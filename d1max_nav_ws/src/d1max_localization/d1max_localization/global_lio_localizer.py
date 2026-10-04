"""Startup-stage extension of the pinned LIO seed owner, not a second owner.

The base LioLocalizer remains byte-identical for the sealed release. The new
launch runs THIS subclass instead of (never alongside) the old seed owner.
"""
from dataclasses import fields
import json
from pathlib import Path
import queue
import signal

import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from .lio_localizer import LioLocalizer, PREFIX, seconds, pose_from
from .global_registration import RegistrationConfig, file_sha256
from .startup_relocalization import StartupConfig, StartupRelocalization
from .relocalization_worker import RegistrationWorker


def augment(data, snapshot):
    value=json.loads(data);value['global_relocalization']=snapshot
    return json.dumps(value,ensure_ascii=False,allow_nan=False)


class _StatusPublisher:
    def __init__(self,publisher,snapshot):self.publisher,self.snapshot=publisher,snapshot
    def publish(self,message):
        self.publisher.publish(String(data=augment(message.data,self.snapshot())))


class _StatusQueue(queue.Queue):
    def __init__(self,snapshot):super().__init__(maxsize=1);self.snapshot=snapshot
    def put_nowait(self,data):super().put_nowait(augment(data,self.snapshot()))


class GlobalLioLocalizer(LioLocalizer):
    def __init__(self):
        super().__init__()
        self.map_pcd=self.declare_parameter('map_pcd','').value
        enabled=self.declare_parameter('global_relocalization.enabled',True).value
        registration={f.name:self.declare_parameter('global_relocalization.registration.'+f.name,
            getattr(RegistrationConfig(),f.name)).value for f in fields(RegistrationConfig)}
        startup={f.name:self.declare_parameter('global_relocalization.startup.'+f.name,
            getattr(StartupConfig(),f.name)).value for f in fields(StartupConfig)}
        self.registration_config=RegistrationConfig(**registration)
        self.startup=StartupRelocalization(self.session['id'],enabled,StartupConfig(**startup))
        path=Path(self.map_pcd or self.session.get('map_pcd',''))
        self.relocalization_map_sha256=file_sha256(path) if enabled and path.is_file() else None
        expected=self.session.get('input_hashes',{}).get(str(path.resolve()))
        if expected and expected!=self.relocalization_map_sha256 and enabled:
            self.startup.fail('localization_map_differs_from_session_snapshot')
        self.relocalization_worker=None;self.relocalization_scan=None;self.relocalization_index={}
        self.relocalization_scan_status={};self.seed_origin='manual'
        self.startup_status_sequence=0
        self.startup_snapshot=self.startup.status()
        self.startup_snapshot['requested_map_sha256']=self.relocalization_map_sha256
        self.relocalization_status_pub=self.create_publisher(String,PREFIX+'global_relocalization/status',5)
        self.create_service(Trigger,PREFIX+'global_relocalization/retry',self.on_relocalization_retry)
        self.subscriptions_.append(self.create_subscription(PointCloud2,PREFIX+'lio/deskewed',
            self.on_relocalization_scan,qos_profile_sensor_data))
        # Reuse the base's one bounded status writer; both ROS and file reports
        # get the same immutable per-tick diagnostic snapshot.
        self.status_pub=_StatusPublisher(self.status_pub,lambda:self.startup_snapshot)
        self.saved=_StatusQueue(lambda:self.startup_snapshot)

    def on_relocalization_retry(self,request,response):
        response.success=self.active_seed is None and self.startup.retry()
        if response.success:self.stop_relocalization_worker()
        response.message='retry_requested' if response.success else 'retry_not_allowed_use_manual_initial_pose'
        return response

    def stop_relocalization_worker(self):
        if self.relocalization_worker is not None:self.relocalization_worker.close()
        self.relocalization_worker=None;self.relocalization_scan=None

    def on_relocalization_scan(self,message):
        if not self.startup.enabled or self.startup.ever_seeded:return
        age=self.now_s()-seconds(message);count=message.width*message.height
        reason=('tracking_frame_mismatch' if message.header.frame_id!=self.p['tracking_frame'] else
                'scan_source_stale_or_future' if not 0<=age<=self.startup.config.max_source_age_s else
                'scan_point_budget_exceeded' if count>self.registration_config.max_scan_points else 'source_ready')
        self.relocalization_scan_status=dict(reason=reason,frame_id=message.header.frame_id,
            source_stamp=seconds(message),points=count)
        if self.startup.request is None and self.startup.state!='failed' and reason=='source_ready':
            self.relocalization_scan=message

    def on_local_sample(self,message):
        super().on_local_sample(message)
        if not hasattr(self,'startup'):return
        if self.core.local_epoch!=self.startup.epoch:
            self.startup.reset_epoch(self.core.local_epoch)
            if self.startup.ever_seeded:self.stop_relocalization_worker()
        try:
            value=json.loads(message.data);t=int(value.get('stamp_ns',0))*1.e-9
            if (value.get('valid') is True and value.get('epoch')==self.core.local_epoch
                    and self.core.stream_valid and self.core.local
                    and abs(self.core.local[-1][0]-t)<1.e-6 and not self.core.fault):
                self.startup.observe(self.core.local_epoch,self.core.local[-1][0],self.core.local[-1][1],value['linear'],value['angular'])
        except (ValueError,KeyError,TypeError):pass

    def publish_seed(self,pose,stamp):
        # Manual input, startup candidate and bounded local recovery all pass
        # the SAME existing seed owner and native fine-matcher channel.
        if hasattr(self,'startup'):self.stop_relocalization_worker()
        super().publish_seed(pose,stamp)
        if hasattr(self,'startup'):
            origin='recovery' if self.core.recovery else getattr(self,'seed_origin','manual')
            self.startup.mark_seed(self.active_seed,stamp,origin)

    def poll_relocalization(self,now):
        if not self.startup.enabled:return
        if self.startup.ever_seeded or self.startup.state=='failed':self.stop_relocalization_worker();return
        if self.relocalization_worker is None:
            try:
                path=Path(self.map_pcd or self.session.get('map_pcd',''))
                if not path.is_file():raise ValueError('missing_original_localization_pcd')
                self.relocalization_worker=RegistrationWorker(path,self.directory.parent/'global_relocalization_cache',
                    self.registration_config,self.relocalization_map_sha256)
                self.startup.state=self.startup.reason='indexing'
            except (ValueError,OSError,RuntimeError) as error:self.startup.fail(str(error))
            return
        result=self.relocalization_worker.poll()
        if result is not None:
            if result['kind']=='error':self.startup.fail(result['reason']);self.stop_relocalization_worker();return
            if result['kind']=='index_ready':
                self.startup.map_sha256=result['map_sha256'];self.relocalization_index=result
                self.startup.state=self.startup.reason='waiting_stationary'
            elif result['kind']=='result':
                if not self.seed_ready():self.startup.fail('initial_seed_inputs_no_longer_ready');self.stop_relocalization_worker();return
                pose=self.startup.accept(result,now)
                if pose is not None:
                    self.core.seed(pose,now)
                    self.seed_origin='global'
                    try:self.publish_seed(pose,now)
                    finally:self.seed_origin='manual'
                elif self.startup.state=='failed':self.stop_relocalization_worker();return
        if self.relocalization_worker is None or self.relocalization_worker.state!='ready':return
        if not self.seed_ready():self.startup.state=self.startup.reason='waiting_lio_and_head_forward';return
        if not self.startup.stationary(now):self.startup.state=self.startup.reason='waiting_stationary';return
        message=self.relocalization_scan
        if message is None:
            self.startup.state='waiting_scan';self.startup.reason='waiting_fresh_deskewed_tracking_scan';return
        request=self.startup.capture(seconds(message),now)
        if request is None:return
        try:
            from .relocalization_cloud import pointcloud_xyz
            points=pointcloud_xyz(message,self.registration_config.max_scan_points)
            if not self.relocalization_worker.submit(points,request['request_id']):self.startup.fail('global_worker_not_ready')
        except (ValueError,OSError,EOFError,BrokenPipeError) as error:self.startup.fail(str(error))
        finally:self.relocalization_scan=None

    def tick(self):
        if hasattr(self,'startup'):
            self.read_command()  # Manual input wins even if a worker result is ready.
            now=self.now_s();self.poll_relocalization(now)
            valid=(self.continuous_pose(now)[0] if self.p.get('navigation_output_enabled') else self.core.tracking(now))
            self.startup.verification(self.active_seed,self.confirmed_seed,valid,now)
            # Lifecycle progress while ROS time is paused is not a sensor
            # observation. The sequence fences duplicate/late status reports.
            self.startup_status_sequence+=1
            self.startup_snapshot=self.startup.status()
            self.startup_snapshot.update(index=self.relocalization_index,scan=self.relocalization_scan_status,
                requested_map_sha256=self.relocalization_map_sha256,
                status_sequence=self.startup_status_sequence,received_at_unix=now)
            self.relocalization_status_pub.publish(String(data=json.dumps(self.startup_snapshot,allow_nan=False)))
        super().tick()

    def close(self):
        self.stop_relocalization_worker()
        super().close()


def main(args=None):
    rclpy.init(args=args);node=GlobalLioLocalizer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.close();node.destroy_node();rclpy.try_shutdown()
