"""Read-only, TF-aware 3D cloud projection for Nav2 obstacle observations.

Missing height-slice returns are NaN (unknown), never invented free-space rays.
This projection does not change SLAM calibration or publish a TF.
"""
import json
import math
import signal
import threading
import time

import numpy as np


def project_points(points, translation, quaternion, *, min_height=-0.45, max_height=0.8,
                   range_min=0.15, range_max=12.0, bins=720):
    cloud = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    t = np.asarray(translation, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if t.shape != (3,) or q.shape != (4,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise ValueError('Invalid rigid transform')
    norm = np.linalg.norm(q)
    if abs(norm-1.0) > .01:
        raise ValueError('Invalid transform quaternion')
    if not (0 < range_min < range_max and min_height < max_height and 8 <= bins <= 4096):
        raise ValueError('Invalid obstacle slice configuration')
    x, y, z, w = q/norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    cloud = cloud[np.isfinite(cloud).all(axis=1)] @ rotation.T + t
    distance = np.hypot(cloud[:, 0], cloud[:, 1])
    keep = ((cloud[:, 2] >= min_height) & (cloud[:, 2] <= max_height)
            & (distance >= range_min) & (distance <= range_max))
    cloud, distance = cloud[keep], distance[keep]
    index = np.floor((np.arctan2(cloud[:, 1], cloud[:, 0])+math.pi)/(2*math.pi)*bins).astype(int) % bins
    ranges = np.full(bins, np.inf, dtype=np.float32)
    np.minimum.at(ranges, index, distance)
    ranges[~np.isfinite(ranges)] = np.nan
    return ranges


def projection_health(*, received_monotonic, source_stamp, now_monotonic, now_wall,
                      max_cloud_age, frame, finite_bins, bins, error, input_topic):
    """Report transport freshness separately from usable endpoint evidence.

    A fresh all-NaN scan is a fresh message, not a valid obstacle observation.
    Conversely, even 720 endpoint bins cannot certify observed free volume:
    this producer has neither per-sensor acquisition provenance nor ray origins.
    """
    def real(value):
        return type(value) in (int, float) and math.isfinite(value)

    source_fresh = bool(
        all(real(value) for value in (received_monotonic, source_stamp, now_monotonic,
                                     now_wall, max_cloud_age))
        and max_cloud_age > 0 and 0 <= now_monotonic-received_monotonic <= max_cloud_age
        and 0 <= now_wall-source_stamp <= max_cloud_age)
    counts_valid = (type(bins) is int and bins > 0 and type(finite_bins) is int
                    and 0 <= finite_bins <= bins)
    has_returns = counts_valid and finite_bins > 0
    valid = bool(source_fresh and has_returns and not error)
    return dict(source='lio_cloud_projection' if input_topic == '/d1max/localization/lio/deskewed'
                else 'endpoint_cloud_projection', input_topic=input_topic,
                fresh=valid, valid=valid, source_fresh=source_fresh,
                wall_time=now_wall, source_stamp_sec=source_stamp, frame=frame,
                finite_bins=finite_bins, total_bins=bins,
                unknown_bins=bins-finite_bins if counts_valid else None,
                has_obstacle_returns=bool(has_returns), coverage_verified=False,
                unobserved_semantics='unknown_nan_not_clear',
                error=error or ('' if valid else 'no_obstacle_slice_returns' if source_fresh
                               and not has_returns else 'stale_cloud'))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, LaserScan
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, TransformException

    class Projection(Node):
        def __init__(self):
            super().__init__('navigation_cloud_to_scan')
            defaults = {'input_topic': '/d1max/localization/lio/deskewed',
                'output_topic': '/d1max/navigation/scan', 'target_frame': 'd1max_loc_base_link',
                'min_height': -.45, 'max_height': .8, 'range_min': .15,
                'range_max': 12., 'bins': 720, 'max_cloud_age': .5}
            self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            project_points([], [0, 0, 0], [0, 0, 0, 1], **{k: self.p[k] for k in
                ('min_height', 'max_height', 'range_min', 'range_max', 'bins')})
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.pub = self.create_publisher(LaserScan, self.p['output_topic'], qos_profile_sensor_data)
            self.health = self.create_publisher(String, '/d1max/navigation/scan_status', 1)
            self.sub = self.create_subscription(PointCloud2, self.p['input_topic'], self.cloud, qos_profile_sensor_data)
            self.received = None; self.source_stamp = None
            self.last_error = 'waiting_cloud'; self.hits = 0
            self.timer = self.create_timer(.5, self.status)

        def cloud(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9 - stamp
            if not 0 <= age <= self.p['max_cloud_age']:
                self.last_error = 'stale_cloud'; return
            try:
                transform = self.tf.lookup_transform(self.p['target_frame'], msg.header.frame_id,
                    Time.from_msg(msg.header.stamp))
                xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                t, q = transform.transform.translation, transform.transform.rotation
                ranges = project_points(xyz, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w],
                    **{k: self.p[k] for k in ('min_height', 'max_height', 'range_min', 'range_max', 'bins')})
                out = LaserScan(); out.header = msg.header; out.header.frame_id = self.p['target_frame']
                out.angle_min = -math.pi; out.angle_increment = 2*math.pi/self.p['bins']
                out.angle_max = out.angle_min+(self.p['bins']-1)*out.angle_increment
                out.scan_time = .1; out.range_min = self.p['range_min']; out.range_max = self.p['range_max']
                out.ranges = ranges.tolist()
                self.pub.publish(out); self.received = time.monotonic(); self.source_stamp = stamp
                self.hits = int(np.count_nonzero(np.isfinite(ranges)))
                self.last_error = '' if self.hits else 'no_obstacle_slice_returns'
            except (TransformException, ValueError, TypeError, AssertionError) as error:
                self.last_error = type(error).__name__ + ': ' + str(error)[:150]

        def status(self):
            msg = String(); msg.data = json.dumps(projection_health(
                received_monotonic=self.received, source_stamp=self.source_stamp,
                now_monotonic=time.monotonic(), now_wall=self.get_clock().now().nanoseconds*1e-9,
                max_cloud_age=self.p['max_cloud_age'], frame=self.p['target_frame'],
                finite_bins=self.hits, bins=self.p['bins'], error=self.last_error,
                input_topic=self.p['input_topic']), allow_nan=False)
            self.health.publish(msg)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set()); signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Projection()
        while not stop.is_set() and rclpy.ok(): rclpy.spin_once(node, timeout_sec=.1)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None: node.destroy_node()
        rclpy.try_shutdown()
