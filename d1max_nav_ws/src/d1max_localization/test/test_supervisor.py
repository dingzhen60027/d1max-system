import json
import math
import time
from unittest.mock import Mock
import pytest
import rclpy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from d1max_localization.supervisor import Supervisor

@pytest.fixture
def node(tmp_path):
    (tmp_path/'session.json').write_text(json.dumps({'id':'test-session','version_id':'test-map'}))
    rclpy.init(args=['--ros-args','-p','session_dir:='+str(tmp_path),'-p','allow_sdk_state_velocity_fallback:=true'])
    value=Supervisor();value.twist_pub=Mock();value.initial_pub=Mock()
    try:yield value
    finally:value.destroy_node();rclpy.shutdown()

def sdk(node,head=1,fast=False,age=0,speed=0,**changes):
    now=node.now_seconds()-age
    value={'received_at_unix':now,'head_direction':head,'forward_speed':speed,'lateral_speed':0.,'yaw_speed':0.,
        'source':'sdk_mc' if fast else 'sdk_monitor_estop','frame':'sdk_body','v_body':[speed,0.,0.],'omega_body':[0.,0.,0.],
        'session':'test','generation':0,'source_timestamp_ns':str(int(now*1e9)),'stamp_unix':now,'clock_mode':'source_delta_host_anchor'}
    value.update(changes);message=String();message.data=json.dumps(value)
    node.on_sdk(message,fast)
    return message

def test_speed_requires_fresh_front_heading_and_finite_values(node):
    sdk(node,fast=True);node.twist_pub.publish.assert_not_called()
    sdk(node);assert node.twist_pub.publish.call_count==0
    sdk(node,fast=True);assert node.twist_pub.publish.call_count==1
    sdk(node,head=2);assert node.sdk_sample()==(None,'missing')
    sdk(node,fast=True);assert node.twist_pub.publish.call_count==1
    sdk(node,head=1);sdk(node,fast=True,speed=float('nan'));assert node.twist_pub.publish.call_count==1
    sdk(node,fast=True,age=5);assert node.twist_pub.publish.call_count==1

def test_web_body_heading_converts_without_direct_ekf_reset(node):
    command={'id':'pose-1','session_id':'test-session','created_at':time.time(),'heading_frame':'body','x':1,'y':2,'z':.5,'yaw':0}
    (node.directory/'initial_pose.json').write_text(json.dumps(command));node.read_command(True)
    assert node.command_result['accepted']
    pose=node.initial_pub.publish.call_args.args[0].pose.pose
    assert pose.position.z==.5
    assert pose.orientation.z==pytest.approx(math.sin(-node.p['sdk_to_tracking_yaw']/2))
    assert not node.aligned
    node.read_command(True);assert node.initial_pub.publish.call_count==1

def test_stale_diagnostic_cannot_restore_alignment_after_reseed(node):
    node.seeded=True;node.seed_time=node.now_seconds();node.aligned=False
    m=DiagnosticArray();m.header.stamp=rclpy.time.Time(seconds=node.seed_time-.5).to_msg()
    m.status=[DiagnosticStatus(name='d1max_localization/icp_fusion',values=[KeyValue(key='alignment_locked',value='true')])]
    node.on_diagnostic(m);assert not node.aligned
    m.header.stamp=node.get_clock().now().to_msg();node.on_diagnostic(m);assert node.aligned

def test_explicit_body_position_and_heading_transform_once(node):
    command={'id':'pose-body','session_id':'test-session','created_at':time.time(),'reference':'body','x':1.,'y':2.,'z':0.,'yaw':math.pi/2}
    (node.directory/'initial_pose.json').write_text(json.dumps(command));node.read_command(True)
    pose=node.initial_pub.publish.call_args.args[0].pose.pose
    offset=node.p['tracking_offset_body']
    assert pose.position.x==pytest.approx(1.-offset[1])
    assert pose.position.y==pytest.approx(2.+offset[0])
    assert pose.position.z==pytest.approx(offset[2])
    assert node.command_result['reference']=='body'
    assert not node.aligned

def test_matcher_diagnostics_do_not_grant_alignment(node):
    m=DiagnosticArray();m.header.stamp=node.get_clock().now().to_msg()
    m.status=[DiagnosticStatus(name='d1max_localization/matcher',message='GICP 未收敛',values=[KeyValue(key='attempts',value='3')])]
    node.on_diagnostic(m)
    assert node.matcher=={'attempts':'3','message':'GICP 未收敛'}
    assert not node.aligned


def output_pair(node, roll_deg=0., pitch_deg=0., icp_roll_deg=0., icp_pitch_deg=0., yaw_deg=65.):
    def orientation(pose, roll, pitch, yaw):
        r,p,y=(math.radians(v)/2 for v in (roll,pitch,yaw))
        cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        pose.orientation.x=sr*cp*cy-cr*sp*sy;pose.orientation.y=cr*sp*cy+sr*cp*sy
        pose.orientation.z=cr*cp*sy-sr*sp*cy;pose.orientation.w=cr*cp*cy+sr*sp*sy
    stamp=node.get_clock().now().to_msg()
    node.seeded=True;node.seed_time=node.now_seconds()-1
    local=Odometry();local.header.stamp=stamp;local.header.frame_id=node.p['odom_frame'];local.child_frame_id=node.p['tracking_frame']
    local.pose.pose.orientation.w=1.;node.on_local(local)
    global_odom=Odometry();global_odom.header.stamp=stamp;global_odom.header.frame_id=node.p['map_frame'];global_odom.child_frame_id=node.p['tracking_frame']
    global_odom.pose.pose.position.z=.18;orientation(global_odom.pose.pose,roll_deg,pitch_deg,yaw_deg);node.on_global(global_odom)
    icp=PoseWithCovarianceStamped();icp.header.stamp=stamp;icp.header.frame_id=node.p['map_frame']
    icp.pose.pose.position.z=.18;orientation(icp.pose.pose,icp_roll_deg,icp_pitch_deg,yaw_deg);node.on_accepted_icp(icp)
    node.tf=Mock();node.pose_pub=Mock();node.path_pub=Mock()
    return icp


@pytest.mark.parametrize('roll,pitch', [(90.,-13.3),(0.,45.),(float('nan'),0.)])
def test_invalid_ekf_attitude_never_publishes_verified_tf(node,roll,pitch):
    output_pair(node,roll,pitch,1.4,-1.9)
    assert not node.publish_verified_pose()
    node.tf.sendTransform.assert_not_called();node.pose_pub.publish.assert_not_called();node.path_pub.publish.assert_not_called()
    assert not node.output_guard['ok']


def test_consistent_map_slope_and_height_are_preserved(node):
    output_pair(node,12.,-8.,12.,-8.)
    assert node.publish_verified_pose()
    tf=node.tf.sendTransform.call_args.args[0]
    assert tf.transform.translation.z==pytest.approx(.18)
    assert tf.transform.rotation.x==pytest.approx(node.global_odom.pose.pose.orientation.x)
    assert node.output_guard['tilt_error_deg']==pytest.approx(0.,abs=1e-5)


@pytest.mark.parametrize('missing', ['empty','stale','unsynchronized'])
def test_output_requires_fresh_time_adjacent_accepted_icp(node,missing):
    icp=output_pair(node)
    if missing=='empty':node.accepted_icp.clear()
    else:
        age=2. if missing=='stale' else .4
        icp.header.stamp=rclpy.time.Time(seconds=node.now_seconds()-age).to_msg()
    assert not node.publish_verified_pose()
    node.tf.sendTransform.assert_not_called()


def test_quaternion_sign_does_not_change_tilt_check(node):
    icp=output_pair(node,1.4,-1.9,1.4,-1.9)
    q=icp.pose.pose.orientation;q.x=-q.x;q.y=-q.y;q.z=-q.z;q.w=-q.w
    assert node.publish_verified_pose()


def test_invalid_output_revokes_localized_in_web_status(node):
    output_pair(node,90.,-13.3,1.4,-1.9)
    now=node.now_seconds();node.last_imu=now;node.last_cloud=now
    node.sdk_sample=Mock(return_value=((0.,0.,0.),'mc_stream'))
    node.bias.bias=0.;node.aligned=True;node.diagnostic_time=now
    node.diagnostic={'last_correction_age_sec':'0.0'}
    node.tick()
    status=json.loads((node.directory/'status.json').read_text())
    assert status['state']=='degraded' and not status['localized']
    assert not status['output_guard']['ok']
    assert any('倾角不一致' in w for w in status['warnings'])


def test_reseed_cannot_reuse_previous_accepted_attitude(node):
    output_pair(node)
    command={'id':'pose-new','session_id':'test-session','created_at':time.time(),'reference':'body','x':1.,'y':2.,'z':0.,'yaw':0.}
    (node.directory/'initial_pose.json').write_text(json.dumps(command));node.read_command(True)
    assert not node.accepted_icp and not node.output_guard['ok']
    assert not node.publish_verified_pose()


def test_3d_global_ekf_observes_all_pose_axes():
    from pathlib import Path
    import yaml
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/localization.yaml').read_text())
    global_ekf=config['ekf_global']['ros__parameters']
    assert global_ekf['two_d_mode'] is False
    assert global_ekf['pose0_config'][:6]==[True]*6
    assert global_ekf['publish_tf'] is False
    assert config['ekf_local']['ros__parameters']['two_d_mode'] is False
    assert config['ekf_local']['ros__parameters']['twist0_config'][6:9]==[True]*3
    assert config['ekf_local']['ros__parameters']['imu0_config'][9:12]==[True]*3
    assert global_ekf['odom0_config'][6:12]==[True]*6
    assert global_ekf['smooth_lagged_data'] is True
    assert global_ekf['history_length']>=1.
    assert config['localization_supervisor']['ros__parameters']['allow_sdk_state_velocity_fallback'] is False


def test_dedicated_only_never_fuses_robotstate_velocity(node):
    node.p['allow_sdk_state_velocity_fallback']=False
    sdk(node,speed=.2)
    assert node.head_direction==1
    node.twist_pub.publish.assert_not_called();assert node.sdk_sample()==(None,'missing')
    sdk(node,fast=True,speed=.3)
    assert node.sdk_sample()[1]=='mc_stream'
    assert node.twist_pub.publish.call_count==1
    node.last_fast=node.now_seconds()-1
    sdk(node,speed=.2)
    assert node.sdk_sample()==(None,'missing')
    assert node.twist_pub.publish.call_count==1


def test_speed_report_uses_observed_rate_not_requested_label(node):
    m=String();m.data=json.dumps({'source':'sdk_mc','received_at_unix':node.now_seconds(),'state':'rate_mismatch','observed_hz':20.,'expected_hz':50,'acknowledged':True})
    node.on_speed_report(m)
    assert node.speed_report['observed_hz']==20. and not node.speed_report['rate_ok']


def test_admitted_small_future_mc_is_ready_but_receipt_timeout_still_stops_it(node):
    now=node.now_seconds();node.now_seconds=Mock(return_value=now)
    sdk(node);sdk(node,fast=True,speed=.032,stamp_unix=now+.045)
    assert node.sdk_sample()[1]=='mc_stream'
    assert node.last_fast==pytest.approx(now+.045,rel=0,abs=1e-6)
    node.now_seconds.return_value=now+.31
    assert node.sdk_sample()==(None,'missing')


def test_future_mc_outside_existing_admission_window_remains_rejected(node):
    now=node.now_seconds();node.now_seconds=Mock(return_value=now)
    sdk(node);sdk(node,fast=True,stamp_unix=now+.15)
    node.twist_pub.publish.assert_not_called()
    assert node.sdk_sample()==(None,'missing')


def test_health_uses_same_future_tolerance_for_admitted_imu_and_cloud(node):
    now=node.now_seconds();node.now_seconds=Mock(return_value=now)
    sdk(node);sdk(node,fast=True,stamp_unix=now+.045)
    node.last_imu=now+.012;node.last_cloud=now+.012
    node.tick()
    status=json.loads((node.directory/'status.json').read_text())
    assert all(status['sensors'].values())
    node.now_seconds.return_value=now+1.
    node.tick()
    status=json.loads((node.directory/'status.json').read_text())
    assert not any(status['sensors'].values())


def test_standstill_speed_tolerance_only_gates_gyro_calibration_not_velocity(node):
    from sensor_msgs.msg import Imu
    node.bias.samples=2;node.imu_pub=Mock()
    sdk(node);sdk(node,fast=True,speed=.032)
    assert node.stationary_metric==pytest.approx(.032)
    twist=node.twist_pub.publish.call_args.args[0].twist.twist.linear
    assert math.hypot(twist.x,twist.y,twist.z)==pytest.approx(.032)
    for _ in range(2):
        m=Imu();m.header.frame_id=node.p['lidar_frame'];m.header.stamp=node.get_clock().now().to_msg()
        m.angular_velocity.x=.014;m.angular_velocity.y=.008;m.angular_velocity.z=.006
        node.on_imu(m)
    assert node.bias.bias==pytest.approx((.014,.008,.006))


@pytest.mark.parametrize('linear,angular', [([.06,0.,0.],[0.,0.,0.]),([0.,0.,0.],[0.,.03,0.])])
def test_linear_and_angular_motion_independently_block_bias_calibration(node,linear,angular):
    sdk(node);sdk(node,fast=True,v_body=linear,omega_body=angular)
    assert node.bias.update((.01,.01,.01),node.stationary_metric) is None
    assert node.bias.count==0


@pytest.mark.parametrize('changes',[
    {'source':'sdk_speed_report'}, {'source':'sdk_monitor_estop'}, {'frame':'sdk_world'},
    {'source_timestamp_ns':'0'}, {'source_timestamp_ns':123}, {'source_timestamp_ns':str(2**64)},
    {'v_body':[1,2]}, {'omega_body':[0,float('nan'),0]}, {'stamp_unix':0},
    {'clock_mode':'unix'}, {'generation':True}, {'session':None},
])
def test_mc_contract_rejects_other_sources_and_invalid_payloads(node,changes):
    sdk(node);sdk(node,fast=True,**changes)
    node.twist_pub.publish.assert_not_called()
    assert node.sdk_sample()==(None,'missing')


def test_mc_duplicate_timestamp_is_not_republished(node):
    sdk(node);message=sdk(node,fast=True,speed=.3)
    assert node.twist_pub.publish.call_count==1
    value=json.loads(message.data);value['received_at_unix']=node.now_seconds()
    message.data=json.dumps(value);node.on_sdk(message,True)
    assert node.twist_pub.publish.call_count==1


def test_mc_uses_body_vectors_and_source_aligned_time_not_flat_state_fields(node):
    from d1max_localization.sensor_policy import tracking_velocity
    sdk(node);stamp=node.now_seconds()-.04
    sdk(node,fast=True,speed=2.,v_body=[.2,-.1,0.],omega_body=[0.,0.,.3],stamp_unix=stamp)
    output=node.twist_pub.publish.call_args.args[0]
    expected=tracking_velocity((.2,-.1,.3),node.p['sdk_to_tracking_yaw'],node.p['tracking_offset_body'])
    assert output.twist.twist.linear.x==pytest.approx(expected[0])
    assert output.twist.twist.linear.y==pytest.approx(expected[1])
    assert output.header.stamp.sec+output.header.stamp.nanosec*1e-9==pytest.approx(stamp,rel=0,abs=1e-6)


def test_state_never_fuses_even_when_legacy_fallback_enabled(node):
    node.p['allow_sdk_state_velocity_fallback']=True
    sdk(node,speed=2.)
    node.twist_pub.publish.assert_not_called();assert node.sdk_sample()==(None,'missing')


def test_3axis_imu_basis_and_shared_bias_are_not_cloud_only_leveling(node):
    from sensor_msgs.msg import Imu
    node.bias.samples=2;node.imu_pub=Mock()
    sdk(node);sdk(node,fast=True)
    for _ in range(2):
        m=Imu();m.header.frame_id=node.p['lidar_frame'];m.header.stamp=node.get_clock().now().to_msg()
        m.angular_velocity.x=.01;m.angular_velocity.y=.02;m.angular_velocity.z=.01
        node.on_imu(m)
    assert node.bias.bias==pytest.approx((.01,.02,.01))
    sdk(node,fast=True,v_body=[0.,0.,.2])
    m.header.stamp=node.get_clock().now().to_msg()
    m.angular_velocity.x=.21;m.angular_velocity.y=-.08;m.angular_velocity.z=.31
    node.on_imu(m)
    out=node.imu_pub.publish.call_args.args[0]
    assert out.header.frame_id==node.p['tracking_frame']
    assert (out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z)==pytest.approx((.2,-.1,.3))
    assert out.orientation_covariance[0]==-1.

def test_delayed_icp_uses_relative_3d_motion_for_tilt_guard(node):
    from copy import deepcopy
    node.seeded=True;node.seed_time=node.now_seconds()-2.
    start=node.get_clock().now().nanoseconds-400_000_000
    for i in range(21):
        t=start+i*20_000_000;roll=math.radians(i)
        local=Odometry();local.header.stamp=rclpy.time.Time(nanoseconds=t).to_msg()
        local.header.frame_id=node.p['odom_frame'];local.child_frame_id=node.p['tracking_frame']
        local.pose.pose.orientation.x=math.sin(roll/2);local.pose.pose.orientation.w=math.cos(roll/2)
        local.pose.pose.position.x=i*.02
        node.on_local(local)
        global_odom=deepcopy(local);global_odom.header.frame_id=node.p['map_frame']
        global_odom.pose.pose.position.x+=4.
        node.on_global(global_odom)
    icp=PoseWithCovarianceStamped();icp.header.frame_id=node.p['map_frame']
    icp.header.stamp=rclpy.time.Time(nanoseconds=start).to_msg()
    icp.pose.pose.orientation.w=1.;icp.pose.pose.position.x=4.
    node.on_accepted_icp(icp);node.tf=Mock();node.pose_pub=Mock()
    assert node.publish_verified_pose()
    assert node.output_guard['icp_sync_sec']==pytest.approx(.4)
    assert node.output_guard['tilt_error_deg']==pytest.approx(0.,abs=1e-5)
    assert node.tf.sendTransform.call_args.args[0].transform.translation.x==pytest.approx(4.)

def test_output_timer_does_not_publish_during_sensor_loss(node):
    node.seeded=True;node.aligned=True;node.bias.bias=(0.,0.,0.)
    node.publish_verified_pose=Mock()
    node.output_tick()
    node.publish_verified_pose.assert_not_called()

def test_vertical_mc_motion_includes_pitch_lever_arm(node):
    from d1max_localization.sensor_policy import tracking_twist
    sdk(node);sdk(node,fast=True,v_body=[0.,0.,.2],omega_body=[0.,.3,0.])
    out=node.twist_pub.publish.call_args.args[0]
    expected,_=tracking_twist((0.,0.,.2),(0.,.3,0.),node.p['sdk_to_tracking_yaw'],node.p['tracking_offset_body'])
    assert out.twist.twist.linear.z==pytest.approx(expected[2])
    assert out.twist.covariance[14]>0
    assert out.twist.covariance[35]==1e6


def test_fresh_icp_not_reported_stale_between_slow_diagnostic_updates(node):
    output_pair(node)
    now=node.now_seconds();node.last_imu=now;node.last_cloud=now
    node.sdk_sample=Mock(return_value=((0.,0.,0.),'mc_stream'))
    node.bias.bias=(0.,0.,0.);node.aligned=True
    node.diagnostic_time=now-2.;node.diagnostic={'last_correction_age_sec':'0.1'}
    node.tick()
    status=json.loads((node.directory/'status.json').read_text())
    assert status['correction_age']<.1
    assert status['localized']
