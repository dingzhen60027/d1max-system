from copy import deepcopy
import pytest
from nav_msgs.msg import Odometry
from d1max_localization.estimation_ros import stamp_time
from d1max_localization.navigation_wire import body_pair_message, local_body_message


def values():
    local = Odometry()
    local.header.stamp = stamp_time(10.2)
    local.header.frame_id, local.child_frame_id = 'd1max_loc_odom', 'd1max_loc_base_link'
    local.pose.pose.orientation.w = 1.
    local.pose.pose.position.x = 2.
    global_ = deepcopy(local)
    global_.header.frame_id = 'd1max_loc_map'
    global_.pose.pose.position.x = 3.
    return dict(local=local,global_=global_,session_id='s',map_version_id='m',epoch=2,
        seed_id='seed',posterior_stamp=10.1,imu_stamp=10.18,extrapolation_sec=.02)


def test_exact_pair_preserves_all_distinct_times_and_measured_geometry():
    result = body_pair_message(**values())
    assert result.schema_version == 2 and result.usable
    assert result.local_odometry.pose.pose.position.x == 2.
    assert result.global_odometry.pose.pose.position.x == 3.
    assert result.source_stamp == stamp_time(10.2)
    assert result.posterior_stamp == stamp_time(10.1)
    assert result.imu_stamp == stamp_time(10.18)
    assert not hasattr(result,'motion_authorized')


@pytest.mark.parametrize('name,value',[('epoch',0),('seed_id',''),('session_id',''),
    ('map_version_id',''),('posterior_stamp',10.3),('imu_stamp',10.3),
    ('extrapolation_sec',0.),('imu_stamp',float('nan'))])
def test_unbound_or_falsified_time_is_not_valid_state(name,value):
    item=values(); item[name]=value
    with pytest.raises(ValueError): body_pair_message(**item)


def test_latest_global_with_old_local_cannot_be_called_atomic_pair():
    item=values(); item['global_'].header.stamp=stamp_time(10.21)
    with pytest.raises(ValueError): body_pair_message(**item)


def test_invalid_pose_not_encoded_as_good_state():
    item=values(); item['local'].pose.pose.position.z=float('nan')
    with pytest.raises(ValueError): body_pair_message(**item)


def test_local_provider_uses_same_integer_imu_bound_as_consumers():
    item = values()
    item.pop('global_')
    item.update(imu_stamp=10.1, extrapolation_sec=.1)
    result = local_body_message(**item)
    assert result.schema_version == 2 and result.usable
    assert result.valid_until == stamp_time(10.3)
    # A float consistency tolerance is not permission to exceed the source
    # provenance bound by a few microseconds and make consumers disagree.
    item['imu_stamp'] = 10.099995
    with pytest.raises(ValueError):
        local_body_message(**item)


def test_pair_and_local_provider_agree_at_exact_provenance_bound():
    item=values();item.update(posterior_stamp=9.8,imu_stamp=10.1,extrapolation_sec=.1)
    pair=body_pair_message(**item)
    local_item=dict(item);local_item.pop('global_')
    local=local_body_message(**local_item)
    assert pair.source_stamp==local.source_stamp
    assert pair.posterior_stamp==local.posterior_stamp==stamp_time(9.8)
    assert pair.imu_stamp==local.imu_stamp==stamp_time(10.1)
    assert local.valid_until==stamp_time(10.3)


@pytest.mark.parametrize('measurement', ['posterior_stamp','imu_stamp'])
def test_pair_provider_rejects_overdue_provenance_at_original_output(measurement):
    item=values()
    if measurement=='posterior_stamp':item[measurement]=9.799999999
    else:item.update(imu_stamp=10.099999999,extrapolation_sec=.1)
    with pytest.raises(ValueError):body_pair_message(**item)


def test_real_navigation_publisher_uses_estimator_identity_and_same_output_sample():
    from unittest.mock import Mock
    from collections import deque
    from d1max_localization.navigation_output import NavigationOutput
    from d1max_localization.math_utils import Pose3
    from test_navigation_estimation import locked, diagonal
    node=NavigationOutput.__new__(NavigationOutput)
    node.core=locked()
    node.p=dict(map_frame='d1max_loc_map',odom_frame='d1max_loc_odom',
                body_frame='d1max_loc_base_link',trajectory_rate_hz=5.)
    node.wire_session=dict(id='frozen-session',version_id='frozen-map')
    node.extrinsic=Pose3((0.,0.,0.),(0.,0.,0.,1.))
    node.global_pub,node.state_pair_pub,node.tf,node.pose_pub,node.path_pub=(Mock() for _ in range(5))
    node.global_times,node.global_arrivals,node.path=deque(),deque(),deque()
    node.last_path=0.
    node.now_s=lambda:10.22
    local=node.core.local[-1]
    node.publish_global(local,Pose3((1.,0.,0.),(0.,0.,0.,1.)),diagonal())
    pair=node.state_pair_pub.publish.call_args.args[0]
    assert pair.local_odometry.header.stamp == pair.global_odometry.header.stamp == stamp_time(local.stamp)
    assert pair.global_odometry == node.global_pub.publish.call_args.args[0]
    assert pair.localization_epoch == node.core.epoch
    assert pair.localization_seed_id == node.core.key()[1]
    assert pair.session_id == 'frozen-session' and pair.map_version_id == 'frozen-map'
    assert pair.source_stamp != stamp_time(node.now_s())
    assert pair.posterior_stamp == stamp_time(local.source_stamp)
    assert node.tf.sendTransform.call_args.args[0].header.stamp == pair.source_stamp
