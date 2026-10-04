from copy import deepcopy
from types import SimpleNamespace as N
import pytest

from d1max_pct_scan.atomic_projection_state import validate


def stamp(value):
    return N(sec=value // 10**9, nanosec=value % 10**9)


def message():
    t = 10_000_000_000
    def odom(frame):
        return N(header=N(stamp=stamp(t), frame_id=frame), child_frame_id='d1max_loc_base_link',
            pose=N(pose=N(position=N(x=1., y=2., z=.5), orientation=N(x=0., y=0., z=0., w=1.))),
            twist=N(twist=N(linear=N(x=.1,y=0.,z=0.), angular=N(x=0.,y=0.,z=.1))))
    return N(schema_version=2, usable=True, session_id='s', map_version_id='m',
        localization_epoch=1, localization_seed_id='seed', source_stamp=stamp(t),
        posterior_stamp=stamp(t), imu_stamp=stamp(t), extrapolation_sec=0.,
        local_odometry=odom('d1max_loc_odom'), global_odometry=odom('d1max_loc_map'))


def check(m, now=10_020_000_000):
    return validate(m, session_id='s', map_version_id='m', now_ns=now)


def test_same_sample_not_latest_tf():
    m = message()
    m.global_odometry.pose.pose.position.x += 1.
    out = check(m)
    assert out.local_body.position == (1.,2.,.5)
    assert out.global_body.position == (2.,2.,.5)
    assert out.source_ns == 10_000_000_000


@pytest.mark.parametrize('field,value', [('schema_version',1),('usable',False),
    ('session_id','other'),('map_version_id','other'),('localization_epoch',0),
    ('localization_seed_id',''),('extrapolation_sec',float('nan'))])
def test_reject_identity_and_invalid_state(field,value):
    m=message(); setattr(m,field,value)
    with pytest.raises(ValueError): check(m)


def test_published_fresh_but_measurement_old():
    m=message(); m.imu_stamp=stamp(9_800_000_000)
    with pytest.raises(ValueError): check(m)
    m=message(); m.posterior_stamp=stamp(9_500_000_000)
    with pytest.raises(ValueError): check(m)


def test_local_global_different_time_forbidden():
    m=message(); m.global_odometry.header.stamp=stamp(10_000_000_001)
    with pytest.raises(ValueError): check(m)


def test_changed_body_or_relabelled_frame_forbidden():
    for container, field, value in [('header','frame_id','map'),('', 'child_frame_id','lidar')]:
        m=message(); target=getattr(m.local_odometry,container) if container else m.local_odometry
        setattr(target,field,value)
        with pytest.raises(ValueError): check(m)


def test_historical_sample_expiry_and_future_are_not_renewed():
    for now in (9_999_999_999,10_401_000_000):
        with pytest.raises(ValueError): check(message(),now)


def test_nonfinite_twist_or_nonunit_quaternion():
    m=message(); m.local_odometry.twist.twist.linear.x=float('nan')
    with pytest.raises(ValueError): check(m)
    m=message(); m.global_odometry.pose.pose.orientation.w=0.
    with pytest.raises(ValueError): check(m)


def test_value_snapshot_is_not_mutated_by_ros_object_reuse():
    m=message(); before=deepcopy(m); out=check(m)
    m.local_odometry.pose.pose.position.x=99.
    assert out.local_body.position[0] == before.local_odometry.pose.pose.position.x
