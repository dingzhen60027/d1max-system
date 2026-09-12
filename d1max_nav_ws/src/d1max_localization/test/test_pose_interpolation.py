from math import sin,cos,pi
import pytest
from d1max_localization.math_utils import Pose3,interpolate_pose,yaw_from_quaternion

def pose(x,yaw):
    return Pose3((x,2*x,3*x),(0.,0.,sin(yaw/2),cos(yaw/2)))

def test_interpolates_xyz_and_quaternion_across_yaw_wrap():
    result=interpolate_pose([(10,pose(0,170*pi/180)),(30,pose(2,-170*pi/180))],20,20)
    assert result.position==pytest.approx((1,2,3))
    assert abs(yaw_from_quaternion(result.orientation))==pytest.approx(pi)

@pytest.mark.parametrize('t,gap',[(9,30),(31,30),(20,19)])
def test_never_uses_nearest_sample_or_extrapolates(t,gap):
    assert interpolate_pose([(10,pose(0,0)),(30,pose(2,0))],t,gap) is None

def test_exact_stamp_needs_no_future_sample():
    assert interpolate_pose([(10,pose(1,0))],10,20)==pose(1,0)
