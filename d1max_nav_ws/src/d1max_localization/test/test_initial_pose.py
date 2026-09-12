import math
import pytest
from d1max_localization.initial_pose import initial_tracking_pose
from d1max_localization.math_utils import yaw_from_quaternion

@pytest.mark.parametrize('yaw',[0,math.pi/2,-math.pi/2,math.pi])
def test_body_pose_uses_one_full_rigid_transform(yaw):
    result=initial_tracking_pose(dict(x=2,y=3,z=.4,yaw=yaw,reference='body'),[.4,.1,-.04],math.pi/2)
    assert result.position==pytest.approx((2+.4*math.cos(yaw)-.1*math.sin(yaw),3+.4*math.sin(yaw)+.1*math.cos(yaw),.36))
    assert math.sin(yaw_from_quaternion(result.orientation))==pytest.approx(math.sin(yaw-math.pi/2))
    assert math.cos(yaw_from_quaternion(result.orientation))==pytest.approx(math.cos(yaw-math.pi/2))

def test_tracking_reference_does_not_apply_body_offset_again():
    result=initial_tracking_pose(dict(x=2,y=3,z=.4,yaw=.1,reference='tracking'),[.4,0,-.04],1.57)
    assert result.position==(2,3,.4)
    assert yaw_from_quaternion(result.orientation)==pytest.approx(.1)

def test_no_automatic_floor_height_or_map_adjustment():
    result=initial_tracking_pose(dict(x=0,y=0,z=0,yaw=0,reference='body'),[.4,0,-.0377],1.56646)
    assert result.position[2]==-.0377

def test_legacy_heading_only_mailbox_keeps_original_position():
    result=initial_tracking_pose(dict(x=1,y=2,z=0,yaw=0,heading_frame='body'),[.4,0,-.04],1.57)
    assert result.position==(1,2,0)

def test_unknown_reference_rejected():
    with pytest.raises(ValueError):initial_tracking_pose(dict(x=0,y=0,z=0,yaw=0,reference='floor'),[0,0,0],0)
