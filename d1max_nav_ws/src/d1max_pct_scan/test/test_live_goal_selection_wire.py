"""Actual draft callbacks and messages, without starting ROS or a robot."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback

from d1max_pct_scan.live_view import LiveView


class Selection:
    # Deterministic UI seam; native-map authority is tested separately with the
    # real LiveGoalSelection and the default tomogram, not this fixture.
    mode = 'ground'
    floor_choices = []
    region_choices = []
    point = [1., 2., -.5]

    def set_mode(self, mode):
        self.mode = mode

    def edit(self, point):
        self.point = list(point)
        if self.mode == 'ground':
            self.point[2] = -.5
        return self.feedback()

    def snap_ground(self):
        self.point[2] = -.5
        return self.feedback()

    def feedback(self):
        return dict(xyz=list(self.point), valid=self.point[0] >= 0,
                    ground_z=-.5, floor_id='floor1', layer_id=2,
                    height_error_m=self.point[2]+.5, reason='请选择地面')


def node():
    n = SimpleNamespace(goal_pose=Pose(), goal_selection=Selection(),
        single_floor=True, session={'id':'s'}, frame='d1max_loc_map',
        goal_marker_signature=None, goal_visual_dirty=False, goal_has_yaw=False,
        markers=Mock(), menu=Mock(), ground_menu=1, free_menu=2,
        goal_support_pub=Mock(), goal_editor_pub=Mock(),
        reason='', reason_scope='', goal_request_pending=False,
        navigate_client=Mock(), motion_capable=False, goal_request_sequence=0)
    n.goal_pose.position.x, n.goal_pose.position.y, n.goal_pose.position.z = 1., 2., -.5
    n.goal_pose.orientation.w = 1.
    n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10)))
    for name in ('publish_goal_editor','on_feedback','set_goal_mode','snap_goal_ground'):
        setattr(n, name, lambda *a, _name=name, **kw: getattr(LiveView,_name)(n,*a,**kw))
    return n


def command(**updates):
    return String(data=json.dumps(dict(schema=1,session_id='s',frame_id='d1max_loc_map',**updates)))


def feedback(x, y, z, event=InteractiveMarkerFeedback.POSE_UPDATE):
    f=InteractiveMarkerFeedback(event_type=event)
    f.header.frame_id='d1max_loc_map'
    f.pose.position.x,f.pose.position.y,f.pose.position.z=x,y,z
    f.pose.orientation.w=1.
    return f


def test_drag_ground_follow_is_edit_only_and_does_not_reinsert_same_handles():
    n=node()
    LiveView.on_feedback(n,feedback(3.,4.,8.))
    assert n.goal_pose.position.z == -.5
    LiveView.refresh_goal_visual(n)
    marker=n.markers.insert.call_args.args[0]
    assert [c.name for c in marker.controls]==['move_rotate_3d','move_x','move_y','rotate_z']
    LiveView.on_feedback(n,feedback(4.,4.,7.))
    LiveView.refresh_goal_visual(n)
    n.markers.insert.assert_called_once()
    assert n.markers.setPose.call_count==1
    n.navigate_client.send_goal_async.assert_not_called()
    assert len(n.goal_support_pub.publish.call_args.args[0].markers)==3


def test_invalid_target_remains_editable_and_submit_preserves_existing_task():
    n=node()
    LiveView.on_feedback(n,feedback(-2.,3.,4.,InteractiveMarkerFeedback.MOUSE_UP))
    assert n.goal_pose.position.x == -2.
    assert '调整' in n.markers.insert.call_args.args[0].description
    old_sequence=n.goal_request_sequence
    LiveView.commit_goal(n,None)
    n.navigate_client.send_goal_async.assert_not_called()
    assert n.goal_request_sequence==old_sequence and not n.goal_request_pending
    status=json.loads(n.goal_editor_pub.publish.call_args.args[0].data)
    assert status['schema']==1 and status['can_submit'] is False
    assert status['goal_validation']['valid'] is False


def test_free_xyz_and_snap_use_current_numeric_draft_not_previous_location():
    n=node()
    LiveView.edit_goal(n,command(selection_mode='free',goal_xyz=[6.,7.,2.]))
    assert n.goal_selection.mode=='free' and n.goal_pose.position.z==2.
    names=[c.name for c in n.markers.insert.call_args.args[0].controls]
    assert 'move_z' in names and 'rotate_x' in names
    LiveView.edit_goal(n,command(operation='snap_ground',selection_mode='ground',goal_xyz=[8.,9.,3.]))
    assert (n.goal_pose.position.x,n.goal_pose.position.y,n.goal_pose.position.z)==(8.,9.,-.5)
    n.navigate_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('update',[
    {'schema':2}, {'session_id':'other'}, {'frame_id':'odom'},
    {'selection_mode':'unknown'}, {'goal_xyz':[1.,2.]},
    {'goal_xyz':[True,0.,0.]}, {'goal_xyz':[1.,2.,float('nan')]},
])
def test_bad_or_foreign_commands_do_not_mutate_draft(update):
    n=node()
    value=dict(schema=1,session_id='s',frame_id='d1max_loc_map',selection_mode='free',goal_xyz=[7.,8.,9.])
    value.update(update)
    LiveView.edit_goal(n,String(data=json.dumps(value)))
    assert n.goal_selection.mode=='ground'
    assert (n.goal_pose.position.x,n.goal_pose.position.y,n.goal_pose.position.z)==(1.,2.,-.5)


def test_formal_entry_missing_map_cannot_bypass_validation():
    n=node();n.goal_selection=None
    LiveView.commit_goal(n,None)
    n.navigate_client.send_goal_async.assert_not_called()
    assert '目标地图未就绪' in n.reason
