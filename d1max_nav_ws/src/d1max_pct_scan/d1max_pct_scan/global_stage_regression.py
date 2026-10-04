"""Real ROS/BT/native-PCT global-stage probes; no robot movement/SDK."""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String


def require_private_domain():
    if (os.environ.get('D1MAX_NAV_ISOLATED') != '1'
            or os.environ.get('ROS_DOMAIN_ID') != '219'
            or os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp'):
        raise ValueError('Global-stage probes require the private offline Zenoh domain')


class Probe(Node):
    def __init__(self):
        super().__init__('crossfloor_global_stage_probe')
        self.status, self.bt, self.worker = {}, {}, {}
        self.messages = 0
        self.records = []
        self.worker_trace = []
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for attr, topic, qos in (('status','/d1max/pct_preview/status',durable),
                               ('bt','/d1max/live_planning/bt/status',durable),
                               ('worker','/d1max/live_planning/global_status',5)):
            self.create_subscription(String, topic, lambda m, attr=attr: self.receive(attr,m), qos)
        prefix = '/d1max/pct_preview/'
        self.plan = self.create_publisher(Empty,prefix+'plan',5)
        self.clear = self.create_publisher(Empty,prefix+'clear',5)
        self.mode = self.create_publisher(String,prefix+'selection_mode',5)
        self.points = {role: self.create_publisher(PointStamped,prefix+role,5) for role in ('start','goal')}

    def receive(self, attr, message):
        setattr(self,attr,json.loads(message.data))
        if attr=='worker':
            self.worker_trace.append(getattr(self,attr))
            del self.worker_trace[:-200]
        self.messages += 1

    def wait(self, predicate, timeout=15., label='condition'):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError(f'Timed out {label}: editor={self.status.get("state")}/{self.status.get("reason")} '
                           f'bt={self.bt.get("reason")} worker={self.worker.get("reason")}')

    def settle(self, seconds=.3):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(self,timeout_sec=.025)

    def select(self, start, goal):
        self.mode.publish(String(data='free'))
        self.wait(lambda:self.status.get('selection_mode')=='free', label='free XYZ mode')
        for role, xyz in (('start',start),('goal',goal)):
            message = PointStamped()
            message.header.frame_id = self.status['frame_id']
            message.header.stamp = self.get_clock().now().to_msg()
            message.point.x, message.point.y, message.point.z = map(float,xyz)
            self.points[role].publish(message)
            self.wait(lambda role=role,xyz=xyz:self.status.get(role+'_xyz') is not None
                and np.allclose(self.status[role+'_xyz'],xyz,atol=1e-6), label=role+' point receipt')
        self.settle(.65)  # identity replacement and fresh fixture window

    def compute(self,name,start,goal,expected_direction):
        self.select(start,goal)
        revision=self.status['revision']
        self.plan.publish(Empty())
        self.wait(lambda:self.status.get('state') in ('planned','failed')
                  and self.status.get('revision')==revision, label=name+' route')
        if self.status['state']!='planned':
            raise RuntimeError(name+': '+self.status['reason'])
        result=self.status['result']
        assert result['stage']=='global_planning_only' and result['motion_enabled'] is False
        assert result['navigation_arrival_tested'] is False
        assert result['route_hash'] and result['source_frame']=='d1max_loc_map'
        if expected_direction in ('up','down'):
            assert {'floor1','floor2'} <= set(result['floors'])
            assert any(s['kind'].startswith('stair') for s in result['segments'])
        else:
            assert not any(s['kind'].startswith('stair') for s in result['segments'])
        self.wait(lambda:any(n['node']=='ComputeRouteOnce' and n['current']=='SUCCESS'
                  for n in self.bt.get('transitions',[])),label='real BT compute SUCCESS')
        assert self.bt['root_status']=='SUCCESS' and self.bt['motion_enabled'] is False
        assert all(n['type'] not in ('FollowCommittedRoute','VerifyMeasuredArrival') for n in self.bt['nodes'])
        digest=result['route_hash']
        self.settle(1.)
        assert self.status['result']['route_hash']==digest
        self.records.append(dict(case=name,status='passed',result=result,bt=self.bt))
        print(json.dumps(dict(case=name,status='passed',elapsed_s=result['elapsed_s'],
                             route_hash=digest)),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--first-route-only',action='store_true',
                        help='Show the first real BT cross-floor route; leave its draft and path visible')
    args=parser.parse_args()
    require_private_domain()  # Check before opening any ROS transport.
    rclpy.init()
    probe=Probe()
    failure=None
    try:
        probe.wait(lambda:probe.status.get('ready') is True,timeout=90.,label='native warmup + lifecycle')
        start,goal=probe.status['start_xyz'],probe.status['goal_xyz']
        probe.compute('floor1_to_floor2',start,goal,'up')
        if args.first_route_only:
            probe.mode.publish(String(data='ground'))
            probe.wait(lambda:probe.status.get('selection_mode')=='ground',label='restore ground editing')
            return  # finally still writes evidence and closes only this probe
        probe.compute('floor2_to_floor1',goal,start,'down')
        # Actual same-floor measured route sample, not a hand-drawn substitute.
        ws=Path(__file__).resolve().parents[3]
        audit=json.loads((ws/'maps/processed/sc_pgo_20260923_crossfloor_complete_001/route_001/audit.json').read_text())
        route=np.asarray(audit['path_xyz'])
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
        index=int(np.searchsorted(arc,3.))
        probe.compute('same_floor_three_metres',start,route[index].tolist(),'same')
        before=probe.worker.get('last_goal_stamp')
        air=np.asarray(goal)+[0.,0.,.4]
        probe.select(start,air)
        assert probe.status['goal_validation']['valid'] is False
        assert probe.status['goal_validation']['reason_code']=='height_off_surface'
        probe.plan.publish(Empty())
        probe.wait(lambda:probe.status.get('state')=='failed',label='invalid goal rejection')
        probe.settle(.5)
        assert probe.worker.get('last_goal_stamp')==before
        probe.records.append(dict(case='air_goal_rejected_without_native_dispatch',status='passed',
                                  validation=probe.status['goal_validation']))
        # Select a truly blocked production PCT cell near the original start.
        # Endpoint UI must reject it without relocating XY or borrowing level 2.
        from d1max_pct_planner.tomogram_map import TomogramMap
        grid=TomogramMap(ws/'maps/processed/sc_pgo_20260923_crossfloor_complete_001/pct/tomogram.npz',
                         max_ground_step_m=.17)
        lower=grid.ground_known & (grid.ground>=-1.) & (grid.ground<=0.)
        all_lower_slices_blocked=~np.any(lower & grid.valid,axis=0)
        blocked=np.argwhere(lower & (grid.cost>20.) & all_lower_slices_blocked[None,:,:])
        xy=grid.center+(blocked[:,1:]-grid.offset)*grid.resolution
        selected=int(np.argmin(np.linalg.norm(xy-np.asarray(start[:2]),axis=1)))
        key=tuple(blocked[selected])
        target=[*xy[selected],float(grid.ground[key])]
        del grid
        probe.select(start,target)
        assert probe.status['goal_validation']['valid'] is False
        assert probe.status['goal_validation']['reason_code']=='pct_cost_blocked'
        probe.plan.publish(Empty())
        probe.wait(lambda:probe.status.get('state')=='failed',label='blocked-cell rejection')
        probe.settle(.5)
        assert probe.worker.get('last_goal_stamp')==before
        probe.records.append(dict(case='blocked_map_cell_rejected_without_xy_snap',status='passed',
                                  target=target,validation=probe.status['goal_validation']))
        # Clear while a real Action request is being accepted/computed. Late
        # results must not resurrect the route after all endpoints are erased.
        probe.select(start,goal)
        probe.plan.publish(Empty())
        probe.settle(.05)
        probe.clear.publish(Empty())
        probe.wait(lambda:probe.status.get('start_xyz') is None and probe.status.get('goal_xyz') is None,
                   label='clear receipt')
        probe.settle(3.)
        assert probe.status.get('result') is None
        probe.records.append(dict(case='cancel_clear_does_not_resurrect_route',status='passed'))
        probe.compute('after_cancel_new_crossfloor_task',start,goal,'up')
    except (Exception,AssertionError) as exc:
        failure=str(exc) or repr(exc)
    finally:
        output=dict(scope='global_BT_stage_not_navigation_arrival',robot_connected=False,
            simulated_input='stationary_start_fixture_only',motion_enabled=False,
            passed=failure is None,error=failure,cases=probe.records,
            last_editor=probe.status,last_bt=probe.bt,last_worker=probe.worker)
        output['worker_trace']=probe.worker_trace
        Path(args.output).write_text(json.dumps(output,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
        probe.destroy_node()
        rclpy.shutdown()
    if failure:
        raise SystemExit(failure)


if __name__=='__main__':
    main()
