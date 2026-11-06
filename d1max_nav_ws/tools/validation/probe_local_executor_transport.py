#!/usr/bin/env python3
"""Actual LocalFollower wire smoke in a private Zenoh graph, without goals/SDK.

Core motion/occupancy tests are separate. This probe verifies generated ABI,
late durable readers, named ports and a live 20 Hz timer, not robot motion.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def stop_peer(process):
    # All peers stay in the observer's owned process group. The outer timeout
    # can therefore retire the whole private graph, even if its observer fails.
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.)


def observe(args):
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from d1max_planning_interfaces.msg import LocalExecutionStatus, LocalTrajectoryCommit, MotionDemand
    rclpy.init()
    node = rclpy.create_node('local_executor_transport_observer')
    durable = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    seen, demands, received = [], [], []
    follower = None
    peers=[]
    logs=[]
    try:
        if args.session is not None:
            commands=json.loads((args.session/'runtime_bundle.json').read_text())['commands']
            for role in ('reference','scan'):
                command=commands[role]
                if (role=='reference' and command[1:3]!=['-m','d1max_pct_scan.continuous_reference_node']
                        or role=='scan' and Path(command[0]).name!='scan_planner_node'):
                    raise ValueError('unexpected_passive_peer_command')
                log=(args.output/(role+'.log')).open('w')
                logs.append(log)
                peers.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
        with (args.output/'follower.log').open('w') as log:
            follower = subprocess.Popen([str(args.follower), '--ros-args',
                '-p', 'session_id:=transport_probe', '-p', 'map_version_id:=transport_probe_map',
                '-p', 'transport_mode:=isolated_mock', '-p', 'controller_frequency_hz:=20.0'],
                stdout=log, stderr=subprocess.STDOUT)
            # Subscribe after startup: the real BT/reference use durable readers.
            time.sleep(.3)
            def status(message):
                seen.append(message)
                received.append(time.monotonic())
            subscriptions = [
                node.create_subscription(LocalExecutionStatus,
                    '/d1max/live_planning/execution/local_status', status, durable),
                node.create_subscription(LocalTrajectoryCommit,
                    '/d1max/live_planning/execution/local_commit', lambda _: None, durable),
                node.create_subscription(MotionDemand,
                    '/d1max/live_planning/execution/demand_unchecked', demands.append, 1),
            ]
            deadline = time.monotonic()+3.
            while time.monotonic()<deadline:
                if follower.poll() is not None:
                    raise RuntimeError('actual_follower_exited')
                if any(p.poll() is not None for p in peers):
                    raise RuntimeError('actual_passive_peer_exited')
                rclpy.spin_once(node, timeout_sec=.03)
            if len(seen)<20 or len(demands)<20:
                raise RuntimeError('actual_wire_delivery_failed')
            if any(s.schema_version!=1 or s.ready or s.at_goal or s.body_source_stamp.sec
                   or s.body_source_stamp.nanosec for s in seen):
                raise RuntimeError('idle_status_claimed_observation_or_navigation')
            if any(not d.hold or any((d.velocity.linear.x, d.velocity.linear.y, d.velocity.angular.z))
                   for d in demands):
                raise RuntimeError('idle_follower_produced_motion')
            prefix='/d1max/live_planning/execution/'
            for name, typename in (('local_status','LocalExecutionStatus'), ('local_commit','LocalTrajectoryCommit')):
                endpoints=node.get_publishers_info_by_topic(prefix+name)
                if not any(e.node_name=='trajectory_tracker' and e.topic_type.endswith('/'+typename)
                           and e.qos_profile.reliability==ReliabilityPolicy.RELIABLE
                           and e.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for e in endpoints):
                    details=[dict(node=e.node_name,type=e.topic_type,
                        reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability)) for e in endpoints]
                    raise RuntimeError('actual_durable_contract_missing:'+name+':'+json.dumps(details))
            retirement=node.get_subscriptions_info_by_topic(prefix+'route_delivery_receipt')
            if not any(e.node_name=='trajectory_tracker' and e.topic_type.endswith('/RouteDeliveryReceipt')
                       and e.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for e in retirement):
                raise RuntimeError('actual_retirement_port_missing')
            if peers:
                for topic,name,typename in (
                    ('/d1max/localization/navigation/local_state','d1max_continuous_reference','LocalNavigationState'),
                    ('/d1max/live_planning/body_pose','scan_planner_node','Odometry')):
                    endpoints=node.get_subscriptions_info_by_topic(topic)
                    if not any(e.node_name==name and e.topic_type.endswith('/'+typename)
                            and e.qos_profile.reliability==ReliabilityPolicy.RELIABLE
                            and e.qos_profile.depth==8 for e in endpoints):
                        raise RuntimeError('actual_acquisition_reader_contract_missing:'+name)
            intervals=sorted(b-a for a,b in zip(received,received[1:]) if b-a>.001)
            result=dict(schema=1, scope='actual_local_follower_ros_wire_not_navigation_acceptance',
                statuses=len(seen), demands=len(demands),
                observed_interval_p50_s=intervals[len(intervals)//2],
                observed_interval_p95_s=intervals[min(len(intervals)-1,int(len(intervals)*.95))],
                configured_control_hz=20., durable_contracts_verified=True,
                retirement_topic_verified=True, sdk_started=False, goals_submitted=False,
                production_touched=False, nonzero_commands=0)
            result['actual_reference_and_native_readers_verified']=bool(peers)
            print(json.dumps(result))
    finally:
        if follower is not None:
            stop_peer(follower)
        for process in peers:
            stop_peer(process)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--follower', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--session', type=Path, help='Sealed isolated session; only reference and SCAN ports are opened')
    parser.add_argument('--observer', action='store_true', help=argparse.SUPPRESS)
    args=parser.parse_args()
    args.follower=args.follower.resolve(strict=True)
    args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.session is not None:
        args.session=args.session.resolve(strict=True)
        if json.loads((args.session/'session.json').read_text())['transport_mode']!='isolated_mock':
            raise ValueError('wire_probe_requires_isolated_session')
    if args.observer:
        observe(args)
        return
    from d1max_pct_scan.isolated_zenoh import private_router, stop_owned
    with private_router(args.output/'zenoh') as environment:
        command=[sys.executable,str(Path(__file__).resolve()),'--observer',
            '--follower',str(args.follower),'--output',str(args.output)]
        if args.session is not None:
            command+=['--session',str(args.session)]
        observer=subprocess.Popen(command,env=environment,stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,text=True,start_new_session=True)
        try:
            stdout,stderr=observer.communicate(timeout=15.)
        finally:
            stop_owned(observer)
        (args.output/'observer.log').write_text(stdout+stderr)
        if observer.returncode:
            raise RuntimeError('isolated_observer_failed: '+stderr[-2000:])
        payload=json.loads(stdout.strip().splitlines()[-1])
        (args.output/'result.json').write_text(json.dumps(payload,indent=2)+'\n')
        print(json.dumps(payload))


if __name__=='__main__':
    main()
