"""Read-only bag compatibility check for adapter + LIO, NOT accuracy/speed validation.
Owns a loopback Zenoh domain and subprocesses. Never publishes robot controls.
Run after sourcing D1 ROS environment and nav overlay: python3 replay_lio.py BAG [seconds].
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

def run():
    bag=Path(sys.argv[1]).resolve();duration=float(sys.argv[2]) if len(sys.argv)>2 else 20.
    if not bag.is_dir() or not 5<=duration<=60:raise ValueError('Existing bag and 5..60 seconds required')
    root=Path(tempfile.mkdtemp(prefix='d1max-lio-bag-check-'))
    print('ARTIFACTS='+str(root),flush=True)
    package=Path(__file__).resolve().parents[1]
    endpoint='tcp/127.0.0.1:17448'
    for name,mode in [('router','router'),('client','client')]:
        value={'mode':mode,'connect':{'endpoints':[endpoint] if mode=='client' else []},
               'listen':{'endpoints':[endpoint] if mode=='router' else []},
               'scouting':{'multicast':{'enabled':False},'gossip':{'enabled':False}}}
        (root/(name+'.json5')).write_text(json.dumps(value))
    os.environ.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp',ROS_DOMAIN_ID='218',ZENOH_ROUTER_CONFIG_URI=str(root/'router.json5'),
                      ZENOH_SESSION_CONFIG_URI=str(root/'client.json5'),ROS_LOG_DIR=str(root/'roslogs'),RUST_LOG='warn')
    os.environ.pop('ROS_LOCALHOST_ONLY',None)
    children=[];streams=[];report={'bag':str(bag),'duration_seconds':duration,'accuracy_validation':False}
    def start(command,name):
        stream=(root/(name+'.log')).open('w');streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);children.append(child);return child
    node=None
    try:
        router=start(['ros2','run','rmw_zenoh_cpp','rmw_zenohd'],'router');time.sleep(.7)
        assert router.poll() is None,'Router could not start; refusing shared-port use'
        import rclpy
        from std_msgs.msg import String
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        rclpy.init();node=rclpy.create_node('isolated_bag_observer')
        statuses=[];odometry=[];clocks=[]
        def diagnostic(m):
            for item in m.status:
                if item.name=='d1max_localization/input_clock':clocks.append({v.key:v.value for v in item.values})
        subs=[node.create_subscription(String,'/d1max/localization/lio/status',lambda m:statuses.append(json.loads(m.data)),20),
              node.create_subscription(Odometry,'/d1max/localization/lio/odometry',lambda m:odometry.append(m),30),
              node.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,10)]
        config=str(package/'config/localization.yaml')
        adapter=start(['ros2','run','d1max_localization','dual_lidar_adapter','--ros-args','--params-file',config],'adapter')
        lio=start(['ros2','run','faster_lio','run_mapping_online','--ros-args','--params-file',config,
                   '-r','__ns:=/d1max/localization/lio','-r','__node:=laserMapping','-r','Odometry:=odometry'],'lio')
        end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        player=start(['ros2','bag','play',str(bag),'--rate','1.0','--disable-keyboard-controls',
                      '--topics','/front_lidar','/front_lidar/imu','/rear_lidar'],'play')
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert adapter.poll() is None and lio.poll() is None,'Frontend exited; see logs'
            rclpy.spin_once(node,timeout_sec=.01)
        stamps=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in odometry]
        gaps=[b-a for a,b in zip(stamps,stamps[1:])]
        report.update(status=statuses[-1] if statuses else {},input_clock=clocks[-1] if clocks else {},
                      odometry_samples=len(stamps),observed_hz=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0.,
                      max_odometry_gap_seconds=max(gaps) if gaps else None)
        assert len(odometry)>30 and report['status'].get('ready') and not any(s.get('fault') for s in statuses),report
        report['passed']=True
    finally:
        if node is not None:node.destroy_node();rclpy.try_shutdown()
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        for stream in streams:stream.close()
        (root/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
if __name__=='__main__':run()
