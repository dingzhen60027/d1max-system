"""Production read-only checks. No service clients, commands, TF or motion publishers."""
import subprocess
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

def websocket_check():
    node='/home/dndx/go2_nav/install/toolchain/node-v24.19.0-linux-x64/bin/node'
    output=subprocess.check_output([node,str(Path(__file__).with_name('verify-monitor-ws.mjs'))],text=True,timeout=8)
    return json.loads(output)

rclpy.init()
node=Node('d1max_verify_monitor',enable_rosout=False,start_parameter_services=False)
data,counts,clouds={},{},{}
def receive(key,message):
    data[key]=json.loads(message.data);counts[key]=counts.get(key,0)+1
for key,topic in {'robot':'/d1max_sdk_bridge/robot_state','behavior':'/d1max_sdk_bridge/behavior_state','monitor':'/d1max/monitor/status','maps':'/d1max/maps/status'}.items():
    node.create_subscription(String,topic,lambda m,k=key:receive(k,m),10)
def cloud(key,message):
    clouds[key]=dict(points=message.width*message.height,frame=message.header.frame_id,bytes=len(message.data))
    counts[key]=counts.get(key,0)+1
for side in ['front','rear']:
    node.create_subscription(PointCloud2,'/'+side+'_lidar',lambda m,k=side:cloud(k,m),qos_profile_sensor_data)
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(PointCloud2,'/d1max/maps/building/points',lambda m:cloud('pcd',m),qos)
start=time.monotonic()
try:
    while time.monotonic()-start<12:
        rclpy.spin_once(node,timeout_sec=.1)
        if time.monotonic()-start>5 and 'pcd' in clouds and counts.get('robot',0)>=3:break
    services=node.get_service_names_and_types_by_node('d1max_sdk_monitor','/')
    subscribers=node.get_subscriber_names_and_types_by_node('d1max_sdk_monitor','/')
    graph=node.get_service_names_and_types()
    assert counts.get('robot',0)>=3,counts
    assert time.time()-data['robot']['received_at_unix']<2.5,data['robot']
    assert data['behavior']['motion_control_enabled'] is False
    assert data['behavior']['sdk_commands_sent']==0,'A real estop request was issued; check operator event history'
    assert len(services)==1 and services[0][0].endswith('/soft_estop'),services
    assert all(name in ['/clock','/parameter_events'] for name,_ in subscribers),subscribers
    assert not any(name.startswith('/d1max/console/') for name,_ in graph),graph
    assert not any(name.startswith('/d1max_sdk_bridge/') for name,_ in graph),graph
    assert len(node.get_publishers_info_by_topic('/d1max_sdk_bridge/robot_state'))==1
    for topic in ['/d1max/console/control_lease','/d1max/console/guarded_velocity']:
        assert not node.get_publishers_info_by_topic(topic),topic
    assert all(clouds[k]['points']>0 for k in ['front','rear','pcd']),clouds
    assert clouds['pcd']['points']==data['maps']['maps'][0]['display_points']
    assert clouds['pcd']['frame'] not in [clouds['front']['frame'],clouds['rear']['frame']]
    report=dict(time=time.time(),counts=counts,last=data,clouds=clouds,services=services,subscribers=subscribers,websocket=websocket_check())
    (Path(__file__).resolve().parents[1]/'artifacts/monitor-live-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
