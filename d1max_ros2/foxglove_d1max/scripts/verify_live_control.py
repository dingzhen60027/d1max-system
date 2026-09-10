"""Read-only checks after deployment. No service client or command publisher."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init();n=Node('d1max_verify_locked_control',enable_rosout=False,start_parameter_services=False)
samples={k:[] for k in ('robot','behavior','gateway')}
subs=[n.create_subscription(String,t,lambda m,key=k:samples[key].append(json.loads(m.data)),10)
      for k,t in [('robot','/d1max_sdk_bridge/robot_state'),('behavior','/d1max_sdk_bridge/behavior_state'),('gateway','/d1max/console/status')]]
try:
    start=time.monotonic()
    while time.monotonic()-start<7:rclpy.spin_once(n,timeout_sec=.1)
    assert all(len(v)>=3 for v in samples.values()),'Missing live state streams'
    assert samples['robot'][-1]['received_at_unix']>samples['robot'][0]['received_at_unix']
    b=samples['behavior'][-1];g=samples['gateway'][-1]
    assert b['control_adapter']=='guarded_console_v1'
    assert all(x['sdk_commands_sent']==0 for x in samples['behavior']),'An SDK command was issued; do not claim passive startup'
    assert b['sdk_has_control'] is False and b['lease_valid'] is False
    assert b['ready_for_navigation'] is False and b['fsm_state']=='LOCKED'
    assert g['mode']=='live' and g['armed'] is False and g['can_arm'] is True
    subscribers=n.get_subscriber_names_and_types_by_node('d1max_sdk_bridge','/')
    assert not any(t=='/cmd_vel' for t,_ in subscribers)
    services=n.get_service_names_and_types_by_node('d1max_sdk_bridge','/')
    assert any(t=='/d1max_sdk_bridge/prepare_navigation' for t,_ in services)
    publishers=n.get_publishers_info_by_topic('/d1max_sdk_bridge/robot_state')
    assert len(publishers)==1
    report=dict(time=time.time(),samples={k:len(v) for k,v in samples.items()},last_robot=samples['robot'][-1],behavior=b,gateway=g,subscribers=subscribers,services=services,sdk_commands_sent=0)
    (Path(__file__).resolve().parents[1]/'artifacts/live-control-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
finally:n.destroy_node();rclpy.shutdown()
