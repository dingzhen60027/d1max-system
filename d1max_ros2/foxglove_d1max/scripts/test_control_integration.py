#!/usr/bin/env python3
"""TEST ONLY: isolated Domain 91, SDK-free mock executable, never a robot."""
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
MOCK = ROOT.parent / 'sdk_bridge_ws/build/d1max_sdk_bridge/sdk_console_bridge_mock'
assert os.environ.get('ROS_DOMAIN_ID') == '91', 'Only isolated ROS Domain 91 is allowed'
assert os.environ.get('RMW_IMPLEMENTATION') == 'rmw_zenoh_cpp'
assert Path(os.environ['ZENOH_SESSION_CONFIG_URI']).resolve() == ROOT / 'config/zenoh-local.json5'
assert 'librobot_sdk' not in subprocess.check_output(['ldd', str(MOCK)], text=True)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

rclpy.init()
n = Node('d1max_control_mock_test', enable_rosout=False, start_parameter_services=False)
latest, history, children, clients = {}, [], [], {}
renew = False
last_renew = 0

def receive(key, message):
    latest[key] = json.loads(message.data)
    if key == 'behavior':
        history.append(latest[key].copy())

subs = [n.create_subscription(String, topic, lambda m, k=key: receive(k, m), 10)
        for key, topic in [('robot', '/d1max_sdk_bridge/robot_state'),
                           ('behavior', '/d1max_sdk_bridge/behavior_state'),
                           ('gateway', '/d1max/console/status')]]

def client(action, kind=Trigger):
    key=(latest['gateway']['service_prefix'],action)
    if key not in clients:
        clients[key]=n.create_client(kind,key[0]+'/'+action)
    return clients[key]

def spin(seconds=.03):
    global last_renew
    rclpy.spin_once(n, timeout_sec=seconds)
    if renew and 'gateway' in latest and time.monotonic()-last_renew>.35:
        last_renew=time.monotonic()
        c=client('keepalive')
        if c.service_is_ready():c.call_async(Trigger.Request())

def wait(predicate, seconds=10):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        spin()
        if predicate():return
    raise AssertionError('Timed out; latest='+json.dumps(latest,ensure_ascii=False))

def call(action, value=None, success=True):
    kind=Trigger if value is None else SetBool
    c=client(action,kind)
    wait(c.service_is_ready,5)
    request=kind.Request()
    if value is not None:request.data=value
    f=c.call_async(request)
    wait(f.done,6)
    response=f.result()
    assert response.success is success,(action,response.message)
    return response

try:
    # No pre-existing nodes may share the mock namespaces.
    end=time.monotonic()+1
    while time.monotonic()<end:spin()
    assert not latest, 'Test domain already has state publishers'
    children.append(subprocess.Popen([str(MOCK),'--ros-args','-p','robot_ip:=127.0.0.1'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True))
    children.append(subprocess.Popen(['python3',str(ROOT/'gateway/console_gateway.py'),'--live-controls'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True))
    wait(lambda:latest.get('gateway',{}).get('can_arm') is True)
    assert latest['behavior']['sdk_commands_sent']==0
    assert latest['behavior']['sdk_has_control'] is False
    call('stand',success=False)
    call('arm',True);renew=True
    wait(lambda:latest['behavior'].get('lease_valid') is True)
    call('prepare_navigation')
    wait(lambda:latest['robot']['motion_status']==1)
    assert latest['behavior']['ready_for_navigation'] is False
    call('lie_down',success=False)
    wait(lambda:latest['behavior']['ready_for_navigation'] is True)
    assert latest['robot']['motion_status']==5
    assert latest['behavior']['goal_status']=='SUCCEEDED'
    assert any(h['active_transition']=='stand' for h in history)
    velocity=n.create_publisher(String,latest['gateway']['service_prefix']+'/velocity',1)
    seq=0
    end=time.monotonic()+.6
    while time.monotonic()<end:
        seq+=1
        velocity.publish(String(data=json.dumps(dict(session=latest['gateway']['session'],seq=seq,stamp=time.time(),x=.2,y=0.,yaw=0.))))
        spin(.04)
    wait(lambda:latest['robot']['forward_speed']>.1)
    wait(lambda:latest['robot']['forward_speed']==0,3)
    renew=False
    wait(lambda:latest['gateway']['armed'] is False,3)
    wait(lambda:latest['behavior']['ready_for_navigation'] is False)
    # An explicit unlock alone must not restore old velocity/preparation.
    call('arm',True);renew=True
    wait(lambda:latest['behavior']['lease_valid'] is True)
    assert latest['behavior']['ready_for_navigation'] is False
    call('soft_estop',True);renew=False
    wait(lambda:latest['robot']['software_emergency_status']==2)
    wait(lambda:latest['behavior']['goal_status']=='SUCCEEDED')
    assert latest['behavior']['ready_for_navigation'] is False
    call('arm',True);renew=True
    wait(lambda:latest['behavior']['lease_valid'] is True)
    call('recover_estop',False)
    wait(lambda:latest['robot']['software_emergency_status']==1)
    wait(lambda:latest['behavior']['goal_status']=='SUCCEEDED')
    assert latest['behavior']['ready_for_navigation'] is False
    call('release_control');renew=False
    call('arm',False)
    wait(lambda:latest['behavior']['sdk_has_control'] is False)
    report=dict(test_only=True,domain=91,mock_sdk_no_vendor_link=True,startup_commands=0,
                checks=['startup_locked','invalid_transition_rejected','prepare_confirmed_stepwise',
                        'standing_is_not_complete','busy_rejects_second_goal','velocity_watchdog_zero',
                        'lease_expiry_locks','rearm_does_not_resume','estop_preempts_and_confirms',
                        'recovery_does_not_resume','release_survives_ui_lock'],
                final=latest['behavior'])
    (ROOT/'artifacts/control-mock-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
finally:
    for p in reversed(children):
        p.terminate()
        try:out,_=p.communicate(timeout=5)
        except subprocess.TimeoutExpired:p.kill();out,_=p.communicate()
        if p.returncode not in (0,-15):print(out[-6000:])
    n.destroy_node();rclpy.shutdown()
