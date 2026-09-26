#!/usr/bin/env python3
"""Supervise a read-only Foxglove bridge + replay publisher in a dedicated ROS domain."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session',type=Path);args=parser.parse_args()
    app=Path(__file__).resolve().parents[1];zenoh=app.parent/'local/opt/ros/humble'
    sys.path.insert(0,str(app))
    from backend.mapping.display_session import active_session
    follow=args.session.resolve().parent==(app/'data/mola').resolve()
    def viewer_command():
        target=active_session(app) if follow else args.session.resolve()
        return [sys.executable,'-m','backend.mapping.visualization',str(target)]+(['--follow-active'] if follow else [])
    for port in (8769,17749):
        with socket.socket() as probe:
            # A clean restart can leave TIME_WAIT sockets, not a live listener.
            # Reuse addresses but never SO_REUSEPORT; listen still rejects an owner.
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind(('127.0.0.1',port))  # No killing or taking over unknown listeners.
            probe.listen(1)
    env={**os.environ,'RMW_IMPLEMENTATION':'rmw_zenoh_cpp','ROS_DOMAIN_ID':'215','ROS_LOCALHOST_ONLY':'1',
        'ZENOH_ROUTER_CONFIG_URI':str(app/'config/mapping/view-router.json5'),
        'ZENOH_SESSION_CONFIG_URI':str(app/'config/mapping/view-session.json5'),
        'AMENT_PREFIX_PATH':f'{zenoh}:/opt/ros/humble',
        'LD_LIBRARY_PATH':f'{zenoh}/lib:{zenoh}/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib',
        'PYTHONPATH':f'{app}:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages',
        'OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'1'}
    children=[];stopping=False
    def stop(_s,_f):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        children.append(subprocess.Popen([str(zenoh/'lib/rmw_zenoh_cpp/rmw_zenohd')],env=env))
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            try:
                with socket.create_connection(('127.0.0.1',17749),timeout=.1):break
            except OSError:time.sleep(.05)
        else:raise RuntimeError('Offline router did not start')
        children.append(subprocess.Popen(['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge','--ros-args',
            '-p','address:=127.0.0.1','-p','port:=8769','-p','num_threads:=2','-p','max_qos_depth:=2',
            '-p','send_buffer_limit:=30000000','-p','capabilities:=[connectionGraph]',
            '-p',"client_topic_whitelist:=['a^']",'-p',"service_whitelist:=['a^']",
            '-p',"param_whitelist:=['a^']",'-p',"asset_uri_allowlist:=['a^']"],env=env))
        children.append(subprocess.Popen(viewer_command(),env=env))
        while not stopping:
            failed=next((p for p in children if p.poll() is not None),None)
            if failed is children[2] and failed.returncode==75 and follow:
                children[2]=subprocess.Popen(viewer_command(),env=env)
                continue
            if failed:raise RuntimeError(f'Display child exited: {failed.args[0]} ({failed.returncode})')
            time.sleep(.2)
    finally:
        for p in reversed(children):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        deadline=time.monotonic()+5
        for p in children:
            try:p.wait(timeout=max(.05,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:p.kill();p.wait()


if __name__=='__main__':main()
