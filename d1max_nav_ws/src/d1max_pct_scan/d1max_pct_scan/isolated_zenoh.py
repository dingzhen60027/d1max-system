"""Owned loopback-only ROS test graph. Never changes the caller's environment."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid


def environment(directory, port, inherited=None):
    directory = Path(directory).resolve()
    endpoint = f'tcp/127.0.0.1:{port}'
    if not 1024 <= port <= 65535:
        raise ValueError('invalid_private_router_port')
    common = dict(scouting={'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
                  timestamping={'enabled': True, 'drop_future_timestamp': False})
    configs = dict(router=dict(common, mode='router', listen={'endpoints': [endpoint],
        'exit_on_failure': True}, connect={'endpoints': []}),
        client=dict(common, mode='client', connect={'endpoints': [endpoint],
        'exit_on_failure': True}, listen={'endpoints': []}))
    directory.mkdir(parents=True, exist_ok=True)
    for name, value in configs.items():
        (directory/(name+'.json5')).write_text(json.dumps(value, indent=2)+'\n')
    env = dict(os.environ if inherited is None else inherited)
    for name in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG','ZENOH_ROUTER_CONFIG',
                 'ROS_LOCALHOST_ONLY'):
        env.pop(name, None)
    env.update(RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='219',
        ZENOH_SESSION_CONFIG_URI=str(directory/'client.json5'),
        ZENOH_ROUTER_CONFIG_URI=str(directory/'router.json5'),
        D1MAX_OFFLINE_ZENOH_TEST='1', D1MAX_NAV_TRANSPORT='isolated_mock',
        D1MAX_NAV_ISOLATED='1', D1MAX_NAV_ISOLATION_TOKEN=uuid.uuid4().hex,
        ROS_LOG_DIR=str(directory/'ros_logs'),OPENBLAS_NUM_THREADS='1',
        MKL_NUM_THREADS='1',OMP_NUM_THREADS='2')
    validate_environment(env)
    return env


def validate_environment(env=None):
    env = os.environ if env is None else env
    if (env.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or env.get('ROS_DOMAIN_ID') != '219'
            or env.get('D1MAX_OFFLINE_ZENOH_TEST') != '1'
            or env.get('D1MAX_NAV_TRANSPORT') != 'isolated_mock'
            or env.get('D1MAX_NAV_ISOLATED') != '1'
            or len(env.get('D1MAX_NAV_ISOLATION_TOKEN','')) != 32
            or any(env.get(k) for k in ('ZENOH_CONFIG_OVERRIDE','ZENOH_SESSION_CONFIG','ZENOH_ROUTER_CONFIG'))):
        raise ValueError('private_zenoh_isolation_environment_required')
    router = json.loads(Path(env['ZENOH_ROUTER_CONFIG_URI']).read_text())
    client = json.loads(Path(env['ZENOH_SESSION_CONFIG_URI']).read_text())
    endpoint = router.get('listen',{}).get('endpoints',[])
    if (len(endpoint) != 1 or not endpoint[0].startswith('tcp/127.0.0.1:')
            or router.get('mode') != 'router' or client.get('mode') != 'client'
            or router.get('connect',{}).get('endpoints') != []
            or client.get('listen',{}).get('endpoints') != []
            or client.get('connect',{}).get('endpoints') != endpoint):
        raise ValueError('private_zenoh_endpoint_contract_invalid')
    for value in (router, client):
        if value.get('scouting') != {'multicast': {'enabled': False}, 'gossip': {'enabled': False}}:
            raise ValueError('private_zenoh_discovery_must_be_disabled')
    return endpoint[0]


def stop_owned(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3.)


@contextmanager
def private_router(directory):
    from ament_index_python.packages import get_package_prefix
    with socket.socket() as reserve:
        reserve.bind(('127.0.0.1', 0))
        port = reserve.getsockname()[1]
    env = environment(directory, port)
    binary = Path(get_package_prefix('rmw_zenoh_cpp'))/'lib/rmw_zenoh_cpp/rmw_zenohd'
    with (Path(directory)/'router.log').open('w') as log:
        router = subprocess.Popen([str(binary)], env=env, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+5.
            while time.monotonic()<deadline:
                if router.poll() is not None:
                    raise RuntimeError('private_router_start_failed')
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):
                        break
                except OSError:
                    time.sleep(.05)
            else:
                raise RuntimeError('private_router_start_timeout')
            yield env
        finally:
            stop_owned(router)
