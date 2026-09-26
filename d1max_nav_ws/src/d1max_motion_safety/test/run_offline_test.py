#!/usr/bin/env python3
"""Run native memory-fixture tests through private loopback Zenoh only."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: run_offline_test.py TEST_BINARY [GTEST_ARGS...]")
    router_binary = Path(get_package_prefix("rmw_zenoh_cpp")) / "lib/rmw_zenoh_cpp/rmw_zenohd"
    with tempfile.TemporaryDirectory(prefix="d1max_collision_offline_") as directory:
        directory = Path(directory)
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        endpoint = f"tcp/127.0.0.1:{port}"
        common = {"scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                  "timestamping": {"enabled": True, "drop_future_timestamp": False}}
        router = {**common, "mode": "router", "listen": {"endpoints": [endpoint], "exit_on_failure": True},
                  "connect": {"endpoints": []}}
        session = {**common, "mode": "client", "connect": {"endpoints": [endpoint], "exit_on_failure": True}}
        (directory / "router.json5").write_text(json.dumps(router))
        (directory / "client.json5").write_text(json.dumps(session))
        env = dict(os.environ)
        for name in ("ZENOH_CONFIG_OVERRIDE", "ZENOH_SESSION_CONFIG", "ZENOH_ROUTER_CONFIG", "ROS_LOCALHOST_ONLY"):
            env.pop(name, None)
        env.update(RMW_IMPLEMENTATION="rmw_zenoh_cpp", ROS_DOMAIN_ID="219", D1MAX_OFFLINE_ZENOH_TEST="1",
                   ZENOH_SESSION_CONFIG_URI=str(directory / "client.json5"),
                   ZENOH_ROUTER_CONFIG_URI=str(directory / "router.json5"),
                   ROS_LOG_DIR=str(directory / "ros_logs"))
        with (directory / "router.log").open("w+") as log:
            child = subprocess.Popen([str(router_binary)], env=env, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            try:
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    if child.poll() is not None:
                        log.seek(0)
                        raise RuntimeError("Private Zenoh router exited: " + log.read())
                    try:
                        with socket.create_connection(("127.0.0.1", port), timeout=.1):
                            break
                    except OSError:
                        time.sleep(.05)
                else:
                    raise RuntimeError("Private Zenoh router did not become ready")
                print(f"OFFLINE: rmw_zenoh_cpp domain=219 endpoint={endpoint}; no uplink, no SDK", flush=True)
                return subprocess.run(sys.argv[1:], env=env, timeout=25, check=False).returncode
            finally:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=2)


if __name__ == "__main__":
    raise SystemExit(main())
