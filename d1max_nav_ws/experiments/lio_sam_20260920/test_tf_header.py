#!/usr/bin/env python3
"""Synthetic TransformFusion TF-header regression, isolated Zenoh/domain 221.

Run after compiling the isolated workspace:
    /usr/bin/python3 /absolute/path/to/test_tf_header.py

No rosbag, sensor adapter, robot SDK, or mapping process is started. This starts
only an owned loopback router and the isolated imuPreintegration executable,
then drives its TransformFusion subscriptions with synthetic Odometry messages.
The IMU-preintegration component receives no IMU or mapping-incremental input.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
ROBOT_ROS = Path("/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2")
DOMAIN = "221"
PORT = 7449
IMU_ODOM = "/lio_sam/tf_header_test/imu"
MAPPING_ODOM = "/lio_sam/mapping/odometry"


def bootstrap():
    """Load the SDK's existing Zenoh library only, never start the robot SDK."""
    if os.environ.get("LIO_SAM_TF_TEST_BOOTSTRAPPED") == "1":
        return
    env = os.environ.copy()
    for key in ("LD_LIBRARY_PATH", "PYTHONPATH", "AMENT_PREFIX_PATH",
                "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH", "ZENOH_CONFIG_OVERRIDE",
                "ZENOH_SESSION_CONFIG_URI", "ZENOH_ROUTER_CONFIG_URI"):
        env.pop(key, None)
    command = (
        "set -eo pipefail; "
        f"source {shlex.quote(str(ROBOT_ROS / 'd1max_ros2_env.sh'))}; "
        f"source {shlex.quote(str(HERE / 'ws/install/local_setup.bash'))}; "
        "export LIO_SAM_TF_TEST_BOOTSTRAPPED=1 RMW_IMPLEMENTATION=rmw_zenoh_cpp "
        f"ROS_DOMAIN_ID={DOMAIN} OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1; "
        f"exec /usr/bin/python3 {shlex.quote(str(Path(__file__).resolve()))} "
        + " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    )
    os.execve("/bin/bash", ["bash", "--noprofile", "--norc", "-c", command], env)


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Discovery and each assertion deadline, seconds")
    args = parser.parse_args()
    if not 1 <= args.timeout <= 30:
        parser.error("--timeout must be between 1 and 30 seconds")
    bootstrap()
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp" or os.environ.get("ROS_DOMAIN_ID") != DOMAIN:
        raise RuntimeError("Refusing non-isolated RMW/domain")

    # Lock and port checks precede process creation; existing services are never
    # killed or reused by this test.
    lock = (HERE / "tf_header_test.lock").open("a+")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    with socket.socket() as sock:
        try:
            sock.bind(("127.0.0.1", PORT))
        except OSError as exc:
            raise RuntimeError(f"Port {PORT} is occupied; leaving existing process untouched") from exc

    executable = HERE / "ws/install/lio_sam/lib/lio_sam/lio_sam_imuPreintegration"
    router_exe = ROBOT_ROS / "local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
    for required in (executable, router_exe):
        if not required.is_file():
            raise FileNotFoundError(required)
    output = HERE / "tf_header_tests" / (time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    output.mkdir(parents=True, exist_ok=False)
    router_config = {
        "mode": "router", "connect": {"endpoints": []},
        "listen": {"endpoints": [f"tcp/127.0.0.1:{PORT}"], "exit_on_failure": True},
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        "timestamping": {"enabled": {"router": True, "peer": True, "client": True},
                         "drop_future_timestamp": False},
    }
    session_config = {
        "mode": "client", "connect": {"endpoints": [f"tcp/127.0.0.1:{PORT}"], "exit_on_failure": True},
        "listen": {"endpoints": []},
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        "timestamping": {"enabled": True, "drop_future_timestamp": False},
    }
    write_json(output / "router.json5", router_config)
    write_json(output / "session.json5", session_config)
    os.environ["ZENOH_ROUTER_CONFIG_URI"] = str(output / "router.json5")
    os.environ["ZENOH_SESSION_CONFIG_URI"] = str(output / "session.json5")
    os.environ.pop("ZENOH_CONFIG_OVERRIDE", None)

    # Import only after the isolated domain/session are selected.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    import yaml

    params = yaml.safe_load((HERE / "ws/src/LIO-SAM/config/params.yaml").read_text())
    params["/**"]["ros__parameters"].update(
        imuTopic="/lio_sam/tf_header_test/unused_imu", odomTopic=IMU_ODOM,
        lidarFrame="lio_lidar", baselinkFrame="lio_lidar", odometryFrame="odom",
        mapFrame="odom", savePCD=False, use_sim_time=False,
        extrinsicRot=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
        extrinsicRPY=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
        extrinsicTrans=[0., 0., 0.], numberOfCores=2,
    )
    (output / "params.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
    result = {"passed": False, "domain": int(DOMAIN), "rmw": "rmw_zenoh_cpp",
              "router": f"127.0.0.1:{PORT}", "synthetic_only": True,
              "executable": str(executable),
              "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
              "cases": [], "transforms": [], "cleanup": {}}
    children, logs = {}, {}
    node = None
    cancelled = False

    def cancel(*_):
        nonlocal cancelled
        cancelled = True

    def start(name, command):
        logs[name] = (output / f"{name}.log").open("w")
        child = subprocess.Popen(command, cwd=HERE, env=os.environ.copy(),
                                 stdout=logs[name], stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children[name] = child
        return child

    def check():
        if cancelled:
            raise KeyboardInterrupt()
        for name, child in children.items():
            if child.poll() is not None:
                raise RuntimeError(f"{name} exited {child.returncode}; inspect {output / (name + '.log')}")

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            check()
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., end - time.monotonic())))

    def stop(name):
        child = children[name]
        sent = []
        for sig, timeout in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            if child.poll() is not None:
                break
            try:
                os.killpg(child.pid, sig)
                sent.append(sig.name)
                child.wait(timeout=timeout)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        result["cleanup"][name] = {"pid": child.pid, "returncode": child.poll(), "signals": sent}

    def on_tf(message):
        for transform in message.transforms:
            if transform.child_frame_id == "lio_lidar":
                result["transforms"].append({
                    "parent": transform.header.frame_id, "child": transform.child_frame_id,
                    "stamp_ns": transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec,
                })

    def odometry(stamp_ns, x=0.):
        message = Odometry()
        message.header.frame_id = "odom"
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        message.child_frame_id = "odom_imu"
        message.pose.pose.orientation.w = 1.
        message.pose.pose.position.x = x
        return message

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    failure = None
    try:
        start("router", [str(router_exe)])
        deadline = time.monotonic() + args.timeout
        while True:
            check()
            try:
                with socket.create_connection(("127.0.0.1", PORT), timeout=.1):
                    break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Owned router failed to listen")
                time.sleep(.05)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("lio_sam_tf_header_regression")
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        mapping_pub = node.create_publisher(Odometry, MAPPING_ODOM, reliable)
        imu_pub = node.create_publisher(Odometry, IMU_ODOM + "_incremental", reliable)
        tf_sub = node.create_subscription(TFMessage, "/tf", on_tf, reliable)
        start("imuPreintegration", [str(executable), "--ros-args", "--params-file", str(output / "params.yaml")])
        deadline = time.monotonic() + args.timeout
        while not (mapping_pub.get_subscription_count() and imu_pub.get_subscription_count()
                   and node.count_publishers("/tf")):
            spin(.05)
            if time.monotonic() >= deadline:
                raise TimeoutError("TransformFusion subscriptions/TF publisher not discovered")

        mapping_stamp = 100_123_456_789
        mapping = odometry(mapping_stamp, x=1.)
        mapping.child_frame_id = "odom_mapping"
        for _ in range(10):
            mapping_pub.publish(mapping)
            spin(.05)

        # A stale/equal increment empties the queue: the previously fixed guard
        # must leave the node alive and publish no transform for these samples.
        for stamp in (mapping_stamp - 100_000_000, mapping_stamp):
            imu_pub.publish(odometry(stamp))
            spin(.1)
        if result["transforms"]:
            raise AssertionError("Unexpected TF for increments not newer than mapping correction")
        result["cases"].append({"name": "stale_and_equal_increment_wait", "passed": True})

        for offset in (100_000_000, 200_000_000):
            expected = mapping_stamp + offset
            before = len(result["transforms"])
            deadline = time.monotonic() + args.timeout
            increment = odometry(expected, x=offset * 1e-9)
            while True:
                imu_pub.publish(increment)
                spin(.05)
                observed = result["transforms"][before:]
                if any(t["parent"] == "odom" and t["stamp_ns"] == expected for t in observed):
                    break
                if observed:
                    raise AssertionError(f"TF header lost/changed: expected odom/{expected}, got {observed}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"No TF received for synthetic stamp {expected}")
            result["cases"].append({"name": "same_lidar_base_frame_header", "passed": True,
                                    "expected_parent": "odom", "expected_stamp_ns": expected,
                                    "observed": observed})
        check()
        result["passed"] = True
    except BaseException as exc:
        failure = exc
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        # Keep the router alive until the node and this rclpy context shut down.
        try:
            if "imuPreintegration" in children:
                stop("imuPreintegration")
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as cleanup_exc:
            result["passed"] = False
            result["cleanup_error"] = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
        finally:
            if "router" in children:
                stop("router")
        for log in logs.values():
            log.close()
        if any(child.poll() is None for child in children.values()):
            result["passed"] = False
            result["cleanup_error"] = "Owned process did not terminate"
        write_json(output / "result.json", result)
        lock.close()
    print(json.dumps({"passed": result["passed"], "output": str(output),
                      "cases": len(result["cases"]), "error": result.get("error")}), flush=True)
    if failure is not None:
        raise failure
    if not result["passed"]:
        raise RuntimeError("Regression cleanup failed")


if __name__ == "__main__":
    main()
