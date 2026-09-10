#!/usr/bin/env python3
"""D1 Max console gateway. Startup NEVER connects to SDK or takes control.

The existing SDK bridge remains responsible for posture transitions and control
ownership. This gateway adds a short UI lease, replay lockout and a per-process
command namespace. Run on the robot-side computer for meaningful link-loss protection.
"""
import argparse
import json
from pathlib import Path
import threading
import time
import signal

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml

from policy import Policy


class ConsoleGateway(Node):
    ACTIONS = ("take_control", "release_control", "stand", "lie_down", "crawl", "general_mode",
               "in_place_mode", "stair_mode", "prepare_navigation", "unlock_to_stand", "reset_error", "halt", "soft_estop", "recover_estop")

    def __init__(self, config, live):
        super().__init__("d1max_console_gateway")
        self.config = config
        self.policy = Policy(config, live)
        self.guard = threading.RLock()
        self.telemetry = ReentrantCallbackGroup()
        self.commands = MutuallyExclusiveCallbackGroup()
        self.safety = MutuallyExclusiveCallbackGroup()
        self.sdk_clients = {}
        self.handles = []
        self.publisher = self.create_publisher(String, "/d1max/console/status", 10)
        self.velocity_pub = self.create_publisher(String, config["cmd_vel_output"], 1) if live else None
        self.lease_pub = self.create_publisher(String, "/d1max/console/control_lease", 1) if live else None
        self.wire_seq = time.time_ns()
        prefix = config["sdk_namespace"].rstrip("/")
        for name, key in (("robot_state", "robot"), ("behavior_state", "behavior"), ("connection_state_text", "connection")):
            self.handles.append(self.create_subscription(String, f"{prefix}/{name}", lambda msg, k=key: self.update(k, msg), 10, callback_group=self.telemetry))
        self.handles.append(self.create_subscription(Clock, "/clock", self.clock_received, qos_profile_sensor_data, callback_group=self.telemetry))
        if live:
            session_prefix = self.policy.status()["service_prefix"]
            self.handles.append(self.create_service(SetBool, f"{session_prefix}/arm", self.arm, callback_group=self.telemetry))
            self.handles.append(self.create_service(Trigger, f"{session_prefix}/keepalive", self.keepalive, callback_group=self.telemetry))
            for action in self.ACTIONS:
                kind = SetBool if action in ("soft_estop", "recover_estop") else Trigger
                target = action
                self.sdk_clients[action] = self.create_client(kind, f"{prefix}/{target}", callback_group=self.telemetry)
                group = self.safety if action in ("soft_estop", "halt") else self.commands
                self.handles.append(self.create_service(kind, f"{session_prefix}/{action}", lambda req, res, a=action: self.forward(a, req, res), callback_group=group))
            self.handles.append(self.create_subscription(String, f"{session_prefix}/velocity", self.velocity, 1, callback_group=self.telemetry))
        self.create_timer(0.05, self.watchdog, callback_group=self.telemetry)
        self.create_timer(1 / config["status_hz"], self.status, callback_group=self.telemetry)
        self.create_timer(1.0, self.check_graph, callback_group=self.telemetry)
        self.get_logger().info("LIVE CONTROLS available but LOCKED" if live else "READ ONLY: no control services or velocity publisher created")

    def update(self, key, message):
        with self.guard:
            if key == "connection":
                self.policy.connected = message.data == "connected"
                self.policy.connection_at = time.monotonic()
            else:
                try:
                    value = json.loads(message.data)
                    if not isinstance(value, dict):
                        return
                    setattr(self.policy, key, value)
                    setattr(self.policy, key + "_at", time.monotonic())
                    if key == "behavior" and value.get("replay_latched") is True:
                        self.policy.replay = True
                        self.policy.disarm()
                        self.stop()
                except (ValueError, TypeError):
                    return

    def clock_received(self, _message):
        with self.guard:
            self.policy.replay = True
            self.policy.disarm()
            self.stop()

    def check_graph(self):
        if any("rosbag2_player" in name for name in self.get_node_names()):
            self.clock_received(None)

    def stop(self):
        if self.policy.moving and self.velocity_pub is not None:
            self.publish_velocity([0., 0., 0.])
            self.policy.stopped()

    def publish_velocity(self, values):
        self.wire_seq += 1
        self.velocity_pub.publish(String(data=json.dumps(dict(
            session=self.policy.session, seq=self.wire_seq, stamp=time.time(),
            x=values[0], y=values[1], yaw=values[2]))))

    def publish_lease(self):
        if self.lease_pub is not None:
            self.wire_seq += 1
            self.lease_pub.publish(String(data=json.dumps(dict(
                session=self.policy.session, seq=self.wire_seq, stamp=time.time(),
                armed=self.policy.armed))))

    def watchdog(self):
        with self.guard:
            if self.policy.should_stop():
                self.stop()
            if self.policy.arm_reason():
                self.policy.disarm()

    def status(self):
        with self.guard:
            self.publish_lease()
            self.publisher.publish(String(data=json.dumps(self.policy.status(), ensure_ascii=False)))

    def arm(self, request, response):
        with self.guard:
            if request.data:
                reason = self.policy.arm()
            else:
                self.policy.disarm()
                self.stop()
                reason = ""
            response.success = not reason
            response.message = reason or ("操作租约已启用" if request.data else "操作已锁定")
            self.publish_lease()
        return response

    def keepalive(self, _request, response):
        with self.guard:
            reason = self.policy.renew()
            response.success, response.message = not reason, reason or "lease renewed"
        return response

    def velocity(self, message):
        try:
            command = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self.guard:
            values = self.policy.velocity(command, time.time())
            if values is None:
                return
            self.publish_velocity(values)

    def forward(self, action, request, response):
        with self.guard:
            reason = self.policy.action_reason(action)
            if reason:
                response.success, response.message = False, reason
                return response
            # Distinct endpoints prevent accidental release through the red stop button.
            if action == "soft_estop" and not request.data or action == "recover_estop" and request.data:
                response.success, response.message = False, "急停与解除请求不匹配"
                return response
            self.stop()
            if action == "soft_estop":
                self.policy.disarm()
                self.publish_lease()
            client = self.sdk_clients[action]
            if not client.service_is_ready():
                response.success, response.message = False, "SDK 服务未就绪"
                return response
            self.policy.request_inflight = True
            try:
                future = client.call_async(request)
            except Exception as error:
                self.policy.request_inflight = False
                self.policy.transport_error = "发送 SDK 服务请求异常，执行状态未知：" + str(error)
                self.policy.disarm()
                self.publish_lease()
                response.success, response.message = False, self.policy.transport_error
                return response
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(self.config["service_timeout_seconds"]):
            # Timeout is ambiguous: do not claim failure to execute or retry motion.
            response.success, response.message = False, "等待 SDK 响应超时；执行状态未知，请检查机器人反馈，勿重复点击"
            with self.guard:
                self.policy.request_inflight = False
                self.policy.transport_error = response.message + "；需重启网关后重新确认"
                self.policy.disarm()
                self.publish_lease()
            return response
        try:
            result = future.result()
            response.success, response.message = result.success, result.message
            with self.guard:
                if result.success and action not in ("halt", "reset_error"):
                    try:
                        self.policy.pending_goal_id = max(self.policy.pending_goal_id, int(json.loads(result.message)["goal_id"]))
                    except (ValueError, KeyError, TypeError):
                        self.policy.transport_error = "控制桥未返回有效目标编号；执行状态未知"
                        self.policy.disarm()
                self.policy.request_inflight = False
        except Exception as error:
            response.success, response.message = False, str(error)
            with self.guard:
                self.policy.request_inflight = False
                self.policy.transport_error = "控制服务异常；执行状态未知，需重新核对"
                self.policy.disarm()
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).resolve().parents[1] / "config/gateway.yaml"))
    parser.add_argument("--live-controls", action="store_true", help="explicitly expose protected control services; does not arm or take control")
    args, ros_args = parser.parse_known_args()
    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("namespace") != "/d1max/console":
        raise ValueError("This panel version requires namespace /d1max/console")
    if config.get("cmd_vel_output") != "/d1max/console/guarded_velocity":
        raise ValueError("Console velocity must use the private guarded channel, never /cmd_vel")
    for key in ("state_stale_seconds", "lease_seconds", "command_timeout_seconds", "service_timeout_seconds", "max_forward", "max_lateral", "max_yaw", "status_hz"):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not 0 < value < 100:
            raise ValueError(f"Invalid configuration: {key}")
    if config["command_timeout_seconds"] > 0.30 or config["lease_seconds"] > 2.0:
        raise ValueError("Safety timeouts exceed supported limits")
    if any(config[key] > limit for key, limit in (("max_forward", 0.30), ("max_lateral", 0.20), ("max_yaw", 0.50))):
        raise ValueError("Control proportions exceed this console's verified limits")
    # Keep the ROS context alive until the final disarm/zero has been published.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = ConsoleGateway(config, args.live_controls)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with node.guard:
            node.policy.disarm()
            node.stop()
            node.publish_lease()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
