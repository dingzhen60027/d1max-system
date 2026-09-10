"""Pure safety policy. No ROS, SDK or network side effects in this module."""
import math
import secrets
import time


class Policy:
    def __init__(self, config, live=False, monotonic=time.monotonic):
        self.config = config
        self.live = live
        self.monotonic = monotonic
        self.session = secrets.token_hex(8)
        self.robot = {}
        self.behavior = {}
        self.robot_at = self.behavior_at = self.connection_at = -math.inf
        self.connected = False
        self.replay = False
        self.lease_until = -math.inf
        self.last_seq = -1
        self.last_velocity_at = -math.inf
        self.moving = False
        self.request_inflight = False
        self.pending_goal_id = 0
        self.transport_error = ""

    def fresh(self, timestamp):
        return self.monotonic() - timestamp <= self.config["state_stale_seconds"]

    def base_reason(self):
        if not self.live:
            return "只读模式：需明确启动 --live-controls 才能提供操作"
        if self.replay:
            return "检测到 /clock 或 rosbag2_player，控制已锁存禁用；结束回放后重启网关"
        if not self.connected or not self.fresh(self.connection_at):
            return "SDK 未连接或连接状态过期"
        if self.behavior.get("control_adapter") != "guarded_console_v1":
            return "未连接受保护的控制状态机"
        return ""

    def arm_reason(self):
        if self.base_reason():
            return self.base_reason()
        if self.transport_error:
            return self.transport_error
        if not self.fresh(self.robot_at) or not self.fresh(self.behavior_at):
            return "机器人或状态机反馈过期"
        if self.behavior.get("fault_latched") is not False:
            return "机器人故障锁存或故障状态未知"
        if self.behavior.get("control_adapter") != "guarded_console_v1":
            return "未连接受保护的控制状态机"
        if self.behavior.get("replay_latched") is True:
            return "SDK 控制桥检测到回放，控制已锁存禁用"
        motion = self.robot.get("motion_status")
        if type(motion) is not int or motion not in range(1, 11):
            return "机器人姿态未知"
        return ""

    @property
    def armed(self):
        return not self.arm_reason() and self.monotonic() < self.lease_until

    def arm(self):
        reason = self.arm_reason()
        if reason:
            return reason
        self.lease_until = self.monotonic() + self.config["lease_seconds"]
        return ""

    def renew(self):
        if not self.armed:
            return "操作租约已失效，请重新解锁"
        return self.arm()

    def disarm(self):
        self.lease_until = -math.inf

    def action_reason(self, action):
        # Stop must remain requestable even if posture/fault feedback is stale.
        if action in ("soft_estop", "halt"):
            return self.base_reason()
        if self.arm_reason():
            return self.arm_reason()
        if not self.armed:
            return "本次操作未解锁或租约过期"
        if action != "release_control" and self.busy:
            return "转换进行中：请等待机器人反馈，或停止/急停取消后续步骤"
        reasons = self.behavior.get("available_actions", {})
        return reasons.get(action, "状态机未提供此动作的许可") if isinstance(reasons, dict) else "动作许可未知"

    @property
    def busy(self):
        goal_id = self.behavior.get("goal_id", 0)
        return (self.request_inflight
                or not isinstance(goal_id, int) or goal_id < self.pending_goal_id
                or self.behavior.get("goal_status") in ("QUEUED", "TRANSITIONING"))

    def motion_reason(self):
        if not self.armed:
            return "操作未解锁"
        if self.busy:
            return "姿态或模式转换中"
        if (self.robot.get("control_source") != 2
                or self.robot.get("motion_status") != 5
                or self.robot.get("sport_mode") != 1
                or self.robot.get("software_emergency_status") != 1
                or self.robot.get("hardware_emergency_status") != 1
                or self.behavior.get("sdk_has_control") is not True
                or self.behavior.get("ready_for_navigation") is not True):
            return "请完成控制权、站稳、通用模式和低速确认"
        return ""

    def velocity(self, command, wall_time):
        if self.motion_reason():
            return None
        if not isinstance(command, dict) or command.get("session") != self.session:
            return None
        seq, stamp = command.get("seq"), command.get("stamp")
        if type(seq) is not int or seq <= self.last_seq:
            return None
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or not -0.1 <= wall_time - stamp <= self.config["command_timeout_seconds"]:
            return None
        values = [command.get(k) for k in ("x", "y", "yaw")]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            return None
        limits = [self.config[k] for k in ("max_forward", "max_lateral", "max_yaw")]
        if any(abs(v) > limit for v, limit in zip(values, limits)):
            return None
        self.last_seq = seq
        self.last_velocity_at = self.monotonic()
        self.moving = any(v != 0 for v in values)
        return values

    def should_stop(self):
        return self.moving and (bool(self.motion_reason()) or self.monotonic() - self.last_velocity_at > self.config["command_timeout_seconds"])

    def stopped(self):
        self.moving = False

    def status(self):
        return {"session": self.session, "service_prefix": f"/d1max/console/s_{self.session}",
                "mode": "replay" if self.replay else "live" if self.live else "read_only",
                "armed": self.armed, "can_arm": not bool(self.arm_reason()), "reason": self.arm_reason(),
                "busy": self.busy, "motion_reason": self.motion_reason(),
                "safety_available": not bool(self.base_reason()),
                "wall_time": time.time()}
