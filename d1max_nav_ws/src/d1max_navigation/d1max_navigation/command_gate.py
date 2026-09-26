"""Fail-closed SI velocity boundary; this module never connects to the robot SDK.

Simulation and hardware are deliberately different admission paths.  Hardware
requires an explicitly enabled, explicitly armed session AND independent fresh
localization/sensor/SDK evidence.  Existing monitor-only SDK adapters therefore
cannot accidentally become motion adapters by receiving a Nav2 Twist.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import time
import uuid

HARD_PLANAR_SPEED_LIMIT = 1.5  # m/s; a user safety boundary, not a tuning parameter.
INITIAL_COMMAND_TIMEOUT = 0.6  # Bounded wait after live arm; never an indefinite zero heartbeat.


def limit_velocity(velocity, forward, lateral, yaw, planar):
    """Component limits AND a Euclidean planar limit; never a diagonal loophole."""
    x, y, z = (max(-limit, min(limit, value))
               for value, limit in zip(velocity, (forward, lateral, yaw)))
    length = math.hypot(x, y)
    scale = min(1., min(planar, HARD_PLANAR_SPEED_LIMIT) / max(length, 1e-12))
    return x * scale, y * scale, z


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def monitor_endpoint(status, now_wall, max_age=1.0):
    """Discover only the monitor's session-scoped endpoint, never arbitrary ROS URLs."""
    if not isinstance(status, dict) or status.get('motion_control_enabled') is not True:
        return None
    session = status.get('session')
    stamp = status.get('wall_time')
    if (not isinstance(session, str) or not re.fullmatch('[a-f0-9]{16}', session)
            or not finite(stamp) or not finite(now_wall) or not 0 <= now_wall-stamp <= max_age
            or status.get('mode') != 'monitor'):
        return None
    prefix = '/d1max/monitor/s_' + session
    if status.get('service_prefix') != prefix:
        return None
    return session, prefix


def sdk_command_envelope(*, sdk_session, generation, sequence, navigation_session,
                         map_version_id, velocity, wall, command_source):
    """SI wire contract consumed by optional navigation_motion_core in the same SDK process."""
    if (not re.fullmatch('[a-f0-9]{16}', sdk_session) or type(generation) is not int or generation < 1
            or type(sequence) is not int or sequence < 1 or not navigation_session or not map_version_id
            or not all(finite(v) for v in velocity) or len(velocity) != 3 or not finite(wall)
            or math.hypot(velocity[0], velocity[1]) > HARD_PLANAR_SPEED_LIMIT
            or not re.fullmatch('[a-f0-9]{32}', command_source)):
        raise ValueError('invalid guarded SDK command context')
    return {'sdk_session': sdk_session, 'arm_generation': generation, 'seq': sequence,
            'navigation_session': navigation_session, 'map_version_id': map_version_id,
            'command_source': command_source, 'stamp': wall, 'x': velocity[0], 'y': velocity[1], 'yaw': velocity[2]}


def arm_generation_revoked(observed, observed_armed, expected, ack_age):
    """A queued pre-arm status gets only a bounded generation catch-up window."""
    return (type(observed) is not int or observed > expected
        or (observed == expected and observed_armed is not True)
        or not finite(ack_age) or ack_age < 0
        or (observed < expected and ack_age > .6))


@dataclass(frozen=True)
class GateConfig:
    mode: str = 'live'
    motion_enabled: bool = False
    expected_version_id: str = ''
    expected_session_id: str = ''
    navigation_session_id: str = ''
    require_execution_permit: bool = False
    execution_permit_timeout: float = 0.35
    map_frame: str = 'd1max_loc_map'
    odom_frame: str = 'd1max_loc_odom'
    base_frame: str = 'd1max_loc_base_link'
    max_forward: float = 0.30
    max_lateral: float = 0.0
    max_yaw: float = 0.50
    max_planar_speed: float = HARD_PLANAR_SPEED_LIMIT
    command_timeout: float = 0.25
    odom_timeout: float = 0.40
    scan_timeout: float = 0.60
    localization_timeout: float = 1.5
    sdk_timeout: float = 2.0

    def __post_init__(self):
        if self.mode not in ('live', 'simulation'):
            raise ValueError('mode must be live or simulation')
        if type(self.motion_enabled) is not bool:
            raise ValueError('motion_enabled must be boolean')
        if type(self.require_execution_permit) is not bool:
            raise ValueError('require_execution_permit must be boolean')
        for name, ceiling in [('max_forward', 1.), ('max_lateral', .5), ('max_yaw', 1.),
                              ('max_planar_speed', HARD_PLANAR_SPEED_LIMIT),
                              ('command_timeout', .25), ('odom_timeout', .5),
                              ('scan_timeout', 1.), ('localization_timeout', 1.5),
                              ('sdk_timeout', 2.), ('execution_permit_timeout', .35)]:
            value = getattr(self, name)
            if not finite(value) or value < 0 or value > ceiling or (name != 'max_lateral' and value == 0):
                raise ValueError(f'unsafe {name}')
        for name in ('map_frame', 'odom_frame', 'base_frame'):
            if not isinstance(getattr(self, name), str) or not getattr(self, name):
                raise ValueError(f'{name} required')


@dataclass(frozen=True)
class GateOutput:
    velocity: tuple[float, float, float]
    allowed: bool
    reason: str


class CommandGate:
    """Pure gate core. All freshness timers use local monotonic receipt time."""

    def __init__(self, config: GateConfig):
        self.config = config
        self.armed = config.mode == 'simulation'
        self.samples = {}
        self.command = None
        self.command_at = None
        self.armed_at = None
        self.execution_permit_sequence = 0
        self.execution_permit_generation = None

    def observe(self, kind, payload, received_at):
        if kind not in ('odom', 'scan', 'localization', 'robot', 'behavior', 'mc', 'execution_permit'):
            raise ValueError('unknown evidence type')
        if kind == 'execution_permit':
            return self.observe_execution_permit(payload, received_at)
        self.samples[kind] = (payload, received_at)

    def observe_execution_permit(self, payload, received_at):
        """Renew execution authority, never freshness for duplicate/replayed permits."""
        valid = (isinstance(payload, dict) and type(payload.get('schema')) is int
                 and payload['schema'] == 1 and type(payload.get('allow')) is bool
                 and type(payload.get('seq')) is int and payload['seq'] > 0
                 and type(payload.get('generation')) is int and payload['generation'] > 0
                 and bool(self.config.navigation_session_id)
                 and payload.get('session_id') == self.config.navigation_session_id
                 and bool(self.config.expected_version_id)
                 and payload.get('map_version_id') == self.config.expected_version_id
                 and finite(payload.get('received_at_unix')) and finite(received_at))
        required = self.config.mode == 'live' and self.config.require_execution_permit
        if not valid:
            self.samples['execution_permit'] = (None, received_at)
            if required:
                self.arm(False, received_at, 0.)
            return False
        if payload['seq'] <= self.execution_permit_sequence:
            return False
        changed = (self.execution_permit_generation is not None
                   and payload['generation'] != self.execution_permit_generation)
        expired = (self.armed and self._sample('execution_permit', received_at,
                   self.config.execution_permit_timeout) is None)
        self.execution_permit_sequence = payload['seq']
        self.execution_permit_generation = payload['generation']
        self.samples['execution_permit'] = (dict(payload), received_at)
        if required and (changed or expired or payload['allow'] is not True):
            self.arm(False, received_at, 0.)
        return True

    def receive_command(self, velocity, now):
        # Check the previous receipt before replacing it. A delayed timer must
        # not let a newly arrived command hide a period of lost input.
        expired = self._live_command_expiry(now)
        if expired:
            self.arm(False, now, 0.)
            return False
        # Always invalidate the previous command first: malformed data is never
        # treated as permission to continue the last nonzero command.
        self.command = None
        self.command_at = None
        if (len(velocity) != 3 or not all(finite(x) for x in velocity) or not finite(now)):
            if self.config.mode == 'live':
                self.arm(False, now, 0.)
            return False
        self.command = tuple(velocity)
        self.command_at = now
        return True

    def _live_command_expiry(self, now):
        if self.config.mode != 'live' or not self.armed:
            return ''
        if self.command_at is None:
            if (not finite(now) or not finite(self.armed_at)
                    or not 0 <= now - self.armed_at <= INITIAL_COMMAND_TIMEOUT):
                return 'initial_command_timeout_rearm_required'
        elif (not finite(now) or not finite(self.command_at)
              or not 0 <= now - self.command_at <= self.config.command_timeout):
            return 'command_timeout_rearm_required'
        return ''

    def _sample(self, kind, now, timeout):
        payload, received = self.samples.get(kind, (None, None))
        if (not finite(now) or not finite(received) or not 0 <= now - received <= timeout
                or not isinstance(payload, dict)):
            return None
        return payload

    def health_reason(self, now, wall, *, check_execution_permit=True):
        """Read health; skipping the permit is reserved for readiness display.

        arm(), output() and every SDK mutation retain the default full check.
        """
        c = self.config
        if c.mode == 'live' and not c.motion_enabled:
            return 'live_motion_disabled'
        odom = self._sample('odom', now, c.odom_timeout)
        if not odom or odom.get('valid') is not True:
            return 'odometry_missing_or_stale'
        if odom.get('frame') != c.odom_frame or odom.get('child_frame') != c.base_frame:
            return 'odometry_frame_mismatch'
        if finite(odom.get('planar_speed')) and odom['planar_speed'] > HARD_PLANAR_SPEED_LIMIT:
            return 'measured_speed_exceeds_hard_limit'
        scan = self._sample('scan', now, c.scan_timeout)
        if not scan or scan.get('valid') is not True:
            return 'scan_missing_or_stale'
        if scan.get('frame') != c.base_frame:
            return 'scan_frame_mismatch'
        if c.mode == 'simulation':
            return ''
        if not c.expected_version_id or not c.expected_session_id:
            return 'localization_context_not_pinned'
        state = self._sample('localization', now, c.localization_timeout)
        if not state:
            return 'localization_missing_or_stale'
        stamp = state.get('wall_time')
        if not finite(wall) or not finite(stamp) or not 0 <= wall - stamp <= c.localization_timeout:
            return 'localization_timestamp_invalid'
        if (state.get('session_id') != c.expected_session_id
                or state.get('map_version_id') != c.expected_version_id):
            return 'localization_context_mismatch'
        if not isinstance(state.get('frames'), dict) or state['frames'].get('map') != c.map_frame:
            return 'localization_frame_mismatch'
        if (state.get('state') != 'tracking' or state.get('localized') is not True
                or state.get('navigation_ready') is not True or state.get('local_fault')):
            return 'localization_not_navigation_ready'
        nav = state.get('navigation')
        calibration = state.get('calibration')
        if (not isinstance(nav, dict) or nav.get('valid') is not True or nav.get('fault')
                or not isinstance(calibration, dict)
                or calibration.get('extrinsics_verified') is not True
                or calibration.get('time_alignment_verified') is not True):
            return 'localization_not_verified'
        if c.require_execution_permit and check_execution_permit:
            permit = self._sample('execution_permit', now, c.execution_permit_timeout)
            if not permit:
                return 'execution_permit_missing_or_stale'
            stamp = permit.get('received_at_unix')
            if not finite(stamp) or not 0 <= wall - stamp <= c.execution_permit_timeout:
                return 'execution_permit_timestamp_invalid'
            if permit.get('allow') is not True:
                return 'execution_not_permitted'
        robot = self._sample('robot', now, c.sdk_timeout)
        if not robot:
            return 'sdk_robot_missing_or_stale'
        stamp = robot.get('received_at_unix')
        if not finite(stamp) or not 0 <= wall - stamp <= c.sdk_timeout:
            return 'sdk_robot_timestamp_invalid'
        # Values are from the actual SDK EmergencyStatus/CtrlSource/SportMode
        # enums, not booleans. UNKNOWN (0) is not equivalent to released (1).
        if (robot.get('software_emergency_status') != 1
                or robot.get('hardware_emergency_status') != 1):
            return 'sdk_estop_active_or_unknown'
        if robot.get('control_source') != 2:
            return 'sdk_control_not_owned'
        if (robot.get('sport_mode') != 1 or robot.get('motion_status') != 5
                or robot.get('speed_level') != 1 or robot.get('head_direction') != 1):
            return 'sdk_general_low_speed_head_forward_required'
        behavior = self._sample('behavior', now, c.sdk_timeout)
        if not behavior:
            return 'sdk_behavior_missing_or_stale'
        if (behavior.get('telemetry_only') is not False
                or behavior.get('ready_for_navigation') is not True
                or behavior.get('sdk_has_control') is not True
                or behavior.get('replay_latched') is not False
                or behavior.get('fault_latched') is not False
                or behavior.get('requires_review') is not False):
            return 'sdk_motion_adapter_not_ready'
        mc = self._sample('mc', now, c.sdk_timeout)
        if (not mc or mc.get('source') != 'sdk_mc' or mc.get('callback') != 'OnMcData'
                or mc.get('stream_fresh') is not True or mc.get('rate_ok') is not True):
            return 'sdk_mc_missing_or_stale'
        stamp = mc.get('received_at_unix')
        if not finite(stamp) or not 0 <= wall - stamp <= c.sdk_timeout:
            return 'sdk_mc_timestamp_invalid'
        return ''

    def arm(self, enabled, now, wall):
        self.armed = False
        self.command = None
        self.command_at = None
        self.armed_at = None
        if enabled is not True:
            return True, 'disarmed'
        reason = self.health_reason(now, wall)
        if reason:
            return False, reason
        self.armed = True
        self.armed_at = now
        return True, 'armed; waiting for a new velocity command'

    def output(self, now, wall):
        reason = self.health_reason(now, wall)
        if reason:
            # Hardware loss of trust requires a new explicit arm after repair.
            # It cannot silently resume an interrupted goal when data returns.
            if self.config.mode == 'live':
                self.arm(False, now, wall)
            return GateOutput((0., 0., 0.), False, reason)
        if not self.armed:
            return GateOutput((0., 0., 0.), False, 'not_armed')
        expired = self._live_command_expiry(now)
        if expired:
            self.arm(False, now, wall)
            return GateOutput((0., 0., 0.), False, expired)
        if (self.command is None or not finite(self.command_at)
                or not 0 <= now - self.command_at <= self.config.command_timeout):
            return GateOutput((0., 0., 0.), False, 'command_missing_or_stale')
        bounded = limit_velocity(self.command, self.config.max_forward,
                                 self.config.max_lateral, self.config.max_yaw,
                                 self.config.max_planar_speed)
        return GateOutput(bounded, True, 'simulation_only' if self.config.mode == 'simulation' else 'live_guard_passed')


def preflight_status(gate, now, wall, *, monitor=None, monitor_received=None,
                     sdk_session='', sdk_service_ready=False, sdk_arm_pending=False):
    """Read-only preparation status, not authority to arm or emit velocity.

    An idle coordinator intentionally withholds its execution permit. Show the
    underlying blockers anyway so the operator can repair them before execute.
    """
    reason = gate.health_reason(now, wall, check_execution_permit=False)
    if not reason and gate.config.mode == 'live':
        endpoint = monitor_endpoint(monitor, wall)
        if (endpoint is None or not finite(monitor_received)
                or not 0 <= now-monitor_received <= 1.):
            reason = 'navigation_enabled_sdk_monitor_required'
        elif sdk_session != endpoint[0] or sdk_service_ready is not True:
            reason = 'sdk_arm_service_unavailable'
        elif not isinstance(monitor.get('navigation_armed'), bool):
            reason = 'sdk_arm_state_unknown'
        elif monitor['navigation_armed']:
            reason = 'sdk_already_armed'
    if not reason:
        if gate.armed:
            reason = 'gate_already_armed'
        elif sdk_arm_pending:
            reason = 'sdk_arm_pending'
    return {'preflight_ready': not reason, 'preflight_block_reason': reason}


def main(args=None):
    import signal
    import threading
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    class GateNode(Node):
        def __init__(self):
            super().__init__('navigation_command_gate')
            defaults = GateConfig().__dict__
            params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            self.gate = CommandGate(GateConfig(**params))
            topics = {
                'input_topic': '/d1max/navigation/cmd_vel_collision_checked',
                'output_topic': '/d1max/navigation/cmd_vel_safe',
                'odom_topic': '/d1max/localization/odometry/local',
                'scan_topic': '/d1max/navigation/scan',
                'localization_topic': '/d1max/localization/status',
                'robot_topic': '/d1max_sdk_bridge/robot_state',
                'behavior_topic': '/d1max_sdk_bridge/behavior_state',
                'mc_topic': '/d1max_sdk_bridge/speed_report_status',
                'status_topic': '/d1max/navigation/command_gate/status',
                'monitor_topic': '/d1max/monitor/status',
                'execution_permit_topic': '/d1max/live_planning/execution_permit',
            }
            self.topics = {k: self.declare_parameter(k, v).value for k, v in topics.items()}
            self.publisher = self.create_publisher(Twist, self.topics['output_topic'], 1)
            self.status = self.create_publisher(String, self.topics['status_topic'], 1)
            self.subscriptions_owned = [
                self.create_subscription(Twist, self.topics['input_topic'], self.command, 1),
                self.create_subscription(Odometry, self.topics['odom_topic'], self.odom, qos_profile_sensor_data),
                self.create_subscription(LaserScan, self.topics['scan_topic'], self.scan, qos_profile_sensor_data),
            ]
            self.monitor_status = None
            self.monitor_received = None
            self.sdk_session = ''
            self.sdk_generation = 0
            self.sdk_sequence = 0
            self.sdk_publisher = None
            self.sdk_client = None
            self.sdk_arm_future = None
            self.sdk_arm_requested_at = None
            self.sdk_arm_confirmed_at = None
            self.sdk_request_serial = 0
            self.command_source = uuid.uuid4().hex
            if self.gate.config.mode == 'live':
                for key in ('localization', 'robot', 'behavior', 'mc'):
                    self.subscriptions_owned.append(self.create_subscription(
                        String, self.topics[key + '_topic'], lambda m, k=key: self.json_status(k, m), 1))
                self.subscriptions_owned.append(self.create_subscription(String,
                    self.topics['monitor_topic'], self.monitor, 1))
                if self.gate.config.require_execution_permit:
                    self.subscriptions_owned.append(self.create_subscription(String,
                        self.topics['execution_permit_topic'],
                        lambda m: self.json_status('execution_permit', m), 1))
            self.arm_service = self.create_service(SetBool, '~/arm', self.arm_request)
            self.timer = self.create_timer(.05, self.tick)
            self.last_status_reason = None
            self.ticks = 0
            self.get_logger().info(f'Command gate: {params["mode"]}; NO SDK connection; live motion enabled={params["motion_enabled"]}')

        def stamp_fresh(self, stamp, timeout):
            value = stamp.sec + stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - value
            return finite(age) and 0 <= age <= timeout

        def command(self, msg):
            if (not all(finite(v) for v in (msg.linear.z, msg.angular.x, msg.angular.y))
                    or any(abs(v) > 1e-6 for v in (msg.linear.z, msg.angular.x, msg.angular.y))):
                self.gate.receive_command((float('nan'), 0., 0.), time.monotonic())
                return
            self.gate.receive_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

        def odom(self, msg):
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            v = msg.twist.twist
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                      v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
            valid = (all(finite(v) for v in values) and .9 < sum(v*v for v in values[3:7]) < 1.1
                     and self.stamp_fresh(msg.header.stamp, self.gate.config.odom_timeout))
            self.gate.observe('odom', {'valid': valid, 'frame': msg.header.frame_id,
                                      'child_frame': msg.child_frame_id,
                                      'planar_speed': math.hypot(v.linear.x, v.linear.y)}, time.monotonic())

        def scan(self, msg):
            valid = (self.stamp_fresh(msg.header.stamp, self.gate.config.scan_timeout)
                     and len(msg.ranges) > 0 and finite(msg.range_min) and finite(msg.range_max)
                     and 0 <= msg.range_min < msg.range_max
                     and finite(msg.angle_increment) and msg.angle_increment > 0
                     and any(finite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges))
            self.gate.observe('scan', {'valid': valid, 'frame': msg.header.frame_id}, time.monotonic())

        def json_status(self, kind, msg):
            try:
                payload = json.loads(msg.data) if len(msg.data) <= 262144 else None
            except (ValueError, TypeError):
                payload = None
            previous_generation = self.gate.execution_permit_generation
            self.gate.observe(kind, payload, time.monotonic())
            if (kind == 'execution_permit' and self.gate.config.require_execution_permit
                    and (self.sdk_arm_future is not None or self.sdk_generation)
                    and (previous_generation != self.gate.execution_permit_generation
                         or self.gate.health_reason(time.monotonic(), time.time()))):
                self.gate.arm(False, time.monotonic(), time.time())
                self.sdk_disarm()

        def monitor(self, msg):
            self.monitor_received = time.monotonic()
            try:
                self.monitor_status = json.loads(msg.data) if len(msg.data) <= 16384 else None
            except (ValueError, TypeError):
                self.monitor_status = None
            endpoint = self.sdk_context()
            if (self.gate.config.motion_enabled and endpoint and not self.gate.armed
                    and self.sdk_arm_future is None and endpoint[0] != self.sdk_session):
                self.sdk_endpoints(*endpoint)

        def sdk_endpoints(self, session, prefix):
            if self.sdk_client:
                self.destroy_client(self.sdk_client)
            if self.sdk_publisher:
                self.destroy_publisher(self.sdk_publisher)
            self.sdk_session = session
            self.sdk_client = self.create_client(SetBool, prefix + '/navigation_arm')
            self.sdk_publisher = self.create_publisher(String, prefix + '/navigation_velocity', 1)

        def sdk_context(self):
            if (self.monitor_received is None or not 0 <= time.monotonic()-self.monitor_received <= 1.0):
                return None
            return monitor_endpoint(self.monitor_status, time.time())

        def sdk_disarm(self):
            self.sdk_request_serial += 1
            self.sdk_generation = 0
            self.sdk_arm_future = None
            self.sdk_arm_requested_at = None
            self.sdk_arm_confirmed_at = None
            if self.sdk_client and self.sdk_client.service_is_ready():
                request = SetBool.Request()
                request.data = False
                self.sdk_client.call_async(request)

        def sdk_arm_done(self, future, serial, session):
            if serial != self.sdk_request_serial:
                return  # Late arm replies cannot re-enable a disarmed gate.
            self.sdk_arm_future = None
            self.sdk_arm_requested_at = None
            try:
                response = future.result()
                data = json.loads(response.message)
                generation = data.get('arm_generation')
                if (not response.success or data.get('armed') is not True
                        or data.get('sdk_session') != session or type(generation) is not int or generation < 1
                        or self.sdk_context() != (session, '/d1max/monitor/s_' + session)):
                    raise ValueError('SDK arm rejected or context changed')
                ok, reason = self.gate.arm(True, time.monotonic(), time.time())
                if not ok:
                    raise ValueError(reason)
                self.sdk_generation = generation
                self.sdk_arm_confirmed_at = time.monotonic()
                self.get_logger().info('Explicit SDK navigation arm confirmed; fresh Nav2 command required')
            except (ValueError, TypeError, AttributeError, RuntimeError) as error:
                self.gate.arm(False, time.monotonic(), time.time())
                self.sdk_disarm()
                self.get_logger().error(f'SDK navigation remains locked: {error}')

        def arm_request(self, request, response):
            if self.gate.config.mode == 'simulation':
                response.success, response.message = self.gate.arm(request.data, time.monotonic(), time.time())
            elif not request.data:
                self.gate.arm(False, time.monotonic(), time.time())
                self.sdk_disarm()
                response.success, response.message = True, 'disarmed'
            else:
                reason = self.gate.health_reason(time.monotonic(), time.time())
                endpoint = self.sdk_context()
                if reason or endpoint is None:
                    response.success, response.message = False, reason or 'fresh navigation-enabled SDK monitor required'
                    return response
                if self.sdk_arm_future is not None or self.gate.armed:
                    response.success, response.message = False, 'arm already pending or active; disarm before rearming'
                    return response
                session, prefix = endpoint
                if session != self.sdk_session:
                    self.sdk_endpoints(session, prefix)
                if not self.sdk_client.service_is_ready():
                    response.success, response.message = False, 'SDK arm service not discovered yet; remains locked'
                    return response
                self.sdk_request_serial += 1
                serial = self.sdk_request_serial
                sdk_request = SetBool.Request()
                sdk_request.data = True
                self.sdk_arm_requested_at = time.monotonic()
                self.sdk_arm_future = self.sdk_client.call_async(sdk_request)
                self.sdk_arm_future.add_done_callback(lambda f: self.sdk_arm_done(f, serial, session))
                response.success = True
                response.message = 'arm request queued; gate remains locked until SDK confirmation'
            self.tick()
            return response

        def tick(self):
            if self.gate.config.mode == 'live':
                if self.sdk_arm_future is not None and time.monotonic()-self.sdk_arm_requested_at > 1.0:
                    self.sdk_disarm()
                    self.get_logger().error('SDK arm timed out; no automatic retry')
                if self.gate.armed:
                    endpoint = self.sdk_context()
                    changed = endpoint != (self.sdk_session, '/d1max/monitor/s_' + self.sdk_session)
                    if not changed:
                        observed = self.monitor_status.get('navigation_arm_generation')
                        # An older 5 Hz status may arrive after the service ACK.
                        # Allow only a bounded 0.6 s status-generation catch-up;
                        # newer generations or disarm in OUR generation revoke
                        # permission immediately.
                        changed = arm_generation_revoked(observed, self.monitor_status.get('navigation_armed'),
                            self.sdk_generation, time.monotonic()-self.sdk_arm_confirmed_at)
                    if changed:
                        self.gate.arm(False, time.monotonic(), time.time())
                        self.sdk_disarm()
            result = self.gate.output(time.monotonic(), time.time())
            if self.gate.config.mode == 'live' and self.sdk_generation:
                if not self.gate.armed:
                    self.sdk_disarm()
                else:
                    # Idle Nav2 is a zero heartbeat, not the previous velocity.
                    self.sdk_sequence += 1
                    packet = sdk_command_envelope(sdk_session=self.sdk_session, generation=self.sdk_generation,
                        sequence=self.sdk_sequence, navigation_session=self.gate.config.expected_session_id,
                        map_version_id=self.gate.config.expected_version_id, velocity=result.velocity, wall=time.time(),
                        command_source=self.command_source)
                    payload = String()
                    payload.data = json.dumps(packet, allow_nan=False)
                    self.sdk_publisher.publish(payload)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = result.velocity
            self.publisher.publish(msg)
            self.ticks += 1
            if self.ticks % 4 == 0 or self.last_status_reason != result.reason:
                arm_reason = self.gate.health_reason(time.monotonic(), time.time())
                if self.gate.config.mode == 'live' and not arm_reason:
                    if self.sdk_context() is None:
                        arm_reason = 'navigation_enabled_sdk_monitor_required'
                    elif not self.sdk_client or not self.sdk_client.service_is_ready():
                        arm_reason = 'sdk_arm_service_unavailable'
                preflight = preflight_status(self.gate, time.monotonic(), time.time(),
                    monitor=self.monitor_status, monitor_received=self.monitor_received,
                    sdk_session=self.sdk_session,
                    sdk_service_ready=bool(self.sdk_client and self.sdk_client.service_is_ready()),
                    sdk_arm_pending=self.sdk_arm_future is not None)
                status = String()
                status.data = json.dumps({'mode': self.gate.config.mode, 'armed': self.gate.armed,
                    'motion_enabled': self.gate.config.motion_enabled, 'allowed': result.allowed,
                    'reason': result.reason, 'velocity_si': list(result.velocity),
                    'navigation_session_id': self.gate.config.navigation_session_id,
                    'map_version_id': self.gate.config.expected_version_id,
                    'limits': {'max_forward': self.gate.config.max_forward,
                               'max_lateral': self.gate.config.max_lateral,
                               'max_yaw': self.gate.config.max_yaw,
                               'max_planar_speed': self.gate.config.max_planar_speed,
                               'hard_planar_speed': HARD_PLANAR_SPEED_LIMIT},
                    'sdk_connected_by_gate': False, 'sdk_arm_pending': self.sdk_arm_future is not None,
                    'arm_ready': not arm_reason and not self.gate.armed and self.sdk_arm_future is None,
                    'arm_block_reason': arm_reason,
                    **preflight,
                    'sdk_session': self.sdk_session, 'sdk_arm_generation': self.sdk_generation,
                    'execution_permit_required': self.gate.config.require_execution_permit,
                    'execution_generation': self.gate.execution_permit_generation,
                    'execution_permit_seq': self.gate.execution_permit_sequence,
                    'wall_time': time.time()}, allow_nan=False)
                self.status.publish(status)
                self.last_status_reason = result.reason

        def stop(self):
            self.gate.arm(False, time.monotonic(), time.time())
            if self.gate.config.mode == 'live':
                self.sdk_disarm()
            self.publisher.publish(Twist())

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = GateNode()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
