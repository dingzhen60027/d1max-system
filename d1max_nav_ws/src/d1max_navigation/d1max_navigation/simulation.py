"""Explicit SOFTWARE SIMULATION for offline Nav2 wiring acceptance.

This is a bounded planar kinematic test plant, NOT a localization algorithm or
robot driver. It can publish synthetic TF only on the isolated offline Zenoh
router. No SDK, Web, control ownership or live sensor interfaces are imported.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_isolation(environment, config):
    """Fail closed before ROS initialization; no discovery or live endpoint."""
    if environment.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise ValueError('SIMULATION requires rmw_zenoh_cpp')
    if environment.get('ZENOH_CONFIG_OVERRIDE') or environment.get('ZENOH_SESSION_CONFIG'):
        raise ValueError('SIMULATION refuses additional Zenoh configuration overrides')
    if not isinstance(config, dict) or config.get('mode') != 'client':
        raise ValueError('SIMULATION requires a client-only Zenoh session')
    if config.get('connect', {}).get('endpoints') != ['tcp/127.0.0.1:7460']:
        raise ValueError('SIMULATION may connect only to isolated loopback port 7460')
    scouting = config.get('scouting', {})
    if (scouting.get('multicast', {}).get('enabled') is not False
            or scouting.get('gossip', {}).get('enabled') is not False):
        raise ValueError('SIMULATION requires multicast and gossip discovery disabled')
    if config.get('listen', {}).get('endpoints'):
        raise ValueError('SIMULATION must not listen on additional endpoints')


@dataclass
class PlanarPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = math.pi / 2


class OccupancyWorld:
    """ROS OccupancyGrid geometry, including rotated map origins and unknowns."""
    def __init__(self, data, width, height, resolution, origin=(0., 0., 0.)):
        if not isinstance(width, int) or not isinstance(height, int) or width < 1 or height < 1:
            raise ValueError('Invalid occupancy dimensions')
        if not _finite(resolution) or not 0.005 <= resolution <= 1.:
            raise ValueError('Invalid occupancy resolution')
        if len(origin) != 3 or not all(_finite(v) for v in origin):
            raise ValueError('Invalid occupancy origin')
        grid = np.asarray(data, dtype=np.int16)
        if grid.size != width * height or np.any((grid < -1) | (grid > 100)):
            raise ValueError('Invalid occupancy contents')
        self.width, self.height, self.resolution = width, height, float(resolution)
        self.origin = tuple(float(v) for v in origin)
        self.blocked = (grid.reshape(height, width) < 0) | (grid.reshape(height, width) >= 50)
        self.cos, self.sin = math.cos(origin[2]), math.sin(origin[2])
        self.dynamic_obstacle = None

    def set_dynamic_obstacle(self, obstacle):
        """One bounded test cylinder; never changes the pinned map on disk."""
        if obstacle is None:
            self.dynamic_obstacle = None
            return
        if (len(obstacle) != 3 or not all(_finite(v) for v in obstacle)
                or not .05 <= obstacle[2] <= .5):
            raise ValueError('Invalid simulated obstacle')
        x, y, radius = (float(v) for v in obstacle)
        gx, gy = self.grid_coordinates(x, y)
        if not 0 <= gx < self.width or not 0 <= gy < self.height:
            raise ValueError('Simulated obstacle must lie in the pinned map')
        self.dynamic_obstacle = (x, y, radius)

    def grid_coordinates(self, x, y):
        dx, dy = np.asarray(x) - self.origin[0], np.asarray(y) - self.origin[1]
        return ((self.cos * dx + self.sin * dy) / self.resolution,
                (-self.sin * dx + self.cos * dy) / self.resolution)

    def clear(self, x, y, radius=.4):
        if not all(_finite(v) for v in (x, y, radius)) or radius < 0:
            return False
        if self.dynamic_obstacle is not None:
            ox, oy, obstacle_radius = self.dynamic_obstacle
            if math.hypot(x-ox, y-oy) <= radius+obstacle_radius:
                return False
        gx, gy = self.grid_coordinates(x, y)
        # Include cell diagonal so footprint cannot cut occupied-cell corners.
        r = radius / self.resolution + math.sqrt(.5)
        x0, x1 = math.floor(gx-r), math.ceil(gx+r)
        y0, y1 = math.floor(gy-r), math.ceil(gy+r)
        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            return False
        yy, xx = np.ogrid[y0:y1+1, x0:x1+1]
        footprint = (xx+.5-gx)**2 + (yy+.5-gy)**2 <= r*r
        return not bool(np.any(self.blocked[y0:y1+1, x0:x1+1] & footprint))

    def segment_clear(self, a, b, radius=.4):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        steps = max(1, math.ceil(length / (self.resolution * .5)))
        return all(self.clear(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, radius)
                   for t in np.linspace(0., 1., steps+1))

    def suggest_goal(self, pose, radius=.4, distance=5.):
        """Suggest an unobstructed test goal; never sends it to Nav2 itself."""
        angles = [pose.yaw] + [pose.yaw+s*i*math.pi/12 for i in range(1, 13) for s in (1, -1)]
        for length in (distance, 4., 3.):
            for yaw in angles:
                goal = (pose.x+length*math.cos(yaw), pose.y+length*math.sin(yaw))
                if self.segment_clear((pose.x, pose.y), goal, radius+.1):
                    return {'x': goal[0], 'y': goal[1], 'yaw': math.atan2(math.sin(yaw), math.cos(yaw))}
        return None

    def raycast(self, pose, count=360, max_range=18., min_range=.05):
        if count < 4 or count > 1440 or not .1 < max_range <= 40:
            raise ValueError('Invalid synthetic scan configuration')
        angles = pose.yaw + np.arange(count) * (2*math.pi/count) - math.pi
        # At half-cell increments a thin one-cell wall cannot be stepped over.
        ranges = np.arange(min_range, max_range+self.resolution*.25, self.resolution*.5)
        xs = pose.x + np.cos(angles)[:, None]*ranges
        ys = pose.y + np.sin(angles)[:, None]*ranges
        gx, gy = self.grid_coordinates(xs, ys)
        ix, iy = np.floor(gx).astype(np.int32), np.floor(gy).astype(np.int32)
        outside = (ix < 0) | (iy < 0) | (ix >= self.width) | (iy >= self.height)
        hit = outside | self.blocked[np.clip(iy, 0, self.height-1), np.clip(ix, 0, self.width-1)]
        if self.dynamic_obstacle is not None:
            ox, oy, obstacle_radius = self.dynamic_obstacle
            hit |= (xs-ox)**2+(ys-oy)**2 <= obstacle_radius**2
        first = np.argmax(hit, axis=1)
        return np.where(np.any(hit, axis=1), ranges[first], np.inf).astype(np.float32)


class SimulationPlant:
    def __init__(self, pose=None, *, radius=.62, command_timeout=.3, max_linear=.3, max_angular=.5):
        if (not all(_finite(v) and v > 0 for v in (radius, command_timeout, max_linear, max_angular))
                or radius > 2 or command_timeout > .5 or max_linear > 1 or max_angular > 2):
            raise ValueError('Unsafe simulation bounds')
        self.pose = pose or PlanarPose()
        if not all(_finite(v) for v in (self.pose.x, self.pose.y, self.pose.yaw)):
            raise ValueError('Invalid simulation pose')
        self.radius, self.command_timeout = radius, command_timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self.command = (0., 0., 0.)
        self.command_time = None
        self.velocity = (0., 0., 0.)
        self.distance = 0.
        self.collision_stops = 0
        self.commands_received = 0

    def set_command(self, vx, vy, wz, now):
        if not all(_finite(v) for v in (vx, vy, wz, now)):
            self.command, self.command_time = (0., 0., 0.), None
            return False
        length = math.hypot(vx, vy)
        scale = min(1., self.max_linear/max(length, 1e-12))
        # Current Nav2 controller is nonholonomic; do not invent lateral motion.
        self.command = (vx*scale, 0., max(-self.max_angular, min(self.max_angular, wz)))
        self.command_time = now
        self.commands_received += 1
        return True

    def step(self, world, dt, now):
        self.velocity = (0., 0., 0.)
        if (world is None or not _finite(dt) or not 0 < dt <= .2 or not _finite(now)
                or self.command_time is None or not 0 <= now-self.command_time <= self.command_timeout):
            return
        vx, vy, wz = self.command
        # Midpoint planar integration, swept collision check prevents tunnelling.
        yaw_mid = self.pose.yaw+wz*dt*.5
        x = self.pose.x+(math.cos(yaw_mid)*vx-math.sin(yaw_mid)*vy)*dt
        y = self.pose.y+(math.sin(yaw_mid)*vx+math.cos(yaw_mid)*vy)*dt
        if not world.segment_clear((self.pose.x, self.pose.y), (x, y), self.radius):
            if abs(vx)+abs(vy)+abs(wz) > 0:
                self.collision_stops += 1
            return
        self.distance += math.hypot(x-self.pose.x, y-self.pose.y)
        self.pose = PlanarPose(x, y, math.atan2(math.sin(self.pose.yaw+wz*dt), math.cos(self.pose.yaw+wz*dt)))
        self.velocity = self.command

    def reset(self, pose, world):
        if (world is None or not all(_finite(v) for v in (pose.x, pose.y, pose.yaw))
                or not world.clear(pose.x, pose.y, self.radius)):
            return False
        self.pose = pose
        self.command = self.velocity = (0., 0., 0.)
        self.command_time = None
        return True


def main(args=None):
    import yaml
    config_path = os.environ.get('ZENOH_SESSION_CONFIG_URI', '')
    if not config_path:
        raise RuntimeError('SIMULATION requires an explicit offline Zenoh session file')
    config = yaml.safe_load(Path(config_path).read_text())
    validate_isolation(os.environ, config)

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from visualization_msgs.msg import Marker
    from .initial_pose import pose_payload

    class NavigationSimulator(Node):
        def __init__(self):
            super().__init__('navigation_simulator')
            defaults = {
                'allow_simulation': False, 'map_frame': 'd1max_loc_map',
                'odom_frame': 'd1max_loc_odom', 'base_frame': 'd1max_loc_base_link',
                'initial_x': 0., 'initial_y': 0., 'initial_yaw': math.pi/2,
                'footprint_radius': .62, 'command_timeout': .3, 'max_linear': .3,
                'max_angular': .5, 'map_version_id': '', 'scan_max_range': 18., 'scan_bins': 720,
            }
            p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            if p['allow_simulation'] is not True:
                raise RuntimeError('Simulator is disabled; require explicit allow_simulation:=true')
            if len({p['map_frame'], p['odom_frame'], p['base_frame']}) != 3:
                raise ValueError('Simulation frames must be distinct')
            if type(p['scan_bins']) is not int or not 360 <= p['scan_bins'] <= 1440:
                raise ValueError('Simulation scan_bins must be an integer between 360 and 1440')
            self.p = p
            self.world = None
            self.plant = SimulationPlant(PlanarPose(p['initial_x'], p['initial_y'], p['initial_yaw']),
                                         radius=p['footprint_radius'], command_timeout=p['command_timeout'],
                                         max_linear=p['max_linear'], max_angular=p['max_angular'])
            self.ready = False
            self.fault = 'waiting_for_map'
            self.goal = None
            self.previous = time.monotonic()
            self.scans = 0
            self.tf = TransformBroadcaster(self)
            self.static_tf = StaticTransformBroadcaster(self)
            identity = TransformStamped()
            identity.header.stamp = self.get_clock().now().to_msg()
            identity.header.frame_id, identity.child_frame_id = p['map_frame'], p['odom_frame']
            identity.transform.rotation.w = 1.
            self.static_tf.sendTransform(identity)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.map_sub = self.create_subscription(OccupancyGrid, '/d1max/navigation/map', self.receive_map, qos)
            self.cmd_sub = self.create_subscription(Twist, '/d1max/navigation/cmd_vel_safe', self.receive_command, 1)
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/d1max/navigation/initialpose', self.receive_pose, 1)
            self.scan_pub = self.create_publisher(LaserScan, '/d1max/navigation/scan', 2)
            self.local_pub = self.create_publisher(Odometry, '/d1max/localization/odometry/local', 5)
            self.global_pub = self.create_publisher(Odometry, '/d1max/localization/odometry/global', 5)
            self.pose_pub = self.create_publisher(PoseStamped, '/d1max/localization/pose', 5)
            self.path_pub = self.create_publisher(RosPath, '/d1max/localization/trajectory', 1)
            self.status_pub = self.create_publisher(String, '/d1max/navigation/runtime_status', 1)
            self.marker_pub = self.create_publisher(Marker, '/d1max/navigation/simulation_status', qos)
            self.path = RosPath()
            self.path.header.frame_id = p['map_frame']
            self.motion_timer = self.create_timer(1./30., self.tick)
            self.scan_timer = self.create_timer(.1, self.scan)
            self.status_timer = self.create_timer(.5, self.status)
            # These test controls exist only after the isolated-simulation
            # environment guard above. A live launch never creates this node.
            self.obstacle_service = self.create_service(SetBool, '/d1max/navigation/simulation/obstacle',
                lambda request, response: self.set_obstacle(request, response, False))
            self.close_obstacle_service = self.create_service(SetBool, '/d1max/navigation/simulation/close_obstacle',
                lambda request, response: self.set_obstacle(request, response, True))
            self.get_logger().warning('SOFTWARE SIMULATION ONLY — isolated Zenoh 7460; no robot connection')

        def set_obstacle(self, request, response, close):
            if not self.ready or self.world is None:
                response.success, response.message = False, 'simulation_not_ready'
                return response
            if not request.data:
                self.world.set_dynamic_obstacle(None)
                response.success, response.message = True, 'cleared'
                return response
            self.world.set_dynamic_obstacle(None)
            pose = self.plant.pose
            angles = [pose.yaw] if close else [pose.yaw+i*math.pi/8 for i in range(16)]
            distance, radius = (.72, .16) if close else (1.5, .2)
            for angle in angles:
                x, y = pose.x+distance*math.cos(angle), pose.y+distance*math.sin(angle)
                # Keep the far test region separated from pre-existing lethal
                # cells, so its before/after costmap evidence is attributable.
                if self.world.clear(x, y, radius+(.05 if close else .15)):
                    self.world.set_dynamic_obstacle((x, y, radius))
                    response.success = True
                    response.message = json.dumps({'x': x, 'y': y, 'radius': radius, 'close': close})
                    return response
            response.success, response.message = False, 'no_clear_test_obstacle_position'
            return response

        def receive_map(self, msg):
            if self.world is not None:
                return  # Session pins one immutable map; restart to change it.
            if msg.header.frame_id != self.p['map_frame']:
                self.fault = 'map_frame_mismatch'
                return
            q = msg.info.origin.orientation
            if abs(q.x)+abs(q.y) > 1e-6 or abs(q.z*q.z+q.w*q.w-1.) > .01:
                self.fault = 'non_planar_map_origin'
                return
            origin = (msg.info.origin.position.x, msg.info.origin.position.y, 2*math.atan2(q.z, q.w))
            try:
                self.world = OccupancyWorld(msg.data, msg.info.width, msg.info.height, msg.info.resolution, origin)
                self.ready = self.world.clear(self.plant.pose.x, self.plant.pose.y, self.plant.radius)
                self.fault = '' if self.ready else 'initial_pose_collides_draw_new_arrow'
                self.goal = self.world.suggest_goal(self.plant.pose, self.plant.radius) if self.ready else None
                self.get_logger().info(f'SIMULATION map ready={self.ready}; suggested test goal={self.goal}')
            except ValueError as exc:
                self.fault = str(exc)

        def receive_command(self, msg):
            if self.ready:
                self.plant.set_command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

        def receive_pose(self, msg):
            try:
                stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                payload = pose_payload(frame=msg.header.frame_id, expected_frame=self.p['map_frame'],
                                       stamp=stamp, now=self.get_clock().now().nanoseconds*1e-9,
                                       position=(p.x, p.y, p.z), quaternion=(q.x, q.y, q.z, q.w))
                if not self.plant.reset(PlanarPose(payload['x'], payload['y'], payload['yaw']), self.world):
                    raise ValueError('initial pose collides or map not ready')
                self.ready, self.fault = True, ''
                self.path.poses.clear()
                self.goal = self.world.suggest_goal(self.plant.pose, self.plant.radius)
                self.get_logger().warning('SIMULATION pose reset; not an actual robot localization result')
            except ValueError as exc:
                self.get_logger().error(f'Rejected simulated initial pose: {exc}')

        def tick(self):
            now = time.monotonic()
            dt, self.previous = now-self.previous, now
            if not self.ready:
                return
            self.plant.step(self.world, dt, now)
            pose = self.plant.pose
            stamp = self.get_clock().now().to_msg()
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.p['odom_frame'], self.p['base_frame']
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(pose.yaw/2), math.cos(pose.yaw/2)
            self.tf.sendTransform(tf)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.p['odom_frame'], self.p['base_frame']
            odom.pose.pose.position.x, odom.pose.pose.position.y = pose.x, pose.y
            odom.pose.pose.orientation = tf.transform.rotation
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = self.plant.velocity
            odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = .0001
            self.local_pub.publish(odom)
            odom.header.frame_id = self.p['map_frame']
            self.global_pub.publish(odom)
            body = PoseStamped()
            body.header, body.pose = odom.header, odom.pose.pose
            self.pose_pub.publish(body)

        def scan(self):
            if not self.ready:
                return
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p['base_frame']
            bins = self.p['scan_bins']
            msg.angle_min, msg.angle_increment = -math.pi, 2*math.pi/bins
            msg.angle_max = msg.angle_min+(bins-1)*msg.angle_increment
            msg.scan_time, msg.time_increment = .1, 0.
            msg.range_min, msg.range_max = .05, float(self.p['scan_max_range'])
            msg.ranges = self.world.raycast(self.plant.pose, count=bins, max_range=msg.range_max).tolist()
            self.scan_pub.publish(msg)
            self.scans += 1
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = msg.header.stamp, self.p['map_frame']
            pose.pose.position.x, pose.pose.position.y = self.plant.pose.x, self.plant.pose.y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(self.plant.pose.yaw/2), math.cos(self.plant.pose.yaw/2)
            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-1800:]
            self.path_pub.publish(self.path)

        def status(self):
            age = None if self.plant.command_time is None else time.monotonic()-self.plant.command_time
            state = {'mode': 'SIMULATION', 'robot_connected': False, 'real_motion_enabled': False,
                     'ready': self.ready, 'fault': self.fault, 'wall_time': time.time(),
                     'map_version_id': self.p['map_version_id'], 'router': '127.0.0.1:7460',
                     'pose': vars(self.plant.pose), 'velocity': self.plant.velocity,
                     'distance_m': self.plant.distance, 'commands_received': self.plant.commands_received,
                     'command_age_sec': age, 'command_timed_out': age is None or age > self.plant.command_timeout,
                     'collision_stops': self.plant.collision_stops, 'synthetic_scans': self.scans,
                     'scan_bins': self.p['scan_bins'],
                     'dynamic_obstacle': self.world.dynamic_obstacle if self.world is not None else None,
                     'suggested_test_goal': self.goal}
            msg = String()
            msg.data = json.dumps(state, allow_nan=False)
            self.status_pub.publish(msg)
            marker = Marker()
            marker.header.stamp, marker.header.frame_id = self.get_clock().now().to_msg(), self.p['map_frame']
            marker.ns, marker.id, marker.type, marker.action = 'simulation_only', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = self.plant.pose.x, self.plant.pose.y+2., 2.
            marker.pose.orientation.w, marker.scale.z = 1., .45
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .65, .05, 1.
            marker.text = 'NAV2 SOFTWARE SIMULATION\nROBOT DISCONNECTED' + ('' if self.ready else '\n'+self.fault)
            marker.lifetime.sec = 1
            self.marker_pub.publish(marker)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = NavigationSimulator()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
