"""Display-only history of the last immutable, BT-committed global route.

This cache is deliberately NOT RouteIngress: cancellation retires execution,
but does not erase the last route someone may still need to inspect. Nothing
returned here is an active reference, collision proof, or execution permission.
No wall-clock lease is extended and the original source header is preserved.
"""
from copy import deepcopy

from .source_route_ros import from_message


class CommittedRouteView:
    def __init__(self, session_id, map_version_id):
        self.session_id = self.map_version_id = None
        self.select_context(session_id, map_version_id)

    def select_context(self, session_id, map_version_id):
        """Explicit view configuration, never inferred from a foreign packet."""
        if not session_id or not map_version_id:
            raise ValueError('committed_route_view_context_required')
        if (session_id, map_version_id) == (self.session_id, self.map_version_id):
            return False
        self.session_id, self.map_version_id = session_id, map_version_id
        self.last_sequence = 0
        self.snapshot = None
        self.identity = None
        self.historical = False
        self.retired = set()
        return True

    def receive(self, message):
        """Return a NEW display Path, or None when no display update is needed.

        Valid inactive messages mark the matching route historical. They never
        publish an empty Path. Source time need not be recent: a transient-local
        commit can legitimately be inspected after the navigation has ended.
        Its source timestamp is not replaced with reception/rendering time.
        """
        source = message.snapshot
        if (message.schema_version != 2 or message.session_id != self.session_id
                or source.map_version_id != self.map_version_id
                or message.delivery_sequence <= self.last_sequence):
            return None
        identity = (message.task_id, message.route_id, message.route_hash)
        if (not all(identity) or source.session_id != message.session_id
                or (source.task_id, source.route_id, source.route_hash) != identity
                or source.path.header.stamp != message.source_stamp
                or message.source_stamp.sec < 0
                or not 0 <= message.source_stamp.nanosec < 1_000_000_000):
            return None
        try:
            audited = from_message(source)
        except (ValueError, TypeError, AttributeError):
            return None
        if not audited.payload()['preview_ready'] or len(source.path.poses) < 2:
            return None
        if not message.active:
            if identity != self.identity:
                return None
            self.last_sequence = message.delivery_sequence
            self.historical = True
            self.retired.add(identity)
            return None
        if identity in self.retired:
            return None
        self.last_sequence = message.delivery_sequence
        if identity == self.identity:
            # Re-delivery is not a new map/path or a new source timestamp.
            return None
        if self.identity is not None:
            self.retired.add(self.identity)
        # Only a bounded set of recently retired identities is needed by this
        # non-authoritative view; the monotonic delivery fence remains primary.
        if len(self.retired) > 1024:
            self.retired = {self.identity} if self.identity is not None else set()
        self.identity = identity
        self.snapshot = deepcopy(source)
        self.historical = False
        return deepcopy(self.snapshot.path)

    def displayed_path(self):
        return None if self.snapshot is None else deepcopy(self.snapshot.path)


def publish_committed_route(cache, publisher, message):
    """Production callback: one publication per new committed route, no timer."""
    path = cache.receive(message)
    if path is not None:
        publisher.publish(path)


def main():
    import argparse
    import json
    from pathlib import Path
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from d1max_navigation_bt_interfaces.msg import RouteReference
    from nav_msgs.msg import Path as NavPath

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    args, rosargs = parser.parse_known_args()
    session = json.loads((args.session/'session.json').read_text())
    if session.get('pipeline_contract') != 'single_floor_v3':
        raise ValueError('committed_route_view_requires_v3_session')

    class View(Node):
        def __init__(self):
            super().__init__('single_floor_committed_route_view')
            self.cache = CommittedRouteView(session['id'], session['version_id'])
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.path = self.create_publisher(NavPath,
                '/d1max/live_planning/committed_route_visual', qos)
            # A different configured session/map has no inherited active route.
            empty = NavPath()
            empty.header.frame_id = session['frame_id']
            self.path.publish(empty)
            self.create_subscription(RouteReference,
                '/d1max/live_planning/committed_route',
                lambda message: publish_committed_route(self.cache, self.path, message), qos)

    rclpy.init(args=rosargs)
    node = View()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
