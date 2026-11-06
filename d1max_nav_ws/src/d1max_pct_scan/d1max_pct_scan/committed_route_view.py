"""Display-only history of the last immutable, BT-committed global route.

This cache is deliberately NOT RouteIngress: cancellation retires execution,
but does not erase the last route someone may still need to inspect. Nothing
returned here is an active reference, collision proof, or execution permission.
No wall-clock lease is extended and the original source header is preserved.
"""
from copy import deepcopy
from pathlib import Path

from .source_route_ros import from_message


def session_route_provenance(session):
    """Configured display binding only; no geometry or motion proof is made.

    Formal sessions contain all three pinned hashes. Minimal legacy view/test
    sessions can provide fewer fields; never invent missing artifact hashes.
    """
    result = {}
    if session.get('frame_id'):
        result['frame_id'] = session['frame_id']
    hashes = session.get('input_hashes', {})
    for field, path_field in (('source_map_sha256', 'map_pcd'),
                             ('tomogram_sha256', 'tomogram_npz'),
                             ('conditioning_sha256', 'planning_manifest')):
        if session.get(path_field):
            value = hashes.get(str(Path(session[path_field]).resolve()))
            if value is not None:
                result[field] = value
    return result


class CommittedRouteView:
    def __init__(self, session_id, map_version_id, *, expected_provenance=None):
        self.session_id = self.map_version_id = None
        self.expected_provenance = {}
        self.select_context(session_id, map_version_id, expected_provenance=expected_provenance)

    def select_context(self, session_id, map_version_id, *, expected_provenance=None):
        """Explicit view configuration, never inferred from a foreign packet."""
        if not session_id or not map_version_id:
            raise ValueError('committed_route_view_context_required')
        same_context = (session_id, map_version_id) == (self.session_id, self.map_version_id)
        provenance = dict(self.expected_provenance if same_context and expected_provenance is None
                          else expected_provenance or {})
        if set(provenance) - {'frame_id', 'source_map_sha256', 'tomogram_sha256', 'conditioning_sha256'}:
            raise ValueError('committed_route_view_provenance_fields_invalid')
        if any(not isinstance(value, str) or not value for value in provenance.values()):
            raise ValueError('committed_route_view_provenance_required')
        if any(len(value) != 64 or any(c not in '0123456789abcdef' for c in value)
               for field, value in provenance.items() if field != 'frame_id'):
            raise ValueError('committed_route_view_provenance_hash_invalid')
        if same_context and provenance == self.expected_provenance:
            return False
        self.session_id, self.map_version_id = session_id, map_version_id
        self.expected_provenance = provenance
        self.last_sequence = 0
        self.last_snapshot_stamp_ns = 0
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

    def receive_snapshot(self, source):
        """Display a typed global-route proposal, never synthesize a reference.

        Snapshot messages have no delivery sequence or active/retired signal.
        Strict original-source-time ordering and remembered retired identities
        fence late results. Old source time remains inspectable; no current-time
        freshness or motion permission is inferred from this display cache.
        """
        try:
            identity = (source.task_id, source.route_id, source.route_hash)
            stamp = source.path.header.stamp
            stamp_ns = stamp.sec*1_000_000_000+stamp.nanosec
            if (source.session_id != self.session_id or source.map_version_id != self.map_version_id
                    or not all(identity) or identity == self.identity or identity in self.retired
                    or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000
                    or stamp_ns <= self.last_snapshot_stamp_ns
                    or any(getattr(source, field) != expected
                           for field, expected in self.expected_provenance.items())):
                return None
            audited = from_message(source)
        except (ValueError, TypeError, AttributeError):
            return None
        if not audited.payload()['preview_ready'] or len(source.path.poses) < 2:
            return None
        if self.identity is not None:
            self.retired.add(self.identity)
        if len(self.retired) > 1024:
            self.retired = {self.identity} if self.identity is not None else set()
        self.identity = identity
        self.snapshot = deepcopy(source)
        self.last_snapshot_stamp_ns = stamp_ns
        self.historical = False
        return deepcopy(self.snapshot.path)

    def displayed_path(self):
        return None if self.snapshot is None else deepcopy(self.snapshot.path)


def publish_committed_route(cache, publisher, message):
    """Production callback: one publication per new committed route, no timer."""
    path = cache.receive(message)
    if path is not None:
        publisher.publish(path)


def publish_route_snapshot(cache, publisher, message):
    """Global-route display callback only, with no reference-delivery envelope."""
    path = cache.receive_snapshot(message)
    if path is not None:
        publisher.publish(path)


def main():
    import argparse
    import json
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from d1max_navigation_bt_interfaces.msg import RouteReference, RouteSnapshot
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
            self.cache = CommittedRouteView(session['id'], session['version_id'],
                                            expected_provenance=session_route_provenance(session))
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.path = self.create_publisher(NavPath,
                '/d1max/live_planning/committed_route_visual', qos)
            # A different configured session/map has no inherited active route.
            empty = NavPath()
            empty.header.frame_id = session['frame_id']
            self.path.publish(empty)
            if session.get('planning_scope') == 'global_route':
                self.create_subscription(RouteSnapshot,
                    '/d1max/live_planning/bt/route_snapshot',
                    lambda message: publish_route_snapshot(self.cache, self.path, message), qos)
            else:
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
        try:
            rclpy.try_shutdown()
        except RCLError as error:
            # Humble's signal thread can close the context after the internal
            # check. Only that already-completed shutdown is benign.
            if rclpy.ok() or 'rcl_shutdown already called' not in str(error):
                raise


if __name__ == '__main__':
    main()
