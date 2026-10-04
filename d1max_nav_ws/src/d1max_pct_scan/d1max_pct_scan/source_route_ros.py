"""Typed RouteSnapshot wire codec. Path-only messages have no route authority."""
import json

from .source_route import RouteSnapshot, MAX_JSON_BYTES


def to_message(snapshot, *, session_id, task_id, route_id, epoch, seed_id, stamp):
    from d1max_navigation_bt_interfaces.msg import RouteSnapshot as Message, RouteSegment
    from geometry_msgs.msg import PoseStamped
    value = snapshot.payload()
    message = Message()
    for key in ('schema_version', 'map_version_id', 'source_map_sha256', 'tomogram_sha256',
                'conditioning_sha256', 'frame_id', 'point_reference', 'direction',
                'source_layer_ids', 'layer_ids', 'point_floor_ids', 'edge_segments',
                'preview_ready', 'execution_eligible', 'eligibility_reason',
                'has_goal_yaw', 'goal_yaw', 'goal_yaw_tolerance_rad'):
        setattr(message, key, value[key])
    message.session_id, message.task_id, message.route_id = session_id, task_id, route_id
    message.localization_epoch, message.localization_seed_id = epoch, seed_id
    message.route_hash = snapshot.route_hash
    message.geometry_evidence_json = json.dumps(value['geometry_evidence'], sort_keys=True,
                                               separators=(',', ':'), allow_nan=False)
    message.path.header.frame_id, message.path.header.stamp = value['frame_id'], stamp
    for xyz in value['xyz']:
        point = PoseStamped()
        point.header = message.path.header
        point.pose.position.x, point.pose.position.y, point.pose.position.z = map(float, xyz)
        point.pose.orientation.w = 1.
        message.path.poses.append(point)
    for item in value['segments']:
        segment = RouteSegment()
        for key, entry in item.items():
            setattr(segment, key, entry)
        message.segments.append(segment)
    return message


def from_message(message):
    if (getattr(message, 'schema_version', None) != 2
            or len(message.geometry_evidence_json.encode()) > MAX_JSON_BYTES
            or len(message.path.poses) > 20000 or len(message.segments) > 64):
        raise ValueError('route_snapshot_wire_schema_or_size_invalid')
    if (message.path.header.frame_id != message.frame_id
            or any(point.header.frame_id not in ('', message.frame_id)
                   for point in message.path.poses)):
        raise ValueError('route_snapshot_path_frame_mismatch')
    value = {key: getattr(message, key) for key in (
        'schema_version', 'map_version_id', 'source_map_sha256', 'tomogram_sha256',
        'conditioning_sha256', 'frame_id', 'point_reference', 'direction',
        'preview_ready', 'execution_eligible', 'eligibility_reason',
        'has_goal_yaw', 'goal_yaw', 'goal_yaw_tolerance_rad')}
    for key in ('source_layer_ids', 'layer_ids', 'point_floor_ids', 'edge_segments'):
        value[key] = list(getattr(message, key))
    value['xyz'] = [[getattr(point.pose.position, axis) for axis in 'xyz']
                    for point in message.path.poses]
    keys = ('segment_id', 'kind', 'floor_id', 'begin_index', 'end_index',
        'source_layer_id', 'target_layer_id', 'entry_portal_id', 'exit_portal_id',
        'required_mode', 'direction', 'execution_eligible', 'eligibility_reason')
    value['segments'] = [{key: getattr(segment, key) for key in keys}
                         for segment in message.segments]
    value['geometry_evidence'] = json.loads(message.geometry_evidence_json)
    snapshot = RouteSnapshot.create(value)
    if message.route_hash != snapshot.route_hash:
        raise ValueError('route_snapshot_hash_mismatch')
    return snapshot
