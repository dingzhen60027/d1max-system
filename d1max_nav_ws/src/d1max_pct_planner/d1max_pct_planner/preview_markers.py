"""Native MoveIt-style pose handles for the planning-only RViz editor.

No MoveIt runtime, robot model, SDK, TF publisher or motion interface is needed.
The message contract follows MoveIt 2 Humble's add6DOFControl() and
addViewPlaneControl(): https://github.com/moveit/moveit2/blob/humble/
moveit_ros/robot_interaction/src/interactive_marker_helpers.cpp

Axis visuals intentionally remain empty. Humble's interactive_markers::autoComplete
creates the standard broad arrows and rotation rings in RViz. Its tools.cpp
uses each control quaternion for BOTH the axis and rendered geometry, and
assigns RGB from the world direction (X red, Y green, Z blue). We must not
replace them with tiny custom arrows whose pose forgets that rotation.
"""
import math

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


# The local X direction is the translation/rotation axis in RViz. All six
# controls are FIXED, so rotating the selected pose does not rotate these axes.
WORLD_AXIS_QUATERNIONS = {
    'x': (0.0, 0.0, 0.0, 1.0),
    'y': (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
    'z': (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),
}
ROLE_COLORS = {'start': (0.18, 0.94, 0.48), 'goal': (1.0, 0.57, 0.12)}
INVALID_COLOR = (1.0, 0.12, 0.18)


def _finite_values(values, count, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{label} must contain {count} finite numbers') from exc
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return result


def _orientation(target, xyzw):
    target.x, target.y, target.z, target.w = xyzw


def _color(marker, rgb, alpha=1.0):
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha


def pose_marker(role, xyz, quaternion_xyzw, frame, scale, valid=True, ground_follow=False):
    """Build an editable pose, preserving its true XYZ and selected orientation.

    The centre uses native MOVE_ROTATE_3D: ordinary drag translates in the
    view plane, Shift changes depth, and Ctrl enables rotation (RViz's own
    interaction implementation). Dedicated arrow/ring controls constrain
    translation/rotation to world X, Y or Z.

    ``valid`` changes the centre/label only; invalid locations remain editable.
    Orientation is editor state, not a promise that the PCT path constrains
    the robot's final orientation. The server owns that distinction.

    Geometry uses scale-relative dimensions: centre radius 0.13, orientation
    arrow length 0.38, standard rings radius 0.5--0.65, standard axis arrows
    span 0.5--0.9. Thus axis arrows do not cover the centre drag target.
    """
    if role not in ROLE_COLORS:
        raise ValueError('Marker role must be start or goal')
    position = _finite_values(xyz, 3, 'Position')
    quaternion = _finite_values(quaternion_xyzw, 4, 'Quaternion')
    norm = math.hypot(*quaternion)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('Quaternion must have nonzero finite norm')
    quaternion = tuple(value / norm for value in quaternion)
    if not isinstance(frame, str) or not frame.strip() or frame != frame.strip():
        raise ValueError('An explicit map frame is required')
    try:
        marker_scale = float(scale)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('Handle scale must be finite and positive') from exc
    if isinstance(scale, bool) or not math.isfinite(marker_scale) or marker_scale <= 0:
        raise ValueError('Handle scale must be finite and positive')
    if not isinstance(valid, bool):
        raise ValueError('Marker validity must be a boolean')

    marker = InteractiveMarker()
    marker.header.frame_id = frame
    marker.name = role
    marker.description = role.upper() if valid else role.upper() + ' · INVALID'
    marker.scale = marker_scale
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    _orientation(marker.pose.orientation, quaternion)

    # MoveIt addViewPlaneControl(..., position=true, orientation=true) uses
    # VIEW_FACING + independent_marker_orientation: dragging stays aligned
    # with the viewport while the visible pose indicator keeps its orientation.
    center = InteractiveMarkerControl(name='move_rotate_3d', always_visible=True)
    center.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    center.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    center.orientation.w = 1.0
    center.independent_marker_orientation = True
    center.description = 'Drag: move · Shift: depth · Ctrl: rotate'
    if ground_follow:
        center.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        center.orientation_mode = InteractiveMarkerControl.FIXED
        _orientation(center.orientation, WORLD_AXIS_QUATERNIONS['z'])
        center.description = 'Move in map XY; height follows the selected ground surface'
    rgb = ROLE_COLORS[role] if valid else INVALID_COLOR
    sphere = Marker(type=Marker.SPHERE)
    sphere.pose.orientation.w = 1.0
    sphere.scale.x = sphere.scale.y = sphere.scale.z = marker_scale * 0.26
    _color(sphere, rgb)
    center.markers.append(sphere)

    # This central arrow communicates the selected pose's forward direction;
    # it is part of the free-drag control, not one of the world-axis handles.
    forward = Marker(type=Marker.ARROW)
    forward.pose.orientation.w = 1.0
    forward.scale.x = marker_scale * 0.38
    forward.scale.y = marker_scale * 0.065
    forward.scale.z = marker_scale * 0.11
    _color(forward, tuple(0.30 + 0.70 * component for component in rgb))
    center.markers.append(forward)
    marker.controls.append(center)

    for axis, quaternion in WORLD_AXIS_QUATERNIONS.items():
        for action, mode in (('rotate', InteractiveMarkerControl.ROTATE_AXIS),
                             ('move', InteractiveMarkerControl.MOVE_AXIS)):
            if ground_follow and not ((action == 'move' and axis in ('x', 'y'))
                                       or (action == 'rotate' and axis == 'z')):
                continue
            control = InteractiveMarkerControl(name=f'{action}_{axis}', always_visible=True)
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            _orientation(control.orientation, quaternion)
            control.description = f'{action.capitalize()} world {axis.upper()}'
            # Standard client-generated visuals are deliberate, not missing:
            # autoComplete supplies broad arrows/rings, including pose/color.
            marker.controls.append(control)
    return marker
