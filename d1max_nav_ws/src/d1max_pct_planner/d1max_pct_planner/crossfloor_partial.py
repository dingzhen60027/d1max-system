"""Native PCT subroutes with endpoints on the configured stair or landing.

The original complete floor-to-floor coordinator is intentionally independent.
Portals select the topology; every connection below is a real native query on
an existing masked map, followed by the same original-map geometry check.
"""
from copy import deepcopy

import numpy as np

from .crossfloor_route import CrossfloorError, _check_leg, _inside_roi


def endpoint_region(point, settings):
    """Route category only, not a connectivity or traversability proof.

    The caller separately validates the selected measured support. A platform
    category still requires native PCT connectivity and whole-curve checks.
    """
    point = np.asarray(point, dtype=float)
    inside_stair = bool(_inside_roi(point, settings['stair_roi']))
    landing_z = settings['anchors']['landing']['xyz'][2]
    tolerance = settings['limits']['max_selection_height_error_m']
    for name, (low, high) in settings['floor_z_ranges'].items():
        if low <= point[2] <= high:
            # Floor Z bands deliberately overlap the first/last treads. They
            # select native map slices, not flat-terrain semantics. Inside the
            # declared stair, a measured surface already above the entry or
            # below the exit belongs to that flight. Keep the actual portal
            # level (and all ordinary floor outside the ROI) on its floor.
            # Reuse the map's existing surface-height tolerance; neither the
            # protected ROI alone nor a cost/clearance change grants passage.
            if inside_stair:
                if (name == 'lower' and
                        settings['anchors']['entry']['xyz'][2]+tolerance < point[2] < landing_z):
                    return 'stair_lower'
                if (name == 'upper' and
                        landing_z < point[2] < settings['anchors']['exit']['xyz'][2]-tolerance):
                    return 'stair_upper'
            return name
    if not inside_stair:
        raise CrossfloorError('endpoint_outside_declared_support',
                              '起终点不在已声明楼层或楼梯支撑区域内')
    if abs(point[2]-landing_z) <= tolerance:
        return 'landing'
    return 'stair_lower' if point[2] < landing_z else 'stair_upper'


def _requests(anchors, first, last):
    """At most four connected native legs, never a shortcut across two flights."""
    if first == last:
        return [('stair_landing' if first == 'landing' else first, 'start', 'goal')]
    attachment = {'lower': ('lower_floor', 'entry', 'stair_lower'),
                  'upper': ('upper_floor', 'exit', 'stair_upper'),
                  'landing': ('stair_landing', 'landing', 'landing')}
    requests, a, b = [], 'start', 'goal'
    initial, final = first, last
    if first in attachment:
        name, a, initial = attachment[first]
        requests.append((name, 'start', a))
    suffix = []
    if last in attachment:
        name, b, final = attachment[last]
        suffix.append((name, b, 'goal'))
    if initial == final:
        requests.append((initial, a, b))
    elif initial == 'landing':
        requests.append((final, a, b))
    elif final == 'landing':
        requests.append((initial, a, b))
    else:
        requests.extend([(initial, a, 'landing'), (final, 'landing', b)])
    requests.extend(suffix)
    # Portals are often the selected endpoint itself. Do not manufacture a
    # zero-length segment or a synthetic connector to satisfy segment counts.
    return [(name, a, b) for name, a, b in requests
            if not (anchors[a]['layer_id'] == anchors[b]['layer_id']
                    and np.array_equal(anchors[a]['xyz'], anchors[b]['xyz']))]


def _direction(first, last, start, goal):
    if first == last == 'landing':
        return 'same_floor'
    rank = {'lower': 0, 'stair_lower': 1, 'landing': 2, 'stair_upper': 3, 'upper': 4}
    upward = rank[last] > rank[first] if first != last else goal[2] >= start[2]
    return 'lower_to_upper' if upward else 'upper_to_lower'


def _merge_endpoint_portals(requests, anchors, tomogram):
    """Keep exact user XYZ when its optional portal is in the same native cell.

    Native PCT requires distinct cells. Do not issue an unplannable tiny leg,
    move the endpoint to the portal, or insert a synthetic short connector:
    the neighbouring native query instead plans directly to that exact point.
    """
    result = list(requests)
    attachments = ('lower_floor', 'upper_floor', 'stair_landing')
    def same_cell(first, last):
        a, b = anchors[first], anchors[last]
        return (a['layer_id'] == b['layer_id'] and np.array_equal(
            tomogram.index(a['xyz'][:2]), tomogram.index(b['xyz'][:2])))
    if (len(result) > 1 and result[0][0] in attachments and result[0][1] == 'start'
            and result[0][2] == result[1][1] and same_cell('start', result[0][2])):
        result[1] = (result[1][0], 'start', result[1][2])
        result.pop(0)
    if (len(result) > 1 and result[-1][0] in attachments and result[-1][2] == 'goal'
            and result[-2][2] == result[-1][1] and same_cell(result[-1][1], 'goal')):
        result[-2] = (result[-2][0], result[-2][1], 'goal')
        result.pop()
    return result


def _check_profile(points, layers, edges, settings, direction):
    """The existing grade/reversal limits, without requiring a whole flight."""
    transitions = []
    for index in np.flatnonzero(np.diff(layers)):
        leg = edges[index]
        if not leg.startswith('stair_') and abs(points[index+1, 2]-points[index, 2]) > .05:
            raise CrossfloorError('transition_outside_stair',
                                  'A height-changing layer transition is outside the stair', edge=int(index))
        transitions.append(dict(edge=int(index), leg=leg, from_layer=int(layers[index]),
                                to_layer=int(layers[index+1]),
                                xyz=points[index:index+2].mean(axis=0).tolist()))
    ids = [i for i, name in enumerate(edges) if name.startswith('stair_')]
    if not ids:
        raise CrossfloorError('missing_stair', 'Partial stair route contains no stair support edges')
    stair = points[ids[0]:ids[-1]+2]
    if not np.all(_inside_roi(stair, settings['stair_roi'])):
        raise CrossfloorError('stair_outside_roi', 'Partial stair route leaves its configured 3D ROI')
    delta = np.diff(stair, axis=0)
    horizontal = np.linalg.norm(delta[:, :2], axis=1)
    limits = settings['limits']
    if np.any(np.abs(delta[:, 2]) > limits['max_vertical_grade']*horizontal
              + limits['vertical_step_allowance_m']+1e-6):
        raise CrossfloorError('stair_vertical_jump', 'Partial stair route violates existing grade/step limits')
    sign = -1. if direction == 'upper_to_lower' else 1.
    signed = stair[:, 2]*sign
    reverse = float(np.max(np.maximum.accumulate(signed)-signed))
    if reverse > limits['max_reverse_height_m']+1e-6:
        raise CrossfloorError('stair_height_reversal', 'Partial stair route reverses height progression',
                              maximum_reverse_m=reverse)
    return transitions, dict(direction='level' if direction == 'same_floor' else 'up' if sign > 0 else 'down',
        height_change_m=float(stair[-1, 2]-stair[0, 2]), max_reverse_height_m=reverse,
        max_edge_grade=float(np.max(np.abs(delta[:, 2])/np.maximum(horizontal, 1e-6))),
        scope='selected_stair_subroute_not_complete_floor_transition')


def plan_partial_stair(tomogram, settings, start, goal, start_layer, goal_layer, *, resources):
    """Plan exact selected endpoints, using the existing three native resources."""
    first, last = endpoint_region(start, settings), endpoint_region(goal, settings)
    direction = _direction(first, last, start, goal)
    if direction == 'upper_to_lower':
        # Match the established complete stair coordinator: a static path is
        # computed in canonical ascent order, then reversed exactly. Native
        # slice search/GPMP tie breaks are directional; do not invent a second
        # geometry, change support or drop checks just for descent.
        result = plan_partial_stair(tomogram, settings, goal, start, goal_layer, start_layer,
                                    resources=resources)
        count = len(result['path_xyz'])
        for key in ('path_xyz', 'layer_ids', 'source_layer_ids', 'edge_legs'):
            result[key] = result[key][::-1]
        endpoint_names = {'start': 'goal', 'goal': 'start'}
        result['segments'] = [{**segment, 'first_index': count-1-segment['last_index'],
            'last_index': count-1-segment['first_index'],
            'from': endpoint_names.get(segment['to'], segment['to']),
            'to': endpoint_names.get(segment['from'], segment['from'])}
            for segment in result['segments'][::-1]]
        result['anchors']['start'], result['anchors']['goal'] = (
            result['anchors']['goal'], result['anchors']['start'])
        result.update(direction=direction, start_region=first, goal_region=last)
        result['layer_transitions'], result['stair_profile'] = _check_profile(
            np.asarray(result['path_xyz']), np.asarray(result['layer_ids']),
            result['edge_legs'], settings, direction)
        tomogram.validate_path(result['path_xyz'], result['layer_ids'],
                               max_ground_step_m=settings['limits']['max_ground_step_m'])
        return result
    anchors = deepcopy(settings['anchors'])
    anchors['start'] = dict(xyz=np.asarray(start, dtype=float), layer_id=int(start_layer))
    anchors['goal'] = dict(xyz=np.asarray(goal, dtype=float), layer_id=int(goal_layer))
    if first in ('lower', 'upper') and last in ('lower', 'upper'):
        raise ValueError('Complete floor routes belong to the existing coordinator')
    requests = _merge_endpoint_portals(_requests(anchors, first, last), anchors, tomogram)
    if not requests:
        raise CrossfloorError('empty_partial_route', '起终点位于同一支撑点，无需楼梯路径')
    points, layers, edges, records = [], [], [], []
    for name, begin_name, end_name in requests:
        begin, end = anchors[begin_name], anchors[end_name]
        resource_name = 'stair_lower' if name == 'stair_landing' else name
        try:
            masked, route = resources(resource_name, settings)
            raw = route.plan(begin['xyz'], end['xyz'], begin['layer_id'], end['layer_id'])
        except Exception as exc:
            raise CrossfloorError('segment_planning_failed', f'Native PCT failed on {name}',
                cause_code=getattr(exc, 'code', type(exc).__name__), cause=str(exc),
                cause_details=getattr(exc, 'details', None)) from exc
        if not isinstance(raw, dict):
            raise CrossfloorError('segment_planning_failed', f'Native PCT returned no route on {name}')
        xyz, ids, checks = _check_leg(raw, tomogram, masked, name, begin, end, settings)
        if name == 'stair_landing':
            # Do not visit a flight and return merely to connect two platform
            # points. This uses the already configured support-height tolerance.
            if np.any(np.abs(xyz[:, 2]-anchors['landing']['xyz'][2]) >
                      settings['limits']['max_selection_height_error_m']+1e-6):
                raise CrossfloorError('landing_height_escape', 'Native landing path leaves platform support')
        if points and (not np.array_equal(xyz[0], points[-1]) or int(ids[0]) != layers[-1]):
            raise CrossfloorError('portal_mismatch', f'{name} does not connect to the preceding native segment')
        first_index = max(0, len(points)-1)
        points.extend(xyz[1:] if points else xyz)
        layers.extend((ids[1:] if layers else ids).tolist())
        edges.extend([name]*(len(xyz)-1))
        records.append(dict(name=name, **{'from': begin_name, 'to': end_name},
            kind='landing' if name == 'stair_landing' else 'stair' if name.startswith('stair_') else 'floor',
            first_index=first_index, last_index=len(points)-1, partial=True, checks=checks,
            native_algorithm=raw.get('algorithm', 'PCT route (test seam)'),
            curve_validation=raw.get('curve_validation'), quintic_segments=raw.get('quintic_segments', 0),
            curve_partition_points=raw.get('curve_partition_points', 0), fixed_leg_cache_hit=False,
            length_m=float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
            layer_ids=sorted(set(ids.tolist()))))
    points, layers = np.asarray(points), np.asarray(layers, dtype=int)
    whole = tomogram.validate_path(points, layers, max_ground_step_m=settings['limits']['max_ground_step_m'])
    transitions, profile = _check_profile(points, layers, edges, settings, direction)
    return dict(status='validated_offline_route', route_type='partial_stair', direction=direction,
        start_region=first, goal_region=last, frame_id=settings['frame_id'],
        tomogram_path=str(tomogram.source), tomogram_sha256=tomogram.sha256,
        source_pcd=settings['source_pcd'], source_pcd_sha256=tomogram.provenance.get('source_sha256'),
        path_xyz=points.tolist(), layer_ids=layers.tolist(),
        source_layer_ids=tomogram.source_layers[layers].astype(int).tolist(), edge_legs=edges,
        anchors={name: dict(xyz=value['xyz'].tolist(), layer_id=value['layer_id'])
                 for name, value in anchors.items()}, segments=records,
        layer_transitions=transitions, stair_profile=profile, whole_route_check=whole,
        stair_refinement=dict(applied=False, reason='exact_selected_endpoints_native_subroutes'),
        length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        execution_authorized=False)
