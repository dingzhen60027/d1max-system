"""Explicit smooth strategy for an original-coordinate, measured single floor.

No raster union or flattened height replaces the production tomogram. Every
sample retains an actual PCT slice; adjacent-slice changes require a checked
measured overlap. Polynomial cell crossings are checked before publishing a
polyline. This is not GPMP, nor a certificate of dynamic feasibility.
"""
import numpy as np
from scipy.spatial import cKDTree

from .corridor_refinement import (
    MAX_INPUT_POINTS, MAX_OUTPUT_POINTS, MAX_VISIBILITY_CHECKS,
    MAX_POLYLINE_VERTICES, MAX_XY_LENGTH_M, OUTPUT_SPACING_M,
    VISIBILITY_ANCHOR_SPACING_M, _line, _rounded_segments, _quality_not_worse,
)
from .path_quality import path_quality
from .tomogram_map import TomogramError


class ObservedGround:
    """Ground heights selected from original returns, never fitted flat."""
    def __init__(self, xyz):
        self.xyz = np.asarray(xyz,dtype=float)
        if self.xyz.ndim != 2 or self.xyz.shape[1] != 3 or not len(self.xyz) or not np.isfinite(self.xyz).all():
            raise ValueError('observed_ground_returns_invalid')
        self.tree = cKDTree(self.xyz[:,:2])

    def query(self, xy):
        distance,ids=self.tree.query(xy,k=1,workers=1)
        if np.any(distance > .12):
            raise TomogramError('smooth_curve_source_support_missing','Curve leaves measured source support')
        return self.xyz[np.asarray(ids),2]


def lift_curve(tomogram, segments, allowed_layers, start, goal, start_layer, goal_layer, source_ground=None):
    """Find slice identities over the exact XY curve, using the real validator."""
    from .tomogram_route import curve_partition
    points = []
    partition_count = 0
    for segment in segments:
        partition = curve_partition(segment['coefficients_xy'], tomogram)
        count = max(1, int(np.ceil(segment['derivative_bound_m'] / OUTPUT_SPACING_M)))
        if count + len(points) > MAX_OUTPUT_POINTS:
            raise TomogramError('refinement_output_limit', 'Curve point budget exceeded')
        times = sorted(set(partition) | set(np.linspace(0., 1., count + 1)))
        xy = np.polynomial.polynomial.polyval(times, segment['coefficients_xy']).T
        if points and np.linalg.norm(points[-1] - xy[0]) < 1e-10:
            xy = xy[1:]
        points.extend(xy)
        partition_count += len(partition)
    xy = np.asarray(points, dtype=float)
    if len(xy) < 2 or len(xy) > MAX_OUTPUT_POINTS:
        raise TomogramError('refinement_output_limit', 'Insufficient or excessive curve samples')
    xy[0], xy[-1] = start[:2], goal[:2]
    # Cheap rejection only: a cell blocked in every allowed real slice cannot
    # be rescued by DP. Positive results still go through the full supercover
    # and overlap validator; this is not a union-map collision test.
    index=np.rint((xy-tomogram.center)/tomogram.resolution).astype(int)+tomogram.offset
    indices=(np.asarray(allowed_layers,dtype=int)[:,None],index[:,0][None,:],index[:,1][None,:])
    costs,ground=tomogram.cost[indices],tomogram.ground[indices]
    if np.any(~np.any(np.isfinite(ground)&np.isfinite(costs)&(costs>=0)&(costs<=tomogram.COST_THRESHOLD),axis=0)):
        raise TomogramError('smooth_curve_all_slices_blocked','Curve crosses a blocked/unknown cell in every permitted slice')
    observed_z = None if source_ground is None else source_ground.query(xy)
    # Most single-floor routes can stay in one real overlapping slice. Try
    # these complete, fully checked candidates before the bounded DP. This
    # avoids thousands of tiny Python validator calls, without a merged map
    # or cached acceptance. Boundary vertices explicitly retain layer changes.
    order = sorted(allowed_layers,key=lambda l:(int(l)!=start_layer,int(l)))
    for selected in order:
        if abs(selected-start_layer)>1 or abs(selected-goal_layer)>1:
            continue
        try:
            heights = tomogram.surface_ground_z_many(xy,int(selected))
            candidate = np.c_[xy,heights if observed_z is None else observed_z]
            candidate = np.vstack([start,candidate,goal])
            ids = np.r_[start_layer,np.full(len(xy),selected,dtype=int),goal_layer]
            checked = tomogram.validate_path(candidate,ids)
            return candidate,ids,dict(checked,curve_partition_points=partition_count)
        except TomogramError:
            pass
    parents, vertices = [], []
    states = {int(start_layer): 0.}
    previous = {int(start_layer): np.asarray(start, dtype=float)}
    for i, point in enumerate(xy):
        options, ancestry, costs = {}, {}, {}
        for layer in allowed_layers:
            layer = int(layer)
            if i == 0 and layer != start_layer or i == len(xy)-1 and layer != goal_layer:
                continue
            try:
                ground = tomogram.surface_ground_z(point, layer)
            except TomogramError:
                continue
            vertex = np.r_[point, ground if observed_z is None else observed_z[i]]
            if i == 0:
                vertex = np.asarray(start, dtype=float)
            elif i == len(xy)-1:
                vertex = np.asarray(goal, dtype=float)
            for prior in states:
                if abs(layer-prior) > 1:
                    continue
                # A slice transition is overlap on the same physical floor,
                # not permission to jump up a step or onto another level.
                if layer != prior and abs(vertex[2]-previous[prior][2]) > .1 + 1e-6:
                    continue
                try:
                    tomogram.validate_path([previous[prior], vertex], [prior, layer])
                except TomogramError:
                    continue
                cost = states[prior] + (1. if layer != prior else 0.)
                if layer not in costs or cost < costs[layer]:
                    costs[layer], ancestry[layer], options[layer] = cost, prior, vertex
        if not options:
            raise TomogramError('smooth_curve_no_measured_slice',
                                'No checked same-floor slice supports the smooth curve',
                                sample=i, xy=point.tolist())
        states, previous = costs, options
        parents.append(ancestry)
        vertices.append(options)
    layer = int(goal_layer)
    path, layers = [], []
    for i in range(len(xy)-1, -1, -1):
        path.append(vertices[i][layer]); layers.append(layer)
        layer = parents[i][layer]
    path, layers = np.asarray(path[::-1]), np.asarray(layers[::-1], dtype=int)
    checks = tomogram.validate_path(path, layers)
    return path, layers, dict(checks, curve_partition_points=partition_count)


def refine_source_identity(tomogram, path_xyz, layer_ids, corner_cut_m=1.5, source_ground=None):
    """Refine a *valid native A-star* path, never rescue invalid GPMP output."""
    if tomogram.provenance.get('geometry_operation') != 'source_identity':
        raise TomogramError('source_identity_required', 'This strategy is restricted to source-identity artifacts')
    path, layers = np.asarray(path_xyz, dtype=float), np.asarray(layer_ids, dtype=int)
    if len(path) < 2 or len(path) > MAX_INPUT_POINTS:
        raise TomogramError('refinement_input_limit', 'Native route exceeds refinement bounds')
    tomogram.validate_path(path, layers)
    before = path_quality(path)
    if before['xy_length_m'] > MAX_XY_LENGTH_M or before['closed_xy']:
        raise TomogramError('refinement_input_limit', 'Closed or oversized route is unsupported')
    available = sorted(set(layers.tolist()))
    # Do not promote previously absent floors/layers during smoothing.
    kept = np.r_[True, np.linalg.norm(np.diff(path[:,:2], axis=0), axis=1) > 1e-9]
    kept[-1] = True
    reduced, labels = path[kept], layers[kept]
    cache, geometry, checks_count = {}, {}, 0

    def visible(a, b):
        nonlocal checks_count
        key = (a, b)
        if key in cache:
            return cache[key]
        checks_count += 1
        if checks_count > MAX_VISIBILITY_CHECKS:
            raise TomogramError('refinement_visibility_limit', 'Visibility budget exceeded')
        try:
            result = lift_curve(tomogram, [_line(reduced[a,:2], reduced[b,:2])], available,
                                reduced[a], reduced[b], int(labels[a]), int(labels[b]),source_ground)
            cache[key] = True
            # Only the exact direct route needs reuse; avoid unbounded arrays.
            if a == 0 and b == len(reduced)-1:
                geometry[key] = result
        except TomogramError as exc:
            if exc.code.endswith('_limit'):
                raise
            cache[key] = False
        return cache[key]

    direct = visible(0, len(reduced)-1)
    selected = [0]
    if direct:
        selected.append(len(reduced)-1)
    else:
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reduced[:,:2], axis=0), axis=1))]
        coarse = [0]
        for i in range(1, len(reduced)-1):
            if arc[i]-arc[coarse[-1]] >= VISIBILITY_ANCHOR_SPACING_M:
                coarse.append(i)
        coarse.append(len(reduced)-1)
        while selected[-1] < len(reduced)-1:
            current = selected[-1]
            chosen = next((index for index in reversed(coarse)
                           if index > current and visible(current, index)), None)
            if chosen is None:
                chosen = current+1
                if not visible(current, chosen):
                    raise TomogramError('no_checked_visibility_successor', 'No checked visibility successor')
            selected.append(chosen)
            if len(selected) > MAX_POLYLINE_VERTICES:
                raise TomogramError('refinement_visibility_limit', 'Visibility vertex budget exceeded')
    polygon = reduced[selected,:2]
    failures = []
    for scale in ([1.] if direct else [1., .7, .4, .2, .1]):
        try:
            segments = [_line(polygon[0],polygon[-1])] if direct else _rounded_segments(polygon,corner_cut_m*scale)
            candidate, identities, audit = (geometry[(0,len(reduced)-1)] if direct else
                lift_curve(tomogram,segments,available,path[0],path[-1],layers[0],layers[-1],source_ground))
            # Revalidate even a cached exact straight line.
            audit.update(tomogram.validate_path(candidate,identities))
            after = path_quality(candidate)
            # Measured-ground Z can legitimately differ from the PCT max
            # return. Never flatten it to improve a 3D length metric.
            worse = [m for m in _quality_not_worse(before,after) if m != 'xyz_length_m']
            if worse:
                raise TomogramError('quality_not_improved','Smooth candidate worsens route',metrics=worse)
            return dict(path=candidate.tolist(),layer_ids=identities.tolist(),applied=True,
                reason='checked_same_floor_multislice_smoothing',is_native_gpmp=False,
                algorithm='native PCT A-star + checked visibility/quintic XY smoothing',
                curve_validation='polynomial_cell_boundary_roots_interiors_and_actual_slice_overlap',
                before_quality=before,after_quality=after,visibility_checks=checks_count,
                visibility_polyline_xy=polygon.tolist(),corner_cut_scale=scale,
                continuity='XY geometric C2; measured source ground Z, no 3D/dynamic C2 guarantee',
                physical_safety_certified=False,prior_candidate_failures=failures,
                segments=[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in s.items()} for s in segments],
                **audit)
        except TomogramError as exc:
            failures.append(dict(cut_scale=scale,code=exc.code,details=exc.details))
    raise TomogramError('no_valid_smooth_candidate','No fully checked smooth route',
                        candidate_failures=failures,visibility_checks=checks_count)
