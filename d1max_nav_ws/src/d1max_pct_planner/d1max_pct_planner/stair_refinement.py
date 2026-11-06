"""Bounded joint stair/platform route quality improvement on the real tomogram.

Only XY geometry changes. Retained-slice connectors, measured tread heights,
support, headroom and ROI stay authoritative. This produces a geometric route,
not a timed trajectory or a physical stair-execution certificate.
"""
from __future__ import annotations

import numpy as np

from .corridor_centering import (ClearanceFieldCache, center_corridor, route_clearance_quality,
                                _field_values, _admissible_mask)
from .corridor_refinement import (refine_corridor, _bezier_power, _expanded_candidate,
                                  _curve_peak_curvature)
from .path_quality import path_quality
from .tomogram_map import TomogramError, TomogramMap

MAX_INPUT_POINTS = 12_000
MAX_OUTPUT_POINTS = 20_000
MAX_SLICE_SPANS = 64
MAX_XY_LENGTH_M = 100.
# A sampling-phase epsilon for a soft comparison, not obstacle inflation.
CLEARANCE_COMPARISON_EPS_M = .001
PROOF = 'slice_polynomial_cell_boundaries_and_original_layer_connectors'


def _profile(points, roi, limits):
    low, high = (np.asarray(value, dtype=float) for value in roi)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(low >= high):
        raise TomogramError('invalid_stair_roi', 'Stair ROI must be finite increasing XYZ')
    if not np.all((points >= low-1e-6) & (points <= high+1e-6)):
        raise TomogramError('stair_outside_roi', 'Stair curve leaves the original ROI')
    difference = np.diff(points, axis=0)
    horizontal = np.linalg.norm(difference[:, :2], axis=1)
    if np.any(np.abs(difference[:, 2]) > limits['max_vertical_grade']*horizontal
              + limits['vertical_step_allowance_m']+1e-6):
        raise TomogramError('stair_vertical_jump', 'Stair curve violates the original grade/step limit')
    direction = float(np.sign(points[-1, 2]-points[0, 2]))
    signed = points[:, 2]*direction
    reverse = float(np.max(np.maximum.accumulate(signed)-signed))
    if reverse > limits['max_reverse_height_m']+1e-6:
        raise TomogramError('stair_height_reversal', 'Stair curve reverses the original height progression')
    return dict(direction='up' if direction > 0 else 'down', max_reverse_height_m=reverse)


def _small_turn_tradeoff(before, after):
    old, new = before['geometry_quality'], after['geometry_quality']
    added = float(new['total_abs_turn_rad'] or 0.)-float(old['total_abs_turn_rad'] or 0.)
    if not 0 < added <= min(np.deg2rad(10.), .15*float(old['total_abs_turn_rad'] or 0.))+1e-8:
        return None
    for key in ('curvature_p95_per_m', 'curvature_max_per_m'):
        if old[key] is None or new[key] is None or new[key] >= old[key]-1e-6:
            return None
    return dict(policy='small_extra_turn_for_lower_peak_and_p95_curvature_with_preserved_clearance',
                turn_before_rad=old['total_abs_turn_rad'], turn_after_rad=new['total_abs_turn_rad'],
                added_turn_rad=added, maximum_added_rad=min(np.deg2rad(10.), .15*old['total_abs_turn_rad']),
                curvature_p95_before=old['curvature_p95_per_m'], curvature_p95_after=new['curvature_p95_per_m'],
                curvature_peak_before=old['curvature_max_per_m'], curvature_peak_after=new['curvature_max_per_m'],
                physical_safety_certified=False)


def _quality_admissible(before, after, *, allow_small_turn_tradeoff=False):
    if not before.get('available') or not after.get('available'):
        return False
    if after['continuous_clearance_min_m'] < before['continuous_clearance_min_m']-CLEARANCE_COMPARISON_EPS_M:
        return False
    if after['xy_length_m'] > before['xy_length_m'] + min(.2, .05*before['xy_length_m']):
        return False
    tradeoff = _small_turn_tradeoff(before, after) if allow_small_turn_tradeoff else None
    for key in ('total_abs_turn_rad', 'curvature_p95_per_m', 'curvature_max_per_m'):
        old, new = before['geometry_quality'][key], after['geometry_quality'][key]
        if old is not None and new is not None and new > old+1e-6*max(1., abs(old)):
            if key == 'total_abs_turn_rad' and tradeoff is not None:
                continue
            return False
    return after['score'] < before['score']-1e-6


def _clearance_quality(tomogram, points, layers, cache):
    """Exact bilinear-field minimum along each published polyline chord.

    The regular arc-length samples remain the soft score, not the minimum:
    shortening an earlier span must not change an untouched span's clearance
    verdict merely by shifting the later sampling phase.
    """
    result = route_clearance_quality(tomogram, points, layers, cache=cache)
    if not result.get('available'):
        return result
    minimum, total_intervals = float('inf'), 0
    points, layers = np.asarray(points), np.asarray(layers)
    for layer in np.unique(layers):
        field = cache.get(int(layer))
        selected = np.flatnonzero((layers[:-1] == layer) | (layers[1:] == layer))
        first, last = [], []
        for index in selected:
            a, b = points[index,:2], points[index+1,:2]
            ia = (a-tomogram.center)/tomogram.resolution+tomogram.offset
            ib = (b-tomogram.center)/tomogram.resolution+tomogram.offset
            times = {0., 1.}
            for axis in (0,1):
                delta = ib[axis]-ia[axis]
                if abs(delta) <= 1e-12:
                    continue
                low, high = sorted((ia[axis], ib[axis]))
                for boundary in range(int(np.ceil(low)), int(np.floor(high))+1):
                    value = (boundary-ia[axis])/delta
                    if 0 < value < 1:
                        times.add(float(value))
            values = np.asarray(sorted(times))
            xy = a+(b-a)*values[:,None]
            first.extend(xy[:-1]); last.extend(xy[1:])
        total_intervals += len(first)
        if total_intervals > MAX_OUTPUT_POINTS*8:
            return dict(result, available=False, reason='continuous_clearance_work_limit')
        a, b = np.asarray(first), np.asarray(last)
        if not len(a):
            continue
        fa, fb, fm = (_field_values(tomogram, field, xy) for xy in (a,b,(a+b)/2))
        # On one interpolation cell, bilinear interpolation along a line is
        # quadratic in normalized chord distance: c0+c1*t+c2*t^2.
        c2 = 2*(fa+fb-2*fm)
        c1 = fb-fa-c2
        critical = np.divide(-c1, 2*c2, out=np.zeros_like(c1), where=np.abs(c2)>1e-12)
        internal = (c2>0)&(critical>0)&(critical<1)
        value = np.minimum(fa,fb)
        value[internal] = np.minimum(value[internal],
            fa[internal]+c1[internal]*critical[internal]+c2[internal]*critical[internal]**2)
        minimum = min(minimum, float(value.min()))
    result['continuous_clearance_min_m'] = minimum
    result['continuous_clearance_semantics'] = 'bilinear_slice_distance_field_minimum_along_published_polyline'
    return result


def _refine_stair_spans(tomogram, path_xyz, layer_ids, *, landing_index, stair_roi,
                        limits, corner_cut_m=1.5):
    """Return checked joint stairs, or the unchanged already-valid route.

    The two original flights are supplied joined at ``landing_index``. Runs of
    the *same retained slice* are optimized together, including the platform
    run spanning that join. Cross-slice endpoint pairs never move. A relocated
    landing remains in the same measured platform slice and has a checked
    straight connector to the requested landing; it is reported explicitly.
    """
    points, layers = np.asarray(path_xyz, dtype=float), np.asarray(layer_ids)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
            or layers.shape != (len(points),) or not np.isfinite(points).all()):
        raise TomogramError('invalid_stair_input', 'Expected finite joined stair XYZ and retained layers')
    layers = np.array([tomogram.layer(value) for value in layers], dtype=int)
    if (isinstance(landing_index, (bool, np.bool_)) or int(landing_index) != landing_index
            or not 0 < int(landing_index) < len(points)-1):
        raise TomogramError('invalid_stair_landing', 'Landing must be an interior original route vertex')
    landing_index = int(landing_index)
    quality = path_quality(points)
    base = dict(applied=False, reason='no_checked_quality_improvement', path=points.tolist(),
                layer_ids=layers.tolist(), landing_index=landing_index,
                requested_landing_xyz=points[landing_index].tolist(),
                effective_landing_xyz=points[landing_index].tolist(), landing_displacement_m=0.,
                before_quality=quality, after_quality=quality, span_records=[],
                physical_safety_certified=False, algorithm='joint_measured_stair_slice_refinement',
                continuity='XY piecewise smooth; original slice connectors, no global C2/dynamic claim')
    if len(points) > MAX_INPUT_POINTS or quality['xy_length_m'] > MAX_XY_LENGTH_M:
        return dict(base, reason='stair_refinement_work_limit')
    if (not np.isfinite(corner_cut_m) or isinstance(corner_cut_m, (bool, np.bool_))
            or not 0 < corner_cut_m <= 5):
        raise ValueError('corner_cut_m must be finite in (0, 5]')
    tomogram.validate_path(points, layers, max_ground_step_m=limits['max_ground_step_m'])
    _profile(points, stair_roi, limits)
    boundaries = np.r_[0, np.flatnonzero(np.diff(layers))+1, len(points)]
    if len(boundaries)-1 > MAX_SLICE_SPANS:
        return dict(base, reason='stair_refinement_slice_span_limit')
    cache = ClearanceFieldCache(tomogram, max_layers=min(16, len(set(layers))))
    before = _clearance_quality(tomogram, points, layers, cache)
    base.update(before_clearance=before, after_clearance=before)
    if not before['available']:
        return dict(base, reason='stair_quality_unavailable')
    output, identities, records = [], [], []
    landing_range = None
    partition_count = 0
    for first, last in zip(boundaries[:-1], boundaries[1:]):
        source, source_ids = points[first:last], layers[first:last]
        chosen, chosen_ids = source, source_ids
        selected_proof, selected_partition = 'original_native_geometry', len(source)
        record = dict(first_index=int(first), last_index=int(last-1), layer_id=int(source_ids[0]),
                      selected='original', candidates=[])
        if len(source) > 1 and np.linalg.norm(np.diff(source[:, :2], axis=0), axis=1).sum() > 1e-6:
            original_quality = _clearance_quality(tomogram, source, source_ids, cache)
            record['original_quality'] = original_quality
            candidates = []
            try:
                smooth = refine_corridor(tomogram, source, source_ids, corner_cut_m,
                                         quality_policy='stair_xy')
                record['candidates'].append(dict(kind='visibility', applied=smooth['applied'], reason=smooth['reason']))
                if smooth['applied']:
                    candidates.append(('visibility', smooth))
                # Center both native and simplified candidates without changing
                # ground. Three bounded cubic alternatives per invocation.
                for label, seed in [('original', dict(path=source, layer_ids=source_ids)), *list(candidates)]:
                    centered = center_corridor(tomogram, seed['path'], seed['layer_ids'], cache=cache,
                                               allow_graded_support=True)
                    record['candidates'].append(dict(kind=label+'_centered', applied=centered['applied'],
                                                     reason=centered['reason']))
                    if centered['applied']:
                        candidates.append((label+'_centered', centered))
                eligible = []
                for label, item in candidates:
                    candidate = np.asarray(item['path'], dtype=float)
                    candidate_ids = np.asarray(item['layer_ids'], dtype=int)
                    if not np.array_equal(candidate[[0,-1]], source[[0,-1]]):
                        continue
                    _profile(candidate, stair_roi, limits)
                    score = _clearance_quality(tomogram, candidate, candidate_ids, cache)
                    accepted = _quality_admissible(original_quality, score, allow_small_turn_tradeoff=True)
                    record['candidates'].append(dict(kind=label, quality_accepted=accepted,
                        clearance=score.get('clearance'), score=score.get('score'),
                        continuous_clearance_min_m=score.get('continuous_clearance_min_m'),
                        turn_tradeoff=(_small_turn_tradeoff(original_quality,score)
                                       if accepted else None)))
                    if accepted:
                        eligible.append((score['score'], label, item, candidate, candidate_ids))
                if eligible:
                    _, label, item, chosen, chosen_ids = min(eligible, key=lambda item: item[:2])
                    record['selected'] = label
                    selected_proof = item['curve_validation']
                    selected_partition = item['curve_partition_points']
            except (ValueError, TypeError, FloatingPointError) as error:
                # Only optional quality work failed. The original valid span
                # and all following spans remain available, with its reason.
                record['quality_error'] = dict(code=getattr(error, 'code', type(error).__name__), message=str(error))
        if len(output)+len(chosen) > MAX_OUTPUT_POINTS:
            return dict(base, reason='stair_refinement_output_limit', span_records=records)
        start_index = len(output)
        output.extend(chosen); identities.extend(chosen_ids)
        if first <= landing_index < last:
            landing_range = (start_index, len(output))
        record.update(output_first_index=start_index, output_last_index=len(output)-1,
                      curve_validation=selected_proof, curve_partition_points=int(selected_partition))
        records.append(record)
        partition_count += int(selected_partition)
    result, ids = np.asarray(output), np.asarray(identities, dtype=int)
    if all(record['selected'] == 'original' for record in records):
        return dict(base, span_records=records)
    try:
        if not np.array_equal(result[[0,-1]], points[[0,-1]]):
            raise TomogramError('stair_endpoint_mismatch', 'Optional refinement moved a stair endpoint')
        checks = tomogram.validate_path(result, ids, max_ground_step_m=limits['max_ground_step_m'])
        profile = _profile(result, stair_roi, limits)
        after = _clearance_quality(tomogram, result, ids, cache)
        if not _quality_admissible(before, after):
            return dict(base, reason='joined_stair_quality_not_improved', span_records=records,
                        candidate_quality=path_quality(result), candidate_clearance=after)
        requested = points[landing_index]
        candidates = np.arange(*landing_range)
        order = candidates[np.argsort(np.linalg.norm(result[candidates]-requested, axis=1), kind='stable')]
        effective_index = None
        for index in order:
            displacement = float(np.linalg.norm(result[index]-requested))
            if (displacement > corner_cut_m+1e-6
                    or abs(result[index,2]-requested[2]) > limits.get('max_selection_height_error_m', .08)+1e-6
                    or ids[index] != layers[landing_index] or not 0 < index < len(result)-1):
                continue
            try:
                tomogram.validate_path([requested, result[index]], [layers[landing_index], ids[index]],
                                       max_ground_step_m=limits['max_ground_step_m'])
                effective_index = int(index)
                break
            except TomogramError:
                continue
        if effective_index is None:
            return dict(base, reason='no_checked_same_platform_split', span_records=records)
        return dict(base, applied=True, reason='checked_joint_stair_quality_improvement',
                    path=result.tolist(), layer_ids=ids.tolist(), landing_index=effective_index,
                    effective_landing_xyz=result[effective_index].tolist(),
                    landing_displacement_m=float(np.linalg.norm(result[effective_index]-requested)),
                    platform_evidence='same_retained_slice_checked_original_surface_connector',
                    after_quality=path_quality(result), after_clearance=after, span_records=records,
                    curve_validation=PROOF, curve_partition_points=partition_count,
                    partition_semantics='accepted_polynomial_partitions_plus_unchanged_native_span_vertices',
                    checks=checks, stair_profile=profile)
    except (ValueError, TypeError, FloatingPointError) as error:
        return dict(base, reason='joint_stair_recheck_failed', span_records=records,
                    error_code=getattr(error, 'code', type(error).__name__), error=str(error))


def _refine_shared_platform(tomogram, result, *, stair_roi, limits, corner_cut_m):
    """Round one landing across aliases of the *same measured* surface.

    A retained-slice identifier is not a physical platform boundary. Only
    neighbouring aliases with numerically equal ground at the landing qualify.
    The candidate uses their admissible-cell intersection, with equal measured
    ground in every cell, limited to the existing corner-cut neighbourhood.
    This view only removes cells; no cost, support, height or threshold in the
    input tomogram is changed. Whole-route acceptance uses the original map.
    """
    points, ids = np.asarray(result['path'], dtype=float), np.asarray(result['layer_ids'], dtype=int)
    landing = int(result['landing_index'])
    requested = np.asarray(result['requested_landing_xyz'], dtype=float)
    canonical = int(ids[landing])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
    nearby = np.abs(arc-arc[landing]) <= corner_cut_m+1e-9
    aliases = [canonical]
    for layer in sorted(set(ids[nearby])-{canonical}):
        if abs(int(layer)-canonical) != 1:
            continue
        try:
            left = tomogram.surface_ground_z(points[landing,:2],canonical)
            right = tomogram.surface_ground_z(points[landing,:2],int(layer))
        except TomogramError:
            continue
        # Equality of retained observations, not the selectable-height or
        # stair-step allowance. Different treads/floors must never be merged.
        if abs(left-right) <= 1e-6:
            aliases.append(int(layer))
    if len(aliases) < 2:
        return dict(result, platform_alias_refinement=dict(applied=False,reason='no_shared_landing_alias'))
    aliases.sort()
    shared = _admissible_mask(tomogram,canonical).copy()
    for layer in aliases:
        shared &= (_admissible_mask(tomogram,layer)
                   & (np.abs(tomogram.ground[layer]-tomogram.ground[canonical]) <= 1e-6))
    x = tomogram.center[0]+(np.arange(tomogram.nx)-tomogram.offset[0])*tomogram.resolution
    y = tomogram.center[1]+(np.arange(tomogram.ny)-tomogram.offset[1])*tomogram.resolution
    shared &= ((np.abs(x[:,None]-points[landing,0]) <= corner_cut_m+1e-9)
               & (np.abs(y[None,:]-points[landing,1]) <= corner_cut_m+1e-9))

    def supported(index):
        return (nearby[index] and int(ids[index]) in aliases
                and all(tomogram.contains(cell) and shared[cell]
                        for cell in tomogram.point_cells(points[index,:2])))

    first = last = landing
    while first > 0 and supported(first-1):
        first -= 1
    while last+1 < len(points) and supported(last+1):
        last += 1
    report = dict(applied=False,alias_layers=aliases,canonical_layer=canonical,
                  input_first_index=first,input_last_index=last,
                  reason='no_checked_shared_platform_improvement',candidates=[])
    if not supported(landing) or last-first < 2 or len(set(ids[first:last+1])) < 2:
        return dict(result,platform_alias_refinement=report)
    # A one-layer *restriction* of the existing PCT surface is sufficient for
    # its existing checked polynomial refiner; the native search is not rerun.
    data = tomogram.data[:,canonical:canonical+1].copy()
    data[0,0,~shared] = 50.
    view = TomogramMap(dict(data=data,resolution=tomogram.resolution,center=tomogram.center,
        slice_h0=tomogram.slice_h0,slice_dh=tomogram.slice_dh,
        selected_source_layers=[tomogram.source_layers[canonical]],
        minimum_headroom_m=tomogram.minimum_headroom_m,
        open_sky_verified=tomogram.open_sky_verified[canonical:canonical+1]),
        unknown_ceiling_policy=tomogram.unknown_ceiling_policy,
        max_ground_step_m=tomogram.max_ground_step_m)
    source = points[first:last+1]
    labels = np.zeros(len(source),dtype=int)
    try:
        view.validate_path(source,labels,max_ground_step_m=limits['max_ground_step_m'])
        smooth = refine_corridor(view,source,labels,corner_cut_m,quality_policy='stair_xy')
        candidates = [('visibility',smooth)] if smooth['applied'] else []
        for name,item in [('original',dict(path=source,layer_ids=labels)),*list(candidates)]:
            centered = center_corridor(view,item['path'],item['layer_ids'],allow_graded_support=True)
            if centered['applied']:
                candidates.append((name+'_centered',centered))
        # A shortened platform must not create a new heading corner where it
        # rejoins the untouched flight. Try bounded C2 Bezier connectors with
        # the actual neighbouring polyline tangents and zero end curvature.
        incoming = points[first,:2]-points[max(0,first-1),:2]
        outgoing = points[min(len(points)-1,last+1),:2]-points[last,:2]
        if min(np.linalg.norm(incoming),np.linalg.norm(outgoing)) > 1e-8:
            incoming /= np.linalg.norm(incoming)
            outgoing /= np.linalg.norm(outgoing)
            for scale in (.5,1.,1.5):
                distance = min(corner_cut_m,float(arc[last]-arc[first])*scale/5.)
                a,b = source[0,:2],source[-1,:2]
                control = np.asarray([a,a+incoming*distance,a+2*incoming*distance,
                                      b-2*outgoing*distance,b-outgoing*distance,b])
                segment = dict(kind='quintic_bezier',coefficients_xy=_bezier_power(control),
                    derivative_bound_m=float(5*np.max(np.linalg.norm(np.diff(control,axis=0),axis=1))))
                if not np.isfinite(_curve_peak_curvature([segment])):
                    continue
                try:
                    local,count,_ = _expanded_candidate(view,[segment],0,source[0],source[-1])
                except TomogramError:
                    continue
                candidates.append(('matched_tangents_'+str(scale),dict(path=local.tolist(),
                    curve_partition_points=count,curve_validation=PROOF)))
        cache = ClearanceFieldCache(tomogram,max_layers=min(16,len(set(ids))))
        before = _clearance_quality(tomogram,points,ids,cache)
        accepted = []
        for name,item in candidates:
            local = np.asarray(item['path'],dtype=float)
            if not np.array_equal(local[[0,-1]],source[[0,-1]]):
                continue
            view.validate_path(local,np.zeros(len(local),dtype=int))
            local_ids = np.full(len(local),canonical,dtype=int)
            local_ids[0],local_ids[-1] = ids[first],ids[last]
            candidate = np.concatenate((points[:first],local,points[last+1:]))
            candidate_ids = np.concatenate((ids[:first],local_ids,ids[last+1:]))
            if len(candidate) > MAX_OUTPUT_POINTS:
                continue
            checks = tomogram.validate_path(candidate,candidate_ids,max_ground_step_m=limits['max_ground_step_m'])
            profile = _profile(candidate,stair_roi,limits)
            quality = _clearance_quality(tomogram,candidate,candidate_ids,cache)
            admissible = _quality_admissible(before,quality)
            report['candidates'].append(dict(name=name,quality_accepted=admissible,
                quality=quality['geometry_quality'],score=quality['score'],
                continuous_clearance_min_m=quality['continuous_clearance_min_m']))
            if not admissible:
                continue
            positions = np.arange(first,first+len(local))
            positions = positions[candidate_ids[positions] == canonical]
            positions = positions[np.argsort(np.linalg.norm(candidate[positions]-requested,axis=1),kind='stable')]
            split = None
            for index in positions:
                if (not 0 < index < len(candidate)-1
                        or np.linalg.norm(candidate[index]-requested) > corner_cut_m+1e-6
                        or abs(candidate[index,2]-requested[2]) > limits.get('max_selection_height_error_m',.08)+1e-6):
                    continue
                try:
                    tomogram.validate_path([requested,candidate[index]],[canonical,canonical],
                                           max_ground_step_m=limits['max_ground_step_m'])
                except TomogramError:
                    continue
                split = int(index)
                break
            if split is not None:
                accepted.append((quality['score'],name,item,candidate,candidate_ids,split,checks,profile,quality))
        if not accepted:
            return dict(result,platform_alias_refinement=report)
        _,name,item,candidate,candidate_ids,split,checks,profile,quality = min(accepted,key=lambda value:value[:2])
        report.update(applied=True,reason='checked_same_surface_alias_rounding',selected=name,
                      output_first_index=first,output_last_index=first+len(item['path'])-1,
                      before_quality=path_quality(points),after_quality=path_quality(candidate),
                      support_contract='intersection_of_original_alias_cells_equal_measured_ground_and_original_clearance')
        return dict(result,applied=True,reason='checked_joint_stair_quality_improvement',
            path=candidate.tolist(),layer_ids=candidate_ids.tolist(),landing_index=split,
            effective_landing_xyz=candidate[split].tolist(),
            landing_displacement_m=float(np.linalg.norm(candidate[split]-requested)),
            after_quality=path_quality(candidate),after_clearance=quality,
            curve_validation=PROOF,
            curve_partition_points=int(result.get('curve_partition_points',len(points)))+int(item['curve_partition_points']),
            partition_semantics='original_span_evidence_plus_checked_shared_platform_polynomial_partitions',
            span_records_scope='before_shared_platform_alias_refinement',
            checks=checks,stair_profile=profile,platform_alias_refinement=report,
            platform_evidence='same_measured_surface_in_admissible_retained_alias_intersection')
    except (ValueError,TypeError,FloatingPointError) as error:
        return dict(result,platform_alias_refinement=dict(report,
            reason='shared_platform_optional_recheck_failed',
            error_code=getattr(error,'code',type(error).__name__),error=str(error)))


def refine_stair_route(tomogram,path_xyz,layer_ids,*,landing_index,stair_roi,
                       limits,corner_cut_m=1.5):
    """Refine real stair spans, then a bounded physically shared landing."""
    result = _refine_stair_spans(tomogram,path_xyz,layer_ids,landing_index=landing_index,
                               stair_roi=stair_roi,limits=limits,corner_cut_m=corner_cut_m)
    if result['reason'] in ('stair_refinement_work_limit','stair_refinement_slice_span_limit',
                           'stair_refinement_output_limit'):
        return result
    return _refine_shared_platform(tomogram,result,stair_roi=stair_roi,
                                   limits=limits,corner_cut_m=corner_cut_m)
