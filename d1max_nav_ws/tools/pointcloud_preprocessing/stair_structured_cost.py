"""Explicit offline PCT terrain prior; retained structures stay native seeds.

This changes planning costs only. It cannot write PCD, claim new observations,
modify SCAN inputs, or authorize motion. All ground/ceiling samples are kept.
"""
from __future__ import annotations

import hashlib

import numpy as np
from scipy.spatial import cKDTree

from d1max_pct_planner.cpu_tomography import traversal_cost, _inflate, SENTINEL


def apply_structured_stair_cost(data, *, center, resolution, traversability,
        model, support_arrays, original_source_xyz, planning_xyz,
        input_tomogram_sha256, planning_sha256):
    """Apply measured-model slope cost only to source-covered terrain cells.

    The original native raw cost and inflated tensor must exactly agree.
    A cell is eligible only inside both measured side boundaries, on the same
    observed flight height, with literal normal-qualified support <=12 cm,
    original measured headroom, and no retained elevated entity nearby. Other
    raw seeds, particularly walls/rails/unknown/low headroom, are unchanged.
    Original native inflation is then recomputed; no generic cost clamp is
    used and all noneligible output cells retain their original cost.
    """
    a = np.asarray(data)
    origin, cloud = np.asarray(original_source_xyz, float), np.asarray(planning_xyz, float)
    center = np.asarray(center, float)
    if (a.ndim != 4 or a.shape[0] != 5 or a.dtype != np.float32 or center.shape != (2,)
            or not np.isfinite(center).all() or not np.isfinite(resolution) or resolution <= 0
            or origin.ndim != 2 or origin.shape[1:] != (3,) or not np.isfinite(origin).all()
            or cloud.ndim != 2 or cloud.shape[1:] != (3,) or not np.isfinite(cloud).all()):
        raise ValueError('structured_cost_input_geometry_invalid')
    digest = lambda v: isinstance(v, str) and len(v) == 64 and all(c in '0123456789abcdef' for c in v)
    if (model.get('schema') != 'd1max.structured_stair_support.v1'
            or model.get('model_samples_are_observations') is not False
            or model.get('source_records_are_literal') is not True
            or not all(digest(v) for v in (input_tomogram_sha256, planning_sha256, model.get('source_sha256')))):
        raise ValueError('structured_cost_model_identity_invalid')
    ids = np.asarray(support_arrays['source_indices'])
    xyz = np.asarray(support_arrays['source_xyz'])
    owner = np.asarray(support_arrays['surface_id'])
    normals = np.asarray(support_arrays['source_normal_z'])
    rms = np.asarray(support_arrays['source_plane_rms_m'])
    if (ids.ndim != 1 or ids.dtype != np.int64 or np.any(ids < 0) or np.any(ids >= len(origin))
            or len(np.unique(ids)) != len(ids) or xyz.shape != (len(ids), 3)
            or not np.array_equal(xyz, origin[ids]) or owner.shape != ids.shape
            or owner.dtype != np.int32 or normals.shape != ids.shape or rms.shape != ids.shape
            or not np.isfinite(normals).all() or not np.isfinite(rms).all()
            or np.any(normals < .70) or np.any(normals > 1.) or np.any(rms < 0) or np.any(rms > .035)):
        raise ValueError('structured_cost_literal_support_invalid')
    policy = model['policy']
    radius = float(policy['source_support_radius_m'])
    residual_limit = float(policy['maximum_support_residual_m'])
    if (not 0 < radius <= .12 or not 0 < residual_limit <= .08
            or policy.get('retained_entity_height_m') != residual_limit
            or not np.isfinite(policy.get('retained_entity_top_m', np.nan))
            or policy['retained_entity_top_m'] < traversability['interval_free']
            or not np.isfinite(policy.get('retained_entity_radius_m', np.nan))
            or policy['retained_entity_radius_m'] < .05):
        raise ValueError('structured_cost_support_bounds_invalid')
    ground = np.nan_to_num(a[3], nan=-float(SENTINEL))
    ceiling = np.nan_to_num(a[4], nan=float(SENTINEL))
    raw = traversal_cost(ground, ceiling, resolution, traversability)
    native_cost = np.stack([_inflate(row, resolution, traversability['safe_margin'],
                                     traversability['inflation']) for row in raw])
    if not np.array_equal(native_cost, a[0]):
        raise ValueError('structured_cost_requires_original_native_tensor')
    gx, gy = np.zeros_like(a[0]), np.zeros_like(a[0])
    gx[:, 1:-1] = a[0, :, 2:]-a[0, :, :-2]
    gy[:, :, 1:-1] = a[0, :, :, 2:]-a[0, :, :, :-2]
    if not np.array_equal(gx, a[1]) or not np.array_equal(gy, a[2]):
        raise ValueError('structured_cost_native_gradient_mismatch')
    selected = np.zeros(raw.shape, bool)
    source_witness = np.full(raw.shape, -1, np.int64)
    qualified_source = np.zeros(len(ids), bool)
    prior = raw.copy()
    nx, ny = a.shape[2:]
    offset = np.array([nx//2, ny//2])
    reports = []
    owned_cells = np.zeros(raw.shape, np.int32)
    for flight in model['flights']:
        fid = int(flight['surface_id'])
        fc = np.asarray(flight['frame']['center_xy'], float)
        axis = np.asarray(flight['frame']['uphill_axis_xy'], float)
        lateral = np.array([-axis[1], axis[0]])
        begin, end = flight['frame']['along_range_m']
        grade, intercept = flight['height_model']['grade'], flight['height_model']['intercept_m']
        sides = flight['retained_structure']['sides']
        negative = np.asarray(sides['negative']['lateral_slope_intercept'], float)
        positive = np.asarray(sides['positive']['lateral_slope_intercept'], float)
        guard = flight['wall_boundary_guard_m']
        if (axis.shape != (2,) or fc.shape != (2,) or negative.shape != (2,) or positive.shape != (2,)
                or not np.isfinite(np.r_[fc, axis, negative, positive, begin, end, grade, intercept, guard]).all()
                or abs(np.linalg.norm(axis)-1.) > 1e-6 or not .15 <= grade <= 1. or begin >= end or guard < .06):
            raise ValueError('structured_cost_flight_bounds_invalid')
        corners = np.array([fc+s*axis+t*lateral for s in (begin, end)
            for t in (np.polyval(negative,s)+guard, np.polyval(positive,s)-guard)])
        lo = np.maximum(0, np.floor((corners.min(axis=0)-center)/resolution).astype(int)+offset-1)
        hi = np.minimum([nx-1, ny-1], np.ceil((corners.max(axis=0)-center)/resolution).astype(int)+offset+1)
        ix, iy = np.meshgrid(np.arange(lo[0],hi[0]+1), np.arange(lo[1],hi[1]+1), indexing='ij')
        xy = center+(np.stack((ix,iy),axis=-1)-offset)*resolution
        s, t = (xy-fc)@axis, (xy-fc)@lateral
        expected = grade*s+intercept
        domain = (s >= begin)&(s <= end)&(t > np.polyval(negative,s)+guard)&(t < np.polyval(positive,s)-guard)
        cells = np.c_[ix[domain], iy[domain]]
        expected_xyz = np.c_[xy[domain], expected[domain]]
        support = owner == fid
        if not support.any():
            raise ValueError('structured_cost_flight_source_support_missing')
        # All retained entities in the corridor are respected, not just normal-
        # classified side-wall records. Thin boxes/rails need no semantic label.
        ps, pt = (cloud[:,:2]-fc)@axis, (cloud[:,:2]-fc)@lateral
        dz = cloud[:,2]-(grade*ps+intercept)
        entity = (ps >= begin)&(ps <= end)&(pt >= np.polyval(negative,ps))&(pt <= np.polyval(positive,ps))
        entity &= (dz > policy['retained_entity_height_m'])&(dz <= policy['retained_entity_top_m'])
        entity_tree = cKDTree(cloud[entity,:2]) if entity.any() else None
        entity_distance = (entity_tree.query(xy[domain], workers=1)[0]
                           if entity_tree else np.full(len(cells),np.inf))
        # Half-diagonal covers the entire PCT cell, not just its center.
        entity_radius = max(float(policy['retained_entity_radius_m']), resolution/np.sqrt(2))
        support_positions = np.flatnonzero(support)
        source_clear = (entity_tree.query(xyz[support,:2],workers=1)[0] > entity_radius
                        if entity_tree else np.ones(int(support.sum()),bool))
        support_positions = support_positions[source_clear]
        if not len(support_positions):
            raise ValueError('structured_cost_no_entity_clear_literal_support')
        tree = cKDTree(xyz[support_positions])
        model_distance, _ = tree.query(expected_xyz, workers=1)
        qualified_source[support_positions] = True
        physical = (model_distance <= radius)&(entity_distance > entity_radius)
        count = 0
        for layer in range(len(raw)):
            ii, jj = cells.T
            z = a[3,layer,ii,jj]
            source_query = np.c_[xy[domain], z]
            finite = np.isfinite(z)
            distance = np.full(len(z), np.inf)
            nearest = np.zeros(len(z), int)
            if finite.any(): distance[finite], nearest[finite] = tree.query(source_query[finite], workers=1)
            room = a[4,layer,ii,jj]-z
            valid = physical & finite & (abs(z-expected[domain]) <= residual_limit)
            valid &= (distance <= radius)&np.isfinite(room)&(room >= traversability['interval_min'])
            if np.any(valid & (owned_cells[layer,ii,jj] != 0)):
                raise ValueError('structured_cost_cross_flight_alias')
            use_i, use_j = ii[valid], jj[valid]
            selected[layer,use_i,use_j] = True
            owned_cells[layer,use_i,use_j] = fid
            source_witness[layer,use_i,use_j] = ids[support_positions][nearest[valid]]
            # Exact slope terms of the native PCT equation on this fitted plane.
            dx2, dy2 = (grade*axis*resolution)**2
            flat_limit = (1.2*resolution*np.tan(traversability['slope_max_rad']))**2
            if dx2+dy2 <= flat_limit:
                terrain_cost = 15*(dx2+dy2)/flat_limit
            else:
                # A uniform over-limit plane has no flat neighbors. Calling
                # it crossable would skip the native standability equation.
                raise ValueError('structured_cost_model_exceeds_native_slope')
            native_model = np.maximum(0., 20*(traversability['interval_free']-room[valid]))+terrain_cost
            prior[layer,use_i,use_j] = np.minimum(raw[layer,use_i,use_j], native_model.astype(np.float32))
            count += int(valid.sum())
        reports.append(dict(surface_id=fid, known_source_and_entity_protected_layer_cells=count,
            physical_xy_domain_cells=len(cells), terrain_model_cost=float(terrain_cost) if count else None,
            retained_entity_records=int(entity.sum()), entity_clear_literal_sources=len(support_positions), source_radius_m=radius,
            original_support_residual_m=residual_limit, entity_radius_m=entity_radius))
    candidate_cost = np.stack([_inflate(row,resolution,traversability['safe_margin'],traversability['inflation']) for row in prior])
    output = a.copy()
    output[0][selected] = candidate_cost[selected]
    output[1] = 0; output[2] = 0
    output[1,:,1:-1,:] = output[0,:,2:,:]-output[0,:,:-2,:]
    output[2,:,:,1:-1] = output[0,:,:,2:]-output[0,:,:,:-2]
    changed = output[0] != a[0]
    assert np.array_equal(output[3:].view(np.uint32),a[3:].view(np.uint32))
    assert not np.any(output[0] > a[0])
    channels = [dict(channel=name,
        input_sha256=hashlib.sha256(np.ascontiguousarray(a[k]).tobytes()).hexdigest(),
        output_sha256=hashlib.sha256(np.ascontiguousarray(output[k]).tobytes()).hexdigest(),
        changed_layer_cells=int((a[k].view(np.uint32)!=output[k].view(np.uint32)).sum()))
        for k,name in enumerate(('cost','gradient_x','gradient_y','ground','ceiling'))]
    return dict(data=output, arrays=dict(eligible_terrain=selected, modified_raw_seed=prior != raw,
        changed_cost=changed, literal_source_witness=source_witness,
        qualified_source_indices=ids[qualified_source].copy(), qualified_source_xyz=xyz[qualified_source].copy(),
        qualified_source_surface_id=owner[qualified_source].copy()), diagnostics=dict(
        schema='d1max.structured_stair_cost.v1', input_tomogram_sha256=input_tomogram_sha256,
        current_planning_sha256=planning_sha256, original_source_sha256=model['source_sha256'],
        model_input_planning_sha256=model['planning_sha256'],
        flights=reports, changed_cost_layer_cells=int(changed.sum()),
        channels=channels, tensor_shape=list(a.shape), tensor_dtype=str(a.dtype),
        changed_native_raw_layer_cells=int((prior != raw).sum()),
        restored_layer_cells=int(((a[0]>20)&(output[0]<=20)).sum()),
        raw_pcd_unchanged=True, original_ground_ceiling_unchanged=True,
        model_is_explicit_planning_prior_not_observation=True,
        native_inflation_unchanged=True, live_scan_inputs_unchanged=True,
        policy='native_model_slope_cost_on_source_witnessed_clear_terrain_then_native_inflation'))
