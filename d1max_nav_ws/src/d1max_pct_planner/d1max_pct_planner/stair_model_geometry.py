"""Explicit static stair geometry, distinct from observations and live free space.

This module is shared by offline PCT construction and source-frame route
conversion.  A continuous support envelope is a planning prior, NOT evidence
of individual treads, risers, or empty volume.  It never edits the source PCD.
"""
from __future__ import annotations

import copy
import hashlib

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .cpu_tomography import SENTINEL, _inflate, traversal_cost, round_half_away_from_zero


SCHEMA = 'd1max.parametric_stair_geometry/v1'
IDEAL_STATIC_POLICY = 'bounded_ideal_stair_structure_v1'


def _finite(value, shape, name):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError('invalid_stair_model_' + name)
    return result


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def convex_contains(polygon, points, tolerance=1e-9):
    """Closed convex polygon; no convex-hull conversion of an arbitrary input."""
    polygon, points = np.asarray(polygon, float), np.asarray(points, float)
    edge = np.roll(polygon, -1, axis=0)-polygon
    delta = points[..., None, :]-polygon
    cross = edge[:, 0]*delta[..., 1]-edge[:, 1]*delta[..., 0]
    return (cross >= -tolerance).all(axis=-1)


class ParametricStairModel:
    """Validated bounded source-frame geometry. All returned points are models."""

    def __init__(self, record):
        self.record = copy.deepcopy(record)
        if (not isinstance(record, dict) or record.get('schema') != SCHEMA
                or not _digest(record.get('source_map_sha256'))
                or not _digest(record.get('planning_map_sha256'))
                or not isinstance(record.get('frame_id'), str) or not record['frame_id']
                or record.get('model_points_are_observations') is not False):
            raise ValueError('invalid_stair_model_identity')
        policy = record.get('static_geometry_policy')
        if policy not in (None, IDEAL_STATIC_POLICY):
            raise ValueError('invalid_stair_model_static_geometry_policy')
        self.ideal_static_interior = policy == IDEAL_STATIC_POLICY
        surfaces = record.get('surfaces')
        if not isinstance(surfaces, list) or not 1 <= len(surfaces) <= 64:
            raise ValueError('invalid_stair_model_surfaces')
        self.surfaces, ids, owners = [], set(), set()
        for original in surfaces:
            surface = copy.deepcopy(original)
            sid, owner = surface.get('surface_id'), surface.get('source_surface_id')
            if (not isinstance(sid, str) or not sid or sid in ids
                    or type(owner) is not int or owner < 1 or owner in owners
                    or surface.get('kind') not in ('continuous_support_envelope', 'landing')):
                raise ValueError('invalid_stair_model_surface_identity')
            ids.add(sid); owners.add(owner)
            segments = surface.get('segment_ids')
            if (not isinstance(segments, list) or not segments
                    or any(not isinstance(s, str) or not s for s in segments)
                    or len(set(segments)) != len(segments)):
                raise ValueError('invalid_stair_model_segment_binding')
            frame = surface.get('frame', {})
            origin = _finite(frame.get('origin_xyz'), (3,), 'origin')
            basis = np.stack([_finite(frame.get(k), (3,), k) for k in ('axis_u', 'axis_v', 'axis_up')])
            # Current model contract has map-Z height and no hidden rigid TF.
            if (not np.allclose(basis@basis.T, np.eye(3), atol=1e-8, rtol=0)
                    or np.linalg.det(basis) < .99999999
                    or not np.allclose(basis[2], [0, 0, 1], atol=1e-8, rtol=0)):
                raise ValueError('invalid_stair_model_frame')
            polygon = np.asarray(surface.get('polygon_xy'), float)
            if (polygon.ndim != 2 or polygon.shape[1:] != (2,)
                    or not 3 <= len(polygon) <= 64 or not np.isfinite(polygon).all()):
                raise ValueError('invalid_stair_model_polygon')
            edges = np.roll(polygon, -1, axis=0)-polygon
            turns = np.cross(edges, np.roll(edges, -1, axis=0))
            if (np.any(np.linalg.norm(edges, axis=1) < 1e-8)
                    or np.any(turns < -1e-9) or not np.any(turns > 1e-9)):
                raise ValueError('stair_model_polygon_must_be_convex_ccw')
            values = [surface.get('support_grade'), surface.get('support_intercept_m'),
                      surface.get('contact_tolerance_m'), surface.get('ground_replacement_band_m'),
                      surface.get('static_terrain_band_m')]
            if (not np.isfinite(values).all() or not 0 <= values[0] <= 1
                    or not 0 <= values[2] <= .05 or not 0 < values[3] <= .25
                    or not 0 < values[4] <= .25
                    or (surface['kind'] == 'landing' and values[0] != 0)):
                raise ValueError('invalid_stair_model_height')
            if surface.get('static_stair_interior_assumed_clear') is not True:
                raise ValueError('stair_model_requires_explicit_static_interior_assumption')
            stations = np.asarray(surface.get('tread_stations_m', []), float)
            heights = np.asarray(surface.get('tread_heights_m', []), float)
            if surface['kind'] == 'standard_steps':
                if (stations.ndim != 1 or not 2 <= len(stations) <= 256
                        or heights.shape != (len(stations)-1,)
                        or not np.isfinite(np.r_[stations, heights]).all()
                        or np.any(np.diff(stations) <= 0) or np.any(np.diff(heights) <= 0)):
                    raise ValueError('invalid_stair_model_steps')
            elif len(stations) or len(heights):
                raise ValueError('envelope_must_not_claim_discrete_treads')
            surface.update(_origin=origin, _basis=basis, _polygon=polygon)
            self.surfaces.append(surface)
        self.frame_id = record['frame_id']
        self.schema = SCHEMA
        obstacles = record.get('obstacles', [])
        if not isinstance(obstacles, list) or len(obstacles) > 256:
            raise ValueError('invalid_stair_model_obstacles')
        self.obstacles = []; obstacle_ids=set()
        for obstacle in obstacles:
            structure_id=obstacle.get('structure_id')
            if not isinstance(structure_id,str) or not structure_id or structure_id in obstacle_ids:
                raise ValueError('invalid_stair_model_obstacle_identity')
            obstacle_ids.add(structure_id)
            pose = obstacle.get('pose', {})
            center = _finite(pose.get('position'), (3,), 'obstacle_position')
            q = _finite(pose.get('orientation_xyzw'), (4,), 'obstacle_orientation')
            half = _finite(obstacle.get('half_extents_xyz'), (3,), 'obstacle_half_extents')
            if (abs(np.linalg.norm(q)-1) > 1e-7 or np.any(half <= 0)
                    or np.max(half) > 10 or not obstacle.get('source')):
                raise ValueError('invalid_stair_model_obstacle')
            x, y, z, w = q
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            self.obstacles.append(dict(obstacle, _center=center, _rotation=rotation, _half=half))

    @classmethod
    def from_dict(cls, record):
        return cls(record)

    def local_xy(self, surface, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape[-1:] != (2,) or not np.isfinite(xy).all():
            raise ValueError('invalid_stair_model_query')
        return (xy-surface['_origin'][:2])@surface['_basis'][:2, :2].T

    def contains(self, surface, xy):
        return convex_contains(surface['_polygon'], self.local_xy(surface, xy))

    def ground_at(self, surface, xy):
        local = self.local_xy(surface, xy)
        if surface['kind'] == 'standard_steps':
            stations = np.asarray(surface['tread_stations_m'])
            index = np.clip(np.searchsorted(stations, local[..., 0], side='right')-1,
                            0, len(surface['tread_heights_m'])-1)
            return surface['_origin'][2]+np.asarray(surface['tread_heights_m'])[index]
        return surface['_origin'][2]+surface['support_intercept_m']+surface['support_grade']*local[..., 0]

    def surface_at(self, xy, *, surface_id=None, source_surface_id=None, height_hint=None):
        choices = [s for s in self.surfaces if (surface_id is None or s['surface_id'] == surface_id)
                   and (source_surface_id is None or s['source_surface_id'] == source_surface_id)
                   and bool(self.contains(s, xy))]
        if height_hint is not None:
            if not np.isfinite(height_hint):
                raise ValueError('invalid_stair_model_height_hint')
            choices = [s for s in choices if abs(float(self.ground_at(s, xy))-height_hint)
                       <= s['ground_replacement_band_m']]
        if len(choices) > 1:
            # Equal-height shared edges are one surface seam. Never use this
            # tie-break to resolve overlapping interiors or another floor.
            heights = [float(self.ground_at(s, xy)) for s in choices]
            on_edges = []
            for s in choices:
                point, polygon = self.local_xy(s, xy), s['_polygon']
                edge = np.roll(polygon, -1, axis=0)-polygon
                cross = edge[:, 0]*(point-polygon)[:, 1]-edge[:, 1]*(point-polygon)[:, 0]
                on_edges.append(np.any(abs(cross)/np.linalg.norm(edge, axis=1) <= 1e-8))
            if max(heights)-min(heights) > 1e-6 or not all(on_edges):
                raise ValueError('ambiguous_stair_model_surface')
            choices = sorted(choices, key=lambda s:s['surface_id'])[:1]
        return choices[0] if choices else None

    def obstacle_mask(self, xy, ground, *, xy_padding=0., height=0.55):
        xyz = np.c_[xy, np.asarray(ground)]
        blocked = np.zeros(len(xyz), bool)
        for box in self.obstacles:
            relative = (xyz-box['_center'])@box['_rotation']
            # Builders currently emit yaw-only structural boxes; reject tilt
            # instead of using an under-approximated prism intersection.
            if not np.allclose(box['_rotation'][:, 2], [0, 0, 1], atol=1e-8):
                raise ValueError('tilted_structural_box_not_supported_by_pct')
            half = box['_half']
            blocked |= ((abs(relative[:, 0]) <= half[0]+xy_padding)
                        & (abs(relative[:, 1]) <= half[1]+xy_padding)
                        & (relative[:, 2] <= half[2])
                        & (relative[:, 2]+height >= -half[2]))
        return blocked

    def obstacle_distance(self, xy, ground, *, height=0.55):
        """Exact XY distance to finite OBBs intersecting this height interval.

        These are model boundary distances, not raw occupancy or body-swept
        clearance. PCT applies its unchanged inflation equation once below.
        """
        xyz = np.c_[xy, np.asarray(ground)]
        distance = np.full(len(xyz), np.inf)
        for box in self.obstacles:
            if not np.allclose(box['_rotation'][:, 2], [0, 0, 1], atol=1e-8):
                raise ValueError('tilted_structural_box_not_supported_by_pct')
            local=(xyz-box['_center'])@box['_rotation'];half=box['_half']
            overlap=(local[:,2]<=half[2])&(local[:,2]+height>=-half[2])
            separation=np.maximum(abs(local[:,:2])-half[:2],0.)
            distance[overlap]=np.minimum(distance[overlap],np.linalg.norm(separation[overlap],axis=1))
        return distance


def apply_parametric_stair_geometry(payload, pct_config, model, planning_xyz):
    """Regularize actual PCT ground in finite model domains, then PCT equations.

    Known ground is associated to the same height band before replacement.
    Original near-terrain ceilings are separated from real overhead returns.
    Retained elevated entities and structural boxes block replacement. The result is a model-derived static map, never a new
    observed source point or a runtime occupancy exemption.
    """
    model = model if isinstance(model, ParametricStairModel) else ParametricStairModel(model)
    data = np.asarray(payload['data'])
    if data.dtype != np.float32 or data.ndim != 4 or data.shape[0] != 5:
        raise ValueError('parametric_stair_requires_float32_five_channel_tensor')
    if str(np.asarray(payload.get('source_sha256')).item()) != model.record['planning_map_sha256']:
        raise ValueError('parametric_stair_planning_identity_mismatch')
    points = np.asarray(planning_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or not np.isfinite(points).all():
        raise ValueError('parametric_stair_invalid_retained_cloud')
    resolution = float(payload['resolution'])
    _, layers, nx, ny = data.shape
    center = np.asarray(payload['center'], float)
    ix, iy = np.meshgrid(np.arange(nx), np.arange(ny), indexing='ij')
    xy = center+resolution*(np.c_[ix.ravel(), iy.ravel()]-[nx//2, ny//2])
    known = np.isfinite(data[3]) & (data[3] > -99)
    selected = np.zeros((layers, nx, ny), bool)
    domain = np.zeros_like(selected)
    owner = np.full(selected.shape, -1, np.int16)
    blocked_domain = np.zeros_like(selected)
    reclassified_ceiling = np.zeros_like(selected)
    structural_only = np.zeros_like(selected)
    analytic_structure_cost = np.zeros_like(data[0])
    # Explicit obstacles are separate from raw terrain gradients. Under the
    # opt-in ideal prior, a noisy/non-model terrain neighbor must not become a
    # second wall through inflation. Real retained entities and low overhead
    # remain seeds; fitted structures use their exact distance field once.
    protected_seeds = np.zeros_like(data[0]) if model.ideal_static_interior else None
    external_seed_domains = []
    result = data.copy()
    reports = []
    trav = pct_config['traversability']
    for number, surface in enumerate(model.surfaces):
        inside = model.contains(surface, xy)
        rows = np.flatnonzero(inside)
        if not len(rows):
            continue
        cellxy = xy[rows]
        z = model.ground_at(surface, cellxy)
        structural_distance=model.obstacle_distance(cellxy,z,height=float(trav['interval_min']))
        structure=structural_distance<=0.
        structure_cost=(float(trav['cost_barrier'])*np.clip(
            1.-(structural_distance-float(trav['inflation']))/
            (float(trav['safe_margin'])+resolution),0.,1.)).astype(np.float32)
        entity=np.zeros(len(cellxy),bool)
        blocked=structure.copy()
        # This domain is explicitly authorized as a static stair-interior
        # prior. Returns within its bounded terrain band do not reconstruct
        # the old noisy maximum-height footprint. Higher retained entities
        # remain exclusions. This is NOT a claim of observed empty space;
        # live SCAN occupancy is neither edited nor exempted by this stage.
        pin = model.contains(surface, points[:, :2])
        modeled_structure_rows = np.asarray(surface.get('source',{}).get('planning_record_indices',[]),dtype=np.int64)
        if (np.any(modeled_structure_rows < 0) or np.any(modeled_structure_rows >= len(points))):
            raise ValueError('parametric_stair_structure_record_out_of_range')
        # The same wall records must not be counted twice: once as the
        # explicit fitted wall and once again as its old jagged silhouette.
        # Coherent inward protrusions have their own retained-geometry boxes.
        pin[modeled_structure_rows] = False
        pz = model.ground_at(surface, points[pin, :2])
        dz = points[pin, 2]-pz
        elevated = points[pin][(dz > surface['static_terrain_band_m'])
                                & (dz <= float(trav['interval_min']))]
        if len(elevated):
            entity=cKDTree(elevated[:, :2]).query(cellxy, workers=1)[0] <= resolution*np.sqrt(.5)
            blocked |= entity
        if protected_seeds is not None:
            # Outside cells keep their original geometry, but a real nearby
            # box/low roof must still inflate INTO the modeled corridor.
            # Inspect only the existing native inflation stencil, never raw
            # terrain gradients. Defer ownership until all model surfaces
            # have been matched so adjacent modeled joins are not reimported.
            radius = int((float(trav['safe_margin'])+float(trav['inflation']))/resolution)
            halo = ndimage.binary_dilation(inside.reshape(nx, ny),
                structure=np.ones((2*radius+1, 2*radius+1), bool)).ravel() & ~inside
            halo_rows = np.flatnonzero(halo)
            if len(halo_rows):
                halo_xy = xy[halo_rows]
                margin = resolution*np.sqrt(.5)
                nearby = ((points[:, :2] >= halo_xy.min(0)-margin).all(1)
                          & (points[:, :2] <= halo_xy.max(0)+margin).all(1))
                nearby[modeled_structure_rows] = False
                elevated_z = points[nearby, 2]-model.ground_at(surface, points[nearby, :2])
                elevated_outside = points[nearby][
                    (elevated_z > surface['static_terrain_band_m'])
                    & (elevated_z <= float(trav['interval_min']))]
                halo_entity = np.zeros(len(halo_rows), bool)
                if len(elevated_outside):
                    halo_entity = cKDTree(elevated_outside[:, :2]).query(
                        halo_xy, workers=1)[0] <= margin
                external_seed_domains.append((halo_rows,
                    model.ground_at(surface, halo_xy), halo_entity,
                    surface['ground_replacement_band_m']))
        # Reconstruct only the above-terrain part of the original cell column.
        # A nearby upper tread was previously the "ceiling" in lower aliases;
        # it is not a roof. Real returns above the explicit static terrain
        # band remain the ceiling, including another floor or a low object.
        pcell = round_half_away_from_zero((points[:, :2]-center)/resolution)+[nx//2,ny//2]
        valid_point = ((pcell >= 0) & (pcell < [nx,ny])).all(axis=1)
        point_domain = ((points[:, :2] >= cellxy.min(axis=0)-resolution).all(axis=1)
                        & (points[:, :2] <= cellxy.max(axis=0)+resolution).all(axis=1))
        candidates = np.flatnonzero(valid_point & point_domain)
        relative_height = points[candidates,2]-model.ground_at(surface,points[candidates,:2])
        above = candidates[relative_height > surface['static_terrain_band_m']]
        upper = np.full(nx*ny,np.inf)
        np.minimum.at(upper,pcell[above,0]*ny+pcell[above,1],points[above,2])
        upper = upper[rows]
        selected_count = 0
        for layer in range(layers):
            original = data[3, layer].ravel()[rows]
            match = known[layer].ravel()[rows] & (abs(original-z) <= surface['ground_replacement_band_m'])
            domain[layer].ravel()[rows[match]] = True
            blocked_domain[layer].ravel()[rows[match & blocked]] = True
            structural_only[layer].ravel()[rows[match & structure & ~entity]]=True
            analytic_structure_cost[layer].ravel()[rows[match]]=np.maximum(
                analytic_structure_cost[layer].ravel()[rows[match]],structure_cost[match])
            if protected_seeds is not None:
                protected_seeds[layer].ravel()[rows[match & entity]] = float(trav['cost_barrier'])
            match &= ~blocked
            ceiling = data[4, layer].ravel()[rows].copy()
            terrain_ceiling = match & np.isfinite(ceiling) & (ceiling <= z+surface['static_terrain_band_m'])
            ceiling[terrain_ceiling] = upper[terrain_ceiling]
            if protected_seeds is not None:
                low_overhead = match & np.isfinite(ceiling) & (ceiling-z < float(trav['interval_min']))
                protected_seeds[layer].ravel()[rows[low_overhead]] = float(trav['cost_barrier'])
            match &= (~np.isfinite(ceiling) | (ceiling-z >= float(trav['interval_min'])))
            take = rows[match]
            if np.any(owner[layer].ravel()[take] >= 0):
                raise ValueError('parametric_stair_overlapping_height_domains')
            selected[layer].ravel()[take] = True
            owner[layer].ravel()[take] = number
            result[3, layer].ravel()[take] = z[match].astype(np.float32)
            change_ceiling = terrain_ceiling & match
            result[4,layer].ravel()[rows[change_ceiling]] = np.where(
                np.isfinite(ceiling[change_ceiling]),ceiling[change_ceiling],np.nan).astype(np.float32)
            reclassified_ceiling[layer].ravel()[rows[change_ceiling]] = True
            selected_count += len(take)
        reports.append(dict(surface_id=surface['surface_id'], xy_domain_cells=len(rows),
            structural_or_entity_blocked_cells=int(blocked.sum()), modified_layer_cells=selected_count))
    if protected_seeds is not None:
        for rows, support_z, entity, band in external_seed_domains:
            for layer in range(layers):
                ground_z = data[3, layer].ravel()[rows]
                ceiling_z = data[4, layer].ravel()[rows]
                same_level = (known[layer].ravel()[rows]
                              & (abs(ground_z-support_z) <= band)
                              & ~domain[layer].ravel()[rows])
                low_overhead = (np.isfinite(ceiling_z)
                                & (ceiling_z-ground_z < float(trav['interval_min'])))
                protected_seeds[layer].ravel()[rows[same_level & (entity | low_overhead)]] = float(trav['cost_barrier'])
    ground = np.where(np.isfinite(result[3]), result[3], -SENTINEL)
    ceiling = np.where(np.isfinite(result[4]), result[4], SENTINEL)
    # Never turn an untouched neighboring cell into a new obstacle merely
    # because the modeled cell next to it now has a different planning Z.
    original_ground = np.where(np.isfinite(data[3]), data[3], -SENTINEL)
    raw = traversal_cost(original_ground, ceiling, resolution, trav)
    discrete_raw = None
    # Within an explicit continuous support model, neighboring raw height
    # spikes outside its domain must not re-enter as a fictitious terrain
    # discontinuity. Derive the SAME PCT plane slope/clearance equation from
    # the model; physical structure, ceilings, entities and unknown cells were
    # excluded above. This is not a generic <=20 cost clamp.
    stand_squared = (1.2*resolution*np.tan(float(trav['slope_max_rad'])))**2
    for number, surface in enumerate(model.surfaces):
        if surface['kind'] == 'standard_steps':
            if discrete_raw is None:
                discrete_raw = traversal_cost(ground, ceiling, resolution, trav)
            take = owner == number
            raw[take] = discrete_raw[take]
            continue  # Discrete steps retain ordinary PCT neighbor equations.
        take = owner == number
        gradient_squared = (resolution*surface['support_grade'])**2
        interval = ceiling[take]-ground[take]
        if gradient_squared > stand_squared:
            # Unsupported steep model cannot be converted to low-cost terrain.
            raw[take] = float(trav['cost_barrier'])
        else:
            raw[take] = (np.maximum(0.,20*(float(trav['interval_free'])-interval))
                         +15*gradient_squared/stand_squared).astype(np.float32)
    raw[blocked_domain] = float(trav['cost_barrier'])
    # A finite model wall already supplies a continuous-distance inflation
    # field. Rasterising its footprint with an extra half-cell and inflating
    # those cells again invents an additional inward margin. Only these
    # modeled structural seeds are replaced; unknown, retained entities and
    # the unchanged original terrain outside the model still use native PCT.
    raw[structural_only]=0.
    if protected_seeds is not None:
        # Never write the analytic prior back to the input or to outside
        # cells. Only the candidate costs of selected, same-level model cells
        # use these seeds. Outside-domain output remains the original tensor.
        ideal_raw = protected_seeds
        ideal_raw[selected] = np.maximum(ideal_raw[selected], raw[selected])
    for layer in range(layers):
        seeds = ideal_raw[layer] if protected_seeds is not None else raw[layer]
        candidate = _inflate(seeds, resolution, float(trav['safe_margin']), float(trav['inflation']))
        candidate=np.maximum(candidate,analytic_structure_cost[layer])
        result[0, layer][selected[layer]] = candidate[selected[layer]]
        result[0, layer][blocked_domain[layer]] = float(trav['cost_barrier'])
    changed_cost = result[0] != data[0]
    # Native cost differences are recomputed only where a neighboring cost
    # changed. This is a one-cell derivative halo, not an enlarged free band.
    gx, gy = result[1].copy(), result[2].copy()
    gx[:, 1:-1] = result[0, :, 2:]-result[0, :, :-2]
    gy[:, :, 1:-1] = result[0, :, :, 2:]-result[0, :, :, :-2]
    halo = np.zeros_like(selected)
    for layer in range(layers):
        halo[layer] = ndimage.binary_dilation(changed_cost[layer], structure=np.array([[0,1,0],[1,1,1],[0,1,0]]))
    result[1][halo] = gx[halo]; result[2][halo] = gy[halo]
    changed_ground = ~(np.equal(result[3], data[3]) | (np.isnan(result[3]) & np.isnan(data[3])))
    return dict(data=result, arrays=dict(model_domain=domain, modeled_ground=selected,
        changed_ground=changed_ground, changed_cost=changed_cost, derivative_halo=halo,
        surface_index=owner, structural_or_entity_domain=blocked_domain,
        terrain_ceiling_reclassified=reclassified_ceiling), diagnostics=dict(schema=SCHEMA, surfaces=reports,
        changed_ground_cells=int(changed_ground.sum()), changed_cost_cells=int(changed_cost.sum()),
        ceiling_unchanged=np.array_equal(data[4], result[4], equal_nan=True),
        terrain_ceiling_reclassified_cells=int(reclassified_ceiling.sum()),
        original_points_unchanged=True, model_points_are_observations=False,
        static_geometry_policy=model.record.get('static_geometry_policy', 'legacy_source_conditioned'),
        input_data_sha256=hashlib.sha256(data.tobytes()).hexdigest(),
        output_data_sha256=hashlib.sha256(result.tobytes()).hexdigest()))
