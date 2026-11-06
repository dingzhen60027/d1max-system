"""Build an explicit stair planning model from measured sides and support.

No synthetic point is inserted into either PCD.  Fitted envelopes are labelled
as such; a configured robot step limit is never used as a measured tread rise.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from .pcd_io import read_pcd


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _surface_frame(center, axis):
    return dict(origin_xyz=[*map(float, center), 0.], axis_u=[*map(float, axis), 0.],
                axis_v=[-float(axis[1]), float(axis[0]), 0.], axis_up=[0., 0., 1.])


def _clip_polygon(polygon, a, b, *, inside=True):
    """Clip a convex polygon by one oriented edge; original vertices only."""
    polygon=np.asarray(polygon,float);result=[]
    if not len(polygon):return polygon.reshape(0,2)
    sign=1. if inside else -1.;edge=b-a
    value=lambda p:sign*(edge[0]*(p-a)[1]-edge[1]*(p-a)[0])
    for p,q in zip(polygon,np.roll(polygon,-1,axis=0)):
        vp,vq=value(p),value(q)
        if vp>=-1e-10:result.append(p)
        if (vp>1e-10 and vq<-1e-10) or (vp<-1e-10 and vq>1e-10):
            result.append(p+(q-p)*vp/(vp-vq))
    clean=[]
    for p in result:
        if not clean or np.linalg.norm(p-clean[-1])>1e-8:clean.append(p)
    if len(clean)>1 and np.linalg.norm(clean[-1]-clean[0])<1e-8:clean.pop()
    return np.asarray(clean,float).reshape(-1,2)


def _polygon_difference(polygon, convex_clip):
    result=[];remaining=np.asarray(polygon,float)
    for a,b in zip(convex_clip,np.roll(convex_clip,-1,axis=0)):
        piece=_clip_polygon(remaining,a,b,inside=False)
        if len(piece)>=3 and abs(np.cross(piece,np.roll(piece,-1,axis=0)).sum())>1e-10:
            result.append(piece)
        remaining=_clip_polygon(remaining,a,b)
        if len(remaining)<3:break
    return result


def _distance_polygon(p, polygon):
    edge=np.roll(polygon,-1,axis=0)-polygon
    t=np.clip(((p-polygon)*edge).sum(1)/(edge*edge).sum(1),0.,1.)
    return float(np.linalg.norm(polygon+t[:,None]*edge-p,axis=1).min())


def _triangle_surface(template, xy, heights, sid, owner):
    xy=np.asarray(xy,float);heights=np.asarray(heights,float)
    if np.cross(xy[1]-xy[0],xy[2]-xy[0])<0:xy=xy[[0,2,1]];heights=heights[[0,2,1]]
    plane=np.linalg.solve(np.c_[xy,np.ones(3)],heights)
    grade=float(np.linalg.norm(plane[:2]));axis=plane[:2]/grade if grade>1e-10 else np.array([1.,0.])
    if grade>1.:raise ValueError(f'parametric_transition_excess_grade {sid} {grade} vertices={np.c_[xy,heights].tolist()}')
    center=xy.mean(0);lateral=np.array([-axis[1],axis[0]])
    value=copy.deepcopy(template)
    value.update(surface_id=sid,source_surface_id=owner,kind='continuous_support_envelope',
        frame=_surface_frame(center,axis),polygon_xy=np.c_[(xy-center)@axis,(xy-center)@lateral].tolist(),
        support_grade=grade,support_intercept_m=float(plane[:2]@center+plane[2]))
    value['source']['transition_vertices_xyz']=np.c_[xy,heights].tolist()
    value['source']['transition_semantics']='bounded continuous model join; vertices are priors, not lidar observations'
    return value


def add_supported_transitions(record, package):
    """Join core ramps to the observed landing and legacy floor endpoint.

    All added polygons stay between existing measured side lines. The global
    stage still requires known same-level input ground; raw clouds never gain
    points. Piecewise planes share exact vertices, including platform edges.
    """
    from .ground_path_bridge import GroundPathBridge
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel
    package=Path(package);record=copy.deepcopy(record);model=ParametricStairModel(record)
    bridge=GroundPathBridge.from_artifacts(package/'manifest.json')
    connector=json.loads((package/'compiled_stair_connector.json').read_text())
    route=np.asarray(connector['path'],float);split=connector['landing_index']
    labels=['stair_lower' if i<=split else 'stair_upper' for i in range(len(route))]
    source=bridge.to_localization_ground(route,labels).xyz
    landing=next(s for s in model.surfaces if s['kind']=='landing')
    landing_xy=landing['_origin'][:2]+landing['_polygon']@landing['_basis'][:2,:2]
    landing_z=float(landing['support_intercept_m']+landing['_origin'][2])
    additions=[];evidence=[];next_owner=max(s['source_surface_id'] for s in record['surfaces'])+1
    for index,s in enumerate(model.surfaces):
        if s['kind']=='landing':continue
        original=record['surfaces'][index];center=s['_origin'][:2];u=s['_basis'][0,:2];v=s['_basis'][1,:2]
        lo,hi=s['_polygon'][:,0].min(),s['_polygon'][:,0].max()
        # The endpoint whose model height is nearer the actual landing is its
        # platform side, independent of upper/lower naming conventions.
        platform_high=abs(s['support_grade']*hi+s['support_intercept_m']-landing_z)<abs(s['support_grade']*lo+s['support_intercept_m']-landing_z)
        run=min(s['ground_replacement_band_m']/max(s['support_grade'],.1),(hi-lo)/4)
        inner_lo,inner_hi=lo+run,hi-run
        fits=s['source']['boundary_fit']['sides'];guard=s['source']['boundary_guard_m']
        side=lambda station,positive:np.polyval(fits['positive' if positive else 'negative']['lateral_slope_intercept'],station)+(-guard if positive else guard)
        cross=lambda station:np.array([center+u*station+v*side(station,False),center+u*station+v*side(station,True)])
        core_poly=np.array([[inner_lo,side(inner_lo,False)],[inner_hi,side(inner_hi,False)],
            [inner_hi,side(inner_hi,True)],[inner_lo,side(inner_lo,True)]])
        original['polygon_xy']=core_poly.tolist()
        # Floor-side endpoint: interpolate the existing original-source route
        # exactly where it crosses the old model boundary, not a robot limit.
        floor_s=lo if platform_high else hi;floor_inner=inner_lo if platform_high else inner_hi
        st=(source[:,:2]-center)@u
        crossings=[]
        for j in range(len(source)-1):
            if (st[j]-floor_s)*(st[j+1]-floor_s)<=0 and abs(st[j+1]-st[j])>1e-9:
                a=(floor_s-st[j])/(st[j+1]-st[j]);point=source[j]+a*(source[j+1]-source[j])
                lateral=float((point[:2]-center)@v)
                if side(floor_s,False)<=lateral<=side(floor_s,True):crossings.append((j,point))
        if len(crossings)!=1:raise ValueError('parametric_transition_requires_unique_source_crossing')
        floor_z=float(crossings[0][1][2]);inner_z=float(s['support_grade']*floor_inner+s['support_intercept_m'])
        quad=np.r_[cross(floor_s),cross(floor_inner)[::-1]]
        z=np.array([floor_z,floor_z,inner_z,inner_z])
        for t,tri in enumerate(([0,1,2],[0,2,3])):
            additions.append(_triangle_surface(original,quad[tri],z[tri],s['surface_id']+f'_floor_join_{t}',next_owner));next_owner+=1
        # Platform join extends to the measured landing hull. Convex clipping
        # removes its interior; no overlap or arbitrary nearest surface wins.
        core_s=inner_hi if platform_high else inner_lo
        landing_st=(landing_xy-center)@u
        far=float(landing_st.max() if platform_high else landing_st.min())
        quad=np.r_[cross(core_s),cross(far)[::-1]]
        if np.cross(quad,np.roll(quad,-1,axis=0)).sum()<0:quad=quad[::-1]
        # An oblique corridor extrapolated to the landing's farthest station
        # otherwise produces corner wings beyond the actual landing extent.
        # Bound the join by the convex span of its core edge and the measured
        # landing, as well as the two existing side lines.
        envelope=np.r_[cross(core_s),landing_xy]
        envelope=envelope[ConvexHull(envelope).vertices]
        for a,b in zip(envelope,np.roll(envelope,-1,axis=0)):
            quad=_clip_polygon(quad,a,b)
        pieces=_polygon_difference(quad,landing_xy)
        core_z=float(s['support_grade']*core_s+s['support_intercept_m'])
        overlap=landing_xy.copy()
        for a,b in zip(quad,np.roll(quad,-1,axis=0)):overlap=_clip_polygon(overlap,a,b)
        if len(overlap)<3:raise ValueError('parametric_transition_no_landing_overlap')
        projected=(overlap-center)@u
        landing_start=float(projected.min() if platform_high else projected.max())
        if abs(landing_start-core_s)<1e-6:raise ValueError('parametric_transition_no_supported_run')
        # Reach platform height at the FIRST measured platform boundary.
        # Every farther platform edge then shares exactly its height. Split
        # the facets at that station; no triangulation spans the grade kink.
        splitline=cross(landing_start)
        splitpieces=[]
        for poly in pieces:
            for inside in (False,True):
                part=_clip_polygon(poly,splitline[0],splitline[1],inside=inside)
                if len(part)>=3:splitpieces.append(part)
        count=0
        for poly in splitpieces:
            heights=[]
            for point in poly:
                fraction=np.clip((float((point-center)@u)-core_s)/(landing_start-core_s),0.,1.)
                heights.append(core_z+(landing_z-core_z)*fraction)
            for j in range(1,len(poly)-1):
                tri=np.array([0,j,j+1])
                if abs(np.cross(poly[j]-poly[0],poly[j+1]-poly[0]))<1e-10:continue
                additions.append(_triangle_surface(original,poly[tri],np.asarray(heights)[tri],
                    s['surface_id']+f'_landing_join_{count}',next_owner));next_owner+=1;count+=1
        evidence.append(dict(surface_id=s['surface_id'],join_run_m=run,source_floor_crossing_index=crossings[0][0],
            source_floor_crossing_xyz=crossings[0][1].tolist(),platform_join_triangles=count,
            source_connector_sha256=sha256(package/'compiled_stair_connector.json')))
    record['surfaces'].extend(additions)
    record['evidence']['continuous_endpoint_transitions']=evidence
    ParametricStairModel(record)
    return record


def _boxes(planning, indices, center, axis, bounds, grade, surface_id, planning_hash):
    """Finite retained-structure boxes. No end caps and no inferred side fill."""
    xyz = planning[indices]
    lateral = np.array([-axis[1], axis[0]])
    local = np.c_[(xyz[:, :2]-center)@axis, (xyz[:, :2]-center)@lateral, xyz[:, 2]]
    yaw = float(np.arctan2(axis[1], axis[0]))
    result = []
    height_bin = np.floor((local[:,2]-grade*local[:,0])/.20).astype(int)
    for side in (-1, 1):
        for bin_id, lower in enumerate(np.arange(bounds[0], bounds[1], .40)):
            horizontal = ((local[:, 0] >= lower) & (local[:, 0] <= min(lower+.40, bounds[1]))
                          & (side*local[:, 1] > 0))
            for level in np.unique(height_bin[horizontal]):
                keep = horizontal & (height_bin == level)
                lo, hi = local[keep].min(axis=0), local[keep].max(axis=0)
                # Outward roundoff-only padding, not a second robot inflation.
                # Separate vertical bins stop a high protruding rail from
                # becoming a fictitious solid wall all the way to the ground.
                lo -= 1e-6; hi += 1e-6
                midpoint = (lo+hi)/2
                world = center+axis*midpoint[0]+lateral*midpoint[1]
                result.append(dict(structure_id=f'{surface_id}_side{side}_{bin_id}_{level}',
                    pose=dict(position=[*world.tolist(), float(midpoint[2])],
                        orientation_xyzw=[0., 0., float(np.sin(yaw/2)), float(np.cos(yaw/2))]),
                    half_extents_xyz=((hi-lo)/2).tolist(), source=dict(
                        kind='finite_retained_vertical_return_bounds', planning_map_sha256=planning_hash,
                        planning_record_indices=np.asarray(indices)[keep].tolist(),
                        no_endpoint_cap=True, padding_m=1e-6,
                        longitudinal_bin_m=.40, support_relative_height_bin_m=.20)))
    return result


def _regular_wall_strips(planning, indices, center, axis, bounds, grade, fitted,
                         surface_id, planning_hash, *, static_geometry_policy=None,
                         boundary_guard_m=.06):
    """Straight measured-side prior plus independently coherent protrusions.

    A residual band is derived from all retained side returns, not tuned to
    route success. Isolated residuals do not recreate a jagged wall silhouette.
    Their original points remain in both PCD and real-time SCAN inputs.
    """
    from d1max_pct_planner.stair_model_geometry import IDEAL_STATIC_POLICY
    if static_geometry_policy not in (None, IDEAL_STATIC_POLICY):
        raise ValueError('invalid_static_stair_wall_policy')
    if not np.isfinite(boundary_guard_m) or not 0 <= boundary_guard_m <= .10:
        raise ValueError('invalid_static_stair_boundary_guard')
    ideal = static_geometry_policy == IDEAL_STATIC_POLICY
    lateral=np.array([-axis[1],axis[0]])
    xyz=planning[indices]
    s=(xyz[:,:2]-center)@axis; t=(xyz[:,:2]-center)@lateral
    boxes=[]; evidence=[]
    for name,sign in (('negative',-1),('positive',1)):
        a,b=fitted['sides'][name]['lateral_slope_intercept']
        side=sign*t>0
        residual=(t-(a*s+b))/np.sqrt(1+a*a)
        # Boundary uncertainty is the robust fit of *side-location bins*,
        # not wall thickness. Outboard returns must not widen the uncertainty
        # symmetrically into the walking corridor.
        side_fit=fitted['sides'][name]
        fit_bins=np.asarray(side_fit['bins'])[side_fit['accepted_bins']]
        fit_residual=(fit_bins[:,1]-(a*fit_bins[:,0]+b))/np.sqrt(1+a*a)
        band=max(float(np.percentile(abs(fit_residual),95)),1e-5)
        outward=max(float(np.max(sign*residual[side])),band)
        direction=(axis+a*lateral)/np.sqrt(1+a*a)
        normal=np.array([-direction[1],direction[0]])
        yaw=float(np.arctan2(direction[1],direction[0]))
        for k,lo in enumerate(np.arange(bounds[0],bounds[1],1.0)):
            hi=min(lo+1.,bounds[1]); use=side&(s>=lo)&(s<=hi)
            if not use.any():continue
            endpoints=np.array([center+axis*v+lateral*(a*v+b) for v in (lo,hi)])
            cxy=endpoints.mean(0)+sign*normal*((outward-band)/2)
            zlo,zhi=xyz[use,2].min(),xyz[use,2].max()
            boxes.append(dict(structure_id=f'{surface_id}_{name}_fitted_{k}',
                pose=dict(position=[*cxy.tolist(),float((zlo+zhi)/2)],
                    orientation_xyzw=[0.,0.,float(np.sin(yaw/2)),float(np.cos(yaw/2))]),
                half_extents_xyz=[float(np.linalg.norm(endpoints[1]-endpoints[0])/2+1e-6),
                                  (band+outward)/2+1e-6,float((zhi-zlo)/2+1e-6)],
                source=dict(kind='static_regular_wall_strip_from_retained_side_fit',
                    planning_map_sha256=planning_hash, planning_record_indices=np.asarray(indices)[use].tolist(),
                    residual_band_p95_m=band,outward_observed_thickness_m=outward,no_endpoint_cap=True,
                    not_literal_solid_geometry=True)))
        # The existing finite boundary guard already assigns near-wall
        # residuals to the fitted side. Starting connected components at a
        # millimetric bin-median p95 links those points along an entire wall,
        # then boxes its largest local residual across the whole flight.
        # Recluster only outside that existing guard for the explicit ideal
        # policy. Independent inward structures still produce retained boxes.
        ownership_band = max(band, boundary_guard_m) if ideal else band
        protruding=side&(-sign*residual>ownership_band)
        rows=np.flatnonzero(protruding); coherent=[]
        if len(rows):
            pairs=cKDTree(xyz[rows]).query_pairs(.12,output_type='ndarray')
            graph=coo_matrix((np.ones(len(pairs)*2),(np.r_[pairs[:,0],pairs[:,1]],
                np.r_[pairs[:,1],pairs[:,0]])),shape=(len(rows),len(rows)))
            count,labels=connected_components(graph,directed=False)
            for component in range(count):
                selected=rows[labels==component]
                if len(selected)<6 or np.linalg.norm(np.ptp(xyz[selected],axis=0))<.10:continue
                coherent.extend(selected.tolist())
                q=np.c_[(xyz[selected,:2]-center)@direction,
                        (xyz[selected,:2]-center)@normal,xyz[selected,2]]
                low,high=q.min(0)-1e-6,q.max(0)+1e-6;c=(low+high)/2
                world=center+direction*c[0]+normal*c[1]
                boxes.append(dict(structure_id=f'{surface_id}_{name}_protrusion_{component}',
                    pose=dict(position=[*world.tolist(),float(c[2])],
                        orientation_xyzw=[0.,0.,float(np.sin(yaw/2)),float(np.cos(yaw/2))]),
                    half_extents_xyz=((high-low)/2).tolist(),source=dict(
                        kind='coherent_retained_inward_protrusion',planning_map_sha256=planning_hash,
                        planning_record_indices=np.asarray(indices)[selected].tolist(),
                        connectivity_m=.12,minimum_records=6,minimum_extent_m=.10)))
        evidence.append(dict(side=name,residual_band_p95_m=band,
            side_records=int(side.sum()),inward_outlier_records=len(rows),
            coherent_inward_records=len(coherent),
            isolated_residuals_retained_in_source_not_wall_silhouette=len(rows)-len(coherent)))
        if ideal:
            evidence[-1].update(static_geometry_policy=static_geometry_policy,
                wall_residual_ownership_band_m=ownership_band,
                existing_boundary_guard_m=boundary_guard_m,
                near_wall_records_assigned_to_fitted_boundary=int(
                    (side & (-sign*residual > band) & (-sign*residual <= ownership_band)).sum()))
    return boxes,evidence


def build_parametric_stair_geometry(package, *, periodicity=None, static_geometry_policy=None):
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel, SCHEMA, IDEAL_STATIC_POLICY
    from d1max_pct_planner.stair_centering import extract_measured_flight_structure, fit_measured_sides

    package = Path(package).resolve()
    manifest = json.loads((package/'manifest.json').read_text())
    planning_path = package/manifest['output_file']
    source_path = Path(manifest['source_path'])
    planning_hash = sha256(planning_path)
    if (planning_hash != manifest['output_sha256']
            or sha256(source_path) != manifest['source_sha256']):
        raise ValueError('parametric_model_input_identity')
    planning = read_pcd(planning_path).xyz
    original = read_pcd(source_path).xyz
    with np.load(package/'record_lineage.npz', allow_pickle=False) as archive:
        retained = archive['retained_source_indices']
    with np.load(package/'stair_ground_mapping.npz', allow_pickle=False) as archive:
        mapping = {key: archive[key] for key in archive.files}
    ids = mapping['source_indices']
    if (ids.dtype != np.int64 or np.any(ids < 0) or np.any(ids >= len(original))
            or not np.array_equal(mapping['source_xyz'], original[ids])):
        raise ValueError('parametric_model_literal_source_mapping_mismatch')
    original_models = manifest['stair_regularization']['surfaces']
    flights = extract_measured_flight_structure(planning, original_models)
    result = dict(schema=SCHEMA, frame_id='d1max_loc_map',
        planning_frame_id=manifest['frame_id'],
        source_map_sha256=manifest['source_sha256'], planning_map_sha256=planning_hash,
        model_points_are_observations=False, physical_tread_dimensions_recovered=False,
        geometry_semantics='explicit_static_planning_support_envelope_not_measured_treads',
        live_obstacle_semantics='model_does_not_modify_raw_occupancy_or_certify_free_space',
        surfaces=[], obstacles=[], evidence=dict(source_model_file='stair_structured_model.json',
            source_model_sha256=sha256(package/'stair_structured_model.json'),
            source_mapping_sha256=sha256(package/'stair_ground_mapping.npz'),
            periodicity=periodicity))
    if static_geometry_policy is not None:
        if static_geometry_policy != IDEAL_STATIC_POLICY:
            raise ValueError('invalid_stair_model_static_geometry_policy')
        result['static_geometry_policy'] = static_geometry_policy
    for flight in flights:
        center, axis = np.asarray(flight['center_xy']), np.asarray(flight['uphill_axis_xy'])
        bounds = np.asarray(flight['along_range_m'])
        fitted = fit_measured_sides(flight)
        negative = np.asarray(fitted['sides']['negative']['lateral_slope_intercept'])
        positive = np.asarray(fitted['sides']['positive']['lateral_slope_intercept'])
        # The existing measured-boundary guard is retained, not increased.
        guard = .06
        low, high = np.polyval(negative, bounds)+guard, np.polyval(positive, bounds)-guard
        polygon = [[bounds[0], low[0]], [bounds[1], low[1]],
                   [bounds[1], high[1]], [bounds[0], high[0]]]
        rows = np.asarray(flight['structure_source_indices'])
        if np.any(rows >= len(retained)):
            raise ValueError('parametric_model_structure_requires_original_lineage')
        original_structure = original[retained[rows]]
        if not np.allclose(original_structure, planning[rows], atol=2e-6, rtol=0):
            raise ValueError('parametric_model_structure_frame_not_source_identity')
        support_rows = mapping['surface_id'] == int(flight['id'])
        literal = original[ids[support_rows]]
        source_residual = literal[:, 2]-(float(flight['grade'])*((literal[:, :2]-center)@axis)
                                                +float(flight['intercept_m']))
        result['surfaces'].append(dict(surface_id=flight['leg'], source_surface_id=int(flight['id']),
            segment_ids=[flight['leg']], kind='continuous_support_envelope',
            frame=_surface_frame(center, axis), polygon_xy=np.asarray(polygon).tolist(),
            support_grade=float(flight['grade']), support_intercept_m=float(flight['intercept_m']),
            contact_tolerance_m=0., ground_replacement_band_m=.25,
            static_terrain_band_m=.25, static_stair_interior_assumed_clear=True,
            tread_stations_m=[], tread_heights_m=[],
            source=dict(kind='fitted_measured_support_envelope_and_retained_sides',
                planning_record_indices=flight['structure_source_indices'].tolist(),
                original_structure_indices=retained[rows].tolist(),
                original_structure_xyz_unchanged=True,
                support_source_indices=ids[support_rows].tolist(),
                source_support_residual_rms_m=float(np.sqrt(np.mean(source_residual**2))),
                source_support_absolute_residual_p95_m=float(np.percentile(abs(source_residual),95)),
                source_support_absolute_residual_max_m=float(np.max(abs(source_residual))),
                height_fit_origin='regularize_stair_support(source.xyz[ids]); original-frame support, not conditioned planning Z',
                boundary_fit=fitted, boundary_guard_m=guard,
                original_surface_id=int(flight['id'])),
            limitations=['Continuous envelope does not recover individual treads or certify stair contact.',
                         'Original support remains in the entrance/exit transition beyond this finite domain.']))
        boxes,wall_evidence=_regular_wall_strips(planning,flight['structure_source_indices'],
            center,axis,bounds,flight['grade'],fitted,flight['leg'],planning_hash,
            static_geometry_policy=static_geometry_policy,boundary_guard_m=guard)
        result['obstacles'].extend(boxes)
        result['surfaces'][-1]['source']['wall_model_evidence']=wall_evidence
    landing_models = [m for m in original_models if m.get('kind') == 'landing_plane']
    if len(landing_models) != 1:
        raise ValueError('parametric_model_requires_one_explicit_landing')
    landing = landing_models[0]
    take = mapping['surface_id'] == int(landing['id'])
    points, ids = mapping['source_xyz'][take], mapping['source_indices'][take]
    if len(points) < 12 or np.max(abs(points[:, 2]-landing['height_m'])) > .10:
        raise ValueError('parametric_model_landing_support_inconsistent')
    center = np.median(points[:, :2], axis=0)
    hull = ConvexHull(points[:, :2]-center)
    polygon = points[hull.vertices, :2]-center
    if len(polygon) > 64:
        raise ValueError('parametric_model_landing_hull_budget')
    result['surfaces'].append(dict(surface_id='stair_landing', source_surface_id=int(landing['id']),
        segment_ids=['stair_lower','stair_landing','stair_upper'], kind='landing', frame=_surface_frame(center, np.array([1., 0.])),
        polygon_xy=polygon.tolist(), support_grade=0., support_intercept_m=float(landing['height_m']),
        contact_tolerance_m=0., ground_replacement_band_m=.25, static_terrain_band_m=.25,
        static_stair_interior_assumed_clear=True,
        tread_stations_m=[], tread_heights_m=[], source=dict(kind='literal_landing_support_convex_domain',
            source_indices=ids.tolist(), unknown_cells_stay_unknown=True,
            source_support_residual_rms_m=float(np.sqrt(np.mean((points[:,2]-landing['height_m'])**2))),
            source_support_absolute_residual_p95_m=float(np.percentile(abs(points[:,2]-landing['height_m']),95)),
            height_fit_origin='median cluster of original source records, not conditioned planning Z'),
        limitations=['Convex domain is a prior; no unknown ground cell or ceiling is invented.']))
    ParametricStairModel.from_dict(result)
    return result


def build_candidate(package, output, *, model_path):
    """Apply the explicit geometry stage to a copied, already-built PCT map.

    The pre-model tensor remains an immutable stage input. No search algorithm,
    robot setting, point cloud, original source mapping, or control is edited.
    """
    import yaml
    from .stair_centering_runner import relocate_package
    from d1max_pct_planner.official_pipeline import source_processing_provenance, apply_parametric_geometry
    from d1max_pct_scan.navigation_map_package import validate_package
    from d1max_pct_planner.compiled_stair_connector import (
        map_settings_sha256, derive_model_ground, polyline_sha256, MODEL_DERIVATION_SCHEMA)
    from d1max_pct_planner.stair_model_geometry import ParametricStairModel
    from d1max_pct_planner.crossfloor_route import validate_config
    from d1max_pct_planner.cpu_tomography import export_preview
    package,output=Path(package).resolve(strict=True),Path(output).resolve()
    source=validate_package(package)
    frozen=dict(source['dependency_files'])
    relocate_package(package,output)
    record=json.loads(Path(model_path).read_text())
    target=output/'stair_model.json'
    target.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    manifest=json.loads((output/'manifest.json').read_text())
    manifest['parametric_stair_geometry']=dict(schema='d1max.parametric_stair_geometry_binding/v1',
        model_file=target.name,model_sha256=sha256(target),
        source_map_sha256=manifest['source_sha256'],planning_map_sha256=manifest['output_sha256'])
    manifest['processing_steps'].append(dict(operation='explicit_parametric_stair_geometry',
        schema=record['schema'],model_sha256=sha256(target),
        original_point_cloud_unchanged=True,model_points_are_observations=False))
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    cfg=yaml.safe_load((output/'pct_pipeline.yaml').read_text())
    processing=source_processing_provenance(cfg)
    with np.load(output/'pct/tomogram.npz',allow_pickle=False) as archive:
        payload={k:archive[k] for k in archive.files}
    payload['source_processing_manifest_sha256']=sha256(output/'manifest.json')
    payload,diagnostics=apply_parametric_geometry(payload,cfg,processing,output/'pct')
    np.savez_compressed(output/'pct/tomogram.npz',**payload)
    with (output/'pct/tomogram.pickle').open('wb') as stream:
        pickle.dump(payload,stream,protocol=pickle.HIGHEST_PROTOCOL)
    export_preview(payload,output/'pct/traversable.pcd',traversable_only=True)
    pct=json.loads((output/'pct/manifest.json').read_text())
    pct['source_processing']=processing
    pct['ground_semantics']=payload['ground_semantics']
    pct['ceiling_semantics']=payload['ceiling_semantics']
    pct['parametric_stair_geometry']=diagnostics
    pct['tensor_channels'][3]='explicit_static_stair_model_or_input_derived_ground_z'
    pct['files']={p.name:dict(sha256=sha256(p),bytes=p.stat().st_size)
        for p in (output/'pct').iterdir() if p.is_file() and p.name!='manifest.json'}
    (output/'pct/manifest.json').write_text(json.dumps(pct,indent=2,allow_nan=False)+'\n')
    route=yaml.safe_load((output/'route.yaml').read_text())
    bundle=json.loads((output/'compiled_stair_connector.json').read_text())
    base_file=output/'compiled_stair_geometry_base.json'
    if bundle.get('origin',{}).get('parametric_geometry') is not None:
        raise ValueError('parametric_model_requires_original_connector_not_cumulative_derivative')
    # Preserve the exact originally published connector, not relocate_package's
    # temporary pre-stage identity (that transient tomogram is overwritten).
    base_file.write_bytes((package/'compiled_stair_connector.json').read_bytes())
    original=np.asarray(bundle['path'],float);layers=np.asarray(bundle['layer_ids'],int)
    points=derive_model_ground(original,ParametricStairModel(record))
    bundle['path']=points.tolist()
    for name,index in [('entry',0),('landing',bundle['landing_index']),('exit',len(points)-1)]:
        route['anchors'][name]['xyz']=points[index].tolist()
    bundle['portals']={name:route['anchors'][name] for name in ('entry','landing','exit')}
    bundle['origin']['parametric_geometry']=dict(schema=MODEL_DERIVATION_SCHEMA,
        base_file=base_file.name,base_sha256=sha256(base_file),model_file=target.name,model_sha256=sha256(target),
        input_geometry_sha256=polyline_sha256(original,layers),output_geometry_sha256=polyline_sha256(points,layers),
        changed_vertices=np.flatnonzero(np.any(original!=points,axis=1)).tolist())
    bundle['tomogram_sha256']=sha256(output/'pct/tomogram.npz')
    bundle['map_settings_sha256']=map_settings_sha256(validate_config(route))
    (output/'compiled_stair_connector.json').write_text(json.dumps(bundle,indent=2,allow_nan=False)+'\n')
    route['compiled_stair_connector']['sha256']=sha256(output/'compiled_stair_connector.json')
    (output/'route.yaml').write_text(yaml.safe_dump(route,sort_keys=False,allow_unicode=True))
    if any(sha256(path)!=expected for path,expected in frozen.items()):
        raise ValueError('parametric_model_source_changed_during_build')
    try:
        checked=validate_package(output)
        check=dict(valid=True,dependencies=checked['dependency_files'])
    except Exception as error:
        # Preserve the genuine candidate rejection for debugging; never label
        # it deployable or activate a half-validated map.
        check=dict(valid=False,error_type=type(error).__name__,error=str(error))
    report=dict(schema='d1max.parametric_stair_candidate_build/v1',output=str(output),
        model_sha256=sha256(target),tomogram_sha256=sha256(output/'pct/tomogram.npz'),
        source_dependencies=frozen,geometry=diagnostics,package_check=check,
        activated=False,physical_acceptance=False)
    (output/'parametric_candidate_build.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--periodicity', type=Path)
    parser.add_argument('--candidate-model',type=Path,
                        help='Build a complete independent candidate from this prebuilt model')
    args = parser.parse_args(argv)
    if args.output.exists():
        raise ValueError('refusing_to_overwrite_stair_model_evidence')
    if args.candidate_model:
        print(json.dumps(build_candidate(args.package,args.output,model_path=args.candidate_model),
                         indent=2,allow_nan=False))
        return
    periodicity = (dict(file=str(args.periodicity.resolve()), sha256=sha256(args.periodicity),
        conclusion='periodicity_not_reliably_identified') if args.periodicity else None)
    model = build_parametric_stair_geometry(args.package, periodicity=periodicity)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), sha256=sha256(args.output),
                         surfaces=len(model['surfaces']), obstacles=len(model['obstacles']))))


if __name__ == '__main__':
    main()
