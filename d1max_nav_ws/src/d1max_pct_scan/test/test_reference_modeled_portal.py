"""Finite stair-portal correspondence; no ROS graph, commands or collision bypass."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from d1max_pct_planner.stair_model_geometry import ParametricStairModel
from d1max_pct_scan.control_frame_contract import Rigid, create_anchor
from d1max_pct_scan.continuous_reference import ContinuousReference
from d1max_pct_scan.reference_portal_geometry import ReferencePortalGeometry, PortalBoundary
from d1max_pct_scan.source_route import RouteSnapshot
from test_continuous_reference import CONTEXT, observation, project, reference


def modeled_core(*, down=False, obstacle=False, model=True, flat=False, anchor=None):
    payload=reference(crossfloor=True).snapshot.payload()
    heights=[0.,0.,.4,.8,.8]
    if down:heights=[.8-z for z in heights]
    payload['xyz']=[[x,0.,z] for x,z in zip([0.,.8,1.6,2.4,3.2],heights)]
    if down:
        payload['direction']='upper_to_lower'
        for s in payload['segments']:
            s['direction']='upper_to_lower'
            if s['kind']=='stair_up':s['kind']='stair_down'
    if flat:
        payload['xyz']=[[x,0.,0.] for x in [0.,.8,1.6,2.4,3.2]]
        payload['point_floor_ids']=['floor1']*5
        for s in payload['segments']:s.update(kind='floor',required_mode='general',floor_id='floor1')
    surface=dict(surface_id='finite-flight',source_surface_id=1,
        segment_ids=['stair_lower','stair_upper'],kind='continuous_support_envelope',
        frame=dict(origin_xyz=[2.4,0.,0.] if down else [.8,0.,0.],
            axis_u=[-1.,0.,0.] if down else [1.,0.,0.],
            axis_v=[0.,-1.,0.] if down else [0.,1.,0.],axis_up=[0.,0.,1.]),
        polygon_xy=[[0.,-.6],[1.6,-.6],[1.6,.6],[0.,.6]],
        support_grade=.5,support_intercept_m=0.,contact_tolerance_m=0.,
        ground_replacement_band_m=.25,static_terrain_band_m=.08,
        static_stair_interior_assumed_clear=True,tread_stations_m=[],tread_heights_m=[])
    record=dict(schema='d1max.parametric_stair_geometry/v1',frame_id='d1max_loc_map',
        source_map_sha256='a'*64,planning_map_sha256='b'*64,
        model_points_are_observations=False,surfaces=[surface],obstacles=[])
    if obstacle:
        record['obstacles']=[dict(structure_id='rail',source={'kind':'bounded_structure'},
            pose=dict(position=[.8,.4,.8 if down else .3],orientation_xyzw=[0.,0.,0.,1.]),
            half_extents_xyz=[.1,.03,.4])]
    geometry=ReferencePortalGeometry.from_model(ParametricStairModel(record))
    anchor=anchor or create_anchor(observation(frame='d1max_loc_map').body,observation().body,1)
    core=ContinuousReference(RouteSnapshot.create(payload),context=CONTEXT,task_id='task',
        anchor=anchor,body_height_m=.55,body_height_calibration_id='fixture',
        execution_mode_policy='fixed_stair',portal_geometry=geometry if model else None)
    return core,geometry


def crossing(core, *, down=False, lateral=.4):
    base=1.35 if down else .55
    sign=-1 if down else 1
    project(core,observation(y=lateral,z=base))
    project(core,observation(x=.6,y=lateral,z=base,t=.6))
    project(core,observation(x=.75,y=lateral,z=base,t=.75))
    body=observation(x=.85,y=lateral,z=base+sign*.025,t=.85)
    project(core,body)
    return body


@pytest.mark.parametrize('down',[False,True])
def test_bounded_lateral_crossing_enters_next_stair_without_centerline_return(down):
    core,_=modeled_core(down=down)
    body=crossing(core,down=down)
    assert core._portal_observation is not None
    staged=core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    assert staged._segment==1 and core._segment==0
    assert staged._progress.source_ns==body.body.source_ns
    assert staged._progress.body_source_xyz==body.body.pose.xyz
    assert staged.snapshot.route_hash==core.snapshot.route_hash
    assert staged.anchor==core.anchor and staged.portal_geometry is core.portal_geometry
    assert staged.window(current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic).start_arc_m>.8


@pytest.mark.parametrize('fault',['no_model','wall','outside_polygon','wrong_height','wrong_owner','parallel_before'])
def test_model_cannot_enable_other_floors_structures_or_nonadjacent_space(fault):
    core,geometry=modeled_core(model=fault!='no_model',obstacle=fault=='wall')
    if fault=='wrong_owner':
        core.portal_geometry=replace(geometry,surfaces=tuple(replace(s,owners=frozenset({'unrelated'}))
                                                            for s in geometry.surfaces))
    lateral=.7 if fault=='outside_polygon' else .4
    project(core,observation(y=lateral))
    project(core,observation(x=.6,y=lateral,t=.6))
    project(core,observation(x=.75,y=lateral,t=.75))
    body=observation(x=.75 if fault=='parallel_before' else .85,y=lateral,
        z=1.575 if fault=='wrong_height' else .575,t=1.6 if fault=='wrong_height' else .85)
    try:project(core,body)
    except ValueError:pass
    assert core._portal_observation is None
    with pytest.raises(ValueError):
        core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
            now_monotonic=body.received_monotonic)


def test_reverse_crossing_retires_modeled_witness():
    core,_=modeled_core();crossing(core)
    project(core,observation(x=.75,y=.4,t=1.))
    assert core._portal_observation is None


def test_model_bounds_only_new_lateral_correspondence_not_legacy_center_semantics():
    core,geometry=modeled_core()
    # Explicitly retain the previous centre-sphere correspondence. A source
    # structure here is still adjudicated by native SCAN, not by this layer.
    boundary=PortalBoundary(np.array((.8,0.,.3)),np.eye(3),np.array((.1,.03,.4)))
    core.portal_geometry=replace(geometry,boundaries=(boundary,))
    assert not core.portal_geometry.clear_column((.8,0.,0.),height=.55)
    body=crossing(core,lateral=0.)
    assert core._portal_observation is not None
    assert core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)._segment==1


@pytest.mark.parametrize('down',[False,True])
@pytest.mark.parametrize('landing',[False,True])
def test_lateral_stair_exit_to_adjacent_floor_or_platform(down,landing):
    core,geometry=modeled_core(down=down)
    payload=core.snapshot.payload()
    if landing:
        payload['segments'][-1]['kind']='landing'
    core=ContinuousReference(RouteSnapshot.create(payload),context=CONTEXT,task_id='task',
        anchor=core.anchor,body_height_m=.55,body_height_calibration_id='fixture',
        execution_mode_policy='fixed_stair',portal_geometry=geometry)
    body=crossing(core,down=down)
    core=core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    sign=-1 if down else 1
    base=1.35 if down else .55
    for x in (1.,1.3,1.55,1.65):
        body=observation(x=x,y=.4,z=base+sign*.5*(x-.8),t=x)
        project(core,body)
    core=core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    for x in (1.9,2.2,2.35,2.45):
        body=observation(x=x,y=.4,z=base+sign*.5*(min(x,2.4)-.8),t=x)
        project(core,body)
    result=core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    assert result._segment==3 and result._progress.measured_arc_m>core._arc[3]
    assert result.snapshot.route_hash==core.snapshot.route_hash
    assert result._progress.source_ns==body.body.source_ns


@pytest.mark.parametrize('model',[False,True])
def test_ordinary_floor_boundary_keeps_original_portal_semantics(model):
    core,_=modeled_core(model=model,flat=True)
    body=crossing(core)
    assert core._portal_observation is None
    with pytest.raises(ValueError,match='not_physically_reached'):
        core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
            now_monotonic=body.received_monotonic)


def test_portal_geometry_is_immutable_and_not_a_reanchor_cache():
    core,geometry=modeled_core()
    duplicate=deepcopy(core)
    assert duplicate.portal_geometry is geometry
    for array in (geometry.surfaces[0].origin,geometry.surfaces[0].basis,geometry.surfaces[0].polygon):
        with pytest.raises(ValueError):array.flags.writeable=True
    wrong=replace(geometry,source_hash='d'*64)
    with pytest.raises(ValueError,match='identity_mismatch'):
        ContinuousReference(core.snapshot,context=CONTEXT,task_id='task',anchor=core.anchor,
            body_height_m=.55,body_height_calibration_id='fixture',portal_geometry=wrong)


def test_reanchor_queries_source_geometry_once_with_new_measured_coordinates():
    core,geometry=modeled_core()
    project(core,observation(y=.4));project(core,observation(x=.6,y=.4,t=.6))
    body=observation(x=.65,y=.4,t=.7)
    global_=observation(x=.70,y=.4,t=.7,frame='d1max_loc_map')
    candidate=core.observe_pair(global_,body,revision=2,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    staged=core.prepare_reanchor(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    assert staged.portal_geometry is geometry and staged.anchor==candidate
    # New local x=.8 corresponds to source x=.85 after this +.05 correction.
    actual=observation(x=.8,y=.4,z=.575,t=.85)
    project(staged,actual)
    assert staged._portal_observation is not None
    assert core._portal_observation is None and core._segment==0
    next_=staged.prepare_segment_transition(actual,current_source_ns=actual.body.source_ns,
        now_monotonic=actual.received_monotonic)
    assert next_._progress.body_source_xyz==pytest.approx((.85,.4,.575))
    assert next_._progress.source_ns==actual.body.source_ns


def test_delayed_candidate_can_recover_after_old_projection_ends():
    core,_=modeled_core();body=crossing(core)
    original=core._portal_observation
    # Real measured motion continues while a geometry proposal is outstanding.
    project(core,observation(x=1.2,y=.4,z=.75,t=1.2))
    body=observation(x=1.5,y=.4,z=.9,t=1.5)
    with pytest.raises(ValueError,match='outside_current_semantic_segment'):project(core,body)
    staged=core.prepare_segment_transition(body,current_source_ns=body.body.source_ns,
        now_monotonic=body.received_monotonic)
    assert core._portal_observation==original
    assert staged._segment==1 and staged._progress.source_ns==body.body.source_ns


def test_loaded_model_is_wired_from_node_to_transport_and_every_new_core(monkeypatch):
    from types import SimpleNamespace
    from d1max_pct_scan import reference_support
    from test_continuous_reference_node import harness
    from test_continuous_reference_transport import envelope, BASE_NS
    _,geometry=modeled_core()
    model=object()
    monkeypatch.setattr(reference_support,'load_reference_support',
        lambda *a,**kw:SimpleNamespace(stair_model=model))
    seen=[]
    def adapt(cls,value):
        seen.append(value)
        return geometry
    monkeypatch.setattr(ReferencePortalGeometry,'from_model',classmethod(adapt))
    node,clock,_=harness(planning_manifest='verified-package-fixture',execution_mode_policy='fixed_stair')
    assert seen==[model] and node.transport.portal_geometry is geometry
    node.transport.on_route(envelope(crossfloor=True),current_source_ns=BASE_NS,now_monotonic=100.)
    assert node.transport.core.portal_geometry is geometry
    assert deepcopy(node.transport.core).portal_geometry is geometry
