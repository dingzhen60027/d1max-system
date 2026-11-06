"""Shared stair models are explicit geometry, never manufactured source rows."""
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from d1max_pct_planner.stair_model_geometry import ParametricStairModel
from d1max_pct_scan.navigation_map_package import load_parametric_stair_model
from d1max_pct_scan.source_route import SourceRouteBuilder, SourceSupport, canonical, digest_file
from d1max_pct_scan.reference_support import ConditionedReferenceSupport, parametric_stair_packet
from d1max_pct_scan.control_frame_contract import Rigid
from test_navigation_map_package import conditioned_package


def model_record():
    surface = dict(surface_id='lower', source_surface_id=1, segment_ids=['stairs'],
        kind='continuous_support_envelope', frame=dict(origin_xyz=[0.,0.,0.],
            axis_u=[1.,0.,0.], axis_v=[0.,1.,0.], axis_up=[0.,0.,1.]),
        polygon_xy=[[0.,-.7],[2.,-.7],[2.,.7],[0.,.7]], support_grade=.3,
        support_intercept_m=0., contact_tolerance_m=0., ground_replacement_band_m=.25,
        static_entity_height_m=.08, static_terrain_band_m=.08, static_stair_interior_assumed_clear=True,
        tread_stations_m=[], tread_heights_m=[])
    return dict(schema='d1max.parametric_stair_geometry/v1', frame_id='d1max_loc_map',
        source_map_sha256='a'*64, planning_map_sha256='b'*64,
        model_points_are_observations=False, surfaces=[surface],
        obstacles=[dict(structure_id='measured-wall', source={'kind':'measured_side'},
            pose=dict(position=[1.,.8,.5], orientation_xyzw=[0.,0.,0.,1.]),
            half_extents_xyz=[1.,.05,.5])])


def bind_model(tmp_path, record=None):
    record = record or model_record()
    model_path = tmp_path/'stair_model.json'
    model_path.write_text(canonical(record))
    manifest = dict(source_sha256='a'*64, output_sha256='b'*64,
        parametric_stair_geometry=dict(schema='d1max.parametric_stair_geometry_binding/v1',
            source_map_sha256='a'*64, planning_map_sha256='b'*64,
            model_file=model_path.name, model_sha256=digest_file(model_path)))
    path=tmp_path/'manifest.json';path.write_text(canonical(manifest))
    return path, manifest, model_path


def test_package_model_dependency_is_exact_shared_asset_not_observations(tmp_path):
    path, manifest, model_path=bind_model(tmp_path)
    seen={}
    model, sha, deps=load_parametric_stair_model(path, manifest,
        checked=lambda p,h:seen.update({str(p):h}))
    assert deps==seen=={str(model_path):digest_file(model_path)}
    assert sha==manifest['parametric_stair_geometry']['model_sha256']
    assert model.frame_id=='d1max_loc_map' and not model.record['model_points_are_observations']
    assert load_parametric_stair_model(path, {})==(None,'',{})


@pytest.mark.parametrize('fault',['hash','frame','source','planning','escape','observation'])
def test_model_loader_rejects_foreign_or_unbound_model(tmp_path,fault):
    path, manifest, model_path=bind_model(tmp_path)
    record=model_record()
    if fault=='hash':model_path.write_text(model_path.read_text()+' ')
    elif fault=='escape':manifest['parametric_stair_geometry']['model_file']='../stair_model.json'
    else:
        if fault=='frame':record['frame_id']='d1max_multifloor_planning'
        elif fault=='source':record['source_map_sha256']='c'*64
        elif fault=='planning':record['planning_map_sha256']='c'*64
        else:record['model_points_are_observations']=True
        model_path.write_text(canonical(record))
        manifest['parametric_stair_geometry']['model_sha256']=digest_file(model_path)
    with pytest.raises(ValueError):load_parametric_stair_model(path,manifest)


def test_every_artifact_ground_bridge_constructor_binds_the_shared_model(conditioned_package):
    from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import GroundPathBridge, ParametricGroundPathBridge
    path=conditioned_package['manifest'];manifest=json.loads(path.read_text())
    legacy=GroundPathBridge.from_artifacts(path)
    record=model_record();record.update(source_map_sha256=manifest['source_sha256'],
                                        planning_map_sha256=manifest['output_sha256'])
    model_path=path.parent/'stair_model.json';model_path.write_text(canonical(record))
    manifest['parametric_stair_geometry']=dict(schema='d1max.parametric_stair_geometry_binding/v1',
        model_file=model_path.name,model_sha256=digest_file(model_path),
        source_map_sha256=manifest['source_sha256'],planning_map_sha256=manifest['output_sha256'])
    path.write_text(canonical(manifest))
    bridge=GroundPathBridge.from_artifacts(path)
    assert isinstance(bridge,ParametricGroundPathBridge)
    model,sha,_=load_parametric_stair_model(path,manifest)
    assert bridge.stair_model.record==model.record and bridge.stair_model_sha256==sha
    points=np.array([[-.1,0.,-.65],[-.05,0.,-.65]])
    np.testing.assert_array_equal(bridge.to_localization_ground(points,['floor1']*2).xyz,
                                  legacy.to_localization_ground(points,['floor1']*2).xyz)


class OldBridge:
    source_frame='d1max_loc_map'
    planning_frame='d1max_multifloor_planning'
    def __init__(self):self.calls=[]
    def to_localization_ground(self, points, labels):
        self.calls.append(np.array(points,copy=True))
        xyz=np.array(points,copy=True);xyz[:,2]+=.04
        return SimpleNamespace(xyz=xyz,diagnostics={'legacy':True})


def route_builder(model=None):
    bridge=OldBridge()
    builder=SourceRouteBuilder(bridge=bridge,source_map_sha256='a'*64,
        conditioning_sha256='c'*64,tomogram_sha256='d'*64,
        support=SourceSupport({},'a'*64),stair_model=model,
        stair_model_sha256='e'*64 if model else '')
    return builder,bridge


def test_source_route_model_precedes_old_inverse_and_does_not_forge_returns():
    builder,bridge=route_builder(ParametricStairModel(model_record()))
    points=np.array([[-.2,0.,0.],[.2,0.,.06],[.8,0.,.24],[1.4,0.,.42],[2.,0.,.6],[2.2,0.,.6]])
    converted=builder._planning_ground(points,['stair_lower']*len(points))
    np.testing.assert_array_equal(converted.xyz[1:-1],points[1:-1])
    np.testing.assert_array_equal(converted.xyz[[0,-1],2],points[[0,-1],2]+.04)
    assert [len(v) for v in bridge.calls]==[1,1]
    result=dict(direction='lower_to_upper',layer_ids=[0]*len(points),source_layer_ids=[0]*len(points),
        segments=[dict(name='stair_lower',first_index=0,last_index=len(points)-1,**{'from':'a','to':'b'})])
    value=builder.build(result,points,['stair_lower']*len(points),map_version_id='map').payload()
    evidence=value['geometry_evidence']
    assert evidence['parametric_stair_model']['modeled_indices']==[1,2,3,4]
    assert evidence['measured_source_support']['unsupported_samples']>0
    assert 'observed_source_support_xyz' not in evidence
    assert evidence['ground_to_body_height_applications']==0 and not value['execution_eligible']


def test_ordinary_floor_route_and_hash_are_exactly_unchanged_outside_model():
    points=[[-2.,0.,0.],[-1.5,0.,0.],[-1.,0.,0.]]
    result=dict(route_type='same_floor',layer_ids=[0]*3,source_layer_ids=[0]*3)
    old,_=route_builder();new,_=route_builder(ParametricStairModel(model_record()))
    assert old.build(result,points,['floor1']*3,map_version_id='map')==new.build(
        result,points,['floor1']*3,map_version_id='map')


def provider(record=None):
    record=record or model_record()
    return ConditionedReferenceSupport({'source_sha256':'a'*64}, SourceSupport({},'a'*64),
        stair_model=ParametricStairModel(record),stair_model_sha256=hashlib.sha256(canonical(record).encode()).hexdigest())


@pytest.mark.parametrize('rpy',[[0.,0.,0.],[.24,-.32,1.7],[-.21,.38,-2.4]])
def test_real_typed_model_full_rigid_pose_transform_without_height_or_local_parameter_changes(rpy):
    from rclpy.serialization import serialize_message,deserialize_message
    from d1max_planning_interfaces.msg import SupportReference
    from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
    from geometry_msgs.msg import PoseStamped
    from test_reference_support import request
    p=provider();node=object.__new__(ReferenceCallbacks);node.support_map=p
    inverse=Rigid((2.,-3.,.4),tuple(map(float,Rotation.from_euler('xyz',rpy).as_quat())))
    req=list(request('a'*64,floor='stair_lower',kind='stair_up',mode='stair',inverse=inverse))
    req[0].reference.segment_id='stairs';req[5]=('stairs','stair_lower','stair_up','stair')
    for x in (.2,.5,.8):
        pt=inverse.point((x,0.,.3*x+.55));pose=PoseStamped()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=pt
        req[0].reference.path.poses.append(pose)
    original=deepcopy(req[0]);_,packet=node.build_support(tuple(req))
    assert req[0]==original and packet.version==original.version and packet.source_stamp==original.source_stamp
    assert not packet.verified and not packet.support_ground_xyz and not packet.stair_contact_indices
    assert not packet.stair_riser_xyz and packet.stair_model_hash==p.stair_model_sha256
    assert len(packet.stair_model_surfaces)==len(packet.stair_model_obstacles)==1
    surface=packet.stair_model_surfaces[0]
    actual=np.array([surface.pose.position.x,surface.pose.position.y,surface.pose.position.z])
    np.testing.assert_allclose(actual,inverse.point((0.,0.,0.)),atol=1e-12)
    q=surface.pose.orientation
    local_to_odom=Rigid(tuple(map(float,actual)),(q.x,q.y,q.z,q.w))
    for x in (.2,.5,.8):
        np.testing.assert_allclose(local_to_odom.point((x,0.,surface.support_grade*x)),
            inverse.point((x,0.,.3*x)),atol=1e-12)
    assert surface.support_intercept_m==0. and surface.segment_id=='stairs'
    assert surface.height_association_m==.25
    assert deserialize_message(serialize_message(packet),SupportReference)==packet


def test_model_hash_and_segment_association_affect_content_and_general_window_remains_empty():
    p=provider();inverse=Rigid((0.,0.,0.),(0.,0.,0.,1.))
    context=('stairs','stair_lower','stair_up','stair')
    first=parametric_stair_packet(p,source_hash='a'*64,segment=context,contexts=(),inverse=inverse)
    assert first[0] and first[3]['model_points_are_observations'] is False
    wrong=('other-stair','stairs','stair_up','stair')
    assert parametric_stair_packet(p,source_hash='a'*64,segment=wrong,contexts=(),inverse=inverse)==('',[],[],None)
    floor=('stairs','floor1','floor','general')
    assert parametric_stair_packet(p,source_hash='a'*64,segment=floor,contexts=(),inverse=inverse)==('',[],[],None)
    with pytest.raises(ValueError,match='source_mismatch'):
        parametric_stair_packet(p,source_hash='f'*64,segment=context,contexts=(),inverse=inverse)


def test_landing_model_uses_explicit_associations_not_literal_region_name():
    value=model_record();landing=deepcopy(value['surfaces'][0])
    landing.update(surface_id='platform',source_surface_id=2,kind='landing',support_grade=0.,
                   segment_ids=['stairs','upper'])
    value['surfaces'].append(landing)
    p=provider(value);inverse=Rigid((0.,0.,0.),(0.,0.,0.,1.))
    low=('stairs','stair_lower','stair_up','stair');high=('upper','stair_upper','stair_up','stair')
    for contexts in ((low,),(low,high),(high,)):
        _,surfaces,_,_=parametric_stair_packet(p,source_hash='a'*64,segment=contexts[0],contexts=contexts,inverse=inverse)
        platform=next(s for s in surfaces if s.surface_id=='platform')
        assert platform.segment_id==contexts[0][0]
    assert not p.points and not p.stair_contact_source


def geometry_stage(tmp_path, *, terrain_ceiling=False, interior_wall=False):
    from d1max_pct_planner.cpu_tomography import traversal_cost, _inflate
    from d1max_pct_planner.stair_model_geometry import apply_parametric_stair_geometry
    from d1max_pct_planner.tomogram_map import TomogramMap
    pct=tmp_path/'pct';pct.mkdir()
    x,y=np.meshgrid(np.arange(-.5,2.51,.1),np.arange(-1.,1.01,.1),indexing='ij')
    ground=(.3*x+.025*np.sin(x*35))[None].astype(np.float32)
    ceiling=(ground+.03) if terrain_ceiling else np.full_like(ground,3.)
    cfg=dict(traversability=dict(kernel_size=7,interval_min=.55,interval_free=.7,
        slope_max_rad=.6,step_max=.17,standable_ratio=.2,cost_barrier=50.,safe_margin=.1,inflation=.1))
    raw=traversal_cost(ground,ceiling,.1,cfg['traversability'])
    cost=np.array([_inflate(raw[0],.1,.1,.1)])
    gx=np.zeros_like(cost);gy=gx.copy()
    gx[:,1:-1]=cost[:,2:]-cost[:,:-2];gy[:,:,1:-1]=cost[:,:,2:]-cost[:,:,:-2]
    payload=dict(data=np.stack([cost,gx,gy,ground,ceiling]),resolution=.1,center=np.array([1.,0.]),
        slice_h0=1.,slice_dh=.5,source_sha256='b'*64)
    record=model_record()
    if interior_wall:record['obstacles'][0]['pose']['position'][1]=.4
    model=ParametricStairModel(record)
    from test_navigation_map_package import write_pcd
    from d1max_pct_scan.pointcloud_helpers.pcd_io import read_pcd
    cloud=tmp_path/'planning.pcd'
    points=np.c_[x.ravel(),y.ravel(),(.3*x).ravel()]
    if terrain_ceiling:
        points=np.r_[points,np.c_[x.ravel(),y.ravel(),(.3*x+1.).ravel()]]
    write_pcd(cloud,points)
    result=apply_parametric_stair_geometry(payload,cfg,model,read_pcd(cloud).xyz)
    base=pct/'parametric_geometry_base.npz';np.savez_compressed(base,**payload)
    evidence=pct/'parametric_geometry_evidence.npz';np.savez_compressed(evidence,**result['arrays'])
    (pct/'parametric_geometry_report.json').write_text(canonical(result['diagnostics']))
    current=dict(payload,data=result['data'],parametric_stair_geometry_schema=record['schema'],
        parametric_stair_model_sha256='e'*64,parametric_stair_base_sha256=digest_file(base),
        parametric_stair_evidence_sha256=digest_file(evidence),
        parametric_stair_data_sha256=hashlib.sha256(result['data'].tobytes()).hexdigest())
    return model,TomogramMap(current),payload


@pytest.mark.parametrize('fault',[None,'tensor','frame','outside','owner','evidence_hash','model_hash'])
def test_explicit_stage_preserves_old_prior_and_cannot_relabel_changed_ground_as_observations(tmp_path,fault):
    from d1max_pct_scan.navigation_map_package import _parametric_geometry_dependencies
    model,grid,payload=geometry_stage(tmp_path)
    evidence=tmp_path/'pct/parametric_geometry_evidence.npz'
    if fault=='tensor':grid.data[3,0,10,10]+=.01
    elif fault=='frame':grid.center[0]+=.01
    elif fault in ('outside','owner'):
        with np.load(evidence,allow_pickle=False) as z:arrays={k:z[k] for k in z.files}
        if fault=='outside':arrays['modeled_ground'][0,0,0]=True
        else:arrays['surface_index'][arrays['modeled_ground']]=22
        np.savez_compressed(evidence,**arrays)
        grid.provenance['parametric_stair_evidence_sha256']=digest_file(evidence)
    elif fault=='evidence_hash':grid.provenance['parametric_stair_evidence_sha256']='f'*64
    elif fault=='model_hash':grid.provenance['parametric_stair_model_sha256']='f'*64
    seen={}
    def checked(path,expected=None):
        digest=digest_file(path)
        if expected is not None and expected!=digest:raise ValueError('evidence_changed')
        seen[str(path)]=digest;return path
    if fault:
        with pytest.raises(ValueError):_parametric_geometry_dependencies(tmp_path,grid,model,'e'*64,checked)
    else:
        old=_parametric_geometry_dependencies(tmp_path,grid,model,'e'*64,checked)
        np.testing.assert_array_equal(old.data,payload['data'])
        assert len(seen)==3
        assert np.any(old.ground!=grid.ground)


def test_terrain_ceiling_reclassification_requires_actual_retained_column(tmp_path):
    from d1max_pct_scan.navigation_map_package import _parametric_geometry_dependencies
    model,grid,_=geometry_stage(tmp_path,terrain_ceiling=True)
    def checked(path,expected=None):
        if expected is not None:assert digest_file(path)==expected
        return path
    base=_parametric_geometry_dependencies(tmp_path,grid,model,'e'*64,checked,
        planning_cloud_path=tmp_path/'planning.pcd')
    assert np.any(base.ceiling!=grid.ceiling)
    with np.load(tmp_path/'pct/parametric_geometry_evidence.npz') as z:
        indices=np.argwhere(z['terrain_ceiling_reclassified'])
    cell=tuple(indices[len(indices)//2]);grid.ceiling[cell]+=.1
    grid.provenance['parametric_stair_data_sha256']=hashlib.sha256(grid.data.tobytes()).hexdigest()
    report=tmp_path/'pct/parametric_geometry_report.json';value=json.loads(report.read_text())
    value['output_data_sha256']=grid.provenance['parametric_stair_data_sha256'];report.write_text(canonical(value))
    with pytest.raises(ValueError,match='ceiling_not_retained_column'):
        _parametric_geometry_dependencies(tmp_path,grid,model,'e'*64,checked,
            planning_cloud_path=tmp_path/'planning.pcd')


def test_structural_model_domain_cannot_be_resigned_as_traversable_cost(tmp_path):
    from d1max_pct_scan.navigation_map_package import _parametric_geometry_dependencies
    model,grid,_=geometry_stage(tmp_path,interior_wall=True)
    evidence=tmp_path/'pct/parametric_geometry_evidence.npz'
    with np.load(evidence) as archive:arrays={k:archive[k] for k in archive.files}
    cell=tuple(np.argwhere(arrays['structural_or_entity_domain'])[0])
    assert grid.cost[cell]>grid.COST_THRESHOLD
    grid.cost[cell]=0.
    grid.provenance['parametric_stair_data_sha256']=hashlib.sha256(grid.data.tobytes()).hexdigest()
    report=tmp_path/'pct/parametric_geometry_report.json';value=json.loads(report.read_text())
    value['output_data_sha256']=grid.provenance['parametric_stair_data_sha256'];report.write_text(canonical(value))
    def checked(path,expected=None):
        if expected is not None:assert digest_file(path)==expected
        return path
    with pytest.raises(ValueError,match='structure_cost_not_blocked'):
        _parametric_geometry_dependencies(tmp_path,grid,model,'e'*64,checked)


def test_model_ground_bridge_selection_body_and_route_share_one_height_not_old_return_radius():
    from test_navigation_surfaces import regularized_stair_model
    from d1max_pct_scan.navigation_surfaces import NavigationSurfaceResolver
    from d1max_pct_scan.live_goal_selection import LiveGoalSelection
    from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import ParametricGroundPathBridge
    old,_=regularized_stair_model()
    record=model_record();surface=record['surfaces'][0]
    surface.update(polygon_xy=[[-.3,-.8],[.3,-.8],[.3,.8],[-.3,.8]],
                   support_grade=0.,support_intercept_m=1.)
    model=ParametricStairModel(record)
    bridge=ParametricGroundPathBridge(old.bridge,model,'e'*64)
    resolver=NavigationSurfaceResolver(old.tomogram,bridge,old.settings)
    editor=LiveGoalSelection(old.tomogram,bridge,old.settings['floor_z_ranges'],route_settings=old.settings)
    # This cell is >12cm from the legacy measured-return correspondence. The
    # explicit model domain, not an enlarged nearest-point threshold, admits it.
    point=np.array([0.,.5,1.])
    with pytest.raises(ValueError):old.resolve_goal(point)
    goal=resolver.resolve_goal(point)
    assert goal['source_xyz']==goal['planning_xyz']==point.tolist()
    assert editor.edit(point)['valid']
    body=resolver.resolve_body(point+[0.,0.,.5],[.49,.51])
    np.testing.assert_array_equal(body['source_xyz'],point)
    projected=bridge.project_live_pose_to_ground(point+[0.,0.,.5],'stair_lower',body_height_interval_m=(.49,.51))
    np.testing.assert_array_equal(projected.xyz,point)
    for method in (bridge.to_localization_ground,bridge.to_planning_ground):
        np.testing.assert_array_equal(method(point[None],['stair_lower']).xyz[0],point)
    # Same XY is not sufficient: another height must not be absorbed into this
    # model, and unrelated flat-floor selection is byte-for-byte unchanged.
    assert bridge.model_surface_at([0.,.5,0.]) is None
    assert resolver.resolve_goal([.6,0.,-.1])==old.resolve_goal([.6,0.,-.1])
    old.tomogram.cost[1,10,15]=100.
    with pytest.raises(ValueError):resolver.resolve_goal(point)


def test_model_ground_adds_body_height_once_in_production_continuous_reference():
    from test_continuous_reference import reference,observation,project,window
    points=[[.2,0.,.06],[.8,0.,.24],[1.4,0.,.42]]
    builder,_=route_builder(ParametricStairModel(model_record()))
    result=dict(direction='lower_to_upper',layer_ids=[0]*3,source_layer_ids=[0]*3,
        segments=[dict(name='stair_lower',first_index=0,last_index=2,**{'from':'a','to':'b'})])
    snapshot=builder.build(result,points,['stair_lower']*3,map_version_id='map')
    core=reference(source=snapshot)
    project(core,observation(x=.2,z=.61))
    packet=window(core)
    assert packet.ground_to_body_height_applications==1
    np.testing.assert_allclose(np.array(packet.body_odom_xyz)-np.array(packet.ground_source_xyz),
                              np.tile([0.,0.,.55],(len(packet.body_odom_xyz),1)),atol=1e-12)
