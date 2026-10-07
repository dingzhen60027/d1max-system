"""Full future volumes use an enforced simulation script, not sparse poses."""
import copy
import math
import pytest
from dynamic_collision import (actor_registry, registry_digest, scripted_actor_regions,
    scripted_center_bounds, native_hit_actor_ids, oracle_payload, voxel_veto)
from world_builder import create_large_world, actor_pose, actor_colliders


def registered(actor):
    return actor_registry({'dynamic_actors':[actor]})[0]


def sample(actor, seconds):
    pose = actor_pose(actor, seconds)
    yaw = pose['yaw']
    return dict(position=pose['position'], linear_velocity=pose['linear_velocity'],
        orientation_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])


@pytest.mark.parametrize('index', range(6))
def test_complete_future_shapes_enclosed_through_turns_seams_and_parking(index):
    actor = copy.deepcopy(create_large_world()['dynamic_actors'][index]);actor['enabled'] = True
    entry = registered(actor)
    period = actor['trajectory']['waypoints'][-1]['time_s']
    for start in (0., actor['start_time_s']-.5, actor['start_time_s']+period-3.,
                  actor['end_time_s']-3., actor['end_time_s']+1.):
        start = max(0., start)
        regions = scripted_actor_regions(entry, sample(actor, start), round(start*1e9), 6_000_000_000)
        for step in range(121):
            t = start + step*.05
            by_id = {shape['id']:region for shape,region in zip(entry['shapes'],regions)}
            for shape in actor_colliders(actor, t):
                region=by_id[shape['id']]
                if shape['type'] == 'box':
                    hx,hy,hz = [v/2. for v in shape['size']]
                    corners = [(x,y,z) for x in (-hx,hx) for y in (-hy,hy) for z in (-hz,hz)]
                else:
                    # A complete circumscribed XY box is deliberately stronger
                    # than just sampled cylinder surface points.
                    r=shape['radius'];hz=shape['height']/2.
                    corners = [(x,y,z) for x in (-r,r) for y in (-r,r) for z in (-hz,hz)]
                c,s=math.cos(shape['yaw']),math.sin(shape['yaw'])
                for x,y,z in corners:
                    p=[shape['center'][0]+c*x-s*y,shape['center'][1]+s*x+c*y,shape['center'][2]+z]
                    # Cylinder is yaw invariant, so audit its axis-aligned box
                    # rather than rotate the square into a fictitious corner.
                    if shape['type']=='cylinder':p=[shape['center'][0]+x,shape['center'][1]+y,shape['center'][2]+z]
                    assert all(lo <= value <= hi for value,lo,hi in zip(p,region['min'],region['max'])), (index,t,p,region)


def test_original_six_second_horizon_and_lease_survive_narrower_future_truth():
    actor=create_large_world()['dynamic_actors'][0];entry=registered(actor)
    t=105.14;s=sample(actor,t);s.update(source_stamp_ns=1_000_000_000,present=True)
    proof=oracle_payload([entry],{actor['id']:s},session_id='s',epoch=1,seed_id='seed',
        context_sequence=1,sequence=1,source_stamp_ns=1_000_000_000,
        simulation_source_stamp_ns=round(t*1e9),reachable_horizon_ns=6_000_000_000)
    assert proof['reachable_until_ns']==7_000_000_000
    assert proof['valid_until_ns']==1_300_000_000
    # The v35 first UNKNOWN voxel is far outside the actor's enforced swept
    # solids; the actor's actual whole current body remains an UNKNOWN veto.
    assert voxel_veto(proof,[-17,-89,-1])==0
    assert voxel_veto(proof,[0,math.floor(s['position'][1]/.05),10])==2
    assert all(region['state']==2 for region in proof['actors'][0]['regions'])


def test_measured_deviation_missing_clock_or_bad_contract_revokes_stronger_model():
    actor=create_large_world()['dynamic_actors'][0];entry=registered(actor);s=sample(actor,10.)
    for change in ('position','tilt','missing_clock'):
        measured=copy.deepcopy(s);source=10_000_000_000
        if change=='position':measured['position'][0]+=.000101
        if change=='tilt':measured['orientation_xyzw']=[.001,0.,0.,math.sqrt(1.-.001**2)]
        if change=='missing_clock':source=None
        with pytest.raises(ValueError):scripted_actor_regions(entry,measured,source,6_000_000_000)
    digest=registry_digest([entry]);changed=copy.deepcopy(actor)
    changed['trajectory']['waypoints'][1]['position'][0]+=.1
    assert registry_digest([registered(changed)])!=digest
    changed=copy.deepcopy(actor);changed['end_time_s']-=1
    with pytest.raises(ValueError,match='continuous_parking'):registered(changed)
    changed=copy.deepcopy(actor);changed['future_motion_contract']['unverified']=True
    with pytest.raises(ValueError):registered(changed)


def test_original_native_prim_identity_never_uses_nearby_xyz_or_namespace_guess():
    spec=create_large_world();registry=actor_registry(spec)
    actor=registry[0];path=actor['shapes'][0]['path']
    assert native_hit_actor_ids([path,actor['path'],path+'/unknown','/World/Indoor/wall',''],registry)==[1,1,0,0,0]


def test_native_rigid_body_roots_have_exact_distinct_ordinals_and_no_prefix_aliases():
    registry=actor_registry(create_large_world())
    roots=[actor['path'] for actor in registry]
    assert native_hit_actor_ids(roots,registry)==list(range(1,len(registry)+1))
    root=roots[0]
    assert native_hit_actor_ids(['/World/Dynamic',root+'/',root+'_other',root+'/unlisted',
        '/World/Dynamic/unregistered'],registry)==[0]*5


def test_native_rigid_body_root_or_leaf_identity_collision_rejects_ambiguous_registry():
    registry=actor_registry(create_large_world())
    for duplicate in ('root','leaf'):
        changed=copy.deepcopy(registry)
        if duplicate=='root':changed[1]['path']=changed[0]['path']
        else:changed[1]['shapes'][0]['path']=changed[0]['shapes'][0]['path']
        with pytest.raises(ValueError,match='ambiguous_native_hit_actor_registry'):
            native_hit_actor_ids([changed[0]['path']],changed)
