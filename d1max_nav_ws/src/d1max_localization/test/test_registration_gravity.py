"""Pure evidence/selection tests: no ROS graph, process, SDK or map flattening."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from d1max_localization.global_registration import RegistrationConfig, select_candidate
from d1max_localization.registration_gravity import (
    QueryGravityHistory, load_map_gravity_contract, validate_registration_gravity)


def contract(rotation=None, map_rotation=None):
    rotation=np.eye(3) if rotation is None else rotation
    map_rotation=np.eye(3) if map_rotation is None else map_rotation
    return dict(schema=1, query=dict(direction=(rotation.T @ [0.,0.,-1.]).tolist(), epoch=1,
        source_stamp_ns=10**9, query_stamp_ns=10**9, frame_id='tracking', source='lio_inertial_posterior'),
        map=dict(direction=(map_rotation @ [0.,0.,-1.]).tolist(), frame_id='map', map_sha256='a'*64,
                 evidence_id='b'*64, source='explicit_synthetic_fixture'))


def candidate(rotation=None, *, score=.8, x=0.):
    matrix=np.eye(4);matrix[:3,:3]=np.eye(3) if rotation is None else rotation;matrix[0,3]=x
    return dict(transform=matrix.tolist(),score=score,inlier_ratio=.95,rmse_m=.06,information_ratio=.03)


def test_upside_down_winner_rejected_before_ranking_without_changing_score():
    flipped=candidate(Rotation.from_euler('x',179.,degrees=True).as_matrix(),score=.9)
    true=candidate(score=.7,x=10.)
    result=select_candidate([flipped,true],RegistrationConfig(),contract())
    assert result['accepted'] and result['candidate']['raw_rank']==2
    assert result['candidate']['score']==.7
    diag=result['gravity_filter']
    assert diag['rejected_count']==1
    assert math.degrees(diag['rejected_candidates'][0]['gravity_error_rad'])==pytest.approx(179.)
    assert diag['raw_top_candidates'][0]['score']==.9


def test_gravity_rejection_is_not_floor_pitch_or_tracking_z_rejection():
    # Real 45-degree robot pitch on stairs, plus a map whose Z is tilted 20deg.
    tracking=Rotation.from_euler('zy',[.7,math.pi/4]).as_matrix()
    tilted_map=Rotation.from_euler('x',20.,degrees=True).as_matrix()
    true=candidate(tilted_map @ tracking)
    result=select_candidate([true],RegistrationConfig(),contract(tracking,tilted_map))
    assert result['accepted']
    assert result['candidate']['gravity_error_rad']<1.e-7


def test_repeated_valid_floors_remain_ambiguous_after_flips_removed():
    inverted=candidate(Rotation.from_euler('x',180.,degrees=True).as_matrix(),score=.9)
    a=candidate(score=.7);b=candidate(score=.68,x=20.)
    result=select_candidate([inverted,a,b],RegistrationConfig(),contract())
    assert not result['accepted'] and result['reason']=='ambiguous_place_or_floor'
    assert result['score_margin']==pytest.approx(.02)
    assert [c['raw_rank'] for c in result['candidates']]==[2,3]


def test_all_flipped_and_missing_gravity_have_explainable_rejections():
    flipped=candidate(Rotation.from_euler('x',180.,degrees=True).as_matrix())
    assert select_candidate([flipped],RegistrationConfig(),contract())['reason']=='all_candidates_gravity_inconsistent'
    result=select_candidate([candidate()],RegistrationConfig())
    assert result['reason']=='registration_gravity_contract_missing' and not result['accepted']
    old=select_candidate([candidate()],RegistrationConfig(),geometry_only=True)
    assert old['candidate'] and not old['accepted'] and not old['initialization_eligible']
    assert old['reason']=='geometry_only_not_initialization_eligible'


@pytest.mark.parametrize('change',[lambda c:c['query'].update(epoch=0),
    lambda c:c['query'].update(query_stamp_ns=10**9+1001),
    lambda c:c['query'].update(direction=[0,0,0]),lambda c:c['map'].update(evidence_id='')])
def test_invalid_contract_never_falls_back_to_z(change):
    c=contract();change(c)
    assert not select_candidate([candidate()],RegistrationConfig(),c)['accepted']


def test_map_hash_and_angle_policy_cannot_be_relaxed_silently():
    with pytest.raises(ValueError):validate_registration_gravity(contract(),'c'*64)
    with pytest.raises(ValueError):replace(RegistrationConfig(),gravity_max_error_rad=math.pi)


def sample(stamp_ns=10**9, epoch=1, rotation=None, gravity=None):
    return dict(schema=1,valid=True,fault=False,epoch=epoch,received_at_unix=1.01,
        stamp_ns=str(stamp_ns),frame='odom',child_frame='tracking',position=[0.,0.,0.],
        orientation=(Rotation.identity() if rotation is None else rotation).as_quat().tolist(),
        inertial=dict(schema=1,gravity=[0.,0.,-9.81] if gravity is None else gravity))


def test_query_uses_exact_posterior_gravity_and_inverse_tracking_rotation():
    history=QueryGravityHistory('odom','tracking')
    odom=Rotation.from_euler('x',.3);tracking=Rotation.from_euler('y',.7)
    value=sample(rotation=odom*tracking,gravity=odom.apply([0.,0.,-9.81]).tolist())
    history.observe(value,1.02)
    q=history.at(10**9,1,1.03)
    assert q['direction']==pytest.approx(tracking.inv().apply([0.,0.,-1.]))
    assert history.at(10**9+1001,1,1.03) is None
    assert history.at(10**9,2,1.03) is None
    assert history.at(10**9,1,1.51) is None
    history.observe(dict(schema=1,valid=False,fault=False,epoch=2,received_at_unix=1.04),1.04)
    assert history.at(10**9,2,1.04) is None


@pytest.mark.parametrize('change',[lambda s:s.update(frame='map'),lambda s:s.update(fault=True),
    lambda s:s['inertial'].update(gravity=[0,0,1]),lambda s:s['inertial'].update(gravity=[float('nan'),0,0])])
def test_invalid_query_posterior_has_no_gravity_sample(change):
    history=QueryGravityHistory('odom','tracking');value=sample();change(value)
    with pytest.raises(ValueError):history.observe(value,1.02)
    assert history.at(10**9,1,1.03) is None


def map_record(tmp_path, rotation=None):
    r=np.eye(3) if rotation is None else rotation
    identity=np.column_stack([np.eye(3),np.zeros(3)]).ravel()
    optimized=np.column_stack([r,np.zeros(3)]).ravel()
    sources={
        'frontend_config':'laserMapping:\n  ros__parameters:\n    mapping:\n      gravity_aligned_world: true\n',
        'frontend_state':'gx,gy,gz\n0,0,-9.81\n',
        'pgo_odometry':' '.join(map(str,identity))+'\n',
        'pgo_optimized':' '.join(map(str,optimized))+'\n',
        'processed_manifest':json.dumps(dict(output_sha256='a'*64,frame_id='map',source_unchanged=True,
            record_preservation='byte_exact_original_records_in_original_order'))}
    evidence={}
    for name,content in sources.items():
        path=tmp_path/name;path.write_text(content)
        evidence[name]=dict(path=str(path),sha256=hashlib.sha256(content.encode()).hexdigest())
    return json.dumps(dict(schema=1,kind='lio_pgo_preserved_records',map_sha256='a'*64,
                           frame_id='map',evidence=evidence))


def test_map_gravity_is_transformed_from_evidence_not_assumed_world_z(tmp_path):
    r=Rotation.from_euler('x',.3).as_matrix()
    record=map_record(tmp_path,r)
    c=load_map_gravity_contract(record,'a'*64,'map')
    assert c['direction']==pytest.approx(r @ [0.,0.,-1.])
    with pytest.raises(ValueError):load_map_gravity_contract(record,'b'*64,'map')
    with pytest.raises(ValueError):load_map_gravity_contract(record,'a'*64,'wrong_map_frame')
    (tmp_path/'frontend_config').write_text('changed')
    with pytest.raises(ValueError,match='evidence_changed'):load_map_gravity_contract(record,'a'*64,'map')
