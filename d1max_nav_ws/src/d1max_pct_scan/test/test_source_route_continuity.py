"""Build-time ordinary-segment continuity; committed routes stay immutable."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_scan.source_route import (SourceRouteBuilder, RouteSnapshot,
    _coalesce_continuous_execution_segments, validate_snapshot)
from test_continuous_reference import (reference, observation, project, window,
    segmented_snapshot, snapshot)
from test_source_route import example


class LiteralGroundBridge:
    source_frame = 'd1max_loc_map'
    planning_frame = 'd1max_multifloor_planning'

    def to_localization_ground(self, points, labels):
        return SimpleNamespace(xyz=np.array(points, copy=True), diagnostics={})


def ordinary_then_stairs():
    points = [[float(i), 0., 0.] for i in range(7)]
    points += [[6., 0., .5], [6., 0., 1.], [7., 0., 1.]]
    labels = ['floor1']*6+['stair_lower']*2+['floor2']*2
    layers = [4]*7+[8, 12, 12]
    raw = dict(direction='lower_to_upper', layer_ids=layers,
        source_layer_ids=layers, segments=[
            dict(name='corridor_a', first_index=0, last_index=2, **{'from':'start','to':'ordinary_join'}),
            dict(name='corridor_b', first_index=2, last_index=6, **{'from':'ordinary_join','to':'stair_entry'}),
            dict(name='stair_lower', first_index=6, last_index=8, **{'from':'stair_entry','to':'stair_exit'}),
            dict(name='upper_floor', first_index=8, last_index=9, **{'from':'stair_exit','to':'goal'})],
        anchors={'ordinary_join':[2.,0.,0.], 'stair_entry':[6.,0.,0.]})
    return raw, points, labels


def build(raw, points, labels):
    return SourceRouteBuilder(bridge=LiteralGroundBridge(), source_map_sha256='a'*64,
        conditioning_sha256='b'*64, tomogram_sha256='c'*64).build(
            raw, points, labels, map_version_id='map')


def test_build_allows_forward_window_across_ordinary_portal_but_not_stairs():
    raw, points, labels = ordinary_then_stairs()
    route = build(raw, points, labels)
    ref = reference(source=route)
    project(ref, observation())
    first = window(ref, horizon_m=4.)
    assert first.ground_source_xyz[-1] == (4., 0., 0.), 'ordinary portal at x=2 must not truncate a 4 m window'
    assert first.segment_id == 'corridor_a'
    assert window(ref, horizon_m=10.).ground_source_xyz[-1] == (6.,0.,0.)
    assert ref.goal_ground_xyz == (7.,0.,1.)
    assert not ref.snapshot.payload()['execution_eligible']


def test_merged_metadata_keeps_original_geometry_layers_portals_and_authority():
    raw, points, labels = ordinary_then_stairs()
    raw['source_layer_ids'] = list(raw['source_layer_ids'])
    raw['source_layer_ids'][6] = 9
    before = deepcopy((raw, points, labels))
    value = build(raw, points, labels).payload()
    assert (raw, points, labels) == before
    assert value['xyz'] == points and value['point_floor_ids'] == labels
    assert value['source_layer_ids'] == raw['source_layer_ids']
    assert value['layer_ids'] == raw['layer_ids']
    first, stair, last = value['segments']
    assert (first['segment_id'], first['begin_index'], first['end_index']) == ('corridor_a',0,6)
    assert (first['source_layer_id'], first['target_layer_id']) == (4,9)
    assert (first['entry_portal_id'],first['exit_portal_id']) == ('start','stair_entry')
    assert stair['segment_id'] == 'stair_lower' and stair['required_mode'] == 'stair'
    assert last['exit_portal_id'] == 'goal' and last['end_index'] == len(points)-1
    assert value['edge_segments'] == ['corridor_a']*6+['stair_lower']*2+['upper_floor']
    evidence = value['geometry_evidence']
    assert evidence['native_segments'] == raw['segments'] and evidence['portals'] == raw['anchors']
    provenance = evidence['execution_segment_coalescing']
    assert [s['segment_id'] for s in provenance['original_segments']] == [s['name'] for s in raw['segments']]
    assert provenance['original_edge_segments'] == ['corridor_a']*2+['corridor_b']*4+['stair_lower']*2+['upper_floor']
    assert provenance['merged_groups'] == [dict(execution_segment_id='corridor_a',
        source_segment_ids=['corridor_a','corridor_b'],internal_portal_ids=['ordinary_join'],
        begin_index=0,end_index=6)]
    assert not value['execution_eligible'] and all(not s['execution_eligible'] for s in value['segments'])


@pytest.mark.parametrize('landing_first', [False, True])
def test_floor_landing_runs_are_continuous_and_existing_snapshot_is_immutable(landing_first):
    route = segmented_snapshot(landing=True)
    value = route.payload()
    if landing_first:
        value['segments'][0]['kind']='landing'
        value['segments'][1]['kind']='floor'
    validate_snapshot(value)
    original_bytes, original_hash = route.encoded, route.route_hash
    _coalesce_continuous_execution_segments(value)
    result = RouteSnapshot.create(value)
    assert len(result.payload()['segments']) == 1
    assert route.encoded == original_bytes and route.route_hash == original_hash
    first_normalization = deepcopy(value)
    _coalesce_continuous_execution_segments(value)
    assert value == first_normalization  # no nested provenance or new identity on retry
    assert RouteSnapshot.create(value).route_hash == result.route_hash


@pytest.mark.parametrize('boundary', ['portal_mismatch','empty_portal','floor','mode','kind','eligibility_reason'])
def test_distinct_or_unverified_boundaries_are_not_merged(boundary):
    value = segmented_snapshot(landing=True).payload()
    a,b = value['segments']
    if boundary == 'portal_mismatch': b['entry_portal_id']='another_portal'
    elif boundary == 'empty_portal': a['exit_portal_id']=b['entry_portal_id']=''
    elif boundary == 'floor':
        b['floor_id']='floor2';value['point_floor_ids'][1:]=['floor2','floor2']
    elif boundary == 'mode': b['required_mode']='unverified'
    elif boundary == 'kind': b['kind']='unknown'
    elif boundary == 'eligibility_reason': b['eligibility_reason']='other_evidence'
    validate_snapshot(value)
    before = RouteSnapshot.create(value)
    _coalesce_continuous_execution_segments(value)
    assert RouteSnapshot.create(value) == before


@pytest.mark.parametrize('bad', ['duplicate_id','overlap','gap','mixed_floor'])
def test_original_invalid_ownership_cannot_be_hidden_by_coalescing(bad):
    raw, points, labels = ordinary_then_stairs()
    if bad == 'duplicate_id': raw['segments'][1]['name']='corridor_a'
    elif bad == 'overlap': raw['segments'][1]['first_index']=1
    elif bad == 'gap': raw['segments'][1]['first_index']=3
    elif bad == 'mixed_floor': labels[3]='floor2'
    with pytest.raises(ValueError): build(raw,points,labels)


def test_build_is_deterministic_and_routes_without_merge_keep_historical_bytes():
    raw,points,labels=ordinary_then_stairs()
    assert build(raw,points,labels) == build(deepcopy(raw),deepcopy(points),list(labels))
    # Frozen BEFORE this change, not recomputed expectations from the new code.
    assert example().route_hash == '635cff22cc5af3c6e4efb3c99e1d69552c5d22fccc3af320ef93a644e32ceec2'
    assert snapshot(True).route_hash == '2f0083c617f4390eefce488a46d144062cf2f54ffbdff69d5c00dd68226df955'
    assert 'execution_segment_coalescing' not in example().payload()['geometry_evidence']


def test_three_ordinary_segments_merge_once_with_the_last_endpoint_metadata():
    raw,points,labels=ordinary_then_stairs()
    raw['segments'][:2]=[
        dict(name='corridor_a',first_index=0,last_index=1,**{'from':'start','to':'join_a'}),
        dict(name='corridor_b',first_index=1,last_index=3,**{'from':'join_a','to':'join_b'}),
        dict(name='corridor_c',first_index=3,last_index=6,**{'from':'join_b','to':'stair_entry'})]
    value=build(raw,points,labels).payload()
    assert len(value['segments'])==3 and value['segments'][0]['end_index']==6
    group=value['geometry_evidence']['execution_segment_coalescing']['merged_groups'][0]
    assert group['source_segment_ids']==['corridor_a','corridor_b','corridor_c']
    assert group['internal_portal_ids']==['join_a','join_b']


def test_reverse_crossfloor_route_preserves_stairs_and_merges_only_lower_ordinary_run():
    raw,points,labels=ordinary_then_stairs()
    count=len(points)
    reverse=deepcopy(raw)
    reverse['direction']='upper_to_lower'
    reverse['layer_ids']=raw['layer_ids'][::-1]
    reverse['source_layer_ids']=raw['source_layer_ids'][::-1]
    reverse['segments']=[dict(name=s['name'],first_index=count-1-s['last_index'],
        last_index=count-1-s['first_index'],**{'from':s['to'],'to':s['from']})
        for s in raw['segments'][::-1]]
    reverse_labels=labels[:-1][::-1]+[labels[0]]
    value=build(reverse,points[::-1],reverse_labels).payload()
    assert value['direction']=='upper_to_lower' and value['xyz']==points[::-1]
    assert [s['segment_id'] for s in value['segments']]==['upper_floor','stair_lower','corridor_b']
    stair=value['segments'][1]
    assert stair['kind']=='stair_down' and stair['required_mode']=='stair'
    final=value['segments'][-1]
    assert final['entry_portal_id']=='stair_entry' and final['exit_portal_id']=='start'
    assert final['begin_index']==3 and final['end_index']==9

