"""Static surface provenance, not motion approval or live occupancy clearing."""
import numpy as np
import pytest
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_scan.stair_support import StairSupportIndex
from d1max_pct_scan.reference_support import ConditionedReferenceSupport
from d1max_pct_scan.source_route import SourceSupport


def regularized_mapping(source, planning):
    from tools.pointcloud_preprocessing.ground_path_bridge import StairGroundMapping
    count = len(source)
    return StairGroundMapping(dict(source_sha256='a'*64, mapping_sha256='b'*64,
        surfaces=[dict(id=1, kind='flight_support_envelope')],
        max_displacement_m=.08, association_max_distance_m=.12),
        dict(source_indices=np.arange(count, dtype=np.int64), source_xyz=source.astype(np.float64),
             planning_xyz=planning.astype(np.float64), surface_id=np.ones(count,dtype=np.int32)),
        source_xyz=source)


def fixture():
    data=np.zeros((5,2,13,7),np.float32)
    data[3,0]=np.arange(13)[:,None]//3*.1
    data[3,1]=data[3,0]+3.
    data[4]=data[3]+1.
    grid=TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=3.))
    rows=[]
    for layer in range(2):
        for x in range(13):
            for y in range(7):
                xy=grid.world((x,y))
                rows.append([*xy,float(data[3,layer,x,y])])
    literal=np.asarray(rows)
    return grid,literal


def test_stair_selection_preserves_literal_xyz_separates_overlapping_floors():
    grid,literal=fixture()
    index=StairSupportIndex(literal,grid)
    lower=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    for path in (lower,lower[::-1],lower+[0.,0.,3.]):
        result=index.select(path)
        assert result.verified and result.maximum_distance_m<=.12
        assert all(tuple(p) in set(map(tuple,literal)) for p in result.points)
        assert np.ptp(result.points[:,2])<1.
    wrong=lower+[0.,0.,1.5]
    assert not index.select(wrong).verified


def test_raw_stair_roi_walls_do_not_automatically_become_support():
    grid,literal=fixture()
    walls=literal+[0.,0.,.3]
    walls=walls[(walls[:,2]<1.) & (walls[:,0]<-.2)]
    index=StairSupportIndex(np.r_[literal,walls],grid)
    assert all(tuple(p) in set(map(tuple,literal)) for p in index.points)
    assert not index.select(walls).verified


def test_missing_tread_or_invalid_pct_cell_cannot_be_bridged_as_measured_ground():
    grid,literal=fixture()
    lower=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    grid.valid[:,4:9,:]=False
    index=StairSupportIndex(literal,grid)
    result=index.select(lower)
    assert not result.verified and result.reason=='stair_measured_surface_coverage_missing'
    assert len(result.points)==0


def test_classified_stair_surface_requires_exact_hash_and_semantic_domain():
    grid,literal=fixture()
    source=SourceSupport({'stairs':literal},'a'*64)
    provider=ConditionedReferenceSupport({'source_sha256':'a'*64},source,grid)
    lower=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    kwargs=dict(source_map_sha256='a'*64,floor_id='stair_lower',segment_kind='stair_up',required_mode='stair')
    assert provider.stair_surface(lower,**kwargs).verified
    for key,value in [('source_map_sha256','b'*64),('floor_id','floor1'),
                      ('segment_kind','floor'),('required_mode','general')]:
        assert not provider.stair_surface(lower,**dict(kwargs,**{key:value})).verified
    # Existing blanket classification deliberately still rejects the ROI.
    assert not provider.classified_ground(**kwargs)


def test_bad_empty_oversized_or_unsupported_window_never_fabricates_returns():
    grid,literal=fixture();index=StairSupportIndex(literal,grid)
    for path in ([],[[0.,0.,0.]],[[0.,0.,float('nan')],[.1,0.,0.]],
                 [[0.,0.,0.],[100000.,0.,0.]]):
        result=index.select(path)
        assert not result.verified and not len(result.points)


def test_native_side_candidate_context_retains_real_tread_not_just_centerline_strip():
    data=np.zeros((5,2,13,31),np.float32)
    data[3,0]=np.arange(13)[:,None]//3*.1
    data[3,1]=data[3,0]+3.
    data[4]=data[3]+1.
    grid=TomogramMap(dict(data=data,resolution=.1,center=[0.,0.],slice_h0=0.,slice_dh=3.))
    literal=np.asarray([[*grid.world((x,y)),float(data[3,layer,x,y])]
                        for layer in range(2) for x in range(13) for y in range(31)])
    index=StairSupportIndex(literal,grid)
    center=literal[np.isclose(literal[:,1],0.) & (literal[:,2]<1.)]
    before=index.select(center,context_radius_m=.2)
    after=index.select(center)
    assert before.verified and after.verified
    assert after.maximum_distance_m==before.maximum_distance_m
    assert np.max(np.abs(before.points[:,1]))<=.2+1e-9
    assert np.max(np.abs(after.points[:,1]))>=1.25
    assert np.all(after.points[:,2]<1.)
    assert set(map(tuple,after.points))<=set(map(tuple,literal))
    # Removing lateral returns leaves a narrow supported stair; requesting a
    # larger context must not invent a wider tread or fill a missing centre.
    narrow=literal[np.abs(literal[:,1])<=.2]
    assert np.max(np.abs(StairSupportIndex(narrow,grid).select(center).points[:,1]))<=.2+1e-9
    for bad in (float('nan'),float('inf'),-.1,2.01):
        assert not index.select(center,context_radius_m=bad).verified


def test_regularized_pct_association_keeps_original_support_without_widening_tolerance():
    grid, planning = fixture()
    # Eight centimetres of planning-only correction exceeds the existing five
    # centimetre association: without correspondence a whole flight is lost.
    source = planning.copy(); source[:, 2] -= .08
    lower = source[np.isclose(source[:,1],0.) & (source[:,2]<1.)]
    old = StairSupportIndex(source, grid)
    assert not old.select(lower).verified
    mapping = regularized_mapping(source, planning)
    owned = SourceSupport({'stairs':source}, 'a'*64, stair_mapping=mapping)
    provider = ConditionedReferenceSupport({'source_sha256':'a'*64}, owned, grid)
    assert provider.stair_index.surface_tolerance_m == old.surface_tolerance_m == .05
    for path in (lower, lower[::-1], lower+[0.,0.,3.]):
        selected = provider.stair_surface(path, source_map_sha256='a'*64,
            floor_id='stairs', segment_kind='stair_up', required_mode='stair')
        assert selected.verified
        assert set(map(tuple, selected.points)) <= set(map(tuple, source))
        assert not set(map(tuple, selected.points)) & set(map(tuple, planning))
        assert np.ptp(selected.points[:,2]) < 1.
    with pytest.raises(ValueError, match='mapping_hash_mismatch'):
        SourceSupport({'stairs':source}, 'c'*64, stair_mapping=mapping)


def test_regularization_does_not_promote_wall_returns_or_fill_missing_pct_support():
    grid, planning = fixture()
    source = planning.copy(); source[:, 2] -= .08
    mapping = regularized_mapping(source, planning)
    walls = planning+[0.,0.,.2]
    index = StairSupportIndex(np.r_[source,walls], grid, stair_mapping=mapping)
    assert set(map(tuple,index.points)) == set(map(tuple,source))
    lower = source[np.isclose(source[:,1],0.) & (source[:,2]<1.)]
    grid.valid[:,4:9,:] = False
    index = StairSupportIndex(np.r_[source,walls], grid, stair_mapping=mapping)
    assert not index.select(lower).verified
    assert not index.select(walls[:13]).verified


def test_independent_structured_literal_context_is_not_filtered_by_pct_body_clearance():
    grid, source = fixture()
    mapping = regularized_mapping(source, source)
    lower = source[source[:, 2] < 1.]
    center = lower[np.isclose(lower[:, 1], 0.)]
    # The footprint-inflated cost is not evidence that a measured tread has
    # disappeared. Its independent structural classification is exact XYZ.
    grid.valid[:, :, :] = False
    before = grid.valid.copy(), grid.ground.copy(), grid.ceiling.copy()
    assert len(StairSupportIndex(source, grid, stair_mapping=mapping).points) == 0
    result = StairSupportIndex(source, grid, stair_mapping=mapping,
        structured_source_points=lower)
    assert set(map(tuple, result.points)) == set(map(tuple, lower))
    assert result.select(center).verified and result.select(center[::-1]).verified
    assert not result.select(center+[0., 0., 3.]).verified
    assert result.radius == .12 and result.surface_tolerance_m == .05
    for original, actual in zip(before, (grid.valid, grid.ground, grid.ceiling)):
        assert np.array_equal(original, actual)


def test_structured_support_intersects_caller_and_cannot_invent_missing_tread_or_wall():
    grid, source = fixture()
    mapping = regularized_mapping(source, source)
    grid.valid[:] = False
    lower = source[source[:, 2] < 1.]
    # Explicit evidence for a mapped row is insufficient if the current
    # caller's separately verified source subset does not contain that row.
    caller = source[np.abs(source[:, 0]) >= .25]
    with pytest.raises(ValueError, match='stair_mapping_source_ownership_mismatch'):
        StairSupportIndex(caller, grid, stair_mapping=mapping,
            structured_source_points=lower)
    selected = StairSupportIndex(source, grid, stair_mapping=mapping,
        structured_source_points=lower[np.abs(lower[:, 0]) >= .25])
    assert set(map(tuple, selected.points)) == set(map(tuple, caller)) & set(map(tuple, lower))
    center = lower[np.isclose(lower[:, 1], 0.)]
    assert not selected.select(center).verified
    walls = lower+[0., 0., .30]
    for invalid in (walls, lower+[1e-9, 0., 0.], np.array([[float('nan'), 0, 0]]),
                    np.zeros((20001, 3)), np.zeros((2, 2))):
        with pytest.raises(ValueError, match='structured_stair_support'):
            StairSupportIndex(np.r_[source, walls], grid, stair_mapping=mapping,
                structured_source_points=invalid)
    with pytest.raises(ValueError, match='structured_stair_support_evidence_invalid'):
        StairSupportIndex(source, grid, structured_source_points=source)


def test_absent_or_empty_structured_evidence_preserves_legacy_index_exactly():
    grid, source = fixture()
    mapping = regularized_mapping(source, source)
    grid.valid[:, 4:9, :] = False
    legacy = StairSupportIndex(source, grid, stair_mapping=mapping)
    for evidence in (None, np.empty((0, 3))):
        current = StairSupportIndex(source, grid, stair_mapping=mapping,
            structured_source_points=evidence)
        assert np.array_equal(current.points, legacy.points)
        for path in (source[:13], source[-13:]):
            left, right = legacy.select(path), current.select(path)
            assert left.verified == right.verified and left.reason == right.reason
            assert left.maximum_distance_m == right.maximum_distance_m
            assert np.array_equal(left.points, right.points)
