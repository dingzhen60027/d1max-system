from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from d1max_localization.global_registration import RegistrationConfig, select_candidate
from d1max_localization.startup_relocalization import StartupConfig, StartupRelocalization
from d1max_localization.math_utils import Pose3
from d1max_localization.relocalization_cloud import pointcloud_xyz


P = Pose3((0., 0., 0.), (0., 0., 0., 1.))


def controller():
    c = StartupRelocalization('session')
    c.map_sha256 = 'a' * 64
    for t in np.arange(10., 10.81, .1):
        c.observe(1, float(t), P, (0, 0, 0), (0, 0, 0))
    return c


def candidate(z=0., x=3., **changes):
    t = np.eye(4); t[:3, 3] = [x, 2., z]
    return dict(transform=t.tolist(), inlier_ratio=.9, rmse_m=.05,
        information_ratio=.05, score=.75, **changes)


def result(c, **changes):
    return dict(request_id=c.request['request_id'], map_sha256=c.map_sha256,
        accepted=True, candidate=candidate(), **changes)


def test_coarse_search_never_becomes_ready_without_matching_and_fused_output():
    c = controller()
    assert c.capture(10.75, 10.8)
    pose = c.accept(result(c), 10.8)
    assert pose.position == pytest.approx((3, 2, 0))
    assert c.state == 'searching'
    c.mark_seed('seed', 10.8, 'global')
    for active, confirmed, valid in [('seed', None, True), ('seed', 'old', True), ('seed', 'seed', False)]:
        c.verification(active, confirmed, valid, 11.)
        assert c.state == 'confirming'
    c.verification('seed', 'seed', True, 11.1)
    assert c.state == 'ready'


def test_manual_seed_overrides_late_worker_and_local_reset_never_resumes_auto():
    c = controller(); c.capture(10.75, 10.8)
    old = result(c)
    c.mark_seed('manual', 10.8, 'manual')
    assert c.accept(old, 10.8) is None
    c.observe(2, 11., P, (0, 0, 0), (0, 0, 0))
    assert c.state == 'manual_required_after_local_reset'
    assert not c.retry() and c.capture(11., 11.) is None


def test_request_epoch_map_and_scan_time_are_not_replaceable():
    c = controller()
    assert c.capture(9., 10.8) is None
    assert c.capture(10.9, 10.8) is None
    assert c.capture(10.75, 10.8)
    good = result(c)
    for key in ('request_id', 'map_sha256'):
        assert c.accept(dict(good, **{key:'other'}), 10.8) is None
        assert c.request is not None
    c.observe(2, 10.9, P, (0, 0, 0), (0, 0, 0))
    assert c.accept(good, 10.9) is None


def test_propagates_source_pose_to_latest_local_sample_once():
    c = controller(); c.capture(10.75, 10.8); good = result(c)
    c.observe(1, 10.9, Pose3((.02, 0, 0), P.orientation), (.02, 0, 0), (0, 0, 0))
    assert c.accept(good, 10.9).position == pytest.approx((3.02, 2., 0.))


def test_moving_during_search_and_stale_local_pose_cannot_seed():
    for moved in (False, True):
        c = controller(); c.capture(10.75, 10.8); good = result(c)
        if moved:
            c.observe(1, 10.9, Pose3((.2, 0, 0), P.orientation), (.5, 0, 0), (0, 0, 0))
        assert c.accept(good, 10.9 if moved else 12.) is None
        assert c.state == 'failed'


def test_stationary_requires_distinct_fresh_samples_and_no_history_gap():
    c = controller()
    c.observe(1, 10.8, P, (0, 0, 0), (0, 0, 0))
    assert c.stationary(10.8)
    assert not c.stationary(12.)
    d = StartupRelocalization('session')
    for t in (10., 10.7, 10.8):d.observe(1, t, P, (0, 0, 0), (0, 0, 0))
    assert not d.stationary(10.8)


def test_confirmation_timeout_stays_manual_and_no_retry_after_seed():
    c = controller();c.mark_seed('s', 10.8, 'global')
    c.verification('s', None, False, 26.)
    assert c.state == 'failed' and not c.retry()


def test_initial_search_retry_bounded_and_explicit():
    c = controller()
    for _ in range(2):
        assert c.capture(10.75, 10.8)
        c.fail('no_match')
        allowed = c.retry()
    assert not allowed and c.state == 'failed'


def test_repeated_corridor_or_floor_is_rejected_but_duplicate_tile_is_not():
    config = RegistrationConfig()
    a = candidate(); b = candidate(x=3.05)
    assert select_candidate([a, b], config)['accepted']
    for other in (candidate(z=3.5), candidate(x=20.)):
        decision = select_candidate([a, other], config)
        assert not decision['accepted'] and decision['reason'] == 'ambiguous_place_or_floor'
    rotated = candidate(); rotated['transform'][:2] = [[-1, 0, 0, 3], [0, -1, 0, 2]]
    assert not select_candidate([a, rotated], config)['accepted']


def test_one_plane_low_overlap_and_large_residual_are_not_initial_localization():
    for field, value in [('information_ratio', 0), ('inlier_ratio', .3), ('rmse_m', .4)]:
        a = candidate();a[field] = value
        assert not select_candidate([a], RegistrationConfig())['accepted']


def test_nonrigid_transform_rejected():
    c = controller();c.capture(10.75, 10.8);r = result(c)
    r['candidate']['transform'][0][0] = 1.2
    assert c.accept(r, 10.8) is None and c.state == 'failed'


@pytest.mark.parametrize('big_endian',[False,True])
def test_xyz_parser_respects_row_padding_and_endian(big_endian):
    b = bytearray(32)
    dtype = '>f4' if big_endian else '<f4'
    np.ndarray((2,3),dtype=dtype,buffer=b,strides=(16,4))[:]=[[1,2,3],[4,5,6]]
    m=SimpleNamespace(width=1,height=2,point_step=12,row_step=16,data=b,is_bigendian=big_endian,
        fields=[SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')])
    assert np.array_equal(pointcloud_xyz(m,10),[[1,2,3],[4,5,6]])
    m.row_step=8
    with pytest.raises(ValueError):pointcloud_xyz(m,10)


def test_unbounded_configuration_rejected():
    with pytest.raises(ValueError):replace(RegistrationConfig(),threads=20)
    with pytest.raises(ValueError):replace(StartupConfig(),max_source_age_s=3.)
    with pytest.raises(ValueError):replace(RegistrationConfig(),candidates=True)
