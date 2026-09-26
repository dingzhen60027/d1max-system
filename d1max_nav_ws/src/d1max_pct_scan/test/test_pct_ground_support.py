"""Pure offline centreline support tests; no ROS, SDK or robot control."""
import hashlib

import numpy as np
import pytest

from d1max_pct_scan.pct_ground_support import (
    GroundSupportError, PCTGroundSupport, write_support_index,
)


SOURCE = hashlib.sha256(b'conditioned-pcd').hexdigest()
TOMOGRAM = hashlib.sha256(b'exact-tomogram').hexdigest()
META = dict(frame_id='d1max_loc_map', resolution=.1,
            source_pcd_sha256=SOURCE, source_tomogram_sha256=TOMOGRAM)


def grid_points(*, missing=(), upper=False, different_step=False):
    points = []
    for x in range(6):
        for y in range(3):
            if (x, y) not in missing:
                points.append((x*.1, y*.1, .18 if different_step and x >= 3 else 0.))
                if upper:
                    points.append((x*.1, y*.1, 3.2))
    return np.asarray(points)


def line(z=.55):
    return np.array([[.05, .1, z], [.10, .1, z], [.15, .1, z],
                     [.20, .1, z], [.25, .1, z], [.30, .1, z],
                     [.35, .1, z], [.40, .1, z], [.45, .1, z]])


def test_valid_support_and_second_floor_are_disjoint():
    support = PCTGroundSupport.from_arrays(grid_points(upper=True), META)
    low = support.validate_body_samples(line())
    upper = support.validate_body_samples(line(3.75))
    assert low['checked_support_cells'] >= 5
    assert upper['centreline_support_only'] is True
    assert not low['motion_authorized']
    with pytest.raises(GroundSupportError, match='wrong_height_or_floor'):
        support.validate_body_samples(line(1.8))


def test_exact_cell_hole_is_not_filled_from_neighbour():
    support = PCTGroundSupport.from_arrays(grid_points(missing={(3, 1)}), META)
    with pytest.raises(GroundSupportError, match='hole_or_blocked_cell'):
        support.validate_body_samples(line())


def test_unsupported_boundary_cell_is_not_skipped():
    support = PCTGroundSupport.from_arrays(grid_points(missing={(3, 2)}), META)
    boundary = line().copy()
    boundary[:, 1] = .15
    with pytest.raises(GroundSupportError, match='hole_or_blocked_cell'):
        support.validate_body_samples(boundary)


def test_step_and_height_are_independent_guards():
    support = PCTGroundSupport.from_arrays(grid_points(different_step=True), META)
    with pytest.raises(GroundSupportError, match='ground_step'):
        support.validate_body_samples(line(), height_tolerance_m=.2)
    with pytest.raises(GroundSupportError, match='wrong_height_or_floor'):
        support.validate_body_samples(line(), height_tolerance_m=.05)


def test_sampling_gap_is_rejected_even_over_valid_cells():
    support = PCTGroundSupport.from_arrays(grid_points(), META)
    with pytest.raises(GroundSupportError, match='sampling_too_sparse'):
        support.validate_body_samples(np.array([[.05, .1, .55], [.45, .1, .55]]))


def test_hash_pinned_roundtrip_and_source_mismatch(tmp_path):
    path = tmp_path / 'support.npz'
    digest = write_support_index(path, grid_points(upper=True), META)
    support = PCTGroundSupport.from_npz(path, expected_sha256=digest,
        expected_source_pcd_sha256=SOURCE, expected_tomogram_sha256=TOMOGRAM)
    assert support.validate_body_samples(line())['support_index_sha256'] == digest
    with pytest.raises(GroundSupportError, match='provenance_mismatch'):
        PCTGroundSupport.from_npz(path, expected_sha256=digest,
            expected_source_pcd_sha256='0'*64, expected_tomogram_sha256=TOMOGRAM)
    with pytest.raises(GroundSupportError, match='hash_mismatch'):
        PCTGroundSupport.from_npz(path, expected_sha256='0'*64,
            expected_source_pcd_sha256=SOURCE, expected_tomogram_sha256=TOMOGRAM)


def stacked_stair_loop():
    points = []
    for ix in range(65):
        x = ix*.1
        if ix <= 32:
            points.append((x, .1, 0.))
        if ix >= 32:
            points.append((x, .1, round((ix-32)*.1, 8)))
        points.append((x, .1, 3.2))
    lower = np.column_stack((np.arange(0, 3.2+.01, .05),
                             np.full(65, .1), np.full(65, .55)))
    stair_x = np.arange(3.2, 6.4+.01, .05)
    stairs = np.column_stack((stair_x, np.full(len(stair_x), .1),
                              .55 + stair_x-3.2))
    upper_x = np.arange(6.4, -.01, -.05)
    upper = np.column_stack((upper_x, np.full(len(upper_x), .1),
                             np.full(len(upper_x), 3.75)))
    return np.asarray(points), np.vstack((lower, stairs[1:], upper[1:]))


def test_legal_stair_route_can_revisit_same_xy_on_another_floor_both_directions():
    points, route = stacked_stair_loop()
    support = PCTGroundSupport.from_arrays(points, META)
    forward = support.validate_body_samples(route)
    reverse = support.validate_body_samples(route[::-1])
    assert forward['checked_support_cells'] > 50
    assert reverse['maximum_adjacent_ground_step_m'] <= .17


def test_real_vertical_jump_does_not_pass_a_stacked_floor_cell():
    points, route = stacked_stair_loop()
    support = PCTGroundSupport.from_arrays(points, META)
    bad = route.copy()
    bad[2, 2] = 3.75
    with pytest.raises(GroundSupportError, match='sampling_too_sparse|ground_step|wrong_height'):
        support.validate_body_samples(bad)
