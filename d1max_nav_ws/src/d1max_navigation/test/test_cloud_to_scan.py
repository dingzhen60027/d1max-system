import math
import numpy as np
import pytest
from d1max_navigation.cloud_to_scan import project_points, projection_health


def test_nearest_ray_and_height_filter():
    scan = project_points([[2, 0, 0], [1, 0, .1], [0, 1, -.5], [0, 2, 1.2]], [0,0,0], [0,0,0,1])
    assert scan[360] == pytest.approx(1.)
    assert np.count_nonzero(np.isfinite(scan)) == 1


def test_unobserved_is_unknown_not_infinite_clearing_ray():
    scan = project_points([], [0,0,0], [0,0,0,1])
    assert np.isnan(scan).all() and not np.isinf(scan).any()


def test_rotation_and_translation_are_applied_once():
    scan = project_points([[1,0,0]], [0,1,0], [0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert scan[540] == pytest.approx(2.)


def test_nan_and_far_returns_rejected():
    scan = project_points([[np.nan,1,0],[20,0,0],[.05,0,0]], [0,0,0], [0,0,0,1])
    assert np.isnan(scan).all()


def test_bad_transform_rejected():
    with pytest.raises(ValueError): project_points([], [0,0,0], [0,0,0,0])


def test_low_obstacle_inside_stop_polygon_survives_while_ground_is_excluded():
    # Standing profile's .55 m reference height is an assumption, not a passed
    # calibration. Cloud is in a ground-origin frame, transformed to base once.
    low = [[.5, y, .20] for y in np.linspace(-.15, .15, 25)]
    ground = [[.4, y, 0.] for y in np.linspace(-.2, .2, 25)]
    scan = project_points(low + ground + [[3., 0., .6]], [0, 0, -.55], [0, 0, 0, 1])
    assert np.count_nonzero(np.isfinite(scan) & (scan < .6)) >= 20
    floor_only = project_points(ground, [0, 0, -.55], [0, 0, 0, 1])
    assert np.isnan(floor_only).all()


def test_ground_exclusion_does_not_filter_obstacles_above_its_bound():
    for height in [.101, .12, .20, .34]:
        scan = project_points([[.5, 0, height]], [0, 0, -.55], [0, 0, 0, 1])
        assert scan[360] == pytest.approx(.5)


def health(**changes):
    values = dict(received_monotonic=10., source_stamp=100., now_monotonic=10.1,
                  now_wall=100.1, max_cloud_age=.5, frame='base', finite_bins=1,
                  bins=720, error='', input_topic='/d1max/localization/lio/deskewed')
    values.update(changes)
    return projection_health(**values)


def test_fresh_all_nan_scan_is_not_valid_safety_evidence():
    ranges = project_points([], [0, 0, 0], [0, 0, 0, 1])
    value = health(finite_bins=int(np.isfinite(ranges).sum()), error='no_obstacle_slice_returns')
    assert value['source_fresh'] is True
    assert value['fresh'] is value['valid'] is value['has_obstacle_returns'] is False
    assert value['unknown_bins'] == 720
    assert value['coverage_verified'] is False
    assert value['unobserved_semantics'] == 'unknown_nan_not_clear'


@pytest.mark.parametrize('finite_bins', [1, 720])
def test_usable_endpoints_never_claim_certified_coverage(finite_bins):
    value = health(finite_bins=finite_bins)
    assert value['fresh'] is value['valid'] is True
    assert value['coverage_verified'] is False
    assert value['unknown_bins'] == 720-finite_bins


@pytest.mark.parametrize('changes', [
    {'source_stamp': 99.5}, {'source_stamp': 100.2},
    {'received_monotonic': 9.5}, {'received_monotonic': 10.2},
    {'received_monotonic': None, 'source_stamp': None},
    {'error': 'stale_cloud'}, {'error': 'TransformException'},
    {'finite_bins': 0}, {'finite_bins': 721}, {'finite_bins': True},
])
def test_stale_bad_transform_or_invalid_evidence_never_reports_fresh(changes):
    value = health(**changes)
    assert value['fresh'] is value['valid'] is False


def test_relabelled_raw_topic_does_not_claim_raw_ray_contract():
    value = health(input_topic='/front_lidar')
    assert value['source'] == 'endpoint_cloud_projection'
    assert value['coverage_verified'] is False


@pytest.mark.parametrize('ranges,stamp_fresh,expected', [
    ([float('nan')]*720, True, False),
    ([float('inf')]*720, True, False),
    ([.1]*720, True, False),
    ([2.]+[float('nan')]*719, True, True),
    ([2.]*720, False, False),
])
def test_actual_command_gate_scan_callback_preserves_unknown_rejection(ranges, stamp_fresh, expected):
    # Compile the production callback without creating its ROS node or SDK
    # clients. This proves the real gate still rejects an all-unknown scan;
    # a positive result is endpoint usability only, never full-space coverage.
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    import time
    from d1max_navigation import command_gate

    tree = ast.parse(Path(command_gate.__file__).read_text())
    callback = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == 'scan')
    code = compile(ast.fix_missing_locations(ast.Module(body=[callback], type_ignores=[])),
                   command_gate.__file__, 'exec')
    namespace = {'finite': command_gate.finite, 'time': time}
    exec(code, namespace)
    observations = []
    subject = SimpleNamespace(gate=SimpleNamespace(config=SimpleNamespace(scan_timeout=.6),
        observe=lambda *args: observations.append(args)), stamp_fresh=lambda *args: stamp_fresh)
    message = SimpleNamespace(header=SimpleNamespace(stamp=None, frame_id='base'),
                              ranges=ranges, range_min=.15, range_max=12., angle_increment=.01)
    namespace['scan'](subject, message)
    assert observations[0][0] == 'scan'
    assert observations[0][1] == {'valid': expected, 'frame': 'base'}
