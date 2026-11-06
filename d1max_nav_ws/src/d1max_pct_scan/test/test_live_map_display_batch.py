"""Display-only batching and first-green delivery; no ROS/SDK initialization."""
import ast
from array import array
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from d1max_pct_planner.stair_model_geometry import ParametricStairModel
from d1max_pct_scan.live_map_geometry import map_surfaces, colored_records
from d1max_pct_scan.pointcloud_helpers.ground_path_bridge import ParametricGroundPathBridge


def model_record():
    surface = dict(surface_id='a', source_surface_id=1, segment_ids=['stairs'],
        kind='continuous_support_envelope', frame=dict(origin_xyz=[0., 0., 0.],
            axis_u=[1., 0., 0.], axis_v=[0., 1., 0.], axis_up=[0., 0., 1.]),
        polygon_xy=[[0., -.7], [1., -.7], [1., .7], [0., .7]],
        support_grade=.3, support_intercept_m=0., contact_tolerance_m=0.,
        ground_replacement_band_m=.25, static_terrain_band_m=.08,
        static_stair_interior_assumed_clear=True, tread_stations_m=[], tread_heights_m=[])
    return dict(schema='d1max.parametric_stair_geometry/v1', frame_id='d1max_loc_map',
        source_map_sha256='a'*64, planning_map_sha256='b'*64,
        model_points_are_observations=False, surfaces=[surface], obstacles=[])


class Original:
    source_frame = 'd1max_loc_map'
    planning_frame = 'd1max_multifloor_planning'
    protected_regions = [{'min': [0., -.7, -.5], 'max': [2., .7, 1.]}]
    stair_mapping = None
    floors = {'floor1': SimpleNamespace(reference_z_m=0.),
              'floor2': SimpleNamespace(reference_z_m=3.)}
    limits = SimpleNamespace(max_ground_residual_m=.25)

    def to_localization_ground(self, points, labels):
        # A real unsupported-cell failure must still be isolated, not filled.
        if (np.asarray(points)[:, 0] > 2.).any() or not np.isfinite(points).all():
            raise ValueError('unsupported')
        xyz = np.array(points, copy=True)
        xyz[:, 2] += .04
        return SimpleNamespace(xyz=xyz, diagnostics={})


class ScalarFacade:
    """Force the unchanged strict route method (the pre-fix display behavior)."""
    def __init__(self, bridge): self.bridge = bridge
    def __getattr__(self, name): return getattr(self.bridge, name)


def compare(points, record=None, kind='traversable'):
    bridge = ParametricGroundPathBridge(Original(), ParametricStairModel(record or model_record()), 'e'*64)
    settings = {'stair_roi': ([-3., -2., -.5], [3., 2., 3.5])}
    old = map_surfaces(points, ScalarFacade(bridge), settings, surface_kind=kind)
    new = map_surfaces(points, bridge, settings, surface_kind=kind)
    for before, after in zip(old, new): np.testing.assert_array_equal(before, after)
    costs = np.linspace(0., 20., len(points))
    np.testing.assert_array_equal(colored_records(old[0], costs[old[1]]),
                                  colored_records(new[0], costs[new[1]]))
    return new


@pytest.mark.parametrize('kind', ['traversable', 'blocked'])
def test_batch_retains_each_ground_cell_color_height_and_unsupported_omission(kind):
    x = np.linspace(-.5, 2.5, 41)
    points = np.c_[x, np.zeros(len(x)), np.clip(.3*x, 0., .6)]
    old = points.copy()
    _, ids = compare(points, kind=kind)
    assert len(ids) < len(points)
    np.testing.assert_array_equal(points, old)


@pytest.mark.parametrize('overlap', ['seam', 'interior', 'different_height', 'other_floor'])
def test_shared_seam_and_ambiguous_or_other_floor_keep_scalar_contract(overlap):
    record = model_record()
    extra = deepcopy(record['surfaces'][0]); extra['surface_id'] = 'b'
    extra['source_surface_id'] = 2
    if overlap == 'seam': extra['polygon_xy'] = [[1., -.7], [2., -.7], [2., .7], [1., .7]]
    elif overlap == 'different_height': extra['support_intercept_m'] = .1
    elif overlap == 'other_floor': extra['support_intercept_m'] = 3.
    record['surfaces'].append(extra)
    points = np.array([[.5, 0., .15], [1., 0., .3], [1.5, 0., .45],
                       [.5, 0., 3.15], [2.5, 0., .0]])
    compare(points, record)


def test_nonfinite_independent_cell_does_not_poison_supported_neighbors():
    compare(np.array([[.5, 0., .15], [float('nan'), 0., 0.], [-.1, 0., 0.]]))


def test_empty_cloud_remains_empty():
    xyz, ids = compare(np.empty((0, 3)))
    assert xyz.shape == (0, 3) and ids.size == 0


@pytest.mark.parametrize('yaw', [0., .7, -1.2])
def test_rotated_domains_height_bands_and_polygon_edges_match_scalar(yaw):
    record = model_record(); surface = record['surfaces'][0]
    c, s = np.cos(yaw), np.sin(yaw)
    surface['frame']['axis_u'] = [c, s, 0.]
    surface['frame']['axis_v'] = [-s, c, 0.]
    # Include points on both sides of exact height and polygon boundaries.
    local = np.array([[x, y, .3*x+height] for x in (0., .5, 1.)
        for y in (-.7-1e-8, -.7, -.7+1e-8, 0., .7, .7+1e-8)
        for height in (-.25000001, -.25, .24999999, .25, .25000001)])
    points = local.copy(); points[:, :2] = local[:, :2] @ np.array([[c, s], [-s, c]])
    compare(points, record)


def test_recursive_rejection_does_not_repeat_scalar_model_classification(monkeypatch):
    bridge = ParametricGroundPathBridge(Original(), ParametricStairModel(model_record()), 'e'*64)
    def forbidden(*args, **kwargs): raise AssertionError('point-by-point model query repeated')
    monkeypatch.setattr(bridge, 'model_surface_at', forbidden)
    # Alternating unsupported cells cause many strict-bridge recursive splits.
    points = np.tile([[.2, 0., .06], [2.5, 0., 0.]], (200, 1))
    xyz, ids = map_surfaces(points, bridge, {'stair_roi': ([-3., -2., -.5], [3., 2., 3.5])})
    assert len(xyz) == 200 and np.array_equal(ids, np.arange(0, 400, 2))


def test_real_node_publishes_green_before_red_work_and_republishes_both(tmp_path):
    source = Path(__file__).resolve().parents[1]/'d1max_pct_scan/live_map_layers.py'
    tree = ast.parse(source.read_text())
    node_code = ast.get_source_segment(source.read_text(), next(n for n in tree.body if isinstance(n, ast.ClassDef)))
    events = []
    class Node:
        def __init__(self, *args): pass
        def create_publisher(self, typ, topic, qos):
            return SimpleNamespace(publish=lambda msg: events.append((topic, msg)))
        def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 123))
        def create_timer(self, period, callback): self.timer = callback
        def get_logger(self): return SimpleNamespace(info=lambda value: None)
    class PointField(SimpleNamespace): FLOAT32 = 7
    def build(session, *, on_traversable):
        xyz = np.array([[0., 0., 0.]])
        on_traversable(xyz, np.array([0.]), {'traversable_cells': 1})
        assert [topic.rsplit('/', 1)[-1] for topic, _ in events] == ['map_layer_status', 'traversable_surface']
        assert 'blocked_cells' not in json.loads(events[0][1].data)
        events.append(('red_work', None))
        return xyz, np.array([0.]), xyz, {'traversable_cells': 1, 'blocked_cells': 1}
    namespace = dict(Node=Node, build_layers=build, colored_records=colored_records,
        json=json, array=array, PointCloud2=SimpleNamespace, Header=SimpleNamespace,
        String=SimpleNamespace, PointField=PointField, QoSProfile=SimpleNamespace,
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1), DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1))
    exec(compile(node_code, str(source), 'exec'), namespace)
    (tmp_path/'session.json').write_text(json.dumps({'id': 'task', 'frame_id': 'map'}))
    node = namespace['LiveMapLayers'](tmp_path)
    assert [topic.rsplit('/', 1)[-1] for topic, _ in events] == [
        'map_layer_status', 'traversable_surface', 'red_work', 'blocked_surface', 'map_layer_status']
    green = events[1][1]
    assert green.width == 1 and green.header.frame_id == 'map' and green.header.stamp == 123
    node.timer()
    assert [topic.rsplit('/', 1)[-1] for topic, _ in events[-2:]] == ['traversable_surface', 'blocked_surface']
