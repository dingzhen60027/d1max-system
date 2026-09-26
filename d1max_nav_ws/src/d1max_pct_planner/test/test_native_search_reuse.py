"""Offline regressions for native reuse: no ROS, SDK, or robot control.

The three integer-path digests were captured from the pre-optimization binary
on 2026-09-25, not generated from the implementation under test. They protect
neighbor order, cost preferences, and path geometry while reset becomes sparse.
"""
from pathlib import Path
import subprocess
import sys

import pytest

from d1max_pct_planner.native_runtime import prepare_native_environment


VENDOR = Path(__file__).resolve().parents[2] / 'pct_planner_vendor'
pytestmark = pytest.mark.skipif(
    not (VENDOR / 'planner/lib/ele_planner.cpython-310-x86_64-linux-gnu.so').is_file(),
    reason='Native PCT extension is not built')

PRELUDE = r'''
import gc
import hashlib
import numpy as np
import sys
sys.path.insert(0, str(VENDOR + '/planner'))
from lib import a_star
from d1max_pct_planner.planner_core import TomogramPlanner
from d1max_pct_planner.native_runtime import probe_native_libraries

def fixture_cost():
    cost = np.zeros((61, 61))
    cost[10:51, 30] = 100.
    cost[35:40, 30] = 0.
    cost[10:25, 40:43] = 15.
    return cost

def make_search(cost=None):
    if cost is None:
        cost = fixture_cost()
    p = a_star.Astar()
    p.init(20, 1, .1, 1., cost, np.zeros_like(cost), np.zeros_like(cost))
    return p

def make_planner():
    data = np.zeros((5, 1, 61, 61), np.float32)
    data[0, 0] = fixture_cost()
    data[4] = 3.
    p = TomogramPlanner(VENDOR, ground_z=True, astar_cost_weight=1.)
    p.load_payload(dict(data=data, resolution=.1, center=np.zeros(2),
                        slice_h0=.5, slice_dh=.5))
    assert probe_native_libraries(VENDOR, require_loaded=True)['runtime_verified']
    return p

def digest(matrix):
    return hashlib.sha256(np.asarray(matrix, dtype=np.float64).tobytes()).hexdigest()
'''


def run_native(script):
    result = subprocess.run(
        [sys.executable, '-c', 'VENDOR=' + repr(str(VENDOR)) + '\n' + PRELUDE + script],
        env=prepare_native_environment(VENDOR), capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_sparse_reset_matches_old_geometry_and_touches_only_search_nodes():
    run_native(r'''
cases = [
    ([0,4,4], [0,56,50], '05288334fa66208e9bb7d69b081b9e70fbe8662189adac7a7f37da3ca5ac8181'),
    ([0,4,4], [0,56,4], 'fced6a15f00004b6743e995442ca1d0d9f3be7fbdfe58f66d769d50a4f308472'),
    ([0,56,50], [0,4,4], 'f0ab1921fc3c0f8b6642cab7e9e9e103870bae455b01154a9ba717390092aab4'),
]
p = make_search()
touched = 0
for start, goal, expected in cases * 3:
    assert p.search(start, goal)
    assert p.get_last_reset_count() == touched
    assert digest(p.get_result_matrix()) == expected
    touched = p.get_touched_count()
    assert 0 < touched < 61*61
    assert p.get_result_size() == len(p.get_result_matrix())
assert p.get_search_count() == 9
stats = p.get_memory_stats()
assert stats['dense_cells'] == 61*61
assert stats['allocated_query_nodes'] == 0
assert stats['last_query_nodes'] < stats['dense_cells']
assert stats['owned_dense_grid_bytes'] == 61*61*3*8
assert not stats['grid_shared_with_optimizer']
''')


def test_failed_search_does_not_poison_reuse_or_leave_old_result():
    run_native(r'''
cost = np.zeros((61, 61)); cost[:, 30] = 100.
p = make_search(cost)
assert p.search([0,4,4], [0,20,50])
assert not p.search([0,4,4], [0,56,50])
assert p.get_result_size() == 0
touched = p.get_touched_count()
assert touched > 0
assert p.get_memory_stats()['allocated_query_nodes'] == 0
assert p.search([0,4,4], [0,20,50])
assert p.get_last_reset_count() == touched
fresh = make_search(cost)
assert fresh.search([0,4,4], [0,20,50])
assert np.array_equal(p.get_result_matrix(), fresh.get_result_matrix())
# A blocked-goal early return must also clear previous results/state.
assert not p.search([0,4,4], [0,30,10])
assert p.get_result_size() == 0
assert p.search([0,4,4], [0,20,50])
assert np.array_equal(p.get_result_matrix(), fresh.get_result_matrix())
''')


def test_short_paths_guarded_natively_and_bounds_fail_without_crash():
    run_native(r'''
p = make_planner()
for start, goal in [([0,4,4], [0,4,4]), ([0,4,4], [0,5,4])]:
    assert p.planner.plan(start, goal, False)
    try:
        p.planner.plan(start, goal, True)
    except ValueError as exc:
        assert 'too close' in str(exc)
    else:
        raise AssertionError('short path must not enter GPMP')
for point in ([-1,4,4], [1,4,4], [0,61,4], [0,4,61], [0,-1,4]):
    try:
        p.planner.plan(point, [0,4,4], True)
    except IndexError:
        pass
    else:
        raise AssertionError('out of bounds endpoint was accepted')
for details in (False, True):
    same = p.plan([-2.6,-2.6], [-2.6,-2.6], return_details=details)
    assert (same['path'] if details else same).shape == (2,3)
    try:
        p.plan([-2.6,-2.6], [-2.5,-2.6], return_details=details)
    except ValueError as exc:
        assert 'too close' in str(exc)
    else:
        raise AssertionError('adjacent Python query must raise safely')
assert p.plan([-2.6,-2.6], [2.,2.6], return_details=True) is not None
''')


def test_single_search_details_and_reference_lifetime_no_grid_copy():
    run_native(r'''
p = make_planner()
finder = p.planner.get_path_finder()
assert finder is p.planner.get_path_finder()
optimizer = p.planner.get_trajectory_optimizer_wnoj()
assert optimizer is p.planner.get_trajectory_optimizer_wnoj()
before = finder.get_search_count()
result = p.plan([-2.6,-2.6], [2.,2.6], return_details=True)
assert finder.get_search_count() == before + 1
# An unchanged retained getter sees new results: it cannot be a copied grid.
assert finder.get_result_size() == p.planner.get_search_result_size() == 57
stats = finder.get_memory_stats()
assert stats['grid_shared_with_optimizer']
assert stats['owned_dense_grid_bytes'] == 0
assert stats['allocated_query_nodes'] == 0
assert 0 < stats['last_query_nodes'] < stats['dense_cells']
assert len(result['path']) == 46
assert digest(result['path']) == 'ad4bc1e1e985e4b94b088cb2ee18cab200749b93cd42e8834cded107cae81d0f'
assert abs(result['native_sample_dt'] - .41481481481481486) < 1e-14
path_copy = finder.get_result_matrix().copy()
del p
gc.collect()
# reference_internal pins the owning planner even after Python drops its name.
assert np.array_equal(finder.get_result_matrix(), path_copy)
assert np.isfinite(optimizer.get_result_matrix()).all()
''')


def test_reinitialize_discards_node_pointers_and_map_shape_is_checked():
    run_native(r'''
p = make_search()
assert p.search([0,4,4], [0,56,50])
small = np.zeros((11, 17))
p.init(20, 1, .1, 1., small, small, small)
assert p.get_search_count() == p.get_touched_count() == p.get_result_size() == 0
assert p.search([0,1,1], [0,15,9])
assert p.get_last_reset_count() == 0
try:
    p.init(20, 0, .1, 1., small, small, small)
except ValueError:
    pass
else:
    raise AssertionError('zero layer count was accepted')
assert p.search([0,1,1], [0,15,9])
''')


def test_sparse_nodes_preserve_gateway_and_height_quantization():
    run_native(r'''
height = np.zeros((2,31,41))
height[0] = np.linspace(-.2,.6,41)[None,:]
height[1] = height[0]
height[1,:,21:] += .05
cost = np.zeros_like(height)
cost[0,:,22:] = 100
cost[1,:,:20] = 100
gateway = np.zeros_like(height)
gateway[0,:,20] = 2
gateway[1,:,20] = -2
p = a_star.Astar()
p.init(20,2,.1,1.,cost.reshape(-1,41),height.reshape(-1,41),gateway.reshape(-1,41))
cases = [
    ([0,2,15],[1,38,15], '640e4c7d9cf682cb001b7a377c03bf8c0b6a84af79eee9275fbcd52af16f250c'),
    ([1,38,15],[0,2,15], 'b562b57e4b0ce949283c07ea7fb46ba5498d7652c8015b59a764fc0695d5d394'),
]
for start,goal,expected in cases * 3:
    assert p.search(start,goal)
    assert digest(p.get_result_matrix()) == expected
assert np.array_equal(p.get_cost_layer(1),cost[1])
assert np.array_equal(p.get_ele_layer(0),gateway[0])
''')


def test_finished_large_failed_query_releases_nodes_and_bucket_capacity():
    run_native(r'''
cost = np.zeros((151,151)); cost[:,100] = 100
p = make_search(cost)
assert not p.search([0,4,4],[0,140,140])
stats = p.get_memory_stats()
assert stats['last_query_nodes'] > 10000
assert stats['allocated_query_nodes'] == 0
assert stats['retained_query_buckets'] < 4096
assert stats['retained_path_nodes'] == 0
assert p.search([0,4,4],[0,8,4])
stats = p.get_memory_stats()
assert stats['last_query_nodes'] < 100
assert stats['peak_query_nodes'] > 10000
assert stats['allocated_query_nodes'] == 0
''')


def test_forty_seeded_random_maps_match_dense_native_baseline():
    run_native(r'''
# Baseline generated from the dense native implementation before this storage
# change; includes 14 genuinely unreachable and 26 successful height-varying
# cost maps. Reinitialize one sparse planner instead of constructing 40 maps.
rng = np.random.default_rng(20260925)
combined = hashlib.sha256()
successes = 0
p = a_star.Astar()
for index in range(40):
    cost = rng.choice([0.,6.,15.,100.],size=(37,53),p=[.7,.1,.05,.15])
    rows,cols = np.indices(cost.shape)
    height = rows*.006 + cols*.003
    gateway = np.zeros_like(cost)
    start = np.array([0,3,int(rng.integers(2,35))])
    goal = np.array([0,49,int(rng.integers(2,35))])
    cost[start[2],start[1]] = cost[goal[2],goal[1]] = 0
    if index % 3 == 0:
        cost[:,26] = 100
    p.init(20,1,.1,1.,cost,height,gateway)
    okay = p.search(start,goal)
    successes += okay
    combined.update(bytes([okay]))
    combined.update(np.asarray(p.get_result_matrix(),dtype=np.float64).tobytes())
    assert p.get_memory_stats()['allocated_query_nodes'] == 0
assert successes == 26
assert combined.hexdigest() == 'f07e8aab0efbdaea0d1a7db1b8977dcdb447452880269b01e149c6056b1fd314'
''')


@pytest.mark.parametrize('layout', ['C', 'F', 'strided', 'readonly_C', 'readonly_strided', 'negative'])
def test_native_map_import_layouts_copy_once_into_independent_owned_storage(layout):
    run_native('layout = ' + repr(layout) + '\n' + r'''
from lib import ele_planner, traj_opt

def with_layout(source):
    if layout in ('strided', 'readonly_strided'):
        storage = np.empty((source.shape[0]*2, source.shape[1]*3), dtype=np.float64)
        value = storage[::2, ::3]
        value[:] = source
    elif layout == 'negative':
        storage = np.empty_like(source, dtype=np.float64)
        value = storage[::-1, ::-1]
        value[:] = source
    else:
        value = np.array(source, dtype=np.float64, order='F' if layout == 'F' else 'C')
    if layout.startswith('readonly_'):
        value.flags.writeable = False
    return value

cost = fixture_cost()
zero = np.zeros_like(cost)
inputs = [with_layout(value) for value in (cost, zero, zero+3, zero, zero, -zero)]
p = ele_planner.OfflineElePlanner(10., True)
p.init_map(20., 15., .1, 1, 1., *inputs)
finder = p.get_path_finder()
np.testing.assert_array_equal(finder.get_cost_layer(0), cost)
# All import views are borrowed only for this synchronous call. Mutating and
# releasing every Python buffer afterward must not alter native map contents.
for value in inputs:
    value.flags.writeable = True
    value[:] = 999.
del inputs, value
gc.collect()
np.testing.assert_array_equal(finder.get_cost_layer(0), cost)
assert p.plan([0,4,4],[0,56,50],True)
assert digest(p.get_debug_path()) == '05288334fa66208e9bb7d69b081b9e70fbe8662189adac7a7f37da3ca5ac8181'
assert digest(p.get_trajectory_optimizer_wnoj().get_result_matrix()) == '6b8f267c60ae372fc7dc068057079fbfcab1b4f809d53cf2cf44837064a52114'

# Replace the map with another shape, retaining the same borrowed A* handle.
small = np.zeros((17,23), dtype=np.float64)
second = [with_layout(value) for value in (small,small,small+3,small,small,-small)]
p.init_map(20.,15.,.1,1,1.,*second)
del second
gc.collect()
assert finder.get_cost_layer(0).shape == (17,23)
assert p.plan([0,2,2],[0,20,14],False)
assert finder.get_result_size() > 1
assert finder.get_memory_stats()['grid_shared_with_optimizer']

# Invalid new import is rejected before replacing the existing valid map.
try:
    p.init_map(20.,15.,.1,1,1.,small,small[:3],small+3,small,small,small)
except ValueError:
    pass
else:
    raise AssertionError('mismatched map shape was accepted')
assert p.plan([0,2,2],[0,20,14],False)
''')


def test_old_map_handle_owns_its_snapshot_across_reinit_and_parent_deletion():
    run_native(r'''
from lib import ele_planner, py_map_manager, traj_opt
p = ele_planner.OfflineElePlanner(10.,True)
cost = np.zeros((22,11),dtype=np.float64)
cost[:11] = 20.
zero = np.zeros_like(cost)
p.init_map(20.,15.,.1,2,1.,cost,zero,zero+3,zero,zero,zero)
old_map = p.get_map()
assert old_map.update_layer_safe(0,3.,3.,0.) == 1
small = np.zeros((7,9),dtype=np.float64)
p.init_map(20.,15.,.1,1,1.,small,small,small+3,small,small,small)
new_map = p.get_map()
assert new_map is not old_map
assert new_map.update_layer_safe(0,3.,3.,0.) == 0
del p, cost, zero, small
gc.collect()
# Unlike a reference_internal view, shared ownership pins the old allocation,
# not merely the already-reinitialized/deleted OfflineElePlanner object.
assert old_map.update_layer_safe(0,3.,3.,0.) == 1
assert new_map.update_layer_safe(0,3.,3.,0.) == 0
''')
