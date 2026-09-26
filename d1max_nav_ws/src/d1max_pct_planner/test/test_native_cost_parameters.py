"""Native cost knobs are explicit, bounded, propagated, and keep old defaults."""
from queue import Queue
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from d1max_pct_planner.planner_core import TomogramPlanner, validate_native_parameters
from d1max_pct_planner.tomogram_map import TomogramMap
from d1max_pct_planner import tomogram_route


@pytest.fixture
def fake_extensions(monkeypatch):
    class Native:
        def __init__(self, **kwargs):
            self.constructor = kwargs
        def init_map(self, *args):
            self.arguments = args
        def set_optimizer_sample_interval(self, interval):
            self.sample_interval = interval
    module = ModuleType('lib')
    module.a_star = SimpleNamespace()
    module.ele_planner = SimpleNamespace(OfflineElePlanner=Native)
    module.traj_opt = SimpleNamespace()
    monkeypatch.setitem(sys.modules, 'lib', module)
    return module


def payload():
    data = np.zeros((5, 1, 9, 9), dtype=np.float32)
    data[4] = 2.
    return {'data':data,'resolution':.2,'center':[0.,0.], 'slice_h0':.5,'slice_dh':.5}


@pytest.mark.parametrize('kwargs,expected', [({},(.2,15.)),
    ({'astar_cost_weight':1.,'optimizer_cost_margin':10.},(1.,10.)),
    ({'astar_cost_weight':10.,'optimizer_cost_margin':0.},(10.,0.))])
def test_native_init_map_receives_cost_knobs_and_fixed_hard_cutoff(fake_extensions, kwargs, expected):
    planner = TomogramPlanner('/unused', **kwargs)
    planner.load_payload(payload())
    args = planner.planner.arguments
    assert args[0] == 20 and args[1] == expected[1] and args[4] == expected[0]
    assert args[2:4] == (.2,1)
    assert planner.native_parameters['astar_cost_weight'] == expected[0]
    assert planner.native_parameters['optimizer_cost_margin'] == expected[1]
    assert planner.native_parameters['optimizer_cost_margin_units'] == 'cost_not_metres'


@pytest.mark.parametrize('name,value', [
    ('astar_cost_weight',0), ('astar_cost_weight',-1), ('astar_cost_weight',10.01),
    ('astar_cost_weight',float('inf')), ('astar_cost_weight',float('nan')),
    ('astar_cost_weight',True), ('astar_cost_weight','1'),
    ('optimizer_cost_margin',-1), ('optimizer_cost_margin',20),
    ('optimizer_cost_margin',float('inf')), ('optimizer_cost_margin',float('nan')),
    ('optimizer_cost_margin',False), ('optimizer_cost_margin','10'),
])
def test_invalid_knobs_rejected_before_native_import(name,value):
    with pytest.raises(ValueError, match=name):
        TomogramPlanner('/not/a/vendor', **{name:value})


def test_default_parameter_validation_keeps_legacy_values():
    assert validate_native_parameters() == {'astar_cost_weight':.2,'optimizer_cost_margin':15.}


def test_layered_route_constructor_passes_native_parameters(fake_extensions,monkeypatch):
    from d1max_pct_planner import native_runtime
    monkeypatch.setattr(native_runtime,'probe_native_libraries',lambda *args,**kwargs:{'runtime_verified':True})
    route = tomogram_route.TomogramRoute(TomogramMap(payload()),'/unused',
                                        astar_cost_weight=1.,optimizer_cost_margin=10.)
    assert route.planner.planner.arguments[1] == 10
    assert route.planner.planner.arguments[4] == 1
    assert route.native_parameters == route.planner.native_parameters


def test_metadata_probe_defers_full_map_but_still_verifies_abi(fake_extensions,monkeypatch):
    from d1max_pct_planner import native_runtime
    probes = []
    monkeypatch.setattr(native_runtime,'probe_native_libraries',
        lambda *args,**kwargs: probes.append(kwargs) or {'runtime_verified':True})
    route = tomogram_route.TomogramRoute(TomogramMap(payload()),'/unused',defer_map=True)
    assert probes == [{}, {'require_loaded':True}]
    assert not route._map_loaded
    assert not hasattr(route.planner,'planner')
    route._load_native_map()
    assert route._map_loaded
    native = route.planner.planner
    route._load_native_map()
    assert route.planner.planner is native


@pytest.mark.parametrize('value', [0, -1, 101, 1.5, 5.0, True, '5', float('nan')])
def test_invalid_optimizer_support_spacing_rejected_before_native_import(value):
    with pytest.raises(ValueError, match='optimizer_sample_interval'):
        TomogramPlanner('/not/a/vendor', optimizer_sample_interval=value)


@pytest.mark.parametrize('quintic', [False, True])
def test_support_spacing_sets_actual_planner_after_map_initialization(fake_extensions, quintic):
    planner = TomogramPlanner('/unused', use_quintic=quintic, optimizer_sample_interval=3)
    planner.load_payload(payload())
    assert planner.planner.sample_interval == 3
    assert planner.planner.arguments[0] == 20
    assert planner.native_parameters['optimizer_sample_interval'] == 3


def test_default_support_spacing_preserves_legacy_native_behavior(fake_extensions):
    planner = TomogramPlanner('/unused')
    planner.load_payload(payload())
    assert planner.native_parameters['optimizer_sample_interval'] == 10
    assert not hasattr(planner.planner, 'sample_interval')


def test_layered_route_passes_explicit_support_spacing(fake_extensions, monkeypatch):
    from d1max_pct_planner import native_runtime
    monkeypatch.setattr(native_runtime, 'probe_native_libraries', lambda *args, **kwargs: {})
    route = tomogram_route.TomogramRoute(TomogramMap(payload()), '/unused', optimizer_sample_interval=2)
    assert route.planner.planner.sample_interval == 2


@pytest.mark.parametrize('explicit',[False,True])
def test_layered_worker_uses_legacy_defaults_or_explicit_cost_knobs(tmp_path,monkeypatch,explicit):
    path=tmp_path/'map.npz'
    np.savez_compressed(path,**payload())
    captured={}
    class FakeRoute:
        def __init__(self, tomogram,vendor,**kwargs):
            captured.update(kwargs)
            self.native_runtime={'runtime_verified':True}
            self.native_parameters=validate_native_parameters(kwargs['astar_cost_weight'],kwargs['optimizer_cost_margin'])
    monkeypatch.setattr(tomogram_route,'TomogramRoute',FakeRoute)
    settings={'tomogram_path':str(path),'vendor_root':'/unused'}
    if explicit:settings.update(astar_cost_weight=1.,optimizer_cost_margin=10.)
    requests,results=Queue(),Queue();requests.put(None)
    tomogram_route.worker_main(settings,requests,results)
    ready=results.get_nowait()
    expected=(1.,10.) if explicit else (.2,15.)
    assert ready['kind']=='ready'
    assert captured['astar_cost_weight']==expected[0]
    assert captured['optimizer_cost_margin']==expected[1]
    assert ready['native_parameters']['astar_cost_weight']==expected[0]


def test_legacy_worker_also_honors_explicit_parameters(monkeypatch):
    from d1max_pct_planner import route_engine
    captured={}
    class Grid:
        def __init__(self,*args):self.sha256='grid';self.free=np.ones(1,dtype=bool)
        def payload(self):return payload()
    class Planner:
        def __init__(self,vendor,**kwargs):
            captured.update(kwargs)
            self.native_parameters=validate_native_parameters(kwargs['astar_cost_weight'],kwargs['optimizer_cost_margin'])
        def load_payload(self,p):pass
    monkeypatch.setattr(route_engine,'MeasuredGrid',Grid)
    monkeypatch.setattr(route_engine,'TomogramPlanner',Planner)
    requests,results=Queue(),Queue();requests.put(None)
    route_engine.worker_main({'planning_grid':'unused','cost_margin_m':.6,'minimum_clearance_m':.2,
        'optimization_guard_cells':1,'vendor_root':'unused','max_heading_rate':10,
        'astar_cost_weight':1.,'optimizer_cost_margin':10.},requests,results)
    assert results.get_nowait()['kind']=='ready'
    assert captured['astar_cost_weight']==1 and captured['optimizer_cost_margin']==10
