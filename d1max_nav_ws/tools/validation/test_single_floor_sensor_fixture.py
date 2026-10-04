"""Only analytic sensor geometry tests; not a substitute for native GridMap."""
import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('fixture',Path(__file__).with_name('run_single_floor_graph.py'))
fixture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


def test_box_surface_not_bounding_plane():
    origins=np.array([[0.,0.,.5],[1.,0.,.5],[0.,-1.6,.5],[0.,0.,1.]])
    directions=np.array([[0.,-1.,0.]]*4)
    result=fixture.box_returns(origins,directions,np.array([-.18,-1.8,-.05]),np.array([.18,-1.4,.65]))
    np.testing.assert_allclose(result[[0,2]],[1.4,.2])
    assert np.isinf(result[[1,3]]).all()


def test_box_behind_ray_is_not_return():
    result=fixture.box_returns(np.array([[0.,0.,.5]]),np.array([[0.,1.,0.]]),
        np.array([-.18,-1.8,-.05]),np.array([.18,-1.4,.65]))
    assert np.isinf(result).all()
