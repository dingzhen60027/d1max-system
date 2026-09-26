import numpy as np
import pytest

from pointcloud_preprocessing.structural_obstacles import filter_structural_obstacles


def floor_scene():
    x,y=np.meshgrid(np.arange(0,4.01,.04),np.arange(-1.0,1.001,.04),indexing='ij')
    return np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]


def run(extra, cfg=None):
    floor=floor_scene();points=np.vstack((floor,np.asarray(extra,dtype=float)))
    config={'enabled':True};config.update(cfg or {})
    return points,filter_structural_obstacles(points,config),len(floor)


def test_disabled_stage_is_exactly_nonmutating():
    p=floor_scene();before=p.copy();result=filter_structural_obstacles(p)
    assert result['keep_mask'].all()
    assert not result['removed_mask'].any()
    assert np.array_equal(p,before)


def test_sparse_floating_upper_returns_do_not_mutually_protect_lower_returns():
    floating=np.array([[1.0,.02,.16],[1.01,.01,.18],[1.02,.03,.45],[1.03,.01,.54],[1.00,.01,.63]])
    p,r,n=run(floating)
    assert r['removed_mask'][n:].all()
    assert r['keep_mask'][:n].all()
    assert not r['wall_protected_mask'][n:].any()


def test_large_vertical_wall_remains_continuous():
    y,z=np.meshgrid(np.arange(-.8,.801,.04),np.arange(.06,1.301,.04))
    wall=np.c_[np.full(y.size,2.),y.ravel(),z.ravel()]
    p,r,n=run(wall)
    assert r['keep_mask'][n:].all()
    assert r['statistics']['trusted_structure_points']>100
    assert r['statistics']['removed_observed_floor_points']==0


def test_dense_continuous_thin_post_is_preserved_without_large_planar_patch():
    angle,z=np.meshgrid(np.linspace(0,2*np.pi,9)[:-1],np.arange(.04,1.25,.05))
    pole=np.c_[2+.025*np.cos(angle.ravel()),.3+.025*np.sin(angle.ravel()),z.ravel()]
    p,r,n=run(pole)
    assert r['keep_mask'][n:].all()
    assert r['statistics']['thin_post_columns']>0


def test_coherent_low_box_top_and_lower_side_are_preserved():
    x,y=np.meshgrid(np.arange(.94,1.061,.06),np.arange(-.06,.061,.06))
    top=np.c_[x.ravel(),y.ravel(),np.full(x.size,.16)]
    box=np.vstack((top,[1.0,0.,.06]))
    p,r,n=run(box)
    assert r['keep_mask'][n:].all()
    assert r['low_object_protected_mask'][n:].all()


def test_unknown_columns_are_never_cleared_or_filled():
    p,r,n=run([[1.,1.5,.16],[20.,20.,.50]])
    assert r['keep_mask'][n:].all()
    assert not r['support_mask'][n:].any()
    assert len(r['keep_mask'])==len(p)


def test_candidate_height_bound_does_not_erase_overhead_points():
    p,r,n=run([[1.,0.,.20],[1.,0.,.80],[1.,0.,2.6]])
    assert r['removed_mask'][n]
    assert r['keep_mask'][n+1:].all()


def test_source_order_and_floor_are_preserved():
    p=np.vstack((floor_scene(),[[1.,.01,.17],[2.,.01,.32]]))
    rng=np.random.default_rng(52);p=p[rng.permutation(len(p))];before=p.copy()
    r=filter_structural_obstacles(p,{'enabled':True})
    assert np.array_equal(p,before)
    assert r['keep_mask'][p[:,2]==0].all()
    assert np.array_equal(~r['keep_mask'],r['removed_mask'])


@pytest.mark.parametrize('cfg',[
    {'bogus':1}, {'candidate_min_height_m':.02}, {'normal_neighbors':2},
    {'enabled':1}, {'support_radius_m':0}, {'floor_reference_z_m':np.nan},
    {'horizontal_plane_min_inlier_ratio':1.1},
])
def test_bad_configuration_rejected(cfg):
    with pytest.raises(ValueError):filter_structural_obstacles(floor_scene(),cfg)


def test_no_observed_floor_refuses_enabled_cleanup():
    with pytest.raises(ValueError,match='observed flat-floor'):
        filter_structural_obstacles(floor_scene()+[0,0,1.],{'enabled':True})


def test_nonfinite_cloud_rejected():
    p=floor_scene();p[0,2]=np.nan
    with pytest.raises(ValueError,match='finite'):
        filter_structural_obstacles(p,{'enabled':True})


def test_metric_support_ignores_unrelated_map_origin_changes():
    points = np.vstack((floor_scene(), [[1.037,.019,.40], [1.03,1.4,.40]]))
    cfg = {'enabled': True, 'support_mode': 'metric_disk'}
    baseline = filter_structural_obstacles(points,cfg)
    shifted_origin = filter_structural_obstacles(np.vstack((points, [-10.037,-20.049,0.])),cfg)
    assert np.array_equal(baseline['support_mask'],shifted_origin['support_mask'][:-1])
    assert np.array_equal(baseline['keep_mask'],shifted_origin['keep_mask'][:-1])
    assert baseline['removed_mask'][-2]
    assert not baseline['removed_mask'][-1]  # beyond actual observed floor


def test_metric_support_requires_both_core_and_surrounding_density():
    x,y=np.meshgrid(np.arange(-.16,.161,.08),np.arange(-.16,.161,.08))
    floor=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    cfg={'enabled':True,'support_mode':'metric_disk'}
    p=np.vstack((floor,[.001,.001,.4]))
    assert filter_structural_obstacles(p,cfg)['removed_mask'][-1]
    sparse=np.vstack((floor[:2],[.001,.001,.4]))
    with pytest.raises(ValueError,match='observed flat-floor'):
        filter_structural_obstacles(sparse,cfg)
    ring=floor[np.linalg.norm(floor[:,:2],axis=1)>.12]
    assert not filter_structural_obstacles(np.vstack((ring,[0.,0.,.4])),cfg)['removed_mask'][-1]


@pytest.mark.parametrize('cfg',[{'support_mode':'paint_free'}, {'support_core_radius_m':.30},
                                 {'support_core_min_points':True}])
def test_metric_support_configuration_rejected(cfg):
    with pytest.raises(ValueError):
        filter_structural_obstacles(floor_scene(),cfg)


def test_thin_height_subset_of_volumetric_noise_is_not_a_box_top():
    x,y=np.meshgrid(np.arange(.94,1.061,.06),np.arange(-.06,.061,.06))
    plane=np.c_[x.ravel(),y.ravel(),np.full(x.size,.16)]
    rng=np.random.default_rng(42)
    noise=np.c_[1+rng.uniform(-.15,.15,36),rng.uniform(-.15,.15,36),rng.uniform(.05,.68,36)]
    extras=np.vstack((plane,noise))
    _,strict,_=run(extras)
    _,permissive,_=run(extras,{'horizontal_plane_min_inlier_ratio':.20})
    assert strict['statistics']['low_object_top_points'] < permissive['statistics']['low_object_top_points']
