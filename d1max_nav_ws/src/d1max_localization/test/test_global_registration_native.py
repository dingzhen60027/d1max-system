"""Native Open3D fixtures, not independent robot/physical acceptance."""
from dataclasses import replace
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from d1max_localization.global_registration import GlobalMapIndex, RegistrationConfig


def room():
    rng=np.random.default_rng(18)
    xy=rng.uniform(0,5,(2400,2));floor=np.column_stack([xy,np.zeros(len(xy))])
    xz=rng.uniform([0,0],[5,2.5],(1500,2));wall=np.column_stack([xz[:,0],np.zeros(len(xz)),xz[:,1]])
    yz=rng.uniform([0,0],[5,2.5],(1800,2));side=np.column_stack([np.zeros(len(yz)),yz])
    # Asymmetric fixed structures, not just three featureless infinite planes.
    box=rng.uniform([1.2,3.,.25],[1.65,3.6,1.2],(600,3))
    return np.concatenate([floor,wall,side,box])


def config():
    return replace(RegistrationConfig(),voxel_m=.25,fine_voxel_m=.12,
        tile_xy_m=12.,stride_xy_m=6.,tile_z_m=4.,stride_z_m=2.,min_points=80,
        ransac_iterations=30000,max_map_points=100000,search_timeout_s=20.)


def test_real_native_global_search_finds_unknown_pose_and_cache_is_source_bound(tmp_path):
    points=room();path=tmp_path/'map.pcd'
    o3d.io.write_point_cloud(str(path),o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
    rotation=Rotation.from_euler('z',.9).as_matrix();origin=np.array([2.,2.,.7])
    query=(points-origin)@rotation
    index=GlobalMapIndex(path,tmp_path/'cache',config())
    result=index.search(query)
    assert result['accepted'],result
    transform=np.asarray(result['candidate']['transform'])
    assert np.linalg.norm(transform[:3,3]-origin)<.12
    assert Rotation.from_matrix(transform[:3,:3]@rotation.T).magnitude()<.08
    assert GlobalMapIndex(path,tmp_path/'cache',config()).cache_hit


def test_real_native_repeated_floors_do_not_get_a_startup_seed(tmp_path):
    points=room();path=tmp_path/'map.pcd'
    both=np.concatenate([points,points+[0,0,4.]])
    o3d.io.write_point_cloud(str(path),o3d.geometry.PointCloud(o3d.utility.Vector3dVector(both)))
    query=(points-[2.,2.,.7])@Rotation.from_euler('z',.9).as_matrix()
    result=GlobalMapIndex(path,tmp_path/'cache',config()).search(query)
    assert not result['accepted'],result
    assert result['reason']=='ambiguous_place_or_floor',result
