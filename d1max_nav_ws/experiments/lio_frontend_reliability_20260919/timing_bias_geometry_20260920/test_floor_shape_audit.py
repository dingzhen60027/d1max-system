import unittest
import numpy as np
from floor_shape_audit import profile_map, dominant_floor, angle


class FloorShapeTests(unittest.TestCase):
    def fixture(self, curve=0.):
        rng=np.random.default_rng(42)
        y,x=np.meshgrid(np.arange(0,40.01,.075),np.arange(-1.4,1.401,.075))
        z=-.49-.017*y+.004*x+curve*(y-20)**2+rng.normal(0,.003,y.shape)
        cloud=np.c_[x.ravel(),y.ravel(),z.ravel()]
        t=np.linspace(0,40,401)
        pose=np.c_[np.zeros(len(t)),t,-.017*t+curve*(t-20)**2]
        return cloud,pose,t

    def test_rigid_tilt_is_not_misclassified_as_bending(self):
        cloud,poses,s=self.fixture()
        result=profile_map(cloud,poses,s,1.,.25,.85,.025)
        expected=np.array([-.004,.017,1.]);expected/=np.linalg.norm(expected)
        self.assertEqual(result['accepted_patches'],16)
        self.assertLess(angle(np.array(result['global_plane_normal']),expected),.03)
        self.assertLess(result['floor_center_to_global_plane_rmse_m'],.002)
        self.assertLess(result['equal_patch_sample_to_global_plane_m']['abs_p95'],.010)
        self.assertLess(result['floor_endpoint_delta_z_m'],-.6)

    def test_bending_is_retained(self):
        cloud,poses,s=self.fixture(.0007)
        result=profile_map(cloud,poses,s,1.,.25,.85,.025)
        self.assertEqual(result['accepted_patches'],16)
        self.assertGreater(result['floor_center_to_global_plane_rmse_m'],.06)
        self.assertLess(result['along_path_quadratic_rmse_m'],result['along_path_linear_rmse_m']*.1)

    def test_insufficient_patch_rejected(self):
        self.assertIsNone(dominant_floor(np.zeros((20,3)),.025,42))


if __name__=='__main__':
    unittest.main()
