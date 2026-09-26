"""Pure native-replay contract tests, no ROS or processes."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from capture_frontend import sha256
from replay_native import phase_time_summary, prepare_data, profile_overrides, spline_measurement


def fixture(folder, *, velocity=True):
    (folder/'clouds').mkdir()
    (folder/'result.json').write_text(json.dumps({'passed':True,'owned_processes_surviving':[]}))
    (folder/'localization.yaml').write_text(yaml.safe_dump({'lio_localizer':{'ros__parameters':{
        'tracking_offset_body':[.4043,0.,-.0377],'sdk_to_tracking_yaw':1.56646}}}))
    odom=[];states=[];samples=[]
    for i in range(12):
        stamp=1790264200000000000+i*100000000
        record={'stamp_ns':stamp,'frame':'d1max_loc_odom','child_frame':'d1max_loc_tracking',
                'position':[i*.1,0.,0.],'orientation':[0.,0.,0.,1.],
                'linear_child_frame':[1.,0.,0.],'angular_child_frame':[0.,0.,1.],
                'pose_covariance':(np.eye(6)*.01).reshape(-1).tolist(),
                'twist_covariance':(np.eye(6)*.01).reshape(-1).tolist()}
        state={'valid':True,'epoch':1,'stamp_ns':str(stamp),'frame':'d1max_loc_odom',
               'inertial':{'world_velocity':[1.,0.,0.]} if velocity else {}}
        odom.append(record);states.append(state)
        if i%2==0:
            path=folder/'clouds'/f'{i}.npz';np.savez(path,xyz=np.array([[1.,2.,3.]],dtype=np.float32))
            samples.append({'stamp_ns':stamp,'cloud_file':'clouds/'+path.name,'cloud_sha256':sha256(path),
                            'local_sample':state,'odometry':record,'frame':'d1max_loc_tracking'})
    for name,records in [('odometry',odom),('local_sample',states),('sample',samples)]:
        (folder/(name+'.jsonl')).write_text(''.join(json.dumps(x)+'\n' for x in records))


class ReplayTests(unittest.TestCase):
    def test_v06_only_changes_experiment_speed_not_geometry_or_acceleration(self):
        for name in ('strict2','occupied2'):
            base=profile_overrides(name);fast=profile_overrides(name+'_v06')
            self.assertEqual({k:v for k,v in fast.items() if k not in ('manager.max_vel','optimization.max_vel')},base)
            self.assertEqual(fast['manager.max_vel'],.6)
            self.assertEqual(fast['grid_map.require_observed_free'],name=='strict2')
            self.assertFalse(any('acc' in k or 'radius' in k for k in fast))

    def test_production_body_reference_and_twist_are_used(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);fixture(folder)
            samples,body=prepare_data(folder)
            self.assertEqual(len(body),12)
            self.assertEqual(len(samples),6)
            # Production body-to-tracking has -yaw; its inverse body pose +yaw.
            self.assertGreater(body[0]['orientation'][2],.70)
            self.assertLess(body[0]['linear'][1],-1.4)
            self.assertEqual(body[0]['angular'],(0.,0.,1.))
            self.assertAlmostEqual(body[0]['position'][2],.0377)
            np.testing.assert_array_equal(samples[0]['world_cloud'],[[1,2,3]])

    def test_missing_inertial_velocity_is_error_not_zero_fill(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);fixture(folder,velocity=False)
            with self.assertRaisesRegex(ValueError,'velocity missing'):prepare_data(folder)

    def test_changed_cloud_hash_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);fixture(folder)
            np.savez(folder/'clouds/0.npz',xyz=np.array([[10,20,30]]))
            with self.assertRaisesRegex(ValueError,'hash mismatch'):prepare_data(folder)

    def test_phase_fraction_is_time_not_message_count(self):
        out=phase_time_summary([{'elapsed_s':1.,'phase':'accepted'},
            {'elapsed_s':5.,'phase':'waiting_observed_space'},
            {'elapsed_s':20.,'phase':'accepted'}],30.)
        self.assertEqual(out['waiting_observed_space_fraction'],.5)
        self.assertEqual(out['phase_seconds']['accepted'],14.)
        self.assertIsNone(out['unknown_voxel_fraction'])

    def test_native_curve_speed_and_acceleration_not_assumed(self):
        xyz,measurement=spline_measurement({'order':3,'knots':[0,0,0,0,1,1,1,1],
            'points':[[0,0,0],[1,0,0],[2,0,0],[3,0,0]]})
        self.assertAlmostEqual(measurement['max_speed_mps'],3.)
        self.assertAlmostEqual(measurement['max_acceleration_mps2'],0.)
        np.testing.assert_allclose(xyz[-1],[3,0,0])


if __name__=='__main__':unittest.main()
