#!/usr/bin/env python3
"""Offline unit tests: no ROS node, no replay, no production edits."""
import copy
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

import compare_imu_chains as audit


class ImuChainEvaluationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(__file__).resolve().parent
        cls.central=yaml.safe_load((root.parent/'extrinsic_trials_20260919/calibrations/device_front_rotation.yaml').read_text())
        cls.front=yaml.safe_load((root/'calibrations/device_front_native.yaml').read_text())
        cls.fixture=audit.EV.load(root.parent/'runs/paired_front_180s')

    def fixture_run(self,calibration,role):
        run=copy.deepcopy(self.fixture)
        run['calibration']=copy.deepcopy(calibration)
        run['manifest']['topics']=['/front_lidar','/rear_lidar',calibration['input_topic']]
        run['manifest']['central_timestamp_offset_sec']=calibration['timestamp_offset_sec']
        run['manifest']['imu_input_topic']=calibration['input_topic']
        run['manifest']['imu_frame']=calibration['frame_id']
        parameters=run['frontend']['laserMapping']['ros__parameters']
        parameters['mapping']['extrinsic_R']=calibration['lio_extrinsic']['rotation']
        parameters['mapping']['extrinsic_T']=calibration['lio_extrinsic']['translation']
        parameters['publish']['body_frame']=calibration['frame_id']
        return run

    def test_exact_shared_physical_origin_under_motion(self):
        count=31
        state=np.zeros(count,dtype=[(key,float) for key in ['t','x','y','z','qx','qy','qz','qw']])
        desired_xyz=np.column_stack([np.linspace(0,10,count),np.sin(np.linspace(0,1,count)),np.linspace(0,.7,count)])
        desired_rotation=Rotation.from_euler('xyz',np.column_stack([np.linspace(-.1,.15,count),np.linspace(.04,-.08,count),np.linspace(.1,.5,count)]))
        reconstructions=[]
        for calibration in [self.central,self.front]:
            R_I_N=Rotation.from_matrix(np.array(calibration['lio_extrinsic']['rotation']).reshape(3,3))
            t_I_N=np.array(calibration['lio_extrinsic']['translation'])
            R_W_I=desired_rotation*R_I_N.inv()
            p_W_I=desired_xyz-R_W_I.apply(t_I_N)
            for j,key in enumerate(['x','y','z']):state[key]=p_W_I[:,j]
            for j,key in enumerate(['qx','qy','qz','qw']):state[key]=R_W_I.as_quat()[:,j]
            converted=audit.lidar_origin_state(state,calibration)
            xyz=np.column_stack([converted[key] for key in ['x','y','z']])
            rotation=Rotation.from_quat(np.column_stack([converted[key] for key in ['qx','qy','qz','qw']]))
            np.testing.assert_allclose(xyz,desired_xyz,atol=1e-12)
            np.testing.assert_allclose((rotation.inv()*desired_rotation).magnitude(),0,atol=1e-12)
            reconstructions.append(xyz)
        np.testing.assert_allclose(*reconstructions,atol=1e-12)

    def test_relative_height_and_xy_length_invariant_to_world_yaw_origin(self):
        p=np.column_stack([np.linspace(0,10,20),np.sin(np.linspace(0,1,20)),np.linspace(0,.5,20)])
        q=Rotation.from_euler('z',1.8).apply(p)+[50,-20,2]
        self.assertAlmostEqual(p[-1,2]-p[0,2],q[-1,2]-q[0,2],places=12)
        self.assertAlmostEqual(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum(),np.linalg.norm(np.diff(q[:,:2],axis=0),axis=1).sum(),places=12)

    def test_real_calibrations_only_permitted_changes(self):
        a=self.fixture_run(self.central,'central');b=self.fixture_run(self.front,'front')
        result=audit.control_checks(a,b)
        self.assertEqual(result['unexpected_calibration_differences'],[])
        self.assertEqual(result['unexpected_frontend_differences'],[])
        self.assertTrue(all(audit.role_checks(a,'central').values()))
        self.assertTrue(all(audit.role_checks(b,'front').values()))
        self.assertEqual([key for key,value in result['checks'].items() if value is False],[])

    def test_scale_noise_and_unknown_keys_not_ignored(self):
        a=self.fixture_run(self.central,'central');b=self.fixture_run(self.front,'front')
        b['calibration']['acceleration_scale']=1.
        self.assertFalse(audit.control_checks(a,b)['checks']['only_allowlisted_calibration_changes'])
        self.assertFalse(audit.role_checks(b,'front')['raw_acceleration_scale_9_80665'])
        b=self.fixture_run(self.front,'front');b['frontend']['laserMapping']['ros__parameters']['mapping']['acc_cov']=9.
        self.assertFalse(audit.control_checks(a,b)['checks']['only_allowlisted_frontend_changes'])
        b=self.fixture_run(self.front,'front');b['calibration']['unknown_execution_setting']=True
        self.assertFalse(audit.control_checks(a,b)['checks']['only_allowlisted_calibration_changes'])

    def test_time_shift_must_match_role_even_though_change_is_allowed(self):
        b=self.fixture_run(self.front,'front');b['calibration']['timestamp_offset_sec']=-.013
        self.assertFalse(audit.role_checks(b,'front')['role_time_offset'])

    def test_rear_and_launch_changes_fail(self):
        a=self.fixture_run(self.central,'central');b=self.fixture_run(self.front,'front')
        b['calibration']['lidar_extrinsics']['rear_to_front']['translation'][0]+=.01
        self.assertFalse(audit.control_checks(a,b)['checks']['same_rear_transform_not_used_by_front_subset'])
        b=self.fixture_run(self.front,'front');b['snapshots']['mapping.launch.py']['actual']='changed'
        self.assertFalse(audit.control_checks(a,b)['checks']['same_launch_snapshot'])

    def test_header_matching_and_missing_output(self):
        a=np.arange(10)*.1+100.;b=a[1:]+1e-5
        ia,ib=audit.matched_state_indices(a,b)
        np.testing.assert_array_equal(ia,np.arange(1,10))
        np.testing.assert_array_equal(ib,np.arange(9))
        ia,ib=audit.matched_state_indices(a,np.delete(a,4))
        self.assertNotEqual(ia[-1]-ia[0]+1,len(ia))

    def test_front_to_front_only_rotation_comparison_supported(self):
        a=self.fixture_run(self.front,'front');b=self.fixture_run(self.front,'front')
        b['calibration']['lio_extrinsic']['rotation']=[1.,0.,0.,0.,0.,1.,0.,-1.,0.]
        b['frontend']['laserMapping']['ros__parameters']['mapping']['extrinsic_R']=b['calibration']['lio_extrinsic']['rotation']
        result=audit.control_checks(a,b,'front','front')
        self.assertTrue(result['checks']['same_non_imu_topics'])
        self.assertEqual(result['unexpected_calibration_differences'],[])
        self.assertTrue(all(audit.role_checks(b,'front').values()))


if __name__=='__main__':
    unittest.main(verbosity=2)
