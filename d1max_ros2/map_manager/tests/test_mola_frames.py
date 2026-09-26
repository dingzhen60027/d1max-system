import unittest
import struct
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from backend.mapping.configuration import parse_config
from backend.mapping.inspect_bag import check_frame_contract
from backend.mapping.visualization import select_map, read_live_cloud

APP = Path(__file__).resolve().parents[1]


class FrameContractTests(unittest.TestCase):
    def test_live_map_protocol_preserves_global_points_without_second_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'global.bin'
            xyz=np.array([[1.,2.,3.],[-1.,-2.,-3.]],dtype='<f4')
            p.write_bytes(b'D1MG0001'+struct.pack('<Qd',2,123.5)+xyz.tobytes())
            actual,stamp=read_live_cloud(p)
            np.testing.assert_array_equal(actual,xyz)
            self.assertEqual(stamp,123.5)
            p.write_bytes(p.read_bytes()[:-1])
            with self.assertRaises(ValueError):read_live_cloud(p)
            p.write_bytes(b'D1MG0001'+struct.pack('<Qd',2000001,123.5))
            with self.assertRaises(ValueError):read_live_cloud(p)

    def test_view_selects_one_complete_map_and_no_false_loop_success(self):
        m = {'artifacts': [{'file': 'raw.pcd', 'status': 'complete'},
                           {'file': 'optimized.pcd', 'status': 'writing'}]}
        self.assertEqual(select_map(m), 'raw.pcd')
        m['artifacts'][1]['status'] = 'complete'
        m['loop_counts'] = {'gnc_inliers': 0}
        self.assertEqual(select_map(m), 'raw.pcd')
        m['loop_counts']['gnc_inliers'] = 3
        self.assertEqual(select_map(m), 'optimized.pcd')
        self.assertIsNone(select_map({}))

    def test_d1_same_label_is_not_identity_extrinsic(self):
        config = {'input': {'lidar_topic': '/front_lidar'}, 'sensors': {'source': 'bag_tf'}}
        with self.assertRaisesRegex(ValueError, '物理坐标不同'):
            check_frame_contract(config, 'rslidar_head', 'rslidar_head')
        config['sensors']['source'] = 'fixed'
        check_frame_contract(config, 'rslidar_head', 'rslidar_head')

    def test_unrelated_sensor_contract_is_not_rewritten(self):
        config = {'input': {'lidar_topic': '/points'}, 'sensors': {'source': 'bag_tf'}}
        check_frame_contract(config, 'sensor', 'sensor')

    def test_d1_profile_matches_existing_calibration(self):
        c = parse_config((APP / 'config/mapping/experiments/d1max_903_mola.yaml').read_text())
        self.assertEqual(c.sensors.source, 'fixed')
        lidar = Rotation.from_euler('ZYX', c.sensors.lidar_pose[3:], degrees=True)
        expected = Rotation.from_quat([-.499867275, .503186620, .497953310, .498977388])
        np.testing.assert_allclose(lidar.as_matrix(), expected.as_matrix(), atol=1e-10)
        imu = Rotation.from_euler('ZYX', c.sensors.imu_pose[3:], degrees=True)
        np.testing.assert_allclose(imu.apply([1, 2, 3]), [1, -3, 2], atol=1e-10)
        self.assertGreater((lidar.inv() * imu).magnitude(), 1.)
        self.assertAlmostEqual(c.input.imu_acceleration_scale, 9.80665)


if __name__ == '__main__':
    unittest.main()
