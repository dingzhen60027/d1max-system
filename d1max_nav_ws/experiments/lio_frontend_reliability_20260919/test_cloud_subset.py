import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from cloud_subset import select_cloud


class SubsetTest(unittest.TestCase):
    def cloud(self, endian=False):
        cloud = PointCloud2()
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = 123, 456
        cloud.header.frame_id = 'normalized_lidar'
        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 4, 24, 96
        cloud.is_bigendian = endian
        cloud.fields = [PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)]
        rows = np.arange(96, dtype=np.uint8).reshape(4, 24)
        rings = np.ndarray((4,), dtype='>u2' if endian else '<u2', buffer=rows, offset=16, strides=(24,))
        rings[:] = [0, 96, 95, 191]
        cloud.data = rows.tobytes()
        return cloud

    def test_dual_byte_identical(self):
        cloud = self.cloud()
        self.assertEqual(select_cloud(cloud, 'dual'), cloud)

    def test_subsets_keep_exact_point_records(self):
        for endian in (False, True):
            cloud = self.cloud(endian)
            for mode, indices in [('paired_front', [0, 2]), ('paired_rear', [1, 3])]:
                selected = select_cloud(cloud, mode)
                self.assertEqual(selected.header, cloud.header)
                self.assertEqual(selected.width, 2)
                expected = b''.join(bytes(cloud.data[i*24:(i+1)*24]) for i in indices)
                self.assertEqual(bytes(selected.data), expected)

    def test_invalid_cloud_rejected(self):
        cloud = self.cloud()
        cloud.row_step = 95
        with self.assertRaises(ValueError):
            select_cloud(cloud, 'paired_front')


if __name__ == '__main__':
    unittest.main()
