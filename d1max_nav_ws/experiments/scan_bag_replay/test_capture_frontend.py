"""Pure capture-contract tests. No ROS initialization or subprocesses."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

from capture_frontend import (CaptureWriter, DOMAIN, ExactPairs, cloud_arrays, json_value,
                              stamp_ns, transport_config, validate_isolation)


def cloud(points, stamp=123000000001, *, padding=0, big=False):
    points = np.asarray(points, dtype=('>f4' if big else '<f4'))
    rows = [p.tobytes() + bytes(padding) for p in points]
    return NS(width=1, height=len(points), point_step=12, row_step=12+padding,
              data=b''.join(rows), is_bigendian=big,
              header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9), frame_id='tracking'),
              fields=[NS(name=n, offset=i*4, count=1, datatype=7) for i, n in enumerate('xyz')])


class MemoryWriter:
    def __init__(self): self.records = []
    def submit(self, *value): self.records.append(value)


class CaptureTests(unittest.TestCase):
    def test_ros_uint8_diagnostics_normalized_without_binary_leak(self):
        self.assertEqual(json_value({'level':b'\x02','values':[np.int64(42),np.float32(.5)]}),
                         {'level':2,'values':[42,.5]})
        with self.assertRaises(ValueError):json_value({'cloud':b'1234'})

    def test_writer_handles_ros_diagnostic_byte(self):
        with tempfile.TemporaryDirectory() as folder:
            writer=CaptureWriter(Path(folder),100,1024*1024)
            writer.submit('input_clock',{'level':b'\x01','message':'calibrating'})
            writer.close()
            self.assertEqual(json.loads((Path(folder)/'input_clock.jsonl').read_text())['level'],1)

    def test_stamp_preserves_integer_nanoseconds(self):
        self.assertEqual(stamp_ns(cloud([[1,2,3]],1790253600123456789)),1790253600123456789)

    def test_row_padding_and_big_endian(self):
        for big in [False, True]:
            arrays, info = cloud_arrays(cloud([[1,2,3],[4,5,6]],padding=7,big=big),10)
            np.testing.assert_array_equal(arrays['xyz'],[[1,2,3],[4,5,6]])
            self.assertEqual(info['retained_points'],2)

    def test_finite_downsample_no_synthetic_points(self):
        arrays, info = cloud_arrays(cloud([[float('nan'),0,0],[1,2,3],[4,5,6],[7,8,9]]),2)
        np.testing.assert_array_equal(arrays['xyz'],[[1,2,3],[7,8,9]])
        np.testing.assert_array_equal(arrays['original_point_index'],[1,3])
        self.assertEqual(info['original_points'],4)
        self.assertEqual(info['finite_xyz_points'],3)

    def test_bad_stride_rejected(self):
        msg=cloud([[1,2,3]])
        msg.row_step=4
        with self.assertRaises(ValueError): cloud_arrays(msg,10)

    def test_exact_pair_preserves_velocity_and_inertial(self):
        writer=MemoryWriter(); pairs=ExactPairs(writer,500000000)
        state={'valid':True,'epoch':2,'frame':'odom','child_frame':'tracking',
               'inertial':{'world_velocity':[1,2,3],'gravity':[0,0,-9.81]}}
        odom={'frame':'odom','child_frame':'tracking','linear_child_frame':[.7,.2,.1]}
        pairs.add('cloud',123000000001,cloud([[1,2,3]]))
        pairs.add('state',123000000001,state)
        pairs.add('odom',123000000001,odom)
        self.assertEqual(pairs.counts['paired'],1)
        self.assertIs(writer.records[0][1][1],odom)
        self.assertIs(writer.records[0][1][2],state)

    def test_nearby_timestamp_never_mixed(self):
        writer=MemoryWriter(); pairs=ExactPairs(writer,500000000)
        pairs.add('cloud',123000000001,cloud([[1,2,3]]))
        pairs.add('state',123000000002,{'valid':True})
        pairs.add('odom',123000000001,{})
        self.assertEqual(pairs.counts['paired'],0)
        self.assertEqual(writer.records,[])

    def test_cloud_sampling_and_memory_bounded(self):
        writer=MemoryWriter(); pairs=ExactPairs(writer,500000000)
        for i in range(200):pairs.add('cloud',i*100000000,cloud([[1,2,3]],i*100000000))
        self.assertLessEqual(len(pairs.cloud),8)
        self.assertEqual(pairs.counts['selected_cloud'],40)
        self.assertEqual(pairs.counts['unmatched_evicted'],32)

    def test_frame_mismatch_rejected(self):
        writer=MemoryWriter(); pairs=ExactPairs(writer,500000000)
        pairs.add('cloud',123000000001,cloud([[1,2,3]]))
        pairs.add('state',123000000001,{'valid':True,'frame':'odom','child_frame':'other'})
        with self.assertRaises(ValueError):
            pairs.add('odom',123000000001,{'frame':'odom','child_frame':'tracking'})

    def test_transport_rejects_discovery_or_wrong_domain(self):
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            env={'ROS_DOMAIN_ID':DOMAIN,'RMW_IMPLEMENTATION':'rmw_zenoh_cpp'}
            for kind,key in [('router','ZENOH_ROUTER_CONFIG_URI'),('client','ZENOH_SESSION_CONFIG_URI')]:
                path=folder/(kind+'.json');path.write_text(json.dumps(transport_config(kind)));env[key]=str(path)
            validate_isolation(env)
            with self.assertRaises(ValueError):validate_isolation(dict(env,ROS_DOMAIN_ID='24'))
            wrong=transport_config('client');wrong['scouting']['multicast']['enabled']=True
            Path(env['ZENOH_SESSION_CONFIG_URI']).write_text(json.dumps(wrong))
            with self.assertRaises(ValueError):validate_isolation(env)

    def test_writer_retains_native_twist_and_missing_acceleration(self):
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder);writer=CaptureWriter(out,100,1024*1024)
            writer.submit('sample',(cloud([[1,2,3]]),{'linear_child_frame':[.7,0,.1]},
                                      {'inertial':{'world_velocity':[.8,.2,.1]}}))
            writer.close()
            result=json.loads((out/'sample.jsonl').read_text())
            self.assertIsNone(result['acceleration'])
            self.assertEqual(result['odometry']['linear_child_frame'],[.7,0,.1])
            with np.load(out/result['cloud_file'],allow_pickle=False) as npz:
                np.testing.assert_array_equal(npz['xyz'],[[1,2,3]])
            self.assertEqual(writer.saved_clouds,1)


if __name__ == '__main__':unittest.main()
