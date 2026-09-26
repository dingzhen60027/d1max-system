import unittest
from collections import Counter
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from adapter import R_N_L, ScanPairer, align_up, convert_cloud, merge_clouds, rear_transform


def sample_cloud(ns=100_000_000_000):
    dtype = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                      "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                      "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
    raw = np.zeros(2000, dtype=dtype)
    theta = np.linspace(-np.pi, np.pi, len(raw))
    raw["x"], raw["y"], raw["z"] = 5*np.cos(theta), 5*np.sin(theta), 1.
    raw["ring"], raw["intensity"] = np.arange(len(raw))%96, np.arange(len(raw))
    raw["timestamp"] = ns*1e-9+np.linspace(.1, 0, len(raw))
    fields = [SimpleNamespace(name=n, offset=dtype.fields[n][1], datatype=(4 if n=="ring" else 8 if n=="timestamp" else 7)) for n in dtype.names]
    msg = SimpleNamespace(fields=fields, is_bigendian=False, point_step=26, row_step=2000*26,
                          width=2000, height=1, data=raw.tobytes(),
                          header=SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9, nanosec=ns%10**9)))
    return msg, raw


class AdapterTest(unittest.TestCase):
    def test_gravity_alignment(self):
        for rpy in [[10, 20, 0], [-35, 15, 0], [0, 0, 0]]:
            rot = Rotation.from_euler("xyz", rpy, degrees=True)
            acc = rot.inv().apply([0, 0, 9.80665])
            np.testing.assert_allclose(align_up(acc).apply(acc), [0, 0, 9.80665], atol=1e-10)

    def test_cloud_units_ring_sort_and_projection(self):
        dtype = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                          "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                          "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
        raw = np.zeros(2000, dtype=dtype)
        theta = np.linspace(-np.pi, np.pi, len(raw))
        raw["x"], raw["y"], raw["z"] = 5*np.cos(theta), 5*np.sin(theta), 1.
        raw["ring"] = np.arange(len(raw))%96
        raw["timestamp"] = 100.+np.linspace(.1, 0, len(raw))
        fields = [SimpleNamespace(name=n, offset=dtype.fields[n][1], datatype=(4 if n=="ring" else 8 if n=="timestamp" else 7)) for n in dtype.names]
        msg = SimpleNamespace(fields=fields, is_bigendian=False, point_step=26, row_step=2000*26,
                              width=2000, height=1, data=raw.tobytes(),
                              header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))
        out, detail = convert_cloud(msg)
        self.assertTrue(np.all(np.diff(out["time"]) >= 0))
        self.assertAlmostEqual(out["time"][-1], .1, places=6)
        self.assertEqual(out["ring"][-1], raw["ring"][0])
        recovered = np.column_stack([out[n] for n in ("x", "y", "z")])@R_N_L
        np.testing.assert_allclose(recovered, np.column_stack([raw[n][::-1] for n in ("x", "y", "z")]), atol=4e-7)
        self.assertEqual(detail["input_points"], detail["output_points"])

    def test_lever_direction(self):
        r_c_n = Rotation.from_euler("xyz", [2, 70, -20], degrees=True).as_matrix()
        t_c_n = np.array([.01, -.36, .003])
        t_n_c = -r_c_n.T@t_c_n
        np.testing.assert_allclose(r_c_n@t_n_c+t_c_n, np.zeros(3), atol=1e-12)

    def test_dual_preserves_native_projection_and_lever(self):
        front, _ = sample_cloud()
        rear, raw = sample_cloud(100_000_020_000)
        rotation = Rotation.from_euler("xyz", [15, 80, 170], degrees=True).as_matrix()
        translation = np.array([.1, -.77, .02])
        out, start, detail = merge_clouds(front, rear, rotation, translation)
        self.assertEqual(start, 100_000_000_000)
        self.assertEqual(detail["header_skew_ns"], 20_000)
        self.assertTrue(np.all(np.diff(out["time"]) >= 0))
        self.assertAlmostEqual(float(out["time"][-1]), .10002, places=6)
        self.assertEqual(len(out), 4000)
        front_out, rear_out = out[out["ring"] < 96], out[out["ring"] >= 96]
        self.assertEqual(len(front_out), len(rear_out))
        native_front, _ = convert_cloud(front)
        native_rear, _ = convert_cloud(rear)
        np.testing.assert_array_equal(front_out["column"], native_front["column"])
        np.testing.assert_array_equal(rear_out["column"], native_rear["column"])
        self.assertEqual(len(np.intersect1d(front_out["ring"], rear_out["ring"])), 0)
        xyz_n = np.column_stack([rear_out[k] for k in ("x", "y", "z")])
        recovered = (xyz_n-translation)@rotation
        original = np.column_stack([raw[k][::-1] for k in ("x", "y", "z")])
        np.testing.assert_allclose(recovered, original, atol=5e-7)
        np.testing.assert_allclose(rear_out["sensor_range"], np.linalg.norm(original, axis=1), atol=1e-6)
        self.assertFalse(np.allclose(rear_out["sensor_range"], np.linalg.norm(xyz_n, axis=1)))

    def test_pairer_drops_stale_without_reusing(self):
        counts = Counter()
        pairs = ScanPairer(counts)
        first, _ = sample_cloud()
        second, _ = sample_cloud(100_100_000_000)
        rear, _ = sample_cloud(100_100_020_000)
        self.assertEqual(pairs.add("front", first), [])
        self.assertEqual(pairs.add("rear", rear), [])
        self.assertEqual(counts["front_unpaired"], 1)
        self.assertEqual(pairs.add("front", second), [(second, rear)])
        self.assertEqual(sum(map(len, pairs.queues.values())), 0)
        with self.assertRaises(RuntimeError):
            pairs.add("front", second)

    def test_wrong_scan_pair_is_rejected(self):
        front, _ = sample_cloud()
        rear, _ = sample_cloud(100_100_000_000)
        with self.assertRaises(ValueError):
            merge_clouds(front, rear, np.eye(3), np.zeros(3))

    def test_invalid_rear_rotation_rejected(self):
        config = {"lidar_extrinsics": {"rear_to_front": {"rotation": (2*np.eye(3)).ravel().tolist(), "translation": [0, 0, 0]}}}
        with self.assertRaises(ValueError):
            rear_transform(config)


if __name__ == "__main__":
    unittest.main()
