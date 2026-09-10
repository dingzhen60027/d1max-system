"""Configuration tests only; no ROS or robot connections."""
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("pcd_view", Path(__file__).resolve().parents[1] / "scripts/pcd_map_publisher.py")
pcd_view = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pcd_view)

class DisabledMapTests(unittest.TestCase):
    def test_disabled_does_not_open_any_pcd(self):
        config = "enabled: false\npublish_period_seconds: 10\nmaps:\n  - path: /nonexistent/history.pcd\n"
        with patch.object(Path, "read_text", return_value=config), patch.object(pcd_view.o3d.io, "read_point_cloud") as read:
            self.assertEqual(pcd_view.load_maps("config.yaml"), (10, []))
            read.assert_not_called()

    def test_disabled_main_exits_without_starting_ros(self):
        with patch("sys.argv", ["pcd_map_publisher.py"]), patch.object(pcd_view, "load_maps", return_value=(10, [])), patch("sys.stdout", new_callable=io.StringIO) as output:
            pcd_view.main()
            self.assertIn("no ROS node started", output.getvalue())

    def test_disabled_probe_does_not_parse_clouds(self):
        with patch("sys.argv", ["pcd_map_publisher.py", "--is-enabled"]), patch.object(Path, "read_text", return_value="enabled: false"), patch.object(pcd_view, "load_maps") as load:
            with self.assertRaises(SystemExit) as exit:
                pcd_view.main()
            self.assertEqual(exit.exception.code, 1)
            load.assert_not_called()

if __name__ == "__main__":
    unittest.main()
