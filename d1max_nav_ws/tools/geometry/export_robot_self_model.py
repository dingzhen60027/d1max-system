#!/usr/bin/env python3
"""Compile a compact collision manifest to stdout, without ROS/SDK or writes.

The URDF and all ``asset_paths`` remain the source of sealed surface geometry.
Archive those files with the manifest; never substitute the display-only model.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/d1max_pct_scan'))
from d1max_pct_scan.robot_self_geometry import RobotSelfModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    args = parser.parse_args()
    model = RobotSelfModel.from_urdf(args.urdf)
    print(json.dumps(model.to_dict(), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
