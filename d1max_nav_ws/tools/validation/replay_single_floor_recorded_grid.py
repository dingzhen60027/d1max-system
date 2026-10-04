#!/usr/bin/env python3
"""Reuse the frozen real-ray replay driver with this release's native GridMap.

No ROS initialization or network. This is an input/collision regression, not
closed-loop navigation and not a new certification of sensor calibration.
"""
import importlib.util
from pathlib import Path


if __name__=='__main__':
    ws=Path(__file__).resolve().parents[2]
    driver=ws/'experiments/navigation_system_v2_20260927/bag_evidence/replay_evidence.py'
    spec=importlib.util.spec_from_file_location('frozen_real_ray_driver',driver)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.PROBE=ws/'experiments/single_floor_execution_20260927/native/build/plan_env/offline_projected_rays_probe'
    module.main()
