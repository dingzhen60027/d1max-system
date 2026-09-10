#!/usr/bin/env python3
import argparse
import os
from pathlib import Path

import numpy as np

from .planner_core import TomogramPlanner


def main():
    parser = argparse.ArgumentParser(description='Run PCT global planning from a tomogram.')
    parser.add_argument('--tomogram', required=True, type=Path)
    parser.add_argument('--vendor-root', type=Path,
                        default=Path(os.environ.get('PCT_PLANNER_ROOT',
                            '/home/dndx/d1max_nav_ws/src/pct_planner_vendor')))
    parser.add_argument('--start', required=True, nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--goal', required=True, nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--start-layer', type=int, default=0)
    parser.add_argument('--goal-layer', type=int, default=0)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    planner = TomogramPlanner(args.vendor_root)
    planner.load(args.tomogram)
    trajectory = planner.plan(args.start, args.goal, args.start_layer, args.goal_layer)
    if trajectory is None or len(trajectory) == 0:
        raise RuntimeError('PCT did not find a path for the requested endpoints/layers')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, trajectory, delimiter=',', header='x,y,z', comments='')
    length = np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum()
    print(f'PCT path created: {args.output}')
    print(f'Waypoints: {len(trajectory)}, 3D length: {length:.3f} m')


if __name__ == '__main__':
    main()
