"""Validated single-floor support grid -> native PCT input, not a new planner."""
import argparse
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np
from scipy import ndimage


class MeasuredGrid:
    def __init__(self, source, cost_margin_m=0.6, minimum_clearance_m=0.2,
                 optimization_guard_cells=1):
        self.source = str(Path(source).resolve())
        self.sha256 = hashlib.sha256(Path(source).read_bytes()).hexdigest()
        with np.load(source, allow_pickle=False) as data:
            self.support = np.asarray(data['support'], dtype=bool)
            self.obstacles = np.asarray(data['obstacles'], dtype=bool)
            self.height = np.asarray(data['height'], dtype=float)
            self.free = np.asarray(data['free'], dtype=bool)
            self.origin = np.asarray(data['origin'], dtype=float)
            self.resolution = float(data['resolution'])
        if (self.free.ndim != 2 or min(self.free.shape) < 3
                or any(v.shape != self.free.shape for v in
                       [self.height, self.support, self.obstacles])):
            raise ValueError('Ground grid fields must share an x/y array shape')
        if (self.origin.shape != (2,) or not np.isfinite(self.origin).all()
                or not np.isfinite(self.resolution) or self.resolution <= 0):
            raise ValueError('Invalid ground grid origin/resolution')
        if not np.isfinite(cost_margin_m) or cost_margin_m <= 0:
            raise ValueError('cost_margin_m must be positive')
        if not np.isfinite(minimum_clearance_m) or minimum_clearance_m < 0:
            raise ValueError('minimum_clearance_m must be nonnegative')
        self.free &= self.support & ~self.obstacles & np.isfinite(self.height)
        clearance = ndimage.distance_transform_edt(
            np.pad(self.support & ~self.obstacles, 1))[1:-1, 1:-1] * self.resolution
        self.free &= clearance >= minimum_clearance_m
        self.clearance = clearance
        if (int(optimization_guard_cells) != optimization_guard_cells
                or optimization_guard_cells < 0 or optimization_guard_cells > 5):
            raise ValueError('optimization_guard_cells must be an integer in [0, 5]')
        self.planner_free = (ndimage.binary_erosion(self.free, iterations=int(optimization_guard_cells))
                             if optimization_guard_cells else self.free.copy())
        if not self.planner_free.any():
            raise ValueError('No measured free ground remains after conservative PCT guard')
        # Blocked cells remain 50; the native planner threshold is 20. Within
        # allowed cells add a soft cost to move GPMP away from corridor edges.
        free_clearance = ndimage.distance_transform_edt(np.pad(self.planner_free, 1))[1:-1, 1:-1] * self.resolution
        self.cost = np.where(self.planner_free, 19.0 * np.clip(
            (cost_margin_m - free_clearance) / cost_margin_m, 0.0, 1.0), 50.0)

    def index(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('Endpoint must contain finite x/y')
        return np.floor((xy - self.origin) / self.resolution).astype(int)

    def contains(self, index):
        return all(0 <= i < size for i, size in zip(index, self.free.shape))

    def validate_point(self, xy):
        index = tuple(self.index(xy))
        if not self.contains(index) or not self.free[index]:
            raise ValueError('Endpoint is outside measured free ground; no automatic snapping')
        return float(self.height[index])

    def payload(self):
        gx = np.zeros_like(self.cost)
        gy = np.zeros_like(self.cost)
        gx[1:-1] = self.cost[2:] - self.cost[:-2]
        gy[:, 1:-1] = self.cost[:, 2:] - self.cost[:, :-2]
        ground = np.where(self.support & np.isfinite(self.height), self.height, np.nan)
        # This auxiliary ceiling only satisfies the native planner data format;
        # obstacle/clearance evidence resides in the retained occupancy mask.
        ceiling = np.where(np.isfinite(ground), ground + 2.0, np.nan)
        return {
            'data': np.stack([self.cost, gx, gy, ground, ceiling])[:, None].astype(np.float32),
            'resolution': self.resolution,
            'center': self.origin + (np.array(self.free.shape) // 2 + 0.5) * self.resolution,
            'slice_h0': float(np.nanmax(ground) + 0.5), 'slice_dh': 0.5,
            'backend': 'measured-ground-grid-adapter-to-native-pct',
            'source_grid': self.source, 'source_sha256': self.sha256,
            'height_semantics': 'ground_z', 'floor_count': 1,
        }

    def segment_cells(self, a, b):
        """Exact supercover of a line including both sides of grid boundaries."""
        a = (np.asarray(a, dtype=float) - self.origin) / self.resolution
        b = (np.asarray(b, dtype=float) - self.origin) / self.resolution
        delta = b - a
        ts = [0.0, 1.0]
        for axis in range(2):
            if abs(delta[axis]) > 1e-12:
                low, high = sorted((a[axis], b[axis]))
                for boundary in range(int(np.ceil(low)), int(np.floor(high)) + 1):
                    t = (boundary - a[axis]) / delta[axis]
                    if 0.0 < t < 1.0:
                        ts.append(float(t))
        ts = sorted(set(ts))
        samples = ts + [(x + y) / 2 for x, y in zip(ts, ts[1:])]
        cells = set()
        for t in samples:
            point = a + t * delta
            options = []
            for value in point:
                integer = round(float(value))
                options.append([integer - 1, integer] if abs(value - integer) < 1e-8
                               else [int(np.floor(value))])
            cells.update((x, y) for x in options[0] for y in options[1])
        return cells

    def validate_path(self, path, max_ground_step_m=0.15):
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all():
            raise ValueError('PCT result must contain at least two finite XYZ points')
        visited = set()
        for a, b in zip(path[:-1, :2], path[1:, :2]):
            cells = self.segment_cells(a, b)
            for cell in cells:
                if not self.contains(cell) or not self.free[cell]:
                    raise ValueError(f'Planned curve crosses blocked/unsupported cell {cell}')
            # Check adjacent cells actually crossed, not arbitrary distant
            # ground points on the whole segment (a gradual slope is allowed).
            for x, y in cells:
                for neighbor in ((x + 1, y), (x, y + 1), (x + 1, y + 1), (x + 1, y - 1)):
                    if neighbor in cells and abs(self.height[x, y] - self.height[neighbor]) > max_ground_step_m:
                        raise ValueError('PCT route exceeds the permitted local ground step')
            visited.update(cells)
        return {'checked_cells': len(visited), 'minimum_clearance_m': float(min(
            self.clearance[cell] for cell in visited))}


def main():
    parser = argparse.ArgumentParser(description='Adapt measured support/obstacle NPZ to native PCT input')
    parser.add_argument('--grid', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cost-margin', type=float, default=0.6)
    parser.add_argument('--clearance', type=float, default=0.2)
    parser.add_argument('--optimization-guard-cells', type=int, default=1)
    args = parser.parse_args()
    grid = MeasuredGrid(args.grid, args.cost_margin, args.clearance, args.optimization_guard_cells)
    payload = grid.payload()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    print(json.dumps({'output': str(args.output), 'source_grid': grid.source,
                      'source_sha256': grid.sha256, 'shape': list(grid.free.shape),
                      'free_cells': int(grid.free.sum()), 'backend': payload['backend']}))


if __name__ == '__main__':
    main()
