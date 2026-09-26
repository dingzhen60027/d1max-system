"""Per-cell rule codes at the stair foot: which rule makes each cell a barrier.
 . flat   c crossable   I interval<min   S not flat & not crossable   ' ' no ground
 lower-case letter after = inflated>20;  '#' marks recorded trajectory cells."""
import sys
import numpy as np
import open3d as o3d
from scipy import ndimage
sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import _inflate, height_gradients, height_layers, mapping_geometry, traversal_cost

PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')
POSES = ('/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/'
         'sc_pgo/optimized_poses.txt')
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])
res, slope, margin, infl = 0.10, float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
layers = [int(v) for v in sys.argv[4].split(',')]
lo, hi = np.array([float(v) for v in sys.argv[5].split(',')]), np.array([float(v) for v in sys.argv[6].split(',')])
imin = float(sys.argv[7]) if len(sys.argv) > 7 else 0.55
pts = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
pts = pts[np.all((pts >= CROP[0]) & (pts <= CROP[1]), axis=1)]
trav = {'kernel_size': 7, 'interval_min': imin, 'interval_free': 0.70, 'slope_max_rad': slope,
        'step_max': 0.17, 'standable_ratio': 0.20, 'cost_barrier': 50.0}
geo = mapping_geometry(pts, {'resolution': res, 'slice_dh': 0.5, 'ground_height': -3.0})
ground, ceiling = height_layers(pts, geo, res, 0.5)
raw = traversal_cost(ground, ceiling, res, trav)
gs, gm = height_gradients(ground)
stand = (1.2 * res * np.tan(slope)) ** 2
center, dim = geo['center'], np.array(geo['shape'])
idx = lambda xy: tuple(np.floor((np.asarray(xy) - center) / res + .5).astype(int) + dim // 2)
i0, i1 = idx(lo), idx(hi)
poses = np.loadtxt(POSES)[:, [3, 7, 11]]
track = {idx(p[:2]) for p in poses[700:960]}
for l in layers:
    inf = _inflate(raw[l], res, margin, infl)
    flat = gs[l] <= stand
    nflat = ndimage.convolve((gs[l] < stand).astype(int), np.ones((7, 7), int), mode='constant')
    cross = (~flat) & (gm[l] <= 0.17 ** 2) & (nflat >= int(0.2 * 49) - 1)
    interval = ceiling[l] - ground[l]
    print(f'\nL{l} slice_top={geo["slice_h0"] + l * 0.5:+.2f}  rows: x from {lo[0]} down, cols: y from {lo[1]} (0.1 m)')
    for i in range(i0[0], i1[0]):
        row = []
        for j in range(i0[1], i1[1]):
            if ground[l, i, j] < -1e5:
                ch = ' '
            elif interval[i, j] < imin:
                ch = 'I'
            elif flat[i, j]:
                ch = '.'
            elif cross[i, j]:
                ch = 'c'
            else:
                ch = 'S'
            if inf[i, j] > 20 and ch in '.c':
                ch = {'.': ',', 'c': 'x'}[ch]
            if (i, j) in track:
                ch = '#' if ch in '.c' else ('@' if ch in ',x' else '!')
            row.append(ch)
        x = center[0] + (i - dim[0] // 2) * res
        print(f'{x:6.1f} ' + ''.join(row))
print("\nlegend: . flat  c crossable  , flat but inflated>20  x crossable but inflated>20  I interval barrier  S slope barrier\n"
      "        # track on free cell  @ track on inflated cell  ! track on barrier cell")
