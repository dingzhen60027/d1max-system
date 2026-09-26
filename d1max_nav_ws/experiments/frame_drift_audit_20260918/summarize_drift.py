#!/usr/bin/env python3
"""Summarize existing run products only; never launches a ROS node."""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

out = Path(__file__).resolve().parent
geometry = json.loads((out / 'geometry/geometry_report.json').read_text())
floors = [f for f in geometry['sampled_floors'] if 'error' not in f]
distance = np.array([f['distance_m'] for f in floors])
height = np.array([f['frontend_floor_xyz_m'][2] for f in floors])
height -= height[0]
font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots(figsize=(10, 4.2))
ax.axhline(0, color='0.5', lw=1, ls='--')
ax.plot(distance, height, 'o-', color='#b64e23', lw=2, markersize=5)
ax.set_title('Faster-LIO 原始地图：沿途拟合地面的高度变化', fontproperties=font, fontsize=15, pad=12)
ax.set_xlabel('前端估计的累计行走距离（米）', fontproperties=font)
ax.set_ylabel('相对起点地面的高度（米）', fontproperties=font)
ax.grid(alpha=.2)
for i, offset in [(5, (12, 12)), (6, (12, -28)), (19, (-120, 18))]:
    ax.annotate(f'{distance[i]:.1f} 米 / {height[i]:+.2f} 米', (distance[i], height[i]),
                xytext=offset, textcoords='offset points', fontproperties=font,
                arrowprops={'arrowstyle': '-', 'color': '0.5'})
fig.text(.5, .01, '20 处地面抽样；连线仅辅助阅读；不是实测地面高程真值。来源：9 月 17 日 19:01 完整建图结果。',
         ha='center', fontproperties=font, fontsize=9)
fig.tight_layout(rect=(0, .045, 1, 1))
fig.savefig(out / 'frontend_floor_drift.png', dpi=160)

source = Path('/home/dndx/d1max_nav_ws/maps/runs/20260917_frontend_no_loop_correspondence_fix_02/frontend_state.csv')
a = np.genfromtxt(source, delimiter=',', names=True)
t = a['t'] - a['t'][0]
g = np.column_stack([a[k] for k in ('gx', 'gy', 'gz')])
tilt = np.degrees(np.arccos(np.clip(-g[:, 2] / np.linalg.norm(g, axis=1), -1, 1)))
samples = []
for sec in (0, 30, 60, 120, 180, float(t[-1])):
    i = int(np.argmin(abs(t - sec)))
    samples.append({'elapsed_sec': float(t[i]), 'sensor_z_m': float(a['z'][i]),
                    'gravity_tilt_from_initial_world_vertical_deg': float(tilt[i]),
                    'accelerometer_bias': [float(a[k][i]) for k in ('bax', 'bay', 'baz')],
                    'features': int(a['features'][i]),
                    'horizontal_features': int(a['horizontal_features'][i]),
                    'point_plane_residual_rmse_m': float(a['residual_rmse'][i])})
summary = {'source': str(source), 'same_run_as_full_loop': False, 'complete': False,
           'interpretation': 'Later modified frontend, interrupted partial replay; estimated gravity tilt is not independently measured attitude error.',
           'duration_sec': float(t[-1]), 'samples': samples,
           'maximum_gravity_tilt_deg': float(tilt.max())}
(out / 'partial_state_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(out / 'frontend_floor_drift.png')
print(out / 'partial_state_summary.json')
