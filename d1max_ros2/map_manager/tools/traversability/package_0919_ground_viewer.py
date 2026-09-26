"""Package existing measured ground results for the read-only local viewer."""
import json
import shutil
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
from extract_ground import read_source

root=Path(__file__).resolve().parents[2]
source=Path('/home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260919_ground_20260921')
out=root/'data/traversability/sc_pgo_20260919_ground_20260921'
out.mkdir(parents=True,exist_ok=True)
report=json.loads((source/'report.json').read_text())
cfg=yaml.safe_load((source/'pipeline.yaml').read_text())
ground=read_source(source/'ground.pcd')[:,:3]
color=np.array([65,215,161],dtype=float)/255
np.column_stack((ground,np.tile(color,(len(ground),1)))).astype('<f4').tofile(out/'ground.bin')
cloud=o3d.io.read_point_cloud(report['source']).voxel_down_sample(.20)
xyz=np.asarray(cloud.points)
np.column_stack((xyz,np.full(xyz.shape,.39))).astype('<f4').tofile(out/'source.bin')
poses=np.loadtxt(cfg['source']['poses']).reshape(-1,3,4)[:,:,3]
poses.astype('<f4').tofile(out/'recorded_trajectory.bin')
for name in ['ground.pcd','ground.ply','pipeline.yaml','ground_preview.png']:
    shutil.copy2(source/name,out/name)
report.update(kind='ground_only',name='SC-PGO · 09-19 · 地面提取',display_point_size_px=2,
    layers=[dict(id='ground',label='提取地面',points=len(ground),file='ground.bin',bounds=report['ground_bounds'],color=[65,215,161])],
    downloads=[['ground.pcd','地面 PCD'],['ground.ply','地面 PLY'],['pipeline.yaml','处理配置'],['report.json','检查报告'],['ground_preview.png','预览图']])
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(out)
