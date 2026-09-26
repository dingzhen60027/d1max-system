"""Publish only offline planning artifacts into the existing local viewer."""
import json
import argparse
import shutil
from pathlib import Path
root=Path(__file__).resolve().parents[2]
out=root/'data/traversability/sc_pgo_20260919_ground_20260921'
parser=argparse.ArgumentParser();parser.add_argument('--smooth',action='store_true');args=parser.parse_args()
base=Path('/home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260919_ground_20260921')
source=base/('global_path_smooth' if args.smooth else 'global_path_reachable')
plan=json.loads((source/'path_report.json').read_text());assert plan['success'] and not plan['robot_commands_sent']
for name in ['planned_path.bin','path.csv','path_report.json','planner.yaml']:
    shutil.copy2(source/name,out/name)
shutil.copy2(source.parent/'global_path_checked/path_report.json',out/'unreachable_test.json')
report=json.loads((out/'report.json').read_text())
report['planning']={'file':'planned_path.bin','length_m':plan['length_m'],'start':plan['start_xyz'],'goal':plan['goal_xyz'],'smooth':args.smooth}
extra=[['path.csv','路径 XYZ'],['planner.yaml','规划配置'],['path_report.json','规划结果'],['unreachable_test.json','跨区不可达记录']]
if args.smooth:
    shutil.copy2(base/'global_path_reachable/path.csv',out/'original_path.csv')
    shutil.copy2(base/'global_path_reachable/planner.yaml',out/'grid_planner.yaml')
    extra.extend([['original_path.csv','原始折线路径'],['grid_planner.yaml','栅格规划配置']])
report['downloads']=[item for item in report['downloads'] if item[0] not in {e[0] for e in extra}]+extra
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
