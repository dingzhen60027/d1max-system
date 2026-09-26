"""Disposable browser-QA server. Never scans production data or contacts robot."""
import os
import sys
import tempfile
from pathlib import Path

import yaml

app_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(app_root))
root = Path(tempfile.mkdtemp(prefix='d1max-bag-browser-'))
for name in ('maps', 'bags', 'data'):
    (root / name).mkdir()
os.environ.update(D1MAX_MAPS_ROOT=str(root / 'maps'), D1MAX_MAP_MANAGER_DATA=str(root / 'data'),
                  D1MAX_BAG_DIR=str(root / 'bags'), D1MAX_BAG_LIBRARY_ROOTS='')
from tests.test_bags import fixture_bag
from backend.app import app, bag_recorder
from backend.bags.common import atomic_json
for i, name in enumerate(('一楼大厅 · 测试采集', '楼梯往返 · 测试采集', '不完整记录 · 测试')):
    path = fixture_bag(root / 'bags', f'slam_raw_qa_{i}')
    atomic_json(path / 'd1max_recording.json', {'name': name})
    if i == 2:
        (path / 'metadata.yaml').unlink()
config = bag_recorder.config()
config['transport'].update(endpoint='tcp/127.0.0.1:1', domain_id=184)
bag_recorder.config = lambda: config
print('BAG_QA_ROOT=' + str(root), flush=True)
if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='127.0.0.1', port=18766)
