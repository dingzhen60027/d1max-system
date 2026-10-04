#!/usr/bin/env python3
"""Capture actual navigation sources/artifacts without importing ROS or the SDK."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/d1max_pct_planner'))
from d1max_pct_planner import paths  # noqa: E402  (no ROS import)

WS = paths.nav_root()
SDK = paths.app_root()/'sdk_bridge_ws'
WEB = paths.app_root()/'map_manager'
EXT = {'.py', '.cpp', '.cc', '.h', '.hpp', '.xml', '.yaml', '.yml', '.json',
       '.json5', '.msg', '.srv', '.action', '.rviz', '.md', '.cmake', '.sh',
       '.jsx','.js','.mjs','.tsx','.ts','.css'}


def record(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return dict(path=str(path), resolved=str(path.resolve()), bytes=path.stat().st_size,
                sha256=h.hexdigest())


def capture(out, stage):
    out.mkdir(parents=True, exist_ok=True)
    if (out/(stage+'_inventory.json')).exists() or (out/(stage+'_sources.tar.gz')).exists():
        raise ValueError('baseline_already_exists_do_not_overwrite:'+stage)
    sources = []
    roots = [p for p in (WS/'src').iterdir() if p.name.startswith('d1max_')]
    roots += [WS/'src/scan_planner_vendor', WS/'src/pct_planner_vendor',
              SDK/'src/d1max_sdk_bridge', WS/'tools',WS/'docs',
              WEB/'backend',WEB/'frontend/src',WEB/'tests']
    for root in roots:
        for p in sorted(root.rglob('*')):
            if (p.is_file() and not any(x in p.parts for x in
                ('__pycache__', '.git', 'node_modules', '.pytest_cache', 'build', 'install'))
                and (p.suffix in EXT or p.name == 'CMakeLists.txt')):
                sources.append(p)
    artifacts = [WS/'install/scan_planner/lib/scan_planner/scan_planner_node',
                 WS/'install/d1max_trajectory_tracker/lib/d1max_trajectory_tracker/trajectory_tracker',
                 SDK/'install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge',
                 WS/'maps/processed/sc_pgo_20260923_crossfloor_complete_001/manifest.json',
                 WS/'maps/processed/sc_pgo_20260923_crossfloor_complete_001/route.yaml',
                 WS/'experiments/navigation_system_v2_20260927/bag_input_identity.json']
    for p in sorted((WS/'log').glob('**/session.json')):
        if p.is_file():
            artifacts.append(p)
            artifacts.extend(q for q in p.parent.iterdir()
                             if q.is_file() and q.suffix in ('.yaml', '.xml'))
    inventory = dict(schema=1, stage=stage, captured_at=datetime.now(timezone.utc).isoformat(),
        head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WS, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], cwd=WS, text=True),
        source_records=[record(p) for p in sources],
        artifacts=[record(p) if p.is_file() else dict(path=str(p), missing=True) for p in artifacts],
        safety=dict(ros_initialized=False, sdk_connected=False, production_deployed=False,
                    physical_acceptance=False),
        note='Pre-existing dirty and untracked sources included. Bag chunk hashes are prior evidence, not rehashed here.')
    archive = out/(stage+'_sources.tar.gz')
    with tarfile.open(archive, 'w:gz') as tar:
        for p in sources:
            base = SDK if p.is_relative_to(SDK) else WEB if p.is_relative_to(WEB) else WS
            label='sdk/' if base==SDK else 'web/' if base==WEB else 'workspace/'
            tar.add(p, arcname=label+str(p.relative_to(base)), recursive=False)
    inventory['source_archive'] = record(archive)
    target = out/(stage+'_inventory.json')
    target.write_text(json.dumps(inventory, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(inventory=str(target), sources=len(sources), archive=str(archive))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=('before', 'after'), required=True)
    args = parser.parse_args()
    capture(args.output, args.stage)
