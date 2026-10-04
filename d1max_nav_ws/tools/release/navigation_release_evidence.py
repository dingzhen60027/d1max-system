#!/usr/bin/env python3
"""File-only evidence capture. Never launches ROS, SDK, a service or a build.

The initial inventory was taken after work began; a delta against it is NOT a
clean Git diff. Untracked and pre-existing edits remain part of the snapshot.
This manifest is an engineering record, not deployment or safety acceptance.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PACKAGES = (
    'd1max_localization', 'd1max_navigation', 'd1max_navigation_bt',
    'd1max_navigation_bt_interfaces', 'd1max_pct_scan', 'd1max_pct_planner',
    'd1max_planning_interfaces', 'd1max_scan_planner', 'd1max_trajectory_tracker',
    'scan_planner_vendor',
)
EXTENSIONS = {'.py', '.cpp', '.h', '.hpp', '.xml', '.yaml', '.yml', '.msg',
              '.srv', '.action', '.rviz', '.md', '.cmake', '.sh'}


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def file_record(path):
    return dict(resolved=str(path.resolve()), bytes=path.stat().st_size,
                sha256=digest(path))


def evidence(workspace, experiment):
    initial = json.loads((experiment/'start_inventory.json').read_text())
    source = {}
    for package in PACKAGES:
        for path in sorted((workspace/'src'/package).rglob('*')):
            if (path.is_file() and '__pycache__' not in path.parts
                    and '.git' not in path.parts
                    and (path.suffix in EXTENSIONS or path.name == 'CMakeLists.txt')):
                source[str(path.relative_to(workspace))] = digest(path)
    for relative in ('tools/release/navigation_release_evidence.py', 'tools/validation/run_navigation_unit_tests.py',
                     'docs/reports/NAVIGATION_SYSTEM_V2_IMPLEMENTATION_20260927.md',
                     'experiments/navigation_system_v2_20260927/reproduce_offline.sh'):
        source[relative] = digest(workspace/relative)
    artifacts = {}
    for value in initial['artifacts']:
        path = Path(value)
        artifacts[value] = file_record(path) if path.is_file() else dict(missing=True)
        if path.name == 'session.json' and path.is_file():
            # Historical effective inputs, not regenerated from current defaults.
            for pattern in ('*.yaml', '*.xml', '*.rviz'):
                for effective in sorted(path.parent.glob(pattern)):
                    artifacts[str(effective)] = file_record(effective)
    for prefix, relative in (
        ('native/install', 'scan_planner/lib/scan_planner/scan_planner_node'),
        ('install', 'd1max_navigation_bt/lib/d1max_navigation_bt/navigator_node'),
        ('tracker/install', 'd1max_trajectory_tracker/lib/d1max_trajectory_tracker/trajectory_tracker'),
    ):
        path = experiment/prefix/relative
        artifacts[str(path)] = file_record(path) if path.is_file() else dict(missing=True)
    tests = {}
    for path in sorted(experiment.rglob('*.xml')):
        if 'install' in path.relative_to(experiment).parts:
            continue
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            continue
        if root.tag not in ('testsuite', 'testsuites'):
            continue
        cases = list(root.iter('testcase'))
        tests[str(path.relative_to(experiment))] = dict(
            sha256=digest(path), cases=len(cases),
            failures=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
            skipped=sum(c.find('skipped') is not None for c in cases))
    old = initial['source_sha256']
    binary = str(workspace/'install/scan_planner/lib/scan_planner/scan_planner_node')
    return dict(schema=2, captured_at_utc=datetime.now(timezone.utc).isoformat(),
        workspace=str(workspace), published_baseline=initial['published_baseline'],
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=workspace,text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],cwd=workspace,text=True),
        source_sha256=source,
        delta_from_non_pristine_start=dict(
            changed=[p for p in source if p in old and source[p] != old[p]],
            not_in_start_inventory=[p for p in source if p not in old]),
        artifacts=artifacts, test_reports=tests,
        test_count_note='Reports may overlap; never sum these as independent cases.',
        production_native_binary_unchanged=(artifacts[binary].get('sha256') ==
            initial['artifacts'][binary]['sha256']),
        status='development_not_deployed_not_execution_accepted',
        preexisting_and_untracked_edits_preserved=True,
        clean_pre_edit_baseline_available=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--experiment',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    result=evidence(args.workspace.resolve(strict=True),args.experiment.resolve(strict=True))
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output.resolve()),
        sources=len(result['source_sha256']), reports=len(result['test_reports']),
        production_native_binary_unchanged=result['production_native_binary_unchanged'])))


if __name__ == '__main__':
    main()
