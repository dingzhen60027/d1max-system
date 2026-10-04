#!/usr/bin/env python3
"""Compare this turn's frozen sources, not a dirty worktree against Git HEAD.

Coverage added after the initial capture is reported separately. No deployment,
Git mutation, ROS initialization, or network connection occurs.
"""
import argparse
import difflib
import json
from pathlib import Path
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    root = args.evidence.resolve()
    destination = root/'source_delta.json'
    patch = root/'source_delta.patch'
    if destination.exists() or patch.exists():
        raise ValueError('existing_comparison_is_immutable')
    inventories = [json.loads((root/(stage+'_inventory.json')).read_text())
                   for stage in ('before', 'after')]
    records = [{row['path']: row for row in inventory['source_records']}
               for inventory in inventories]
    before, after = records
    changed = sorted(path for path in before.keys() & after.keys()
                     if before[path]['sha256'] != after[path]['sha256'])
    removed = sorted(before.keys() - after.keys())
    added = sorted(after.keys() - before.keys())
    # Initial capture covered WS src/tools and SDK; docs/Web were added later.
    newly_covered = [p for p in added if '/map_manager/' in p or '/d1max_nav_ws/docs/' in p]
    created = [p for p in added if p not in newly_covered]
    report = dict(schema=1, baseline_head=inventories[0]['head'],
                  modified=changed, added_within_original_coverage=created,
                  deleted_within_original_coverage=removed,
                  newly_covered_not_claimed_created=newly_covered,
                  note='Existing dirty changes included in before baseline are not attributed to this turn.',
                  physical_acceptance=False, deployed=False)
    archives = [tarfile.open(root/(stage+'_sources.tar.gz')) for stage in ('before', 'after')]
    try:
        indexed = []
        for archive in archives:
            indexed.append({entry.name: archive.extractfile(entry).read().decode('utf-8', errors='replace')
                            for entry in archive.getmembers() if entry.isfile()})
        with patch.open('w') as stream:
            for name in sorted(indexed[0].keys() | indexed[1].keys()):
                if name.startswith(('web/', 'workspace/docs/')) and name not in indexed[0]:
                    continue
                stream.writelines(difflib.unified_diff(
                    indexed[0].get(name, '').splitlines(keepends=True),
                    indexed[1].get(name, '').splitlines(keepends=True),
                    fromfile='before/'+name, tofile='after/'+name))
    finally:
        for archive in archives:
            archive.close()
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({key: len(value) for key,value in report.items() if isinstance(value,list)}))


if __name__ == '__main__':
    main()
