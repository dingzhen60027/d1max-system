#!/usr/bin/env python3
"""Count expected dual pairs from 12-byte CDR prefixes, without ROS or clouds."""
from collections import Counter, deque
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import struct
from types import SimpleNamespace

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
BAG = Path('/home/dndx/d1max_nav_ws/bags/slam_raw_20260917_171716_fe8f38')
INIT_CUTOFF_NS = 1772316443262103536
MAX_DELTA_NS = 5_000_000


def header_message(stamp):
    sec, nsec = divmod(stamp, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nsec)))


def run_pairing(rows, adapter):
    counts = Counter()
    pairer = adapter.ScanPairer(counts, max_delta_ns=MAX_DELTA_NS)
    independent = {'front': deque(), 'rear': deque()}
    details = {'stale_header': Counter(), 'queue_capacity': Counter()}
    pending_max = {'front': 0, 'rear': 0}
    pairs, independently_paired = [], []
    for row in rows:
        source, stamp = row['source'], row['stamp_ns']
        counts[source+'_received'] += 1
        if stamp < INIT_CUTOFF_NS:
            counts[source+'_skipped_startup'] += 1
            continue
        for front, rear in pairer.add(source, header_message(stamp)):
            pairs.append((adapter.stamp_ns(front), adapter.stamp_ns(rear)))
        own = independent[source]
        own.append(stamp)
        if len(own) > 8:
            own.popleft()
            details['queue_capacity'][source] += 1
        front, rear = independent['front'], independent['rear']
        while front and rear:
            delta = front[0]-rear[0]
            if abs(delta) <= MAX_DELTA_NS:
                independently_paired.append((front.popleft(), rear.popleft()))
            else:
                older = 'front' if delta < 0 else 'rear'
                independent[older].popleft()
                details['stale_header'][older] += 1
        for source in ('front', 'rear'):
            pending_max[source] = max(pending_max[source], len(independent[source]))
    assert pairs == independently_paired
    final_pending = {source: len(queue) for source, queue in pairer.queues.items()}
    for source in ('front', 'rear'):
        assert counts[source+'_unpaired'] == details['stale_header'][source]+details['queue_capacity'][source]
        assert counts[source+'_received'] == (counts[source+'_skipped_startup']+len(pairs)+
                                             counts[source+'_unpaired']+final_pending[source])
    skew = np.array([abs(a-b) for a,b in pairs], dtype=np.int64)
    return {'counts': dict(counts), 'paired_scans': len(pairs), 'pending_unpaired': final_pending,
            'unpaired_reasons': {key: dict(value) for key, value in details.items()},
            'max_queue_after_pairing': pending_max,
            'first_pair_header_ns': list(pairs[0]) if pairs else None,
            'last_pair_header_ns': list(pairs[-1]) if pairs else None,
            'last_pending_header_ns': {source: list(queue) for source,queue in independent.items()},
            'abs_pair_skew_ns': {key: float(value) for key,value in zip(['min','median','p95','max'],np.percentile(skew,[0,50,95,100]))} if pairs else {},
            'count_conservation': True, 'independent_pairer_crosscheck': True}


def main():
    meta_path = BAG/'metadata.yaml'
    meta = yaml.safe_load(meta_path.read_text())['rosbag2_bagfile_information']
    spec = importlib.util.spec_from_file_location('pair_adapter', HERE/'adapter.py')
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    rows, files = [], []
    for file_index, name in enumerate(meta['relative_file_paths']):
        path = BAG/name
        db = sqlite3.connect(path.as_uri()+'?mode=ro', uri=True)
        db.execute('PRAGMA query_only=ON')
        topics = dict(db.execute('SELECT name,id FROM topics'))
        sources = {topics['/front_lidar']: 'front', topics['/rear_lidar']: 'rear'}
        per_file = Counter()
        for mid, received, topic, prefix in db.execute(
                'SELECT id,timestamp,topic_id,substr(data,1,12) FROM messages WHERE topic_id IN (?,?) ORDER BY timestamp,id', tuple(sources)):
            assert len(prefix) == 12 and prefix[:2] == b'\x00\x01', 'Expected little-endian CDR header'
            sec,nsec = struct.unpack_from('<iI',prefix,4)
            assert 0 <= nsec < 1_000_000_000
            source = sources[topic]
            rows.append({'record_ns': received, 'file_index': file_index, 'message_id': mid,
                         'source': source, 'stamp_ns': sec*1_000_000_000+nsec})
            per_file[source] += 1
        db.close()
        files.append({'path': str(path), 'counts': dict(per_file)})
    rows.sort(key=lambda row: (row['record_ns'],row['file_index'],row['message_id']))
    for source in ('front','rear'):
        stamps = np.array([r['stamp_ns'] for r in rows if r['source']==source], dtype=np.int64)
        assert np.all(np.diff(stamps) > 0)
        expected = next(t['message_count'] for t in meta['topics_with_message_count'] if t['topic_metadata']['name']=='/'+source+'_lidar')
        assert len(stamps) == expected
    report = {'bag': str(BAG), 'init_cutoff_ns': INIT_CUTOFF_NS, 'max_header_delta_ns': MAX_DELTA_NS,
              'queue_capacity': 8, 'read_only': True, 'ros_initialized': False,
              'point_payloads_deserialized': 0, 'sql_payload_selection': 'substr(data,1,12)',
              'prefix_bytes_returned': len(rows)*12,
              'record_order': 'bag record_ns, file_index, SQLite message id',
              'source_files': files,
              'metadata_sha256': hashlib.sha256(meta_path.read_bytes()).hexdigest(),
              'adapter_sha256': hashlib.sha256((HERE/'adapter.py').read_bytes()).hexdigest(),
              'full_bag': run_pairing(rows, adapter)}
    full = report['full_bag']
    assert full['counts']['front_skipped_startup'] == 20
    assert full['counts']['rear_skipped_startup'] == 19
    # Describe a source prefix exactly matching observed short-run input counts,
    # when such a prefix exists; do not assume record-time duration == sensor time.
    seen = Counter()
    for index,row in enumerate(rows):
        seen[row['source']] += 1
        if seen == {'front': 585,'rear': 583}:
            report['prefix_at_585_front_583_rear'] = run_pairing(rows[:index+1], adapter)
            report['prefix_at_585_front_583_rear']['last_record_ns'] = row['record_ns']
            break
    report['limitations'] = [
        'Exact source expectation assumes all recorded clouds are delivered, same per-topic order, and initialization cutoff as supplied.',
        'ROS callback cross-topic scheduling can differ from bag arrival order; capacity-drop counts may then differ. Inspect queue_capacity count.',
        'Only pairing and startup header gate are audited; cloud validity, point counts, IMU coverage, end-of-run pending output and algorithm admission are separate.',
        'Clouds arriving before runtime initialization completes may be skipped even with stamp >= cutoff; compare actual runtime startup counts.',
        'Bag original missing scans are not runtime message loss; compare counts to this source baseline, not an arbitrary percentage.']
    output = HERE/'bag_pair_audit.json'
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(output),'full_bag':full,
                      'short_prefix':report.get('prefix_at_585_front_583_rear')},indent=2))


if __name__ == '__main__':
    main()
