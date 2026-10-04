"""Read-only acquisition/receipt timing audit of the replay's raw inputs.

The one fixed replay clock offset is explicitly NOT a physical calibration.
LiDAR header age is scan-start age, not solve time or per-point scan-end age.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import yaml


def audit(bag, seconds, clock_offset_ns):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from d1max_localization.realtime_navigation_output import quantiles_ms
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    required=('/front_lidar','/rear_lidar','/front_lidar/imu')
    selected=(*required,'/rear_lidar/imu','/imu_driver/imu_central')
    result=dict(bag=str(bag),window_seconds=seconds,clock_receipt_offset_ns=clock_offset_ns,
        physical_clock_calibration=False,lidar_header_reference='scan_start_not_processing_time',topics={})
    # Handle split recordings without silently auditing only the first DB.
    rows={key:[] for key in selected}
    first=None
    for relative in metadata['relative_file_paths']:
        with sqlite3.connect('file:'+str(bag/relative)+'?mode=ro',uri=True) as db:
            types={name:(tid,get_message(kind)) for tid,name,kind in db.execute('SELECT id,name,type FROM topics') if name in selected}
            if types:
                ids=tuple(item[0] for name,item in types.items() if name in required)
                if ids:
                    start=db.execute('SELECT MIN(timestamp) FROM messages WHERE topic_id IN (%s)' %
                        ','.join('?' for _ in ids),ids).fetchone()[0]
                    if start is not None: first=start if first is None else min(first,start)
                if first is None: continue
            for name,(tid,message_type) in types.items():
                for receipt,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<=? ORDER BY timestamp,id',
                        (tid,first+round(seconds*1e9))):
                    message=deserialize_message(data,message_type)
                    source=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                    rows[name].append((receipt,source))
    if first is None: raise ValueError('recording_has_no_raw_topics')
    end=first+round(seconds*1e9)
    for name,values in rows.items():
        values=sorted(v for v in values if first<=v[0]<=end)
        if not values:
            if name in required: raise ValueError('raw_topic_missing:'+name)
            result['topics'][name]=dict(present=False)
            continue
        receipt=[v[0] for v in values];source=[v[1] for v in values]
        receipt_gaps=[(b-a)*1e-9 for a,b in zip(receipt,receipt[1:])]
        source_gaps=[(b-a)*1e-9 for a,b in zip(source,source[1:])]
        source_ages=[(r+clock_offset_ns-s)*1e-9 for r,s in values]
        result['topics'][name]=dict(present=True,used_by_current_lio=name in required,samples=len(values),
            mapped_age_is_physical_latency=False,
            optional_imu_shared_clock_verified=False if name not in required else None,
            receipt_interval_ms=quantiles_ms(receipt_gaps),header_interval_ms=quantiles_ms(source_gaps),
            mapped_header_age_ms=quantiles_ms(source_ages),
            minimum_mapped_header_age_ms=1000*min(source_ages),
            invalid_nonpositive_headers=sum(s<=0 for s in source),
            nonincreasing_headers=sum(v<=0 for v in source_gaps),
            receipt_gaps_over_100ms=sum(v>.1 for v in receipt_gaps),
            receipt_gaps_over_20ms=sum(v>.02 for v in receipt_gaps))
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    report=json.loads(args.report.read_text())
    result=audit(Path(report['bag']),report['played_seconds'],report['clock_receipt_offset_ns'])
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
