#!/usr/bin/env python3
"""Read-only central-IMU availability audit; stdout JSON, no ROS or bag reading."""
import csv
import hashlib
import json
from decimal import Decimal
from pathlib import Path

import numpy as np

ROOT = Path('/home/dndx/d1max_nav_ws')
CACHE = ROOT / 'experiments/central_imu_quality_20260918/central.npz'
RUNS = {
    'central_device_front_180s': ROOT / 'experiments/lio_frontend_reliability_20260919/runs/central_device_paired_front_180s_20260919',
    'central_original_full_frontend': ROOT / 'maps/runs/20260919_125722_855277_central_imu_faster_lio_sc_pgo_zenoh',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def angle(a, b):
    cross = np.linalg.norm(np.cross(a, b), axis=-1)
    dot = np.sum(a*b, axis=-1)
    return np.degrees(np.arctan2(cross, dot))


def groups(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def analyze(run, data):
    adapter = json.loads((run / 'central_imu_adapter.json').read_text())
    assert adapter['config']['input_topic'] == '/imu_driver/imu_central'
    offset = round(adapter['config']['timestamp_offset_sec']*1e9)
    assert offset == -13_000_000
    all_t = data['stamp_ns'] + offset
    source_mask = (all_t >= adapter['first_output_stamp_ns']) & (all_t <= adapter['last_output_stamp_ns'])
    t = all_t[source_mask]
    assert len(t) == adapter['counts']['received'] == adapter['counts']['published']
    assert np.all(np.diff(t) > 0)
    with (run / 'frontend_state.csv').open() as f:
        rows = list(csv.DictReader(f))
    ends = np.array([int(Decimal(r['t'])*Decimal(1_000_000_000)) for r in rows], dtype=np.int64)
    assert np.all(np.diff(ends) > 0)
    xyz = np.array([[float(r[k]) for k in ('x','y','z')] for r in rows])
    g = np.array([[float(r[k]) for k in ('gx','gy','gz')] for r in rows])
    dt = np.diff(ends)*1e-9
    dz = np.diff(xyz[:,2])
    distance = np.linalg.norm(np.diff(xyz[:,:2], axis=0), axis=1)
    speed = distance/dt
    dg = angle(g[:-1], g[1:])
    tilt = np.degrees(np.arctan2(np.linalg.norm(g[:,:2], axis=1), -g[:,2]))
    starts, stops = ends[:-1], ends[1:]
    gap_left, gap_right = t[:-1], t[1:]
    gap_ns = gap_right-gap_left
    covered = (gap_left < ends[-1]) & (gap_right > ends[0])

    def summary(mask):
        indices = np.flatnonzero(mask)
        duration = float(dt[mask].sum())
        travel = float(distance[mask].sum())
        return {
            'interval_count': len(indices), 'total_duration_sec': duration,
            'estimated_xy_distance_m': travel,
            'signed_sum_delta_z_m': float(dz[mask].sum()),
            'sum_downward_delta_z_m': float(-np.minimum(dz[mask],0).sum()),
            'signed_delta_z_per_sec': float(dz[mask].sum()/duration) if duration else None,
            'signed_delta_z_per_xy_m': float(dz[mask].sum()/travel) if travel > .01 else None,
            'estimated_xy_speed_median_mps': float(np.median(speed[mask])) if len(indices) else None,
            'signed_sum_gravity_tilt_change_deg': float(np.diff(tilt)[mask].sum()),
            'gravity_direction_total_variation_deg': float(dg[mask].sum()),
        }

    thresholds = {}
    masks = {}
    detailed_gaps = []
    for threshold in (15,30,50):
        chosen = np.flatnonzero(covered & (gap_ns > threshold*1_000_000))
        affected = np.zeros(len(dt), dtype=bool)
        overlap = np.zeros(len(dt))
        for j in chosen:
            # Strict positive overlap of source availability gap with state-to-state interval.
            use = (starts < gap_right[j]) & (stops > gap_left[j])
            affected |= use
            overlap[use] += (np.minimum(stops[use],gap_right[j])-np.maximum(starts[use],gap_left[j]))*1e-9
        masks[threshold] = affected
        thresholds[str(threshold)] = {
            'source_gaps_intersecting_state_coverage': len(chosen),
            'source_gap_duration_sec_clipped_to_state_coverage': float(overlap.sum()),
            'coverage_duration_fraction_in_source_gaps': float(overlap.sum()/dt.sum()),
            'affected_intervals': int(affected.sum()),
            'affected_interval_fraction': float(affected.mean()),
            'affected': summary(affected), 'unaffected': summary(~affected),
        }
    for j in np.flatnonzero(covered & (gap_ns > 15_000_000)):
        detailed_gaps.append({'left_output_ns':int(gap_left[j]),'right_output_ns':int(gap_right[j]),
                             'duration_ms':float(gap_ns[j]*1e-6),
                             'left_since_first_state_sec':float((gap_left[j]-ends[0])*1e-9),
                             'affected_state_intervals':int(((starts<gap_right[j])&(stops>gap_left[j])).sum())})
    # Source availability only: actual LIO callback/dequeue sequence was not logged.
    idx = np.searchsorted(t, ends, side='right')-1
    assert np.all(idx >= 0) and np.all(idx+1 < len(t))
    tails_ms = (ends-t[idx])*1e-6
    tail_max_index = int(np.argmax(tails_ms))
    segments=[]
    for a,b in groups(~masks[15]):
        duration=(ends[b]-ends[a])*1e-9
        travel=float(distance[a:b].sum())
        if duration < 2:
            continue
        segments.append({'first_state_index':int(a),'last_state_index':int(b),
            'start_since_first_state_sec':float((ends[a]-ends[0])*1e-9),
            'duration_sec':float(duration),'estimated_xy_distance_m':travel,
            'estimated_mean_xy_speed_mps':float(travel/duration),
            'delta_z_m':float(xyz[b,2]-xyz[a,2]),
            'delta_z_per_xy_m':float((xyz[b,2]-xyz[a,2])/travel) if travel>.1 else None,
            'gravity_direction_endpoint_change_deg':float(angle(g[a],g[b])),
            'gravity_tilt_start_deg':float(tilt[a]),'gravity_tilt_end_deg':float(tilt[b]),
            'maximum_source_gap_ms':float(gap_ns[(gap_left<ends[b])&(gap_right>ends[a])].max()*1e-6)})
    speed_strata=[]
    for lower,upper in ((0,.05),(.05,.15),(.15,.30),(.30,2.0),(2.0,float('inf'))):
        base=(speed>=lower)&(speed<upper)
        speed_strata.append({'speed_lower_mps':lower,'speed_upper_mps':upper if np.isfinite(upper) else None,
                             'clean_15ms':summary(base&~masks[15]),'gap_15ms':summary(base&masks[15])})
    cloud=run/'paired_scans.csv'
    if cloud.exists():
        with cloud.open() as f: all_cloud=list(csv.DictReader(f))
        admitted=[r for r in all_cloud if r['admitted']=='True']
        cloud_evidence={'available':True,'file':str(cloud),'sha256':sha(cloud),'admitted':len(admitted),
            'first_header_ns':int(admitted[0]['stamp_ns']),'last_header_ns':int(admitted[-1]['stamp_ns']),
            'interpretation':'Observed admitted headers establish selected scan window; output state times supply frame-end intervals. No per-point scan-end or exact header-to-state pairing reconstructed.'}
    else:
        cloud_evidence={'available':False,'interpretation':'No paired_scans.csv. Only consecutive frontend state intervals; no raw/admitted cloud headers invented.'}
    return {'run':str(run),'analysis_target':'frontend_state, not PGO trajectory','timestamp_offset_ns':offset,
        'source_received_published_and_cache_count':len(t),'source_total_gaps_over_ms':{str(k):int((gap_ns>k*1e6).sum()) for k in (15,30,50)},
        'source_maximum_gap_ms':float(gap_ns.max()*1e-6),
        'state_rows':len(rows),'intervals':len(dt),'first_state_ns':int(ends[0]),'last_state_ns':int(ends[-1]),
        'interval_duration_sec_max':float(dt.max()),'coverage_duration_sec':float(dt.sum()),
        'estimated_total_delta_z_m':float(xyz[-1,2]-xyz[0,2]),'estimated_xy_distance_m':float(distance.sum()),
        'gravity_tilt_start_end_deg':[float(tilt[0]),float(tilt[-1])],
        'state_timestamp_rounding_caveat':'15-significant-digit epoch seconds have up to about 5 us rounding; exact gap boundary classifications can differ within that margin.',
        'thresholds_ms':thresholds,
        'source_available_tail_extrapolation_ms':{'maximum':float(tails_ms.max()),'p50':float(np.percentile(tails_ms,50)),
            'p95':float(np.percentile(tails_ms,95)),'p99':float(np.percentile(tails_ms,99)),
            'over_15ms_state_ends':int((tails_ms>15).sum()),'over_30ms_state_ends':int((tails_ms>30).sum()),'over_50ms_state_ends':int((tails_ms>50).sum()),
            'maximum_at_state_index':tail_max_index,'maximum_at_ns':int(ends[tail_max_index]),
            'left_source_ns':int(t[idx[tail_max_index]]),'right_source_ns':int(t[idx[tail_max_index]+1]),
            'not_actual_consumption_trace':True},
        'clean_15ms_segment_count_at_least_2sec':len(segments),
        'clean_15ms_segments_at_least_2sec':sorted(segments,key=lambda x:x['duration_sec'],reverse=True)[:15],
        'clean_15ms_segments_most_downward':sorted(segments,key=lambda x:x['delta_z_m'])[:10],
        'estimated_speed_stratification':speed_strata,
        'source_gap_examples_first20':detailed_gaps[:20],
        'source_gap_examples_largest20':sorted(detailed_gaps,key=lambda x:x['duration_ms'],reverse=True)[:20],
        'cloud_header_evidence':cloud_evidence,
        'source_evidence_sha256':{str(p):sha(p) for p in (CACHE,run/'central_imu_adapter.json',run/'frontend_state.csv')}}


def main():
    with np.load(CACHE, allow_pickle=False) as data:
        runs={name:analyze(path,data) for name,path in RUNS.items()}
    report={'schema_version':1,'topic':'/imu_driver/imu_central','runs':runs,
        'limits':[
            'No ROS or full bag reading. Cached raw source stamps adjusted by run-recorded -13 ms.',
            'State intervals are nominal integration windows between consecutive emitted frontend states, not per-IMU filter consumption logs; first-state initialization interval excluded.',
            'Gap overlap means only source-availability gaps. No fabricated packet timestamps or gap interpolation.',
            'A gap-free segment can retain error from earlier gaps; correlation/no-correlation does not establish causality.',
            'Height/gravity are estimates without independent motion truth. Different duration, speed, distance and rotations confound affected/unaffected comparisons.',
            'Repeated six-axis values at distinct timestamps are not absent timestamps; this audit does not certify signal freshness or driver internal buffering.',
            'Full run uses its original extrinsic and dual lidar; short run uses device-rotation candidate and front-only. Do not treat cross-run differences as a single-variable timing experiment.'
        ]}
    print(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
