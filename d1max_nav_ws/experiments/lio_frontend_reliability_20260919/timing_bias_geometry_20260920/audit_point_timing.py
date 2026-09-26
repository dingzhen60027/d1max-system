#!/usr/bin/env python3
"""Bounded timing audit: cached IMU plus ten original point-cloud messages.

No ROS initialization, publication, replay, SLAM or configuration edits.
Reconstruction is checked against saved runtime headers/counts/scan ends;
it is not a byte-level capture of live intermediate messages.
"""
import csv
import hashlib
import json
import re
import sqlite3
import struct
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


ROOT=Path('/home/dndx/d1max_nav_ws')
HERE=Path(__file__).resolve().parent
BAG=ROOT/'bags/slam_raw_20260917_171716_fe8f38'
RUN=ROOT/'experiments/lio_frontend_reliability_20260919/runs/internal_device_paired_front_180s_20260919'
CACHE=ROOT/'experiments/central_imu_quality_20260918/front.npz'


def qstats(values):
    a=np.asarray(values,dtype=float)
    return dict(zip(['min','median','p95','p99','max'],np.quantile(a,[0,.5,.95,.99,1]).tolist()))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_nearest_cloud(db,topic_id,header_ns,record_ns):
    # Read tiny CDR headers in a bounded recording-time interval first; fetch
    # only the selected full cloud. This never scans the full bag payload.
    rows=db.execute('SELECT id,timestamp,substr(data,1,12) FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp LIMIT 24',
                    (topic_id,int(record_ns-500_000_000),int(record_ns+500_000_000))).fetchall()
    def stamp(row):
        sec,nsec=struct.unpack_from('<iI',row[2],4)
        return sec*10**9+nsec
    chosen=min(rows,key=lambda row:abs(stamp(row)-int(header_ns)))
    if abs(stamp(chosen)-int(header_ns))>2_000_000:
        raise ValueError('No bounded cloud candidate near desired paired header')
    payload=db.execute('SELECT data FROM messages WHERE id=?',(chosen[0],)).fetchone()[0]
    msg=deserialize_message(payload,PointCloud2)
    fields={field.name:field for field in msg.fields}
    names=['x','y','z','intensity','ring','timestamp']
    dtype=np.dtype({'names':names,'formats':['<f4','<f4','<f4','<f4','<u2','<f8'],
                    'offsets':[fields[key].offset for key in names],'itemsize':msg.point_step})
    assert not msg.is_bigendian and fields['timestamp'].datatype==8
    cloud=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                     strides=(msg.row_step,msg.point_step)).reshape(-1).copy()
    xyz=np.column_stack([cloud[key] for key in ['x','y','z']]).astype(float)
    ranges=np.linalg.norm(xyz,axis=1)
    valid=np.isfinite(xyz).all(axis=1)&(ranges>=.35)&(ranges<=120.)
    meta={'header_ns':stamp(chosen),'record_ns':chosen[1],'frame_id':msg.header.frame_id,
          'rows':msg.height,'columns':msg.width,'point_step':msg.point_step,
          'timestamp_datatype':fields['timestamp'].datatype,'point_count':len(cloud),
          'adapter_valid_count':int(valid.sum()),'candidate_headers_examined':len(rows)}
    return cloud,xyz,valid,meta


def main():
    meta=yaml.safe_load((BAG/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    db=sqlite3.connect((BAG/meta['relative_file_paths'][0]).as_uri()+'?mode=ro',uri=True)
    topics=dict(db.execute('SELECT name,id FROM topics'))
    imu=np.load(CACHE)
    imu_ns=imu['stamp_ns']; imu_s=imu_ns.astype(float)*1e-9
    state=np.genfromtxt(RUN/'frontend_state.csv',delimiter=',',names=True)
    ends=state['t']; previous=np.searchsorted(imu_s,ends,side='right')-1
    following=previous+1
    age_ms=(ends-imu_s[previous])*1000
    bracket_ms=(imu_s[following]-imu_s[previous])*1000
    with (RUN/'paired_scans.csv').open() as stream:
        paired=[row for row in csv.DictReader(stream) if row['admitted']=='True']
    paired_ns=np.array([int(row['stamp_ns']) for row in paired],dtype=np.int64)
    # Five representative state frames: early, two interior, final, and the
    # largest source-derived scan-end extrapolation interval.
    indices=list(dict.fromkeys([0,400,900,len(ends)-1,int(np.argmax(age_ms))]))
    assert len(indices)==5
    R_N_L=Rotation.from_quat([-.499867275,.503186620,.497953310,.498977388]).as_matrix()
    report={'run':str(RUN),'bag':str(BAG),'cached_imu':str(CACHE),'cached_imu_sha256':sha(CACHE),
            'read_only':True,'ros_initialized':False,'full_cloud_messages_read':10,
            'new_full_bag_scan':False,'source_clock_interpretation':'Sensor header time and bag receipt time are separate; only sensor header/point time is used for deskew.',
            'sampled_frames':[]}
    for state_index in indices:
        end=float(ends[state_index])
        pair_index=int(np.argmin(np.abs(paired_ns.astype(float)*1e-9-(end-.1))))
        pair_stamp=int(paired_ns[pair_index])
        imu_index=int(np.argmin(np.abs(imu_ns-pair_stamp)))
        recording_ns=int(imu['record_ns'][imu_index])
        front,xyz,valid,mf=read_nearest_cloud(db,topics['/front_lidar'],pair_stamp,recording_ns)
        back,bxyz,bvalid,mr=read_nearest_cloud(db,topics['/rear_lidar'],pair_stamp,recording_ns)
        all_times=front['timestamp']; times=all_times[valid]; back_times=back['timestamp'][bvalid]
        assert np.isfinite(times).all() and np.isfinite(back_times).all()
        header_s=mf['header_ns']//10**9+(mf['header_ns']%10**9)*1e-9
        # This sample must enter decodePointTime's absolute-seconds branch.
        assert np.all(times>1e8) and np.max(np.abs(times-header_s))<86400
        base=float(min(times.min(),back_times.min()))
        adapter_header_ns=int(np.floor(base*1e9+.5))
        adapter_header_s=adapter_header_ns//10**9+(adapter_header_ns%10**9)*1e-9
        order=np.argsort(times,kind='stable')
        times=times[order]
        relative_ms=((times-base)*1000).astype(np.float32)
        normalized_xyz=(xyz[valid][order]@R_N_L.T).astype(np.float32)
        retained=np.sum(normalized_xyz**2,axis=1)>.5**2
        last_offset=float(relative_ms[retained][-1])/1000
        predicted_end=adapter_header_s+last_offset
        reconstructed=adapter_header_s+relative_ms.astype(float)/1000
        source_relative=(times-base)*1000
        p=int(previous[state_index]);f=int(following[state_index])
        report['sampled_frames'].append({
            'state_row':state_index,'state_elapsed_sec':end-float(ends[0]),
            'paired_row':pair_index,'front':mf,'rear_for_adapter_base_only':mr,
            'point_time_absolute_seconds_range':[float(np.min(all_times)),float(np.max(all_times))],
            'all_points_first_minus_header_ms':float((np.min(all_times)-header_s)*1000),
            'all_points_last_minus_header_ms':float((np.max(all_times)-header_s)*1000),
            'adapter_selected_first_minus_header_ms':float((times[0]-header_s)*1000),
            'raw_array_negative_time_steps':int((np.diff(all_times)<0).sum()),
            'float64_absolute_timestamp_ulp_ns':float(np.spacing(base)*1e9),
            'adapter_header_delta_to_saved_csv_ns':adapter_header_ns-pair_stamp,
            'adapter_merged_input_count_matches_saved':int(valid.sum()+bvalid.sum())==int(paired[pair_index]['input_points']),
            'selected_front_count_matches_saved':int(valid.sum())==int(paired[pair_index]['output_points']),
            'selected_front_relative_time_range_ms':[float(relative_ms[0]),float(relative_ms[-1])],
            'selected_front_relative_time_monotonic':bool(np.all(np.diff(relative_ms)>=0)),
            'relative_float32_rounding_abs_max_ns':float(np.max(np.abs(relative_ms.astype(float)-source_relative))*1e6),
            'absolute_reconstruction_error_abs_max_ns':float(np.max(np.abs(reconstructed-times))*1e9),
            'preprocess_survivor_count':int(retained.sum()),
            'legacy_given_offset_time_condition_true':bool(relative_ms[-1]>0),
            'preprocess_scan_end_offset_ms':last_offset*1000,
            'reconstructed_scan_end_sec':predicted_end,
            'actual_csv_scan_end_sec':end,
            'scan_end_delta_to_actual_csv_us':float((predicted_end-end)*1e6),
            'imu_before_scan_end_ns':int(imu_ns[p]),'imu_after_scan_end_ns':int(imu_ns[f]),
            'imu_end_extrapolation_ms':float(age_ms[state_index]),
            'source_imu_bracketing_interval_ms':float(bracket_ms[state_index]),
            'imu_samples_inside_cloud_interval':int(np.sum((imu_s>=adapter_header_s)&(imu_s<=predicted_end)))})
    db.close()
    first=float(ends[0]);last=float(ends[-1])
    selected=(imu_s>=first-.15)&(imu_s<=last+.15)
    intervals=np.diff(imu_ns[selected])*1e-6
    # Match each saved end to the latest admitted cloud beginning at least
    # 50 ms before it. Validate the unique ~100 ms intervals, not a fixed
    # synthetic begin=end-0.1 approximation.
    pair_s=paired_ns.astype(float)*1e-9
    begin_index=np.searchsorted(pair_s,ends-.05)-1
    beginnings=pair_s[begin_index]
    assert len(np.unique(begin_index))==len(ends)
    assert np.all((ends-beginnings>.09)&(ends-beginnings<.11))
    gap_overlap={}
    for threshold in [.03,.05]:
        gap_mask=np.diff(imu_s)>threshold
        left,right=imu_s[:-1][gap_mask],imu_s[1:][gap_mask]
        inside=(right>beginnings[0])&(left<ends[-1])
        left,right=left[inside],right[inside]
        intersects=((beginnings[:,None]<right)&(ends[:,None]>left)).any(axis=1)
        gap_overlap[str(int(threshold*1000))]={
            'source_gaps_intersecting_processed_window':len(left),
            'saved_scan_intervals_intersecting_gap':int(intersects.sum())}
    adapter=json.loads((RUN/'central_imu_adapter.json').read_text())
    manifest=json.loads((RUN/'manifest.json').read_text())
    config=yaml.safe_load((RUN/'config/frontend.yaml').read_text())['laserMapping']['ros__parameters']
    report['full_saved_state_window_imu_coverage']={
        'state_count':len(ends),'derived_from_cached_source_not_algorithm_callback_capture':True,
        'scan_end_to_previous_imu_ms':qstats(age_ms),'bracketing_imu_interval_ms':qstats(bracket_ms),
        'scan_ends_with_prior_imu_older_than_ms':{str(t):int(np.sum(age_ms>t)) for t in [5,10,20,50]},
        'source_imu_interval_ms_near_state_window':qstats(intervals),
        'source_gap_overlap_by_threshold_ms':gap_overlap,
        'matched_cloud_interval_sec_range':[float((ends-beginnings).min()),float((ends-beginnings).max())],
        'published_source_count':adapter['counts'],
        'published_first_last_ns':[adapter['first_output_stamp_ns'],adapter['last_output_stamp_ns']],
        'no_source_coverage_proves_per_callback_arrival_order':True}
    report['effective_runtime_settings']={
        'imu_source':manifest['imu_source'],'imu_input_topic':manifest['imu_input_topic'],
        'input_offset_sec':adapter['config']['timestamp_offset_sec'],
        'acceleration_scale_to_si':adapter['config']['acceleration_scale'],
        'time_sync_en':config['common']['time_sync_en'],
        'preprocess':config['preprocess'],'point_filter_num':config['point_filter_num'],
        'software_source_offset_zero_is_not_physical_delay_calibration':True,
        'middleware':manifest['rmw']}
    worst=int(np.argmax(age_ms))
    context_indices=[int(np.argmin(np.abs(ends-(ends[worst]-1)))),worst,
                     int(np.argmin(np.abs(ends-(ends[worst]+1))))]
    gravity=np.column_stack([state[k] for k in ['gx','gy','gz']])
    unit=gravity/np.linalg.norm(gravity,axis=1)[:,None]
    tilt=np.degrees(np.arctan2(np.linalg.norm(gravity[:,:2],axis=1),-gravity[:,2]))
    context=(ends>=ends[context_indices[0]])&(ends<=ends[context_indices[-1]])
    report['worst_tail_gap_saved_state_context']={
        'note':'Saved post-update estimates, not ground truth or proof of causation; Z is the IMU origin.',
        'rows':[{'state_row':i,'relative_to_gap_scan_sec':float(ends[i]-ends[worst]),
                 'world_gravity_xyz':gravity[i].tolist(),'gravity_tilt_deg':float(tilt[i]),
                 'imu_origin_z_m':float(state['z'][i]),'residual_rmse_m':float(state['residual_rmse'][i])}
                for i in context_indices],
        'window_row_count':int(context.sum()),
        'window_delta_z_m':float(state['z'][context_indices[-1]]-state['z'][context_indices[0]]),
        'window_gravity_direction_change_deg':float(np.degrees(np.arccos(np.clip(unit[context_indices[-1]]@unit[context_indices[0]],-1,1)))),
        'window_residual_rmse_min_median_max_m':np.quantile(state['residual_rmse'][context],[0,.5,1]).tolist()}
    mean_text=re.search(r'mean acc=([^,]+), gyro bias=',(RUN/'frontend.log').read_text()).group(1)
    mean=np.array([float(x) for x in mean_text.split()])
    report['acceleration_constant_correction']={'common_G_m_s2':9.81,'S2_gravity_radius':9.809,
        'initial_acc_mean_norm_m_s2_from_log':float(np.linalg.norm(mean)),
        'actual_internal_acceleration_scale':float(9.81/np.linalg.norm(mean)),
        'previous_9_809_normalization_claim_was_incorrect':True}
    report['runtime_binary_check']=json.loads((RUN/'runtime_binary_check.json').read_text())
    report['limits']=[
        'Only five paired frames were reconstructed (ten payloads); no intermediate live cloud was recorded.',
        'Input point counts and output headers/end times validate reconstruction, not full bytewise identity.',
        'Source IMU availability does not prove per-callback algorithm arrival/consumption history.',
        'Physical LiDAR firing-to-IMU sampling/filter latency remains uncalibrated; common clock and zero software offset do not prove zero physical lag.',
        'Current source contains post-build guards/sorting absent from the loaded default library; no claim that those guards ran.',
        'The time-unit path can be validated without making a claim that it uniquely explains map descent.']
    output=HERE/'timing_audit.json'
    with output.open('x') as stream:json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
    print(json.dumps({'samples':[{key:row[key] for key in ['state_row','adapter_header_delta_to_saved_csv_ns',
        'selected_front_count_matches_saved','scan_end_delta_to_actual_csv_us','absolute_reconstruction_error_abs_max_ns',
        'imu_end_extrapolation_ms']} for row in report['sampled_frames']],
        'coverage':report['full_saved_state_window_imu_coverage'],
        'acceleration_constant_correction':report['acceleration_constant_correction']},indent=2))


if __name__=='__main__':main()
