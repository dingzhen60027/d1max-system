#!/usr/bin/env python3
"""Compare central/native-front IMU chains at the same physical front-LiDAR origin.

Read-only, offline evaluation. Defaults are central-device reference and
raw-front-device candidate; --reference-role front permits front-IMU R-only trials.
Deliberately changed fields are explicitly
enumerated; unknown changes remain comparison failures. No ROS initialization.
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


HERE=Path(__file__).resolve().parent
EVALUATOR=HERE.parent/'extrinsic_trials_20260919/evaluate_trials.py'
SPEC=importlib.util.spec_from_file_location('extrinsic_trial_evaluator',EVALUATOR)
EV=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EV)
IMU_ROLES={
    'central':{'input_topic':'/imu_driver/imu_central','frame_id':'central_imu','timestamp_offset_sec':-.013},
    'front':{'input_topic':'/front_lidar/imu','frame_id':'front_imu_raw','timestamp_offset_sec':0.},
}
ALLOWED_CALIBRATION_CHANGES={
    'input_topic','frame_id','timestamp_offset_sec','lio_extrinsic.rotation','lio_extrinsic.translation',
}
ALLOWED_FRONTEND_CHANGES={
    'laserMapping.ros__parameters.mapping.extrinsic_R',
    'laserMapping.ros__parameters.mapping.extrinsic_T',
    'laserMapping.ros__parameters.publish.body_frame',
    'laserMapping.ros__parameters.diagnostics.state_log_path',
}


def is_rotation(value):
    matrix=np.asarray(value,dtype=float).reshape(3,3)
    return bool(np.isfinite(matrix).all() and np.max(np.abs(matrix.T@matrix-np.eye(3)))<1e-5
                and abs(np.linalg.det(matrix)-1)<1e-5)


def execution_calibration(calibration):
    return {key:value for key,value in calibration.items()
            if key not in EV.DESCRIPTIVE_CALIBRATION_KEYS}


def metadata(calibration):
    return {key:calibration.get(key) for key in EV.DESCRIPTIVE_CALIBRATION_KEYS}


def lidar_origin_state(state,calibration):
    """p_W_N=p_W_I+R_W_I*t_I_N, R_W_N=R_W_I*R_I_N.

    N axes are normalized cloud axes, whose origin is the physical front LiDAR.
    Bias vectors stay in their native IMU coordinates and are labelled as such.
    """
    result=state.copy()
    xyz=np.column_stack([state[key] for key in ('x','y','z')])
    orientation=Rotation.from_quat(np.column_stack([state[key] for key in ('qx','qy','qz','qw')]))
    rotation=np.asarray(calibration['lio_extrinsic']['rotation'],dtype=float).reshape(3,3)
    translation=np.asarray(calibration['lio_extrinsic']['translation'],dtype=float)
    if not is_rotation(rotation) or translation.shape!=(3,) or not np.isfinite(translation).all():
        raise ValueError('LiDAR-to-IMU extrinsic must be a valid finite rigid transform')
    lidar_xyz=xyz+orientation.apply(translation)
    lidar_orientation=(orientation*Rotation.from_matrix(rotation)).as_quat()
    for index,key in enumerate(('x','y','z')):
        result[key]=lidar_xyz[:,index]
    for index,key in enumerate(('qx','qy','qz','qw')):
        result[key]=lidar_orientation[:,index]
    return result


def matched_state_indices(a,b,tolerance=.0001):
    """One-to-one ordered scan-end matches, tolerating CSV rounding only."""
    i=j=0; aa=[]; bb=[]
    while i<len(a) and j<len(b):
        difference=float(a[i]-b[j])
        if abs(difference)<=tolerance:
            aa.append(i);bb.append(j);i+=1;j+=1
        elif difference<0:
            i+=1
        else:
            j+=1
    return np.asarray(aa,dtype=int),np.asarray(bb,dtype=int)


def role_checks(run,role):
    config=run['calibration']; manifest=run['manifest']
    params=run['frontend']['laserMapping']['ros__parameters']
    desired=IMU_ROLES[role]
    mapping=params['mapping']
    checks={
        'role_input_topic':config.get('input_topic')==desired['input_topic'],
        'role_raw_imu_frame':config.get('frame_id')==desired['frame_id'],
        'role_time_offset':config.get('timestamp_offset_sec')==desired['timestamp_offset_sec'],
        'raw_acceleration_scale_9_80665':config.get('acceleration_scale')==9.80665,
        'raw_gyro_scale_1':config.get('gyro_scale')==1.,
        'same_normalized_imu_output_topic':config.get('output_topic')=='/d1max/slam/imu',
        'duplicates_preserved':config.get('duplicate_policy')=='preserve_and_count',
        'frontend_body_frame_matches_raw_imu':params['publish'].get('body_frame')==config.get('frame_id'),
        'frontend_imu_topic_matches_adapter_output':params['common'].get('imu_topic')==config.get('output_topic'),
        'source_imu_topic_in_manifest':desired['input_topic'] in manifest.get('topics',[]),
        'manifest_imu_input_topic_matches_when_recorded':manifest.get('imu_input_topic',desired['input_topic'])==desired['input_topic'],
        'manifest_imu_frame_matches_when_recorded':manifest.get('imu_frame',desired['frame_id'])==desired['frame_id'],
        'front_only_paired_scan_selection':manifest.get('cloud_selection')=='paired_front',
        'gravity_aligned_world_enabled':mapping.get('gravity_aligned_world') is True,
        'extrinsic_is_SO3':is_rotation(mapping['extrinsic_R']),
    }
    return checks


def control_checks(a,b,reference_role='central',candidate_role='front'):
    ma,mb=a['manifest'],b['manifest']
    calibration_diff=EV.differences(execution_calibration(a['calibration']),execution_calibration(b['calibration']))
    frontend_diff=EV.differences(a['frontend'],b['frontend'])
    unexpected_calibration=sorted(set(calibration_diff)-ALLOWED_CALIBRATION_CHANGES)
    unexpected_frontend=sorted(set(frontend_diff)-ALLOWED_FRONTEND_CHANGES)
    runtime_a=EV.runtime_fingerprint(a['runtime_binary_check'])
    runtime_b=EV.runtime_fingerprint(b['runtime_binary_check'])
    sensor_topics_a=[topic for topic in ma.get('topics',[]) if topic!=IMU_ROLES[reference_role]['input_topic']]
    sensor_topics_b=[topic for topic in mb.get('topics',[]) if topic!=IMU_ROLES[candidate_role]['input_topic']]
    checks={
        'same_source_bag':ma.get('bag')==mb.get('bag'),
        'same_source_file_stats':ma.get('source_stats')==mb.get('source_stats'),
        'same_input_window':ma.get('paired_cloud_window_sec')==mb.get('paired_cloud_window_sec'),
        'same_replay_rate':ma.get('rate')==mb.get('rate'),
        'same_non_imu_topics':sensor_topics_a==sensor_topics_b,
        'same_admitted_header_sequence':np.array_equal(a['scans'][:,0],b['scans'][:,0]),
        'same_admitted_input_point_counts':np.array_equal(a['scans'][:,1],b['scans'][:,1]),
        'same_admitted_front_output_point_counts':np.array_equal(a['scans'][:,2],b['scans'][:,2]),
        'same_binary_manifest_hashes':ma.get('binary_sha256')==mb.get('binary_sha256'),
        'same_actual_loaded_binary_hashes':None if runtime_a is None or runtime_b is None else runtime_a==runtime_b,
        'only_allowlisted_calibration_changes':not unexpected_calibration,
        'only_allowlisted_frontend_changes':not unexpected_frontend,
        'same_rear_transform_not_used_by_front_subset':EV.rear(a['calibration'])==EV.rear(b['calibration']),
        'same_launch_snapshot':a['snapshots']['mapping.launch.py']['actual']==b['snapshots']['mapping.launch.py']['actual'],
        'same_adapter_config_snapshot':a['snapshots']['dual_lidar.yaml']['actual']==b['snapshots']['dual_lidar.yaml']['actual'],
        'same_imu_adapter_script':a['snapshots']['central_imu_adapter.py']['actual']==b['snapshots']['central_imu_adapter.py']['actual'],
        'same_cloud_selector_script':a['snapshots']['cloud_subset.py']['actual']==b['snapshots']['cloud_subset.py']['actual'],
    }
    return {'checks':checks,'all_passed':all(checks.values()),
            'insufficient_evidence':[key for key,value in checks.items() if value is None],
            'calibration_execution_differences':calibration_diff,'frontend_differences':frontend_diff,
            'unexpected_calibration_differences':unexpected_calibration,
            'unexpected_frontend_differences':unexpected_frontend,
            'calibration_metadata_differences':EV.differences(metadata(a['calibration']),metadata(b['calibration']))}


def common_metrics(run,indices,canonical_times,segment_seconds):
    selected=run['state'][indices].copy()
    actual_times=selected['t'].copy()
    # Same scan-end poses have at most 100 us of logged timestamp rounding.
    # Only the analysis time axis is canonicalized; no pose interpolation occurs.
    selected['t']=canonical_times
    lidar=lidar_origin_state(selected,run['calibration'])
    values=EV.metrics(lidar,float(canonical_times[0]),float(canonical_times[-1]),segment_seconds)
    values['relative_lidar_orientation']=values.pop('relative_body_orientation')
    values['relative_lidar_orientation']['definition']='R_W_N(first).inverse * R_W_N(t); normalized front-LiDAR axes, no cross-run world alignment.'
    imu_origin_delta=np.array([selected[key][-1]-selected[key][0] for key in ('x','y','z')])
    values['original_imu_origin_delta_xyz_m']=imu_origin_delta.tolist()
    values['lidar_origin_minus_imu_origin_endpoint_delta_z_m']=values['estimated_delta_xyz_m'][2]-float(imu_origin_delta[2])
    values['native_imu_bias_frame']=run['calibration']['frame_id']
    values['native_imu_bias_comparison_caution']='Bias vector components belong to different sensor axes; do not subtract them across runs. Norms are descriptive and not bias ground truth.'
    values['actual_first_last_state_stamp_sec']=actual_times[[0,-1]].tolist()
    values['max_analysis_time_canonicalization_sec']=float(np.max(np.abs(actual_times-canonical_times)))
    return values


def run_summary(run,role,indices,times,segment_seconds):
    integrity=dict(run['integrity'])
    adapter=run['adapter']
    first,last=adapter.get('first_output_stamp_ns'),adapter.get('last_output_stamp_ns')
    integrity['adapter_imu_covers_common_state_interval']=bool(first is not None and last is not None and first*1e-9<=times[0] and last*1e-9>=times[-1])
    end_delta=float(run['state']['t'][-1]-run['scans'][-1,0]*1e-9)
    role_status=role_checks(run,role)
    return {'directory':str(run['path']),'name':run['name'],'role':role,
            'integrity':integrity,'integrity_all_passed':all(integrity.values()),
            'role_checks':role_status,'role_checks_all_passed':all(role_status.values()),
            'calibration_metadata':metadata(run['calibration']),
            'lio_extrinsic':run['calibration']['lio_extrinsic'],
            'imu_execution_config':{key:run['calibration'].get(key) for key in EV.IMU_ADAPTER_EXECUTION_FIELDS},
            'runtime_binary_check':run['runtime_binary_check'],'snapshots':run['snapshots'],
            'admitted_scan_count':len(run['scans']),'state_rows_total':len(run['state']),
            'matched_common_state_rows':len(indices),
            'state_rows_before_common':int(indices[0]),
            'state_rows_after_common':int(len(run['state'])-indices[-1]-1),
            'first_state_minus_first_admitted_scan_header_sec':float(run['state']['t'][0]-run['scans'][0,0]*1e-9),
            'unmatched_rows_inside_common_range':int(indices[-1]-indices[0]+1-len(indices)),
            'last_state_minus_last_admitted_scan_header_sec':end_delta,
            'adapter_statistics':{key:value for key,value in adapter.items() if key not in ['config','notes']},
            'lidar_origin_common_metrics':common_metrics(run,indices,times,segment_seconds)}


def markdown(report):
    lines=[f"# 最小前雷达链对照：{report['reference_role']} → {report['candidate_role']}",'',
           '**比较前已转换到同一物理前雷达原点，不直接比较不同 IMU 安装位置的轨迹。**','',
           f"严格控制检查：{'通过' if report['all_control_and_integrity_checks_passed'] else '未全部通过'}。共同扫描末端状态 {report['common_state_count']} 个。",'',
           '| 链路 | 前雷达原点 ΔZ m | 前雷达 XY 路程 m | 原 IMU 原点 ΔZ m | 末端重力倾角 ° | 残差中位 m |',
           '|---|---:|---:|---:|---:|---:|']
    for run in report['runs']:
        value=run['lidar_origin_common_metrics']
        lines.append(f"| {run['name']} | {value['estimated_delta_xyz_m'][2]:.4f} | {value['estimated_xy_path_m']:.3f} | "
                     f"{value['original_imu_origin_delta_xyz_m'][2]:.4f} | {value['gravity_tilt_to_world_down_initial_final_max_deg'][1]:.4f} | "
                     f"{value['residual_rmse_median_p95_max_m'][0]:.5f} |")
    failed=[key for key,value in report['controlled_comparison']['checks'].items() if value is False]
    missing=report['controlled_comparison']['insufficient_evidence']
    lines+=['','对照检查失败项：'+(', '.join(failed) if failed else '无。'),
            '证据不足项：'+(', '.join(missing) if missing else '无。')]
    for run in report['runs']:
        lines+=['',f"## {run['name']}",'']
        failures=[key for group in [run['integrity'],run['role_checks']] for key,value in group.items() if not value]
        lines.append('本组检查失败项：'+(', '.join(failures) if failures else '无。'))
        lines+=['','| 时间 s | 雷达 XY 路程 m | 雷达 ΔZ m | 高度/路程斜率 |','|---|---:|---:|---:|']
        for segment in run['lidar_origin_common_metrics']['time_segments']:
            begin,end=segment['elapsed_range_sec']; slope=segment['height_change_per_path_m']
            slope_text=f'{100*slope:.3f}%' if slope is not None else '移动不足'
            lines.append(f"| {begin:.1f}–{end:.1f} | {segment['xy_path_m']:.3f} | {segment['delta_xyz_m'][2]:.4f} | {slope_text} |")
    lines+=['','## 解释边界','']+[f'- {value}' for value in report['limitations']]
    return '\n'.join(lines)+'\n'


def compare(reference,candidate,segment_seconds=30.,legacy_reference=False,
            reference_role='central',candidate_role='front'):
    a,b=EV.load(reference),EV.load(candidate)
    ia,ib=matched_state_indices(a['state']['t'],b['state']['t'])
    if len(ia)<3:
        raise ValueError('Need at least three common scan-end states within 100 us')
    times=a['state']['t'][ia].copy()
    controls=control_checks(a,b,reference_role,candidate_role)
    controls['checks']['no_unmatched_output_inside_common_interval']=bool(
        ia[-1]-ia[0]+1==len(ia) and ib[-1]-ib[0]+1==len(ib))
    controls['all_passed']=all(controls['checks'].values())
    summaries=[run_summary(a,reference_role,ia,times,segment_seconds),run_summary(b,candidate_role,ib,times,segment_seconds)]
    return {'reference':str(a['path']),'candidate':str(b['path']),
            'reference_role':reference_role,'candidate_role':candidate_role,
            'legacy_reference_requested':legacy_reference,
            'legacy_reference_policy':'Legacy data may be shown descriptively; differing launch or missing runtime evidence still fail strict control checks.',
            'trajectory_origin':'Physical front LiDAR / normalized cloud origin N',
            'origin_transform':'p_W_N=p_W_I+R_W_I*t_I_N; R_W_N=R_W_I*R_I_N',
            'world_frames':'Each run retains its own initialized gravity-aligned world; yaw and absolute origin are not assumed common.',
            'cross_run_cartesian_pose_error_computed':False,
            'common_state_count':len(ia),'common_reference_header_interval_sec':times[[0,-1]].tolist(),
            'maximum_matched_header_difference_sec':float(np.max(np.abs(a['state']['t'][ia]-b['state']['t'][ib]))),
            'controlled_comparison':controls,
            'allowed_calibration_change_paths':sorted(ALLOWED_CALIBRATION_CHANGES),
            'allowed_frontend_change_paths':sorted(ALLOWED_FRONTEND_CHANGES),
            'runs':summaries,
            'all_control_and_integrity_checks_passed':bool(controls['all_passed'] and all(run['integrity_all_passed'] and run['role_checks_all_passed'] for run in summaries)),
            'limitations':[
                '各组位置先转换到相同前雷达物理原点，再相对共同起点统计；没有直接用不同 IMU 原点的高度差比较。',
                '各自 world 由各自 IMU 初始化重力对齐，yaw 与绝对原点不保证相同；不计算两条 XYZ 轨迹之间的定位误差。',
                '记录的 p_W_I 必须对应 IMU 状态原点；这里使用实际生效的 p_I=R_I_N*p_N+t_I_N 外参定义。',
                '两种 IMU 的源、frame、时移与各自外参是有意变量；这不是只改一个标量的实验，也不能把改善唯一归因于硬件。',
                '共同状态按扫描末端时间一一匹配，容许 CSV 的 100 微秒舍入差；仅规范化分析时间轴，不插值位姿。初始化前后多余行单独公开。',
                '偏置分量仍属于各自原始 IMU 坐标系，不可直接跨传感器相减；偏置模长和匹配残差也不是外部真值。',
                '同话题扫描头和点数不保证逐字节 IMU 实收历史相同；源 IMU 本来不同，传输差异只能在现有日志范围内审计。',
                '前雷达相对高度及分段曲率是估计轨迹描述，不是地面 GT；未加地面水平先验、回环或高度锁。',
                '历史组若 launch、脚本或运行库证据不同，仅作参考，绝不因 legacy 标志而忽略控制失败。',
                '短段结果不认证全程、高速、室外开阔环境，也不自动批准替换生产参数。']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference',type=Path,help='Central IMU, device-front rotation, paired_front run')
    parser.add_argument('candidate',type=Path,help='Raw front internal IMU, device DIFOP, paired_front run')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--segment-seconds',type=float,default=30.)
    parser.add_argument('--legacy-reference',action='store_true',help='Label older descriptive reference; does not bypass any failed control check.')
    parser.add_argument('--reference-role',choices=tuple(IMU_ROLES),default='central')
    parser.add_argument('--candidate-role',choices=tuple(IMU_ROLES),default='front')
    args=parser.parse_args()
    markdown_path=args.output.with_suffix('.md')
    if args.output==markdown_path or args.output.exists() or markdown_path.exists():
        parser.error('JSON and Markdown output paths must be distinct and new')
    if args.segment_seconds<=0:
        parser.error('Segment duration must be positive')
    report=compare(args.reference,args.candidate,args.segment_seconds,args.legacy_reference,
                   args.reference_role,args.candidate_role)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    text=markdown(report)
    with markdown_path.open('x') as stream:
        stream.write(text)
    print(text)
    print(f'JSON: {args.output.resolve()}')


if __name__=='__main__':
    main()
