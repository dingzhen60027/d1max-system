#!/usr/bin/env python3
"""Read-only, controlled extrinsic-trial comparison; never treats trajectory as GT.

The first positional run is the reference. Outputs JSON plus concise Markdown;
both must be new paths. No ROS initialization, bag replay, or map writes occur.
"""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


DEFAULT_REAR = {'rotation': [1.,0.,0.,0.,-1.,0.,0.,0.,-1.],
                'translation': [0.,0.,-.7323],
                'parent_frame': 'rslidar_head', 'child_frame': 'rslidar_tail'}
STATE_FIELDS = ('t','x','y','z','qx','qy','qz','qw','gx','gy','gz',
                'bax','bay','baz','bgx','bgy','bgz','features','residual_rmse')
DESCRIPTIVE_CALIBRATION_KEYS = ('provenance','trial')
IMU_ADAPTER_EXECUTION_FIELDS = ('input_topic','output_topic','frame_id','acceleration_scale',
                                'gyro_scale','timestamp_offset_sec','duplicate_policy')


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(stream.read()).hexdigest()


def flatten(value, prefix=''):
    if isinstance(value,dict):
        result={}
        for key,item in value.items():
            result.update(flatten(item, f'{prefix}.{key}' if prefix else key))
        return result
    return {prefix:value}


def differences(a,b):
    aa,bb=flatten(a),flatten(b)
    return {key:{'reference':aa.get(key),'candidate':bb.get(key)}
            for key in sorted(aa.keys()|bb.keys()) if aa.get(key)!=bb.get(key)}


def frontend_core(config):
    config=copy.deepcopy(config)
    params=config['laserMapping']['ros__parameters']
    # Only run-specific log destination is ignored, not all diagnostic settings.
    diagnostics=params.get('diagnostics',{})
    diagnostics.pop('state_log_path',None)
    if not diagnostics:
        params.pop('diagnostics',None)
    params['mapping'].pop('extrinsic_R',None)
    params['mapping'].pop('extrinsic_T',None)
    return config


def calibration_core(calibration):
    calibration=copy.deepcopy(calibration)
    # Audited central_imu_adapter.py and mapping.launch.py do not read these
    # two descriptive blocks for execution. Unknown/new fields remain checked.
    # Keep the original blocks in every run's report; do not silently discard.
    for key in DESCRIPTIVE_CALIBRATION_KEYS:
        calibration.pop(key,None)
    calibration.pop('lio_extrinsic',None)
    lidar=calibration.get('lidar_extrinsics',{})
    lidar.pop('rear_to_front',None)
    if not lidar:
        calibration.pop('lidar_extrinsics',None)
    return calibration


def rear(calibration):
    return calibration.get('lidar_extrinsics',{}).get('rear_to_front',DEFAULT_REAR)


def runtime_fingerprint(report):
    """Compare actual loaded artifacts, excluding process IDs and node order."""
    if not isinstance(report,dict) or not report.get('nodes'):
        return None
    nodes=[]
    lio_loaded_library_verified=False
    for node in report['nodes']:
        if not node.get('executable') or not node.get('executable_sha256'):
            return None
        libraries=sorted((str(item.get('path')),str(item.get('sha256')))
                         for item in node.get('loaded_libraries',[]))
        if any(sha in ['None',''] for _,sha in libraries):
            return None
        if Path(node['executable']).name=='run_mapping_online':
            lio_loaded_library_verified=any(Path(path).name=='libfaster_lio_lib.so' for path,_ in libraries)
        nodes.append((node['executable'],node['executable_sha256'],libraries))
    return sorted(nodes) if lio_loaded_library_verified else None


def load(run):
    run=run.resolve()
    manifest=json.loads((run/'manifest.json').read_text())
    result=json.loads((run/'result.json').read_text())
    state=np.atleast_1d(np.genfromtxt(run/'frontend_state.csv',delimiter=',',names=True))
    missing=set(STATE_FIELDS)-set(state.dtype.names or [])
    if missing or len(state)<2:
        raise ValueError(f'{run}: invalid state columns or fewer than two rows: {missing}')
    if not all(np.isfinite(state[k]).all() for k in state.dtype.names):
        raise ValueError(f'{run}: nonfinite state data')
    if not np.all(np.diff(state['t'])>0):
        raise ValueError(f'{run}: state timestamps are not strictly increasing')
    with (run/'paired_scans.csv').open() as stream:
        scans=[row for row in csv.DictReader(stream) if row['admitted'].lower()=='true']
    scan_values=np.array([[int(s[k]) for k in ('stamp_ns','input_points','output_points')] for s in scans],dtype=np.int64)
    if len(scan_values)<2 or np.any(np.diff(scan_values[:,0])<=0):
        raise ValueError(f'{run}: admitted input scan timestamps invalid')
    frontend=yaml.safe_load((run/'config/frontend.yaml').read_text())
    calibration=yaml.safe_load((run/'config/calibration.yaml').read_text())
    adapter=json.loads((run/'central_imu_adapter.json').read_text())
    quaternions=np.column_stack([state[k] for k in ('qx','qy','qz','qw')])
    mapping=frontend['laserMapping']['ros__parameters']['mapping']
    snapshots={}
    for name in ['frontend.yaml','calibration.yaml','mapping.launch.py','dual_lidar.yaml',
                 'central_imu_adapter.py','cloud_subset.py']:
        path=run/'config'/name
        actual=digest(path) if path.is_file() else None
        recorded=manifest.get('snapshot_sha256',{}).get(name)
        snapshots[name]={'actual':actual,'recorded':recorded,
                         'matches_manifest':actual is not None and recorded==actual}
    poses_path=run/'frontend_odometry.tum'
    poses=sum(1 for line in poses_path.read_text().splitlines() if line.strip() and not line.startswith('#')) if poses_path.is_file() else None
    runtime_path=run/'runtime_binary_check.json'
    runtime=json.loads(runtime_path.read_text()) if runtime_path.is_file() else None
    map_path=Path(result['map_path']) if result.get('map_path') else None
    integrity={
        'result_complete':result.get('complete') is True,
        'frontend_only_no_height_lock':all(not obj.get(key,False) for obj in [manifest,result]
                                          for key in ['loop_closure_enabled','height_lock_enabled']),
        'source_bag_unchanged':result.get('source_unchanged') is True,
        'result_pose_count_matches_state':result.get('poses')==len(state),
        'tum_pose_count_matches_state':poses==len(state),
        'quaternion_norms_valid':bool(np.max(np.abs(np.linalg.norm(quaternions,axis=1)-1))<1e-3),
        'effective_lio_rotation_matches_calibration':mapping.get('extrinsic_R')==calibration['lio_extrinsic']['rotation'],
        'effective_lio_translation_matches_calibration':mapping.get('extrinsic_T')==calibration['lio_extrinsic']['translation'],
        'online_extrinsic_estimation_disabled':mapping.get('extrinsic_est_en') is False,
        'adapter_config_matches_snapshot':adapter.get('config')==calibration,
        'imu_offset_matches_manifest':calibration.get('timestamp_offset_sec')==manifest.get('central_timestamp_offset_sec'),
        'snapshot_hashes_match':all(item['matches_manifest'] for item in snapshots.values()),
        'zenoh_preserved':manifest.get('rmw')=='rmw_zenoh_cpp',
        'saved_map_present_nonempty':bool(map_path and map_path.is_file() and map_path.stat().st_size>100),
    }
    return dict(path=run,name=run.name,manifest=manifest,result=result,state=state,scans=scan_values,
                frontend=frontend,calibration=calibration,adapter=adapter,snapshots=snapshots,
                integrity=integrity,runtime_binary_check=runtime)


def norm_stats(values):
    norms=np.linalg.norm(values,axis=1)
    return {'first':values[0].tolist(),'last':values[-1].tolist(),
            'last_minus_first':(values[-1]-values[0]).tolist(),
            'norm_median_p95_max':np.r_[np.quantile(norms,[.5,.95]),np.max(norms)].tolist(),
            'max_change_from_first_norm':float(np.max(np.linalg.norm(values-values[0],axis=1)))}


def metrics(state,lo,hi,segment_seconds):
    data=state[(state['t']>=lo)&(state['t']<=hi)]
    xyz=np.column_stack([data[k] for k in ('x','y','z')])
    relative=xyz-xyz[0]
    times=data['t']-data['t'][0]
    path=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xyz[:,:2],axis=0),axis=1))]
    gravity=np.column_stack([data[k] for k in ('gx','gy','gz')])
    gravity_unit=gravity/np.linalg.norm(gravity,axis=1)[:,None]
    tilt=np.rad2deg(np.arctan2(np.linalg.norm(gravity[:,:2],axis=1),-gravity[:,2]))
    gravity_change=np.rad2deg(np.arccos(np.clip(gravity_unit@gravity_unit[0],-1,1)))
    orientation=Rotation.from_quat(np.column_stack([data[k] for k in ('qx','qy','qz','qw')]))
    relative_orientation=orientation[0].inv()*orientation
    rotvec=np.rad2deg(relative_orientation.as_rotvec())
    angle=np.rad2deg(relative_orientation.magnitude())
    horizontal=data['horizontal_features']/np.maximum(data['features'],1) if 'horizontal_features' in data.dtype.names else None
    linear_matrix=np.column_stack([np.ones(len(path)),path])
    linear_coefs=np.linalg.lstsq(linear_matrix,relative[:,2],rcond=None)[0]
    linear_residual=relative[:,2]-linear_matrix@linear_coefs
    quadratic_matrix=np.column_stack([linear_matrix,path**2])
    quadratic_coefs=np.linalg.lstsq(quadratic_matrix,relative[:,2],rcond=None)[0]
    quadratic_residual=relative[:,2]-quadratic_matrix@quadratic_coefs
    xy_centered=xyz[:,:2]-xyz[:,:2].mean(axis=0)
    xy_singular=np.linalg.svd(xy_centered,compute_uv=False)
    plane_matrix=np.column_stack([np.ones(len(xyz)),relative[:,:2]])
    plane_coef=np.linalg.lstsq(plane_matrix,relative[:,2],rcond=None)[0]
    plane_residual=relative[:,2]-plane_matrix@plane_coef
    segments=[]
    for start in np.arange(0,times[-1],segment_seconds):
        end=min(float(start+segment_seconds),float(times[-1]))
        use=(times>=start)&(times<=end)
        if use.sum()<3:
            continue
        positions=relative[use]; s=path[use]
        traveled=float(s[-1]-s[0])
        slope=np.polyfit(s-s[0],positions[:,2],1)[0] if traveled>.1 else None
        segments.append({'elapsed_range_sec':[float(times[use][0]),float(times[use][-1])],
                         'rows':int(use.sum()),'xy_path_m':traveled,
                         'delta_xyz_m':(positions[-1]-positions[0]).tolist(),
                         'height_change_per_path_m':float(slope) if slope is not None else None,
                         'height_slope_angle_deg':float(np.rad2deg(np.arctan(slope))) if slope is not None else None,
                         'gravity_tilt_first_last_deg':tilt[use][[0,-1]].tolist(),
                         'residual_rmse_median_m':float(np.median(data['residual_rmse'][use]))})
    return {
        'rows':len(data),'first_last_stamp_sec':data['t'][[0,-1]].tolist(),
        'duration_sec':float(times[-1]),'estimated_delta_xyz_m':relative[-1].tolist(),
        'estimated_xy_path_m':float(path[-1]),'estimated_z_span_m':float(np.ptp(xyz[:,2])),
        'gravity_tilt_to_world_down_initial_final_max_deg':[float(tilt[0]),float(tilt[-1]),float(np.max(tilt))],
        'gravity_direction_change_from_initial_final_max_deg':[float(gravity_change[-1]),float(np.max(gravity_change))],
        'relative_body_orientation':{'definition':'R_initial.inverse * R_t; physical motion and estimate drift both contribute.',
            'final_rotvec_deg':rotvec[-1].tolist(),'relative_rotation_angle_final_max_deg':[float(angle[-1]),float(np.max(angle))],
            'relative_rotvec_component_min_max_deg':np.vstack([rotvec.min(axis=0),rotvec.max(axis=0)]).tolist()},
        'accel_bias_m_s2':norm_stats(np.column_stack([data[k] for k in ('bax','bay','baz')])),
        'gyro_bias_rad_s':norm_stats(np.column_stack([data[k] for k in ('bgx','bgy','bgz')])),
        'residual_rmse_median_p95_max_m':np.r_[np.quantile(data['residual_rmse'],[.5,.95]),data['residual_rmse'].max()].tolist(),
        'features_min_median_max':np.quantile(data['features'],[0,.5,1]).tolist(),
        'horizontal_feature_fraction_median':float(np.median(horizontal)) if horizontal is not None else None,
        'state_dt_median_p95_max_sec':np.r_[np.quantile(np.diff(data['t']),[.5,.95]),np.diff(data['t']).max()].tolist(),
        'height_trend_descriptive_only':{
            'linear_path_slope_m_per_m':float(linear_coefs[1]),
            'linear_path_residual_rmse_m':float(np.sqrt(np.mean(linear_residual**2))),
            'linear_path_residual_span_m':float(np.ptp(linear_residual)),
            'quadratic_path_term_per_m':float(quadratic_coefs[2]),
            'quadratic_path_residual_rmse_m':float(np.sqrt(np.mean(quadratic_residual**2))),
            'affine_spatial_height_plane_z_equals_c_plus_ax_plus_by':plane_coef.tolist(),
            'affine_spatial_height_plane_residual_rmse_m':float(np.sqrt(np.mean(plane_residual**2))),
            'xy_centered_singular_values_m':xy_singular.tolist(),
            'xy_small_to_large_singular_ratio':float(xy_singular[-1]/max(xy_singular[0],1e-12)),
            'full_2d_plane_slope_well_conditioned_heuristic':bool(xy_singular[-1]/max(xy_singular[0],1e-12)>.05),
            'caution':'These fits describe estimated sensor-origin height only. A small residual does not prove fixed extrinsic tilt; large residual does not prove dynamic drift. Real motion and true terrain are not separated. A near-straight route poorly identifies a 2D height plane.'},
        'time_segments':segments}


def compare(reference,candidate,allow_rear):
    a,b=reference,candidate
    ma,mb=a['manifest'],b['manifest']
    rear_changed=rear(a['calibration'])!=rear(b['calibration'])
    same_state_count=len(a['state'])==len(b['state'])
    runtime_a=runtime_fingerprint(a['runtime_binary_check'])
    runtime_b=runtime_fingerprint(b['runtime_binary_check'])
    checks={
        'same_bag':ma.get('bag')==mb.get('bag'),
        'same_bag_source_stats':ma.get('source_stats')==mb.get('source_stats'),
        'same_replay_rate':ma.get('rate')==mb.get('rate'),
        'same_input_window':ma.get('paired_cloud_window_sec')==mb.get('paired_cloud_window_sec'),
        'same_cloud_selection':ma.get('cloud_selection')==mb.get('cloud_selection'),
        'same_topics':ma.get('topics')==mb.get('topics'),
        'same_admitted_scan_header_sequence':np.array_equal(a['scans'][:,0],b['scans'][:,0]),
        'same_admitted_input_point_counts':np.array_equal(a['scans'][:,1],b['scans'][:,1]),
        'same_admitted_output_point_counts':np.array_equal(a['scans'][:,2],b['scans'][:,2]),
        'same_binary_manifest_hashes':ma.get('binary_sha256')==mb.get('binary_sha256'),
        'same_actual_loaded_binary_hashes':None if runtime_a is None or runtime_b is None else runtime_a==runtime_b,
        'same_launch_snapshot':a['snapshots']['mapping.launch.py']['actual']==b['snapshots']['mapping.launch.py']['actual'],
        'same_dual_adapter_config_snapshot':a['snapshots']['dual_lidar.yaml']['actual']==b['snapshots']['dual_lidar.yaml']['actual'],
        'same_imu_adapter_script':a['snapshots']['central_imu_adapter.py']['actual']==b['snapshots']['central_imu_adapter.py']['actual'],
        'same_cloud_selector_script':a['snapshots']['cloud_subset.py']['actual']==b['snapshots']['cloud_subset.py']['actual'],
        'same_frontend_except_explicit_extrinsics_and_log_path':frontend_core(a['frontend'])==frontend_core(b['frontend']),
        'same_imu_configuration_except_extrinsics_and_provenance':calibration_core(a['calibration'])==calibration_core(b['calibration']),
        'same_imu_time_shift':ma.get('central_timestamp_offset_sec')==mb.get('central_timestamp_offset_sec'),
        'rear_extrinsic_change_explicitly_permitted':not rear_changed or b['name'] in allow_rear,
        'same_output_pose_count':same_state_count,
        'same_output_header_sequence_to_100us':same_state_count and bool(np.allclose(a['state']['t'],b['state']['t'],rtol=0,atol=.0001)),
    }
    return {'checks':checks,'all_passed':all(checks.values()),
            'insufficient_evidence_checks':[key for key,value in checks.items() if value is None],
            'frontend_parameter_differences':differences(a['frontend'],b['frontend']),
            'calibration_execution_differences':differences({k:v for k,v in a['calibration'].items() if k not in DESCRIPTIVE_CALIBRATION_KEYS},
                                                          {k:v for k,v in b['calibration'].items() if k not in DESCRIPTIVE_CALIBRATION_KEYS}),
            'calibration_descriptive_metadata_differences':differences(
                {k:a['calibration'].get(k) for k in DESCRIPTIVE_CALIBRATION_KEYS},
                {k:b['calibration'].get(k) for k in DESCRIPTIVE_CALIBRATION_KEYS}),
            'effective_rear_transform_changed':rear_changed,
            'effective_rear_transform':rear(b['calibration']),
            'snapshot_hash_differences':differences({k:v['actual'] for k,v in a['snapshots'].items()},
                                                  {k:v['actual'] for k,v in b['snapshots'].items()})}


def markdown(report):
    lines=['# 外参隔离对照评估','',f"共同时间范围：{report['common_state_interval_sec']}。",'',
           '**注意：所有高度都是估计的传感器原点轨迹，不是测量真值，也未加入地面约束。**','',
           '| 运行 | 完整性 | 对照控制 | XY 路程 m | ΔZ m | 末端重力倾角 ° | 匹配残差中位 m |',
           '|---|---|---|---:|---:|---:|---:|']
    for run in report['runs']:
        m=run['common_metrics']
        lines.append(f"| {run['name']} | {'通过' if run['integrity_all_passed'] else '未通过'} | "
                     f"{'通过' if run['reference_comparison']['all_passed'] else '未通过'} | "
                     f"{m['estimated_xy_path_m']:.3f} | {m['estimated_delta_xyz_m'][2]:.4f} | "
                     f"{m['gravity_tilt_to_world_down_initial_final_max_deg'][1]:.4f} | "
                     f"{m['residual_rmse_median_p95_max_m'][0]:.5f} |")
    for run in report['runs']:
        lines+=['',f"## {run['name']}",'']
        failures=[k for k,v in run['integrity'].items() if not v]+[k for k,v in run['reference_comparison']['checks'].items() if v is False]
        lines.append('检查失败项：'+(', '.join(failures) if failures else '无。'))
        missing=run['reference_comparison']['insufficient_evidence_checks']
        if missing:
            lines.append('证据不足项：'+', '.join(missing))
        lines+=['','| 时间 s | XY 路程 m | ΔZ m | 分段高度/路程斜率 |','|---|---:|---:|---:|']
        for segment in run['common_metrics']['time_segments']:
            begin,end=segment['elapsed_range_sec']; slope=segment['height_change_per_path_m']
            slope_text=f'{100*slope:.3f}%' if slope is not None else '移动不足'
            lines.append(f"| {begin:.1f}–{end:.1f} | {segment['xy_path_m']:.3f} | {segment['delta_xyz_m'][2]:.4f} | {slope_text} |")
        trend=run['common_metrics']['height_trend_descriptive_only']
        lines+=['',f"高度对累计 XY 路程线性拟合残差 RMS：{trend['linear_path_residual_rmse_m']:.4f} m；"
                    f"二次拟合残差 RMS：{trend['quadratic_path_residual_rmse_m']:.4f} m。",'',
                '这些量用于观察是否存在分段趋势变化，不能单独判定固定倾斜、动态漂移或真实地形。']
    lines+=['','## 边界','']+[f'- {limit}' for limit in report['limitations']]
    return '\n'.join(lines)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True,help='New JSON path; sibling .md also must not exist.')
    parser.add_argument('--allow-rear-extrinsic-run',action='append',default=[],help='Candidate directory basename explicitly allowed to alter rear-to-front TF.')
    parser.add_argument('--segment-seconds',type=float,default=30.)
    args=parser.parse_args()
    if args.segment_seconds<=0:
        parser.error('--segment-seconds must be positive')
    md_path=args.output.with_suffix('.md')
    if args.output==md_path or args.output.exists() or md_path.exists():
        parser.error('JSON and Markdown must have distinct, new output paths')
    runs=[load(path) for path in args.runs]
    if len({run['name'] for run in runs})!=len(runs):
        parser.error('Run directory basenames must be unique')
    lo=max(run['state']['t'][0] for run in runs); hi=min(run['state']['t'][-1] for run in runs)
    if hi<=lo or any(np.sum((r['state']['t']>=lo)&(r['state']['t']<=hi))<3 for r in runs):
        parser.error('Insufficient common state time coverage')
    report={'reference':str(runs[0]['path']),'common_state_interval_sec':[float(lo),float(hi)],
            'allow_rear_extrinsic_run':args.allow_rear_extrinsic_run,'runs':[],
            'imu_adapter_execution_fields_audited':list(IMU_ADAPTER_EXECUTION_FIELDS),
            'descriptive_calibration_blocks_excluded_from_execution_comparison':list(DESCRIPTIVE_CALIBRATION_KEYS),
            'limitations':[
                '轨迹是估计值，不是测量真值；机器人身体运动、地形和算法误差未被独立分离。',
                '固定倾斜与动态曲率指标只是描述性趋势；近直线轨迹无法可靠确定完整二维高度平面。',
                '同输入 topic/header/count 不等于逐字节内容相同；本试验有意改变点坐标外参。',
                'IMU 配置、时移与覆盖区间可核验，但现有日志不记录逐条算法实收 IMU 哈希，不能证明接收历史逐字节一致。',
                'binary_sha256 来自运行快照；可选 runtime_binary_check.json 单独保留，缺失时不声称已验证实际加载库。',
                '局部匹配残差小不是全局准确度；短段通过不认证长距离、高速、室外开阔场景。']}
    for run in runs:
        first_imu=run['adapter'].get('first_output_stamp_ns')
        last_imu=run['adapter'].get('last_output_stamp_ns')
        run['integrity']['adapter_imu_covers_common_state_interval']=bool(
            first_imu is not None and last_imu is not None and first_imu*1e-9<=lo and last_imu*1e-9>=hi)
        report['runs'].append({'name':run['name'],'directory':str(run['path']),
            'calibration_descriptive_metadata':{k:run['calibration'].get(k) for k in DESCRIPTIVE_CALIBRATION_KEYS},
            'integrity':run['integrity'],'integrity_all_passed':all(run['integrity'].values()),
            'snapshots':run['snapshots'],'runtime_binary_check':run['runtime_binary_check'],
            'reference_comparison':compare(runs[0],run,set(args.allow_rear_extrinsic_run)),
            'admitted_scan_count':len(run['scans']),'state_rows_total':len(run['state']),
            'initialization_scan_difference_count':len(run['scans'])-len(run['state']),
            'adapter_statistics':{k:v for k,v in run['adapter'].items() if k not in ['config','notes']},
            'common_metrics':metrics(run['state'],lo,hi,args.segment_seconds)})
    report['all_control_and_integrity_checks_passed']=all(r['integrity_all_passed'] and r['reference_comparison']['all_passed'] for r in report['runs'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    text=markdown(report)
    with md_path.open('x') as stream:
        stream.write(text)
    print(text)
    print(f'JSON: {args.output.resolve()}')
    print(f'Markdown: {md_path.resolve()}')


if __name__=='__main__':
    main()
