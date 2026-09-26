#!/usr/bin/env python3
"""Check static gravity/plane consistency from saved data, without ROS.

Horizontal floor is used as an offline validation fact only, never fed to LIO.
No output is a production-ready calibration.
"""
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ROOT=Path('/home/dndx/d1max_nav_ws')
HERE=Path(__file__).resolve().parent


def unit(v):
    return v/np.linalg.norm(v)


def angle(a,b):
    return float(np.degrees(np.arccos(np.clip(np.dot(unit(a),unit(b)),-1,1))))


def summary_imu(name):
    source=ROOT/'experiments/central_imu_quality_20260918'/f'{name}.npz'
    data=np.load(source)
    selected=data['stamp_ns']-data['stamp_ns'][0] <= 2_000_000_000
    mean=data['accel'][selected].mean(axis=0)
    return data,{'source':str(source),'first_2s_count':int(selected.sum()),
                 'first_2s_raw_accel_mean':mean.tolist(),
                 'first_2s_raw_accel_std':data['accel'][selected].std(axis=0).tolist(),
                 'first_2s_raw_gyro_norm_p95':float(np.quantile(np.linalg.norm(data['gyro'][selected],axis=1),.95)),
                 'first_2s_specific_force_unit':unit(mean).tolist()}


def main():
    geometry=json.loads((HERE/'dual_geometry_strict_pair_report.json').read_text())
    calibration_path=ROOT/'experiments/central_imu_fasterlio_20260918/calibration.yaml'
    calibration=yaml.safe_load(calibration_path.read_text())
    factory_path=ROOT/'bags/slam_raw_20260917_171716_fe8f38/snapshots/extrinsics_20260917.json'
    factory=json.loads(factory_path.read_text())
    fitted=json.loads((ROOT/'experiments/central_imu_fasterlio_20260918/calibration/gyro_rotation_candidate.json').read_text())
    central,cs=summary_imu('central')
    front,fs=summary_imu('front')
    # L: raw front point frame; F: raw front IMU axes; N: current normalized points;
    # I: historical Rx(+90deg) normalized front IMU; C: raw central IMU.
    R_N_L=np.asarray(geometry['R_NF'])
    R_C_N=np.asarray(calibration['lio_extrinsic']['rotation']).reshape(3,3)
    R_I_F=np.array([[1.,0,0],[0,0,-1],[0,1,0]])
    front_device=next(x for x in factory['factory_devices'] if x['src']=='192.168.1.200')
    R_F_L=Rotation.from_quat(front_device['quaternion_xyzw']).as_matrix()
    R_C_F=np.asarray(fitted['R_c_from_f_raw'])
    R_C_N_difop=R_C_F@R_F_L@R_N_L.T
    predictions={
        'central_current_R_C_N':R_C_N.T@np.asarray(cs['first_2s_specific_force_unit']),
        'front_imu_historical_identity_extrinsic':R_I_F@np.asarray(fs['first_2s_specific_force_unit']),
        'front_imu_conditional_DIFOP':R_N_L@R_F_L.T@np.asarray(fs['first_2s_specific_force_unit']),
        'central_conditional_DIFOP_chain':R_C_N_difop.T@np.asarray(cs['first_2s_specific_force_unit']),
    }
    report={'calibration_input':str(calibration_path),'factory_input':str(factory_path),
            'interpretation':'Offline consistency validation under the user-supplied horizontal-floor assumption; not runtime floor locking.',
            'central_imu':cs,'front_imu':fs,
            'predicted_up_normal_in_N':{k:v.tolist() for k,v in predictions.items()},
            'frames':[],
            'conditional_difop':{
                'assumption':'DIFOP quaternion maps raw front LiDAR point coordinates to raw front IMU vector coordinates; bag driver conventions still need validation.',
                'factory_front_device':front_device,
                'R_F_L':R_F_L.tolist(),
                't_F_L_m':front_device['translation_m'],
                'R_F_N_for_raw_front_IMU_and_current_normalized_cloud':(R_F_L@R_N_L.T).tolist(),
                'R_I_N_for_normalized_front_IMU_and_current_normalized_cloud':(R_I_F@R_F_L@R_N_L.T).tolist(),
                't_I_N_m':(R_I_F@front_device['translation_m']).tolist(),
                'R_C_N_diagnostic_chain':R_C_N_difop.tolist(),
                'rotation_change_from_current_deg':float(np.degrees(Rotation.from_matrix(R_C_N_difop@R_C_N.T).magnitude())),
                'approved_for_production':False,
                'official_decoder_reference':'https://github.com/RoboSense-LiDAR/rs_driver/blob/main/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp',
                'official_decoder_scope':'Current upstream decodeDifopPkt decodes q/t to device_info; decodeImuPkt reports raw scaled axes. This does not prove the exact binary/extra transforms used when recording this bag.'},
            'limitations':['Initial acceleration includes sensor bias and possible small motion; no external calibrated inclinometer.',
                           'DIFOP is a device-reported valid rotation, but its application convention is conditional here.',
                           'Plane normal comparison cannot validate yaw, translation, or dynamic deskew.',
                           'No algorithm parameters, maps, bags, or TFs were modified.']}
    for frame in geometry['frames']:
        target=frame['front']['header_ns']+13_000_000
        select=np.abs(central['stamp_ns']-target)<=150_000_000
        local=central['accel'][select].mean(axis=0)
        local_up=R_C_N.T@unit(local)
        rois=[]
        for roi in frame['floor_roi_comparisons']:
            rois.append({'front_roi':{k:roi['front'][k] for k in ['x_abs_max_m','y_range_m','normal']},
                         'rear_roi':{k:roi['rear'][k] for k in ['x_abs_max_m','y_range_m','normal']},
                         'angles_deg':{name:{'front':angle(pred,np.asarray(roi['front']['normal'])),
                                             'rear':angle(pred,np.asarray(roi['rear']['normal']))}
                                       for name,pred in predictions.items()},
                         'central_local_300ms_angles_deg':{'front':angle(local_up,np.asarray(roi['front']['normal'])),
                                                         'rear':angle(local_up,np.asarray(roi['rear']['normal']))}})
        report['frames'].append({'record_requested_s':frame['requested_record_s'],
                                 'central_local_300ms_count':int(select.sum()),
                                 'central_local_300ms_accel_mean':local.tolist(),
                                 'rois':rois})
    output=HERE/'gravity_geometry_report.json'
    with output.open('x') as stream:
        json.dump(report,stream,indent=2)
    for key in predictions:
        for sensor in ['front','rear']:
            values=[r['angles_deg'][key][sensor] for f in report['frames'] for r in f['rois']]
            print(key,sensor,min(values),max(values))
    print(output)


if __name__=='__main__':
    main()
