#!/usr/bin/env python3
"""Offline actual 500Hz full-scene component audit. No runtime imports."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

DT=2_000_000


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def xy_dist(a,b):
    return math.hypot(b[0]-a[0],b[1]-a[1])


def body_yaw_rate(row):
    w,x,y,z=row['quaternion_wxyz']
    v=row['angular_velocity_world']
    return 2*(x*z+w*y)*v[0]+2*(y*z-w*x)*v[1]+(1-2*(x*x+y*y))*v[2]


def yaw(row):
    w,x,y,z=row['quaternion_wxyz']
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def tilt(row):
    w,x,y,z=row['quaternion_wxyz']
    return math.acos(max(-1.,min(1.,1-2*(x*x+y*y))))


def straight_demand_diagnostics(rows, reference_yaw=0.):
    """Time-integral vs along-path displacement, separate from spatial side.

    This is a straight component reference in the requested world heading,
    not an executed navigation curve, tracker phase or acceptance proof.
    The source-BEGIN demand is held over its own next real physical step.
    """
    if not rows or not math.isfinite(reference_yaw):raise ValueError('invalid_straight_component_reference')
    axis=(math.cos(reference_yaw),math.sin(reference_yaw));normal=(-axis[1],axis[0])
    origin=rows[0]['position'];integral=compensation=0.;previous=None
    maximum=maximum_side=0.;peak=None;last=None
    for row in rows:
        if previous is not None:
            delta=row['source_ns']-previous['source_ns']
            if delta!=DT:raise ValueError('nonconsecutive_component_integral_samples')
            addition=previous['highlevel_command'][0]*delta*1e-9-compensation
            updated=integral+addition;compensation=(updated-integral)-addition;integral=updated
        displacement=[row['position'][i]-origin[i] for i in (0,1)]
        along=sum(a*b for a,b in zip(displacement,axis))
        lateral=sum(a*b for a,b in zip(displacement,normal))
        error=along-integral
        last=dict(source_ns=row['source_ns'],demand_integral_m=integral,
            actual_along_path_m=along,signed_integral_error_m=error,signed_lateral_m=lateral)
        if peak is None or abs(error)>maximum:maximum=abs(error);peak=dict(last)
        maximum_side=max(maximum_side,abs(lateral));previous=row
    return dict(reference='straight requested world heading; not a navigation curve',reference_yaw_rad=reference_yaw,
        source_begin_ns=rows[0]['source_ns'],source_end_ns=rows[-1]['source_ns'],terminal=last,
        peak_absolute_along_demand_integral_error_m=maximum,peak_integral_error_witness=peak,
        peak_absolute_spatial_lateral_error_m=maximum_side,
        original_dot10_time_integral_budget_met=maximum<=.10,
        original_dot10_spatial_side_budget_met=maximum_side<=.10,
        formal_tracking_acceptance=False)


def window(rows,lo,hi):
    values=[r for r in rows if lo<=r['source_ns']<=hi]
    assert values and values[0]['source_ns']==lo and values[-1]['source_ns']==hi
    return dict(samples=len(values),source_begin_ns=lo,source_end_ns=hi,
        net_xy_m=xy_dist(values[0]['position'],values[-1]['position']),
        net_xyz_m=[b-a for a,b in zip(values[0]['position'],values[-1]['position'])],
        cumulative_xy_m=sum(xy_dist(a['position'],b['position']) for a,b in zip(values,values[1:])),
        mean_measured_xy_speed_mps=sum(math.hypot(*r['linear_velocity_world'][:2]) for r in values)/len(values),
        peak_measured_full_xyz_mps=max(math.hypot(*r['linear_velocity_world']) for r in values),
        peak_measured_xy_mps=max(math.hypot(*r['linear_velocity_world'][:2]) for r in values),
        peak_measured_full_angular_radps=max(math.hypot(*r['angular_velocity_world']) for r in values),
        peak_measured_body_yaw_radps=max(abs(body_yaw_rate(r)) for r in values),
        peak_tilt_rad=max(tilt(r) for r in values))


def analyze(run,reference_yaw=0.):
    run=Path(run)
    summary=json.loads((run/'summary.json').read_text())
    domain=summary['policy_component']
    assert domain['enabled'] is True and domain['external_udp_disabled'] is True and domain['navigation_authorization'] is False
    assert summary['physics_hz']==500 and summary['robot_asset']['stall_recovery']['enabled'] is False
    rows=read_rows(run/'policy_component_state_500hz.jsonl')
    frames=summary['physics_frames'];zero=domain['true_zero_begin_source_ns'];duration=frames*DT
    assert len(rows)==frames+1 and frames==6000 and zero==8_000_000_000 and duration-zero>=4_000_000_000
    assert rows[0]['source_ns']==0 and rows[-1]['source_ns']==duration
    assert all(r['source_ns']==i*DT and r['source_tick']==i for i,r in enumerate(rows))
    assert all(r['acquisition_phase']=='actual_pre_policy_PhysX_BEGIN' for r in rows[:-1])
    assert rows[-1]['acquisition_phase']=='actual_final_PhysX_END'
    assert all(b['native_physics_tick']==a['native_physics_tick']+1 for a,b in zip(rows,rows[1:]))
    assert all(b['policy_counter']==a['policy_counter']+1 for a,b in zip(rows,rows[1:]))
    desired=domain.get('desired_navigation_linear_speed')
    if desired is None:
        assert all(r['highlevel_command']==[0.,0.] for r in rows)
        actual_input=struct.unpack('f',struct.pack('f',domain['internal_policy_linear_speed']))[0]
        assert all(r['official_policy_input']==([actual_input,0.,0.] if r['source_ns']<zero else [0.,0.,0.]) for r in rows)
    else:
        assert domain['internal_policy_linear_speed'] is None
        assert all(r['highlevel_command']==([desired,0.] if r['source_ns']<zero else [0.,0.]) for r in rows)
        limits=summary['robot_asset']['velocity_feedback']['policy_command_limits']
        actual_limits=[struct.unpack('f',struct.pack('f',v))[0] for v in limits]
        assert all(abs(r['official_policy_input'][0])<=actual_limits[0] and r['official_policy_input'][1]==0
                   and abs(r['official_policy_input'][2])<=actual_limits[1] for r in rows)
    zero_rows=[r for r in rows if r['source_ns']>=zero]
    start=None;completion=None;max_good_ns=0
    for r in zero_rows:
        good=math.hypot(*r['linear_velocity_world'])<=.03 and math.hypot(*r['angular_velocity_world'])<=.05
        if not good:start=None
        elif start is None:start=r['source_ns']
        if start is not None:
            max_good_ns=max(max_good_ns,r['source_ns']-start)
            if completion is None and r['source_ns']-start>=1_000_000_000:
                completion=dict(window_begin_ns=start,confirmation_source_ns=r['source_ns'],
                    exact_zero_to_confirmation_s=(r['source_ns']-zero)/1e9)
    zero_pose=zero_rows[0]['position'];zero_yaw=yaw(zero_rows[0])
    stop=dict(exact_zero_source_ns=zero,zero_tail_execution_duration_ns=duration-zero,
        complete_500hz_measurement_continuity=True,full_linear_threshold=.03,full_angular_threshold=.05,
        continuous_window_ns=1_000_000_000,completion=completion,
        original_three_second_deadline_pass=completion is not None and completion['exact_zero_to_confirmation_s']<=3.,
        longest_good_source_span_s=max_good_ns/1e9,
        max_zero_displacement_xy_m=max(xy_dist(zero_pose,r['position']) for r in zero_rows),
        cumulative_zero_xy_m=sum(xy_dist(a['position'],b['position']) for a,b in zip(zero_rows,zero_rows[1:])),
        max_zero_yaw_displacement_rad=max(abs(math.remainder(yaw(r)-zero_yaw,2*math.pi)) for r in zero_rows))
    links=read_rows(run/'robot_link_readings.jsonl')
    foot_counts={side:0 for side in ('fl','fr','hl','hr')};last4_counts=dict(foot_counts)
    min_foot=math.inf;max_foot=-math.inf;states=0
    for r in links:
        if not 0<=r['sim_time_ns']<zero:continue
        assert len(r['colliders'])==13
        feet={side:[c for c in r['colliders'] if c['rigid_body_path']==f'/World/Spot/{side}_foot'] for side in foot_counts}
        assert all(len(v)==1 and v[0]['shape']['type']=='Sphere' for v in feet.values())
        for side,value in feet.items():
            bottom=float(value[0]['aabb_min'][2]);min_foot=min(min_foot,bottom);max_foot=max(max_foot,bottom)
            if bottom>.01:
                foot_counts[side]+=1
                if r['sim_time_ns']>=4_000_000_000:last4_counts[side]+=1
        states+=1
    observations=read_rows(run/'policy_observations.jsonl')
    source_obs=[r for r in observations if r['phase']=='source']
    assert len(source_obs)==frames//10 and all(r['source_tick']==i*10 for i,r in enumerate(source_obs))
    assert all(len(r['observation'])==48 for r in observations)
    all_stats=window(rows,0,duration)
    evidence=[]
    for name in ('summary.json','policy_component_state_500hz.jsonl','policy_observations.jsonl','robot_link_readings.jsonl','policy_command_events.jsonl'):
        p=run/name;evidence.append(dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size))
    return dict(schema=1,kind='offline_full_scene_internal_policy_speed_component',run=str(run.resolve()),
        internal_forward_policy_input=domain['internal_policy_linear_speed'],
        desired_navigation_linear_speed=desired,
        servo_mode='direct_internal_input' if desired is None else 'original_measured_PI_servo',
        actual_internal_linear_input_motion_minmax=[min(r['official_policy_input'][0] for r in rows if r['source_ns']<zero),
                                                    max(r['official_policy_input'][0] for r in rows if r['source_ns']<zero)],
        actual_internal_linear_input_zero_minmax=[min(r['official_policy_input'][0] for r in zero_rows),
                                                  max(r['official_policy_input'][0] for r in zero_rows)],
        formal_navigation_pass=False,
        navigation_authorization=False,physical_deployment_acceptance=False,
        source_continuity=dict(actual_native_steps=frames,body_samples=len(rows),physics_hz=500,
            begin_samples=frames,terminal_actual_end_samples=1,policy_source_inferences=len(source_obs),
            inference_counter_first_last=[source_obs[0]['inference_policy_counter'],source_obs[-1]['inference_policy_counter']],
            counter_and_native_tick_increment_verified=True),
        motion_0_8s=window(rows,0,zero),motion_last_4s=window(rows,4_000_000_000,zero),whole_12s=all_stats,
        existing_domain_comparison=dict(full_xyz_dot50=all_stats['peak_measured_full_xyz_mps']<=.50,
            physical_full_xyz_dot60=all_stats['peak_measured_full_xyz_mps']<=.60,
            physical_body_yaw_dot80=all_stats['peak_measured_body_yaw_radps']<=.80,
            full_angular_dot80_separate=all_stats['peak_measured_full_angular_radps']<=.80),
        demand_tracking_diagnostics=(dict(motion_0_8s=straight_demand_diagnostics(
            [r for r in rows if r['source_ns']<=zero],reference_yaw),
            whole_12s_including_zero_drift=straight_demand_diagnostics(rows,reference_yaw)) if desired is not None else None),
        feet=dict(cadence='original50Hz plus native10Hz BEGIN witnesses',measured_motion_bundles=states,
            exact_registered_sphere_bottom_flight_threshold_m=.01,flight_counts_0_8s=foot_counts,
            flight_counts_last4s=last4_counts,min_bottom_m=min_foot,max_bottom_m=max_foot),stop=stop,
        body_envelope_attestation=summary['body_envelope_attestation'],
        static_geometry_attestation=summary['static_geometry_attestation'],native_floor_hit_audit=summary['native_floor_hit_audit'],
        limitations=['Single cold bootstrap and straight internal demand; stable minimum response and history-generalized navigation need further evidence.',
            'Internal policy input is separate from highlevel command and cannot authorize post-safety command floors.',
            'Demand time-integral error and spatial side deviation are distinct; neither substitutes for original tracker current-curve acceptance.',
            'Fullangular norm and body yaw are distinct; all actual Z/tilt/transient speed retained.'],evidence=evidence)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--reference-yaw',type=float,default=0.,help='Requested straight component world heading for diagnostics')
    args=parser.parse_args();result=analyze(args.run,args.reference_yaw)
    data=json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.output:
        if args.output.exists():raise ValueError('offline_result_already_exists')
        args.output.write_text(data)
    else:print(data,end='')
