#!/usr/bin/env python3
"""Compare two physical AIRY beam-calibration fingerprints on four messages."""
import json
import sqlite3

import numpy as np
import yaml

from audit_beam_frame import BAG, HERE, cone_angle_residual, decode_angles, read_cloud, stats


def main():
    metadata = yaml.safe_load((BAG/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    db = sqlite3.connect((BAG/metadata['relative_file_paths'][0]).as_uri()+'?mode=ro', uri=True)
    topics = dict(db.execute('SELECT name,id FROM topics'))
    factory = json.loads((BAG/'snapshots/extrinsics_20260917.json').read_text())
    candidates = []
    for packet in factory['raw_capture']['difop']:
        device = next(v for v in factory['factory_devices'] if v['src']==packet['src'])
        payload = bytes.fromhex(packet['hex'])
        vertical, horizontal = decode_angles(payload,468),decode_angles(payload,756)
        order = np.argsort(vertical)
        candidates.append({'src':packet['src'],'serial_number':device['sn_hex'],
                           'vertical':vertical[order], 'horizontal':horizontal[order]})
    report = {'bag':str(BAG), 'read_only':True, 'ros_initialized':False,
              'cloud_messages_read':4,
              'method':'Unfitted native beam-cone angle residual, using both candidate device calibration arrays on identical points.',
              'limitations':[
                  'Supports topic-to-device calibration fingerprint association, not a direct audit of runtime driver configuration.',
                  'Cloud generation formula accuracy is not physical angular accuracy or extrinsic calibration precision.',
                  'Beam cones do not determine rotation around native scanner Z.'],
              'frames':[]}
    for topic in ['/front_lidar','/rear_lidar']:
        for seconds in [.65,1.5]:
            points,ring,info=read_cloud(db,topics[topic],metadata['starting_time']['nanoseconds_since_epoch']+int(seconds*1e9))
            result={'topic':topic,'requested_s':seconds,**info,'candidate_device_residuals':[]}
            for candidate in candidates:
                result['candidate_device_residuals'].append({
                    'src':candidate['src'],'serial_number':candidate['serial_number'],
                    **stats(cone_angle_residual(points,ring,candidate['vertical'],candidate['horizontal']))})
            report['frames'].append(result)
            print(json.dumps(result,indent=2),flush=True)
    db.close()
    with (HERE/'cross_device_report.json').open('x') as stream:
        json.dump(report,stream,indent=2)


if __name__=='__main__':
    main()
