"""Sequential offline pipeline. Native tools never publish into the live ROS graph."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import yaml
from .configuration import parse_config, validate_bag
from .mola_runtime import atomic_json, now
from .toolchain import binary, live_binary, environment, prefix, availability


class Cancelled(Exception):
    pass


def check_native_log(path):
    # MOLA 3.0 can report an internal fatal error but exit with code 0.
    # Never mark a partial trajectory/map as a successfully completed task.
    with Path(path).open(errors='replace') as stream:
        for line in stream:
            if re.search(r'[|\[]ERROR\s*[|\]]|fatal error|Exit due to exception', line, re.I):
                detail = line.strip() + ''.join(next(stream, '') for _ in range(4))
                raise RuntimeError(f'原生程序报告异常：{detail[:1600]}')


def loop_summary(path):
    # Counts reported by native GNC are diagnostics, not a map-quality certificate.
    result = None
    pattern = re.compile(r'Total accepted loop closures: (\d+) \(GNC: (\d+) inliers, (\d+) outliers rejected\)')
    with Path(path).open(errors='replace') as stream:
        for line in stream:
            match = pattern.search(line)
            if match:
                result = dict(zip(('icp_accepted', 'gnc_inliers', 'gnc_outliers'), map(int, match.groups())))
    return result


def prepare(session, app_root, config):
    """Snapshot trusted, installed upstream pipelines, then apply validated parameters."""
    share = prefix(app_root) / 'opt/ros/humble/share'
    sources = {
        'frontend': share / 'mola_lidar_odometry/pipelines/lidar3d-gicp.yaml',
        'loop': share / 'mola_sm_loop_closure/pipelines/loop-closure-f2f-lidar3d-gicp.yaml',
        'estimator': share / 'mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml',
    }
    provenance = {}
    for name, path in sources.items():
        data = path.read_bytes()
        (session / f'upstream_{name}.yaml').write_bytes(data)
        provenance[name] = {'source': str(path), 'sha256': hashlib.sha256(data).hexdigest()}
    frontend = yaml.safe_load(sources['frontend'].read_text())
    params = frontend['params']
    params['lidar_sensor_labels'] = [config.input.lidar_topic]
    params['imu_sensor_label'] = config.input.imu_topic
    params['publish_vehicle_frame'] = config.input.base_frame
    params['publish_reference_frame'] = 'd1max_loc_map'
    params['publish_deskewed_scans'] = config.visualization.enabled
    params['simplemap'].update(generate=True, save_final_map_to_file='raw.simplemap',
        generate_lazy_load_scan_files=True, save_deskewed_scans=True,
        min_translation_between_keyframes=config.frontend.keyframe_distance_m,
        min_rotation_between_keyframes=config.frontend.keyframe_rotation_deg,
        # Measure from the last inserted keyframe so revisits are not discarded.
        # The native 3.0 API supports this flag (not shown in its default YAML).
        measure_from_last_kf_only=True, min_nearby_poses_occupied=1)
    params['local_map_updates'].update(max_distance_to_keep_keyframes=config.frontend.local_map_radius_m,
        min_translation_between_keyframes=config.frontend.keyframe_distance_m,
        min_rotation_between_keyframes=config.frontend.keyframe_rotation_deg)
    for entry in frontend['localmap_generator']:
        entry['params']['metric_map_definition']['insertOpts']['remove_frames_farther_than'] = config.frontend.local_map_radius_m
    frontend['initial_localization'].update(method='InitLocalization::PitchAndRollFromIMU',
                                            use_imu_orientation=False)
    for filters in frontend.values():
        if isinstance(filters, list):
            for entry in filters:
                if isinstance(entry, dict) and entry.get('class_name') == 'mp2p_icp_filters::FilterDeskew':
                    entry['params'].update(method='MotionCompensationMethod::IMU',
                                           silently_ignore_no_timestamps=False)
    loop = yaml.safe_load(sources['loop'].read_text())
    # MOLA saves both original observations and an additional "deskewed" one.
    # Select exactly the latter, or raw and deskewed clouds would be mixed twice.
    loop['observations_generator'][0]['params']['process_sensor_labels_regex'] = '^deskewed$'
    lc = config.loop_closure
    loop['params'].update(use_gnss=False, gnss_add_horizontality=False, assume_planar_world=False,
        min_icp_goodness=lc.min_icp_quality,
        max_distance_for_lc_candidate=lc.max_candidate_distance_m,
        min_distance_between_frames=.25, min_frames_between_lc=lc.min_frame_separation,
        max_lc_optimization_rounds=lc.max_optimization_rounds,
        lc_optimize_every_n=0, use_kiss_matcher=False)
    # Our simplemap stores already deskewed scans, not raw scans. Do NOT deskew twice.
    loop['observations_filter'] = [f for f in loop['observations_filter']
                                  if f['class_name'] != 'mp2p_icp_filters::FilterDeskew']
    for entry in loop['observations_filter']:
        params = entry['params']
        if params.get('input_pointcloud_layer') == 'deskewed':
            params['input_pointcloud_layer'] = 'raw'
        if 'pointcloud_layer_to_remove' in params:
            params['pointcloud_layer_to_remove'] = [n for n in params['pointcloud_layer_to_remove'] if n != 'deskewed']
    estimator = yaml.safe_load(sources['estimator'].read_text())
    estimator['params']['enforce_planar_motion'] = False
    export = {'generators': [{'class_name': 'mp2p_icp_filters::Generator', 'params': {
        'target_layer': 'raw', 'process_sensor_labels_regex': '^deskewed$',
        'process_class_names_regex': 'mrpt::obs::CObservationPointCloud',
        'throw_on_unhandled_observation_class': True}}], 'filters': [], 'final_filters': []}
    for name, value in (('frontend', frontend), ('loop', loop), ('estimator', estimator), ('export', export)):
        (session / f'{name}.yaml').write_text(yaml.safe_dump(value, sort_keys=False))
    packages = prefix(app_root).parent / 'packages.json'
    if packages.is_file():
        shutil.copyfile(packages, session / 'packages.json')
    compat = prefix(app_root).parent / 'compat.json'
    if compat.is_file():
        shutil.copyfile(compat, session / 'compat.json')
    live = prefix(app_root).parent / 'live.json'
    if config.visualization.enabled and live.is_file():
        shutil.copyfile(live, session / 'live_build.json')
    return provenance


class Worker:
    def __init__(self, session, app_root):
        self.session, self.app_root = Path(session).resolve(), Path(app_root).resolve()
        self.config = parse_config((self.session / 'task.yaml').read_text())
        self.manifest = json.loads((self.session / 'manifest.json').read_text())
        self.cancelled = False
        self.process = None
        self.env = environment(app_root)
        settings = self.session / 'native-config'
        settings.mkdir(exist_ok=True)
        self.env['XDG_CONFIG_HOME'] = str(settings)
        self.env.update(MOLA_DESKEW_METHOD='MotionCompensationMethod::IMU',
            MOLA_IGNORE_NO_POINT_STAMPS='false', MOLA_SIMPLEMAP_GENERATE_LAZY_LOAD='true',
            MOLA_SIMPLEMAP_OUTPUT='raw.simplemap', MOLA_SAVE_DESKEWED_SCANS='true',
            MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION='false', ASSUME_PLANAR_WORLD='false',
            USE_GNSS='false', OMP_NUM_THREADS=str(self.config.execution.threads))
        if self.config.visualization.enabled:
            v = self.config.visualization
            self.env.update(D1MAX_MOLA_LIVE_DIR=str(self.session / 'live'),
                D1MAX_MOLA_LIVE_VOXEL=str(v.voxel_size_m),
                D1MAX_MOLA_LIVE_PERIOD=str(v.update_period_sec),
                D1MAX_MOLA_LIVE_MAX_POINTS=str(v.max_points))
        fixed = self.config.sensors.source == 'fixed'
        for sensor, values in (('LIDAR', self.config.sensors.lidar_pose), ('IMU', self.config.sensors.imu_pose)):
            self.env[f'MOLA_USE_FIXED_{sensor}_POSE'] = str(fixed).lower()
            for key, value in zip(('X', 'Y', 'Z', 'YAW', 'PITCH', 'ROLL'), values):
                self.env[f'{sensor}_POSE_{key}'] = str(value)

    def update(self, **value):
        self.manifest.update(value, updated_at=now())
        atomic_json(self.session / 'manifest.json', self.manifest)

    def check_cancel(self):
        if self.cancelled or (self.session / 'cancel.request').exists():
            raise Cancelled()

    def stage(self, name, progress, command):
        self.check_cancel()
        print(f'[{name}]', flush=True)
        self.update(status='running', stage=name, progress=progress, command=command)
        log = self.session / (name + '.log')
        started = time.monotonic()
        try:
            with log.open('w') as stream:
                self.process = subprocess.Popen(command, cwd=self.session, env=self.env,
                                                stdout=stream, stderr=subprocess.STDOUT)
                while self.process.poll() is None:
                    self.check_cancel()
                    if time.monotonic() - started > self.config.execution.stage_timeout_sec:
                        raise RuntimeError(f'{name} 超时，任务已停止，不会自动重试')
                    time.sleep(.1)
                self.check_cancel()
                if self.process.returncode:
                    with log.open('rb') as reader:
                        reader.seek(0, 2)
                        reader.seek(max(0, reader.tell() - 1800))
                        detail = reader.read().decode(errors='replace')
                    raise RuntimeError(f'{name} 失败 ({self.process.returncode})：{detail}')
        finally:
            if self.process and self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=3)
            self.process = None
        self.check_cancel()
        check_native_log(log)

    def export(self, stem, progress):
        sm = self.session / f'{stem}.simplemap'
        if not sm.is_file() or sm.stat().st_size == 0:
            raise RuntimeError('前端未生成有效关键帧地图')
        self.stage(stem + '_metric_map', progress, [str(binary(self.app_root, 'sm2mm')),
            '-i', str(sm), '-o', stem + '.mm', '-p', 'export.yaml', '--externals-dir', str(self.session / 'raw_Images'),
            '--no-progress-bar'])
        self.stage(stem + '_ply', progress + 5, [str(binary(self.app_root, 'mm2ply')),
            '-i', stem + '.mm', '-o', stem, '-b', '--export-fields', 'x,y,z'])
        # Native mm preserves observations/channels; Web PCD is a voxelized XYZ derivative.
        import open3d as o3d
        choices = list(self.session.glob(stem + '*.ply'))
        if len(choices) != 1:
            raise RuntimeError(f'{stem} 点云层数量异常：{len(choices)}')
        cloud = o3d.io.read_point_cloud(str(choices[0]))
        cloud = cloud.remove_non_finite_points().voxel_down_sample(self.config.export.voxel_size_m)
        if len(cloud.points) == 0:
            raise RuntimeError('导出点云为空')
        destination = self.session / (stem + '.pcd')
        if not o3d.io.write_point_cloud(str(destination), cloud, write_ascii=False, compressed=False):
            raise RuntimeError('PCD 保存失败')
        artifacts = [*self.manifest.get('artifacts', []),
            {'file': destination.name, 'role': 'mola_frontend' if stem == 'raw' else 'mola_optimized',
             'points': len(cloud.points), 'status': 'complete', 'fields': ['x', 'y', 'z']}]
        self.update(artifacts=artifacts)

    def run(self):
        try:
            self.update(status='running', stage='preflight', progress=1)
            validate_bag(self.config)
            tools = availability(self.app_root, refresh=True)
            if not tools['available']:
                raise RuntimeError(tools['reason'])
            lio_binary = live_binary(self.app_root) if self.config.visualization.enabled else binary(self.app_root, 'mola-lidar-odometry-cli')
            if not lio_binary.is_file():
                raise RuntimeError('MOLA 实时地图输出模块未构建，请运行 scripts/build_mola_live.py')
            self.update(provenance=prepare(self.session, self.app_root, self.config),
                        stored_scans='raw_and_deskewed', selected_scan_layer='deskewed',
                        export_fields=['x', 'y', 'z'], loop_validation='unverified')
            # Read-only inspection, no publishers or ROS nodes. Uses system ROS Python bindings.
            probe_env = dict(self.env)
            self.env['PYTHONPATH'] = str(self.app_root) + ':/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages'
            self.stage('input_check', 2, ['/usr/bin/python3', '-m', 'backend.mapping.inspect_bag', str(self.session / 'task.yaml')])
            input_bag = self.config.input.bag_path
            if self.config.input.imu_acceleration_scale != 1.0:
                self.stage('normalize_input', 3, ['/usr/bin/python3', '-m', 'backend.mapping.normalize_bag', str(self.session / 'task.yaml')])
                input_bag = str(self.session / 'input_si')
                self.update(prepared_input=input_bag)
            self.env = probe_env
            self.stage('lio', 5, [str(lio_binary),
                '-c', 'frontend.yaml', '--state-estimator-param-file', 'estimator.yaml',
                '--input-rosbag2', input_bag,
                '--lidar-sensor-label', self.config.input.lidar_topic,
                '--imu-sensor-label', self.config.input.imu_topic,
                '--base-link-frame-id', self.config.input.base_frame,
                '--output-tum-path', 'trajectory.tum', '--output-simplemap', 'raw.simplemap'])
            self.export('raw', 40)
            if self.config.loop_closure.enabled:
                self.stage('loop_closure', 60, [str(binary(self.app_root, 'mola-sm-lc-cli')),
                    '-a', 'mola::FrameToFrameLoopClosure', '-p', 'loop.yaml',
                    '-i', 'raw.simplemap', '-o', 'optimized.simplemap',
                    '--externals-dir', str(self.session / 'raw_Images')])
                counts = loop_summary(self.session / 'loop_closure.log')
                self.update(loop_counts=counts, loop_validation='reported' if counts else 'unverified')
                self.export('optimized', 85)
            self.check_cancel()
            self.update(status='complete', stage='complete', progress=100, completed_at=now(), error=None)
            print('MOLA 离线任务完成，结果已保存；回环约束须另行核对。', flush=True)
            return 0
        except Cancelled:
            self.update(status='cancelled', stage='cancelled', completed_at=now(), error='任务已取消，保留已完成阶段和日志')
            return 130
        except Exception as exc:
            self.update(status='failed', completed_at=now(), error=str(exc))
            print(str(exc), flush=True)
            return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    worker = Worker(args.session, Path(__file__).resolve().parents[2])
    def cancel(_signal, _frame):
        worker.cancelled = True
    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    raise SystemExit(worker.run())


if __name__ == '__main__':
    main()
