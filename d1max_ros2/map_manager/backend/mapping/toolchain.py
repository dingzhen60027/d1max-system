"""Private binary prefix and bounded capability checks; never source robot environments."""
import os
import json
from pathlib import Path
import subprocess
import time

TOOLS = ('mola-lidar-odometry-cli', 'mola-sm-lc-cli', 'sm2mm', 'mm2ply')
_cache = {}


def prefix(app_root):
    return Path(os.environ.get('D1MAX_MOLA_PREFIX', Path(app_root) / '.local/mola/root')).resolve()


def environment(app_root):
    root = prefix(app_root)
    ros = root / 'opt/ros/humble'
    compat = root.parent / 'compat/opt/ros/humble'
    zenoh = Path(app_root).parent / 'local/opt/ros/humble'
    # Do not inherit MOLA expressions, dataset paths, TF, or live Zenoh settings.
    env = {k: os.environ[k] for k in ('HOME', 'LANG', 'LC_ALL', 'USER', 'TMPDIR') if k in os.environ}
    env.update(PATH=f'{ros}/bin:/usr/bin:/bin',
               LD_LIBRARY_PATH=f'{compat}/lib:{compat}/lib/x86_64-linux-gnu:{ros}/lib:{ros}/lib/x86_64-linux-gnu:{root}/usr/lib/x86_64-linux-gnu:{zenoh}/lib:{zenoh}/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib:/usr/local/lib',
               AMENT_PREFIX_PATH=f'{ros}:{zenoh}:/opt/ros/humble',
               RMW_IMPLEMENTATION='rmw_zenoh_cpp', ROS_DOMAIN_ID='214',
               ZENOH_SESSION_CONFIG_URI=str(Path(app_root) / 'config/mapping/zenoh-offline.json5'),
               ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1')
    return env


def binary(app_root, name):
    if name not in TOOLS:
        raise ValueError('Unregistered MOLA executable')
    return prefix(app_root) / 'opt/ros/humble/bin' / name


def live_binary(app_root):
    return prefix(app_root).parent / 'live/opt/ros/humble/bin/d1max-mola-lidar-cli'


def availability(app_root, refresh=False):
    key = str(prefix(app_root))
    if not refresh and key in _cache and time.monotonic() - _cache[key][0] < 30:
        return dict(_cache[key][1])
    details, reason = {}, None
    metric_plugin = prefix(app_root) / 'opt/ros/humble/lib/x86_64-linux-gnu/libmola_metric_maps.so'
    if not metric_plugin.is_file():
        return {'available': False, 'reason': 'MOLA 缺少 KeyframePointCloudMap 地图插件', 'prefix': key}
    compatibility = prefix(app_root).parent / 'compat.json'
    fixed_plugin = prefix(app_root).parent / 'compat/opt/ros/humble/lib/libmola_metric_maps.so.3.0.0'
    try:
        ready = fixed_plugin.is_file() and bool(json.loads(compatibility.read_text())['library_sha256'])
    except (OSError, ValueError, KeyError):
        ready = False
    if not ready:
        return {'available': False, 'reason': 'MOLA Humble 兼容插件未就绪，请运行 build_mola_compat.py', 'prefix': key}
    for name in TOOLS:
        path = binary(app_root, name)
        if not path.is_file():
            reason = f'MOLA 未安装：{name}'
            break
        try:
            result = subprocess.run([str(path), '--help'], env=environment(app_root),
                                    capture_output=True, text=True, timeout=5)
            if result.returncode and not ('USAGE:' in result.stdout and '--help' in result.stdout):
                raise ValueError((result.stderr or result.stdout)[-500:])
            details[name] = result.stdout
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            reason = f'MOLA 依赖不可用：{name}: {exc}'
            break
    required = {'mola-lidar-odometry-cli': ['--input-rosbag2', '--imu-sensor-label', '--output-simplemap'],
                'mola-sm-lc-cli': ['--algorithm', '--externals-dir'],
                'sm2mm': ['--externals-dir']}
    if not reason:
        for name, flags in required.items():
            if any(flag not in details.get(name, '') for flag in flags):
                reason = f'MOLA 版本不兼容：{name}'
                break
    value = {'available': reason is None, 'reason': reason, 'prefix': key}
    _cache[key] = time.monotonic(), value
    return dict(value)
