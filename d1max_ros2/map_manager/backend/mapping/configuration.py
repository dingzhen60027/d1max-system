"""Validated task configuration, not executable MOLA YAML from a browser."""
from pathlib import Path
from typing import Literal
import re
import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)


class Input(StrictModel):
    bag_path: str = ''
    lidar_topic: str = '/front_lidar'
    imu_topic: str = '/front_lidar/imu'
    base_frame: str = 'front_lidar'
    # 1.0 for SI; 9.80665 only for bags known to contain acceleration in g.
    imu_acceleration_scale: float = Field(default=1.0, gt=0, le=100)


class Sensors(StrictModel):
    source: Literal['bag_tf', 'fixed'] = 'bag_tf'
    # x,y,z in metres; yaw,pitch,roll in degrees, relative to input.base_frame.
    lidar_pose: list[float] = Field(default_factory=lambda: [0.] * 6, min_length=6, max_length=6)
    imu_pose: list[float] = Field(default_factory=lambda: [0.] * 6, min_length=6, max_length=6)


class Frontend(StrictModel):
    pipeline: Literal['gicp_imu'] = 'gicp_imu'
    keyframe_distance_m: float = Field(default=.3, ge=.05, le=5)
    keyframe_rotation_deg: float = Field(default=10., ge=1, le=60)
    local_map_radius_m: float = Field(default=30., ge=5, le=150)


class LoopClosure(StrictModel):
    algorithm: Literal['frame_to_frame_gicp'] = 'frame_to_frame_gicp'
    enabled: bool = True
    assume_planar_world: Literal[False] = False
    min_icp_quality: float = Field(default=.75, ge=.5, le=1.)
    max_candidate_distance_m: float = Field(default=2.5, ge=.5, le=20.)
    min_frame_separation: int = Field(default=30, ge=10, le=10000)
    max_optimization_rounds: int = Field(default=10, ge=1, le=100)


class Export(StrictModel):
    voxel_size_m: float = Field(default=.10, ge=.02, le=1.)


class Execution(StrictModel):
    threads: int = Field(default=4, ge=1, le=16)
    stage_timeout_sec: int = Field(default=7200, ge=30, le=86400)


class Visualization(StrictModel):
    enabled: bool = True
    voxel_size_m: float = Field(default=.15, ge=.05, le=1.)
    update_period_sec: float = Field(default=1., ge=.2, le=10.)
    max_points: int = Field(default=1000000, ge=10000, le=2000000)


class MolaConfig(StrictModel):
    schema_version: Literal[1] = 1
    profile: Literal['mola_lio_lc'] = 'mola_lio_lc'
    input: Input = Field(default_factory=Input)
    sensors: Sensors = Field(default_factory=Sensors)
    frontend: Frontend = Field(default_factory=Frontend)
    loop_closure: LoopClosure = Field(default_factory=LoopClosure)
    export: Export = Field(default_factory=Export)
    execution: Execution = Field(default_factory=Execution)
    visualization: Visualization = Field(default_factory=Visualization)

    @model_validator(mode='after')
    def safe_strings(self):
        for topic in (self.input.lidar_topic, self.input.imu_topic):
            if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', topic):
                raise ValueError('雷达/IMU 话题必须是具体 ROS 话题，不能是正则或表达式')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/]*', self.input.base_frame):
            raise ValueError('base_frame 非法')
        # Upstream CLI interpolates bag paths into MOLA's extended YAML parser.
        if any(c in self.input.bag_path for c in ("'", '"', '$', '`', '\n', '\r', ',', '\x00')):
            raise ValueError('rosbag 路径不能包含引号、逗号或配置表达式')
        return self


def parse_config(text: str, bag_path: str | None = None) -> MolaConfig:
    if not isinstance(text, str):
        raise ValueError('配置必须是 YAML 文本')
    if len(text) > 32768:
        raise ValueError('配置过大')
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError('YAML 语法错误') from exc
    if not isinstance(raw, dict):
        raise ValueError('配置必须是 YAML 对象')
    if bag_path is not None:
        if not isinstance(raw.get('input', {}), dict):
            raise ValueError('input 必须是配置对象')
        raw['input'] = {**raw.get('input', {}), 'bag_path': bag_path}
    return MolaConfig.model_validate(raw)


def config_yaml(config: MolaConfig) -> str:
    return yaml.safe_dump(config.model_dump(), allow_unicode=True, sort_keys=False)


def validate_bag(config: MolaConfig) -> Path:
    try:
        path = Path(config.input.bag_path).expanduser().resolve(strict=True)
    except OSError as exc:
        raise ValueError('rosbag 目录不存在或无法读取') from exc
    if not config.input.bag_path or not path.is_dir() or not (path / 'metadata.yaml').is_file():
        raise ValueError('请选择包含 metadata.yaml 的完整 rosbag2 目录；不会自动修改或重建原始 bag')
    # Validate resolved paths too, including any symlink target.
    MolaConfig.model_validate({**config.model_dump(), 'input': {**config.input.model_dump(), 'bag_path': str(path)}})
    try:
        metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
        if not isinstance(metadata, dict) or not isinstance(metadata.get('topics_with_message_count'), list):
            raise ValueError('rosbag metadata.yaml 格式不完整')
    except (OSError, yaml.YAMLError, TypeError, KeyError) as exc:
        raise ValueError('rosbag metadata.yaml 无法读取') from exc
    topics = {t['topic_metadata']['name']: t['topic_metadata']['type'] for t in metadata['topics_with_message_count']}
    for topic, kind in ((config.input.lidar_topic, 'sensor_msgs/msg/PointCloud2'),
                        (config.input.imu_topic, 'sensor_msgs/msg/Imu')):
        if topics.get(topic) != kind:
            raise ValueError(f'rosbag 缺少 {topic} ({kind})')
    for relative in metadata.get('relative_file_paths', []):
        child = (path / relative).resolve(strict=True)
        if not child.is_relative_to(path) or not child.is_file():
            raise ValueError('rosbag 分片路径非法或缺失')
    if not metadata.get('relative_file_paths'):
        raise ValueError('rosbag 没有数据分片')
    return path
