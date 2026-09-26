"""Pure validation and HTTP boundary for RViz localization seeds, never motion."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


class PoseRejected(ValueError):
    """A pose must be redrawn after the stated precondition is satisfied."""


class WebUnavailable(RuntimeError):
    pass


class SubmissionUnknown(WebUnavailable):
    """A POST may have reached the server; automatic retry is forbidden."""


def finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PoseRejected(f'{label} 必须是有限数值')
    if not math.isfinite(value):
        raise PoseRejected(f'{label} 必须是有限数值')
    return float(value)


def validate_pose_age(stamp, now, max_age=3.0):
    stamp = finite_number(stamp, '初值时间戳')
    now = finite_number(now, '当前时间')
    max_age = finite_number(max_age, '初值最大数据龄')
    if max_age <= 0 or stamp <= 0 or not -0.2 <= now - stamp <= max_age:
        raise PoseRejected('初值时间戳过期或超前，请重新画箭头；旧初值不会排队重发')


def pose_payload(*, frame, expected_frame, stamp, now, position, quaternion,
                 body_z=0.0, max_age=3.0):
    """RViz arrow is a body-origin XY/yaw seed; configured body_z supplies Z.

    The clicked plane's Z is intentionally not treated as the robot body height.
    No sensor extrinsics, point clouds or TF are changed here.
    """
    if not expected_frame or frame != expected_frame:
        raise PoseRejected(f'初值坐标系必须为 {expected_frame}，实际为 {frame!r}')
    validate_pose_age(stamp, now, max_age)
    if len(position) != 3 or len(quaternion) != 4:
        raise PoseRejected('初值向量长度错误')
    x, y, _ = (finite_number(v, '位置') for v in position)
    qx, qy, qz, qw = (finite_number(v, '四元数') for v in quaternion)
    z = finite_number(body_z, '配置的机身高度')
    if abs(x) > 100000 or abs(y) > 100000 or abs(z) > 100:
        raise PoseRejected('初值超出定位 API 允许范围')
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if abs(norm - 1.0) > 0.01:
        raise PoseRejected('初值四元数未归一化或为零')
    qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
    if abs(roll) > 0.01 or abs(pitch) > 0.01:
        raise PoseRejected('仅接受平面方向箭头，不能通过初值修改机身俯仰或横滚')
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return {'x': x, 'y': y, 'z': z, 'yaw': yaw, 'reference': 'body'}


def validate_context(overview, *, expected_version_id, expected_session_id='',
                     map_frame='d1max_loc_map', now=None, max_health_age=2.0,
                     require_ready=True):
    """Fail closed on map/session mismatch, stale health or seed gate closure."""
    if not expected_version_id:
        raise PoseRejected('未锁定 2D 地图版本，拒绝提交初值')
    if not isinstance(overview, dict) or overview.get('phase') != 'running':
        raise PoseRejected('定位服务尚未运行；连接机器人并启动对应地图定位后再画箭头')
    session = overview.get('id')
    if not isinstance(session, str) or not session:
        raise PoseRejected('定位会话标识缺失')
    if expected_session_id and session != expected_session_id:
        raise PoseRejected('定位会话已变化，请重新启动本次 RViz 定位测试')
    if overview.get('version_id') != expected_version_id:
        raise PoseRejected('RViz 的 2D 地图与定位 PCD 版本不一致，拒绝提交初值')
    health = overview.get('health')
    if not isinstance(health, dict) or not health:
        raise PoseRejected('等待新鲜定位状态和传感器数据，请就绪后重新画箭头')
    if health.get('session_id') != session or health.get('map_version_id') != expected_version_id:
        raise PoseRejected('定位健康状态不属于当前地图和会话')
    age = finite_number(time.time() if now is None else now, '当前墙钟') - finite_number(
        health.get('wall_time'), '定位状态时间戳')
    if not 0 <= age < max_health_age:
        raise PoseRejected('定位健康状态已过期，请等待数据恢复后重新画箭头')
    if not isinstance(health.get('frames'), dict) or health['frames'].get('map') != map_frame:
        raise PoseRejected('定位地图坐标系与 RViz 固定坐标系不一致')
    if require_ready:
        if health.get('initial_pose_ready') is not True:
            raise PoseRejected('等待 LIO 初始化和传感器就绪；初值不会自动延后执行')
        if health.get('head_direction') != 1:
            raise PoseRejected('初值需要机头前向状态；程序不会自动切换头尾或姿态')
    return session, health


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise HTTPError(req.full_url, code, '拒绝本机 API 重定向', headers, fp)


class LocalWebClient:
    """Only loopback HTTP, no proxy, redirect, retry or lifecycle operations."""
    def __init__(self, base_url='http://127.0.0.1:8766', timeout=1.0):
        parts = urlsplit(base_url)
        if (parts.scheme != 'http' or parts.hostname not in {'127.0.0.1', 'localhost', '::1'}
                or parts.path not in {'', '/'} or parts.query or parts.fragment
                or parts.username or parts.password):
            raise ValueError('web_url 仅允许本机 HTTP 根地址')
        if not 0.1 <= timeout <= 3.0:
            raise ValueError('request_timeout_sec 必须位于 0.1–3.0 秒')
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.opener = build_opener(ProxyHandler({}), _NoRedirect())

    def request(self, path, payload=None, headers=None):
        if path not in {'/api/localization/overview', '/api/localization/initial-pose'}:
            raise ValueError('仅允许定位状态查询和初值提交')
        if (path.endswith('/overview')) != (payload is None):
            raise ValueError('请求方法与固定 API 不匹配')
        request = Request(self.base_url + path,
                          data=None if payload is None else json.dumps(payload, allow_nan=False).encode(),
                          headers={'Content-Type': 'application/json', **(headers or {})})
        try:
            with self.opener.open(request, timeout=self.timeout) as response:
                data = response.read(2*1024*1024 + 1)
                if len(data) > 2*1024*1024:
                    raise ValueError('本机 API 响应过大')
                result = json.loads(data)
                if not isinstance(result, dict):
                    raise ValueError('本机 API 响应不是对象')
                return result
        except HTTPError as error:
            # A 4xx rejection is definite; server/network failures after POST are not.
            try:
                detail = str(json.loads(error.read(4096)).get('detail', error.reason))[:350]
            except (ValueError, AttributeError):
                detail = str(error.reason)[:350]
            if 400 <= error.code < 500:
                raise PoseRejected(f'Web 拒绝初值：{detail}') from None
            problem = f'本机 Web HTTP {error.code}：{detail}'
            if payload is not None:
                raise SubmissionUnknown(problem + '；提交结果未知，不会自动重试') from None
            raise WebUnavailable(problem) from None
        except (OSError, ValueError) as error:
            problem = f'本机 Web 不可用或响应无效：{str(error)[:250]}'
            if payload is not None:
                raise SubmissionUnknown(problem + '；提交结果未知，不会自动重试') from None
            raise WebUnavailable(problem) from None


@dataclass(frozen=True)
class SeedReceipt:
    request_id: str
    session_id: str
    version_id: str


def submit_checked_pose(client, payload, *, expected_version_id,
                        expected_session_id='', map_frame='d1max_loc_map',
                        max_health_age=2.0, wall_now=time.time,
                        check_pose_fresh=lambda: None):
    """One GET then at most one POST, with atomic server-side context guards."""
    check_pose_fresh()
    overview = client.request('/api/localization/overview')
    session, _ = validate_context(
        overview, expected_version_id=expected_version_id,
        expected_session_id=expected_session_id, map_frame=map_frame,
        now=wall_now(), max_health_age=max_health_age)
    check_pose_fresh()
    response = client.request('/api/localization/initial-pose', payload, headers={
        'X-D1max-Session-Id': session, 'X-D1max-Map-Version': expected_version_id})
    if (response.get('status') != 'queued' or response.get('accepted') is not False
            or not isinstance(response.get('request_id'), str) or not response['request_id']):
        raise SubmissionUnknown('Web 初值响应不符合协议；结果未知，不会自动重试')
    return SeedReceipt(response['request_id'], session, expected_version_id)
