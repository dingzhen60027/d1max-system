"""Actual preview callbacks exercised in memory; never initializes ROS."""
from array import array
import json
import time

import numpy as np
import pytest

from d1max_pct_scan.preview_ray_exclusion import from_session
from d1max_pct_scan.ray_projection import PreviewExcludedScan, RAY_DTYPE, session_settings
from test_perception_ray_projector_boundary import node, raw_cloud, ready
from test_preview_ray_exclusion import mixed_points, preview_session
from test_ray_projection import EPOCH, raw_points, session_config


def install(node, mode='preview_drop_rays'):
    exclusion = from_session(preview_session(mode), 'body')
    node.p['preview_exclusion'] = exclusion
    node.core.preview_exclusion = exclusion
    return exclusion


def payload(points):
    message = raw_cloud()
    message.width = len(points)
    message.row_step = len(points)*64
    message.data = array('B', points.tobytes())
    return message


def reported(node):
    node.publish_status(node.current_context(), node.now_ns(), time.monotonic())
    return json.loads(node.status_pub.messages[-1].data)


def test_node_constructor_installs_only_explicit_validated_filter(node):
    # The fixture supplies the in-memory ROS modules; constructing another node
    # checks the real settings-to-core boundary, not only later field assignment.
    from d1max_pct_scan.perception_ray_projector import create_node
    _, localization = session_config()
    settings = session_settings(preview_session(), localization)
    other = create_node(settings)
    try:
        assert other.core.preview_exclusion is settings['preview_exclusion']
        assert other.core.preview_exclusion.mode == 'preview_drop_rays'
        assert other.exclusion_counts == dict(matched=[0, 0], dropped=[0, 0])
    finally:
        other.destroy_node()


def test_all_filtered_callback_reports_counts_without_publishing_or_renewing_source(node):
    exclusion = install(node)
    ready(node)
    points = raw_points(n=5)
    points['x'] = .5
    node.on_rays(payload(points))
    node.tick()
    with pytest.raises(PreviewExcludedScan):
        node.inflight[0].result(timeout=2.)
    node.tick()
    assert not node.publisher.messages
    assert node.last_published == [0, 0]
    assert node.core.last_input == [-1, -1]
    assert node.core.sequence == 0
    assert node.exclusion_counts == dict(matched=[5, 0], dropped=[5, 0])
    status = reported(node)
    assert status['source_fresh'] == [False, False]
    assert status['valid'] is False
    detail = status['preview_ray_exclusion']
    assert detail['config_sha256'] == exclusion.digest
    assert detail['calibration_verified'] is False
    assert detail['warning'] == 'experimental_endpoint_exclusion_not_self_classification'


@pytest.mark.parametrize('mode,remaining,matched,dropped', [
    ('audit', 5, 3, 0), ('preview_drop_rays', 2, 3, 3),
])
def test_callback_preserves_full_scan_times_and_reports_distinct_audit_drop_counts(
        node, mode, remaining, matched, dropped):
    install(node, mode)
    ready(node)
    points = mixed_points()
    node.on_rays(payload(points))
    node.tick()
    node.inflight[0].result(timeout=2.)
    node.tick()
    assert len(node.publisher.messages) == 1
    message = node.publisher.messages[0]
    assert message.rays.width == remaining
    assert message.rays.header.stamp.sec*1000000000+message.rays.header.stamp.nanosec == EPOCH
    assert message.acquisition_end.sec*1000000000+message.acquisition_end.nanosec == EPOCH+100000000
    published = np.frombuffer(message.rays.data, dtype=RAY_DTYPE)
    expected = points if mode == 'audit' else points[[1, 3]]
    for name in ('offset_time', 'source_index', 'timestamp', 'source_timestamp', 'raw_timestamp'):
        assert published[name].tobytes() == expected[name].tobytes()
    assert node.exclusion_counts == dict(matched=[matched, 0], dropped=[dropped, 0])
    assert node.last_published == [EPOCH, 0]


def test_worker_completed_with_old_filter_cannot_publish_or_update_lease(node):
    install(node)
    ready(node)
    node.on_rays(payload(mixed_points()))
    node.tick()
    node.inflight[0].result(timeout=2.)
    node.core.preview_exclusion = from_session(preview_session(pad=.01), 'body')
    node.tick()
    assert not node.publisher.messages
    assert node.last_published == [0, 0]
    assert node.core.sequence == 0
    assert node.exclusion_counts == dict(matched=[0, 0], dropped=[0, 0])
