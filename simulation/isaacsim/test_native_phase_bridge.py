"""Pure BEGIN/END acquisition contract checks; no ROS or simulator started."""
import math
import numpy as np
import pytest

from bridge import native_phase_contract, ray_source_times
from protocol import NATIVE_RAY_PHASE, RayAssembler, decode, ray_packets


CONTRACT = dict(schema=1, phase=NATIVE_RAY_PHASE, physics_dt_ns=2_000_000)


def scan(end_s=math.nextafter(.020, math.inf), begin_ns=18_000_000):
    metadata = dict(native_frame_time_ns=round(end_s*1e9), native_frame_time_s=end_s,
        phase=NATIVE_RAY_PHASE, native_frame_physics_step=10, capture_physics_step=9)
    parts = [decode(data) for data in ray_packets('native-epoch', 1, begin_ns, 0,
        [.2, 0., .2], [[1., 0., -.2]], **metadata)]
    return RayAssembler().add(parts[0], now=0.)


def test_begin_integer_headers_and_native_original_end_float_are_distinct():
    original = math.nextafter(.020, math.inf)
    value = scan(original)
    anchor = 1734000000000000001
    times = ray_source_times(value, anchor, CONTRACT)
    assert times['source_stamp_ns'] == anchor+18_000_000
    assert times['source_timestamp_s'] == (anchor+18_000_000)*1e-9
    assert times['raw_timestamp_s'].hex() == original.hex()
    assert times['native_frame_metadata']['native_frame_time_ns'] == 20_000_000
    # The existing PointCloud2 raw_timestamp FLOAT64 retains the original
    # sensor float; no point dtype or ROS acquisition-end field is introduced.
    point = np.zeros(1, dtype=[('raw_timestamp', '<f8')])
    point['raw_timestamp'] = times['raw_timestamp_s']
    assert np.frombuffer(point.tobytes(), dtype=point.dtype)['raw_timestamp'][0].item().hex() == original.hex()


def test_quad_session_requires_explicit_sealed_phase_and_physics_dt():
    session = dict(static_collision_prior_contract=dict(body_envelope_attestation_required=True))
    with pytest.raises(ValueError, match='phase_contract_required'):
        native_phase_contract(session)
    assert native_phase_contract(dict(session, perception_native_phase_contract=CONTRACT)) == CONTRACT
    for update in (dict(schema=True), dict(schema=2), dict(phase='postphysics'),
                   dict(physics_dt_ns=0), dict(physics_dt_ns=True), dict(physics_dt_ns=2_000_000.)):
        with pytest.raises(ValueError, match='invalid_isaac_native_ray_phase_contract'):
            native_phase_contract(dict(session, perception_native_phase_contract=dict(CONTRACT, **update)))


def test_native_end_may_retain_float_accumulation_within_sealed_step_tolerance():
    # Native time is retained, including substep float accumulation. Integer
    # BEGIN is not synthesized by subtracting dt from that float.
    original = .020000125
    times = ray_source_times(scan(original), 7, CONTRACT)
    assert times['source_stamp_ns'] == 18_000_007
    assert times['raw_timestamp_s'] == original
    for original in (.0198, .0202):
        ray_source_times(scan(original), 7, CONTRACT)
    for original in (.020200001, .019799999, .018, .016):
        with pytest.raises(ValueError, match='end_outside_sealed_physics_step'):
            ray_source_times(scan(original), 7, CONTRACT)


def test_metadata_cannot_be_missing_in_sealed_phase_or_used_without_contract():
    plain = dict(sim_time_ns=18_000_000)
    legacy = ray_source_times(plain, 7)
    assert legacy['source_stamp_ns'] == 18_000_007
    assert legacy['raw_timestamp_s'] == plain['sim_time_ns']*1e-9
    assert legacy['native_frame_metadata'] == {}
    assert native_phase_contract({}) is None
    with pytest.raises(ValueError, match='phase_metadata_required'):
        ray_source_times(plain, 7, CONTRACT)
    with pytest.raises(ValueError, match='phase_contract_required'):
        ray_source_times(scan(), 7)
