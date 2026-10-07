"""Safety-relevant cross-interpreter wire contract checks; no simulator needed."""
import math
import pytest

from protocol import (RayAssembler, decode, ray_packets, state_packet, command_packet, imu_packet,
                      NATIVE_RAY_PHASE, native_ray_metadata, encode)


def test_native_actor_identity_reassembly_preserves_ray_and_registry_bytes():
    import numpy as np
    xyz=np.arange(7500,dtype=np.float32).reshape(2500,3)*.001
    rings=np.arange(2500,dtype=np.uint16)%33;actors=np.arange(2500,dtype=np.uint16)%5
    kwargs=dict(actor_registry_sha256='a'*64)
    binary=list(ray_packets('epoch-a',7,18_000_000,0,[.2,0.,.2],xyz,rings=rings,actor_ids=actors,**kwargs))
    scalar=list(ray_packets('epoch-a',7,18_000_000,0,[.2,0.,.2],xyz.tolist(),rings=rings.tolist(),actor_ids=actors.tolist(),**kwargs))
    assert binary==scalar
    assembler=RayAssembler();result=None
    for i in (2,0,1):result=assembler.add(decode(binary[i]),now=.01*i)
    assert result['sim_time_ns']==18_000_000 and result['actor_registry_sha256']=='a'*64
    assert result['actor_ids']==actors.tolist() and result['rings']==rings.tolist()
    np.testing.assert_array_equal(np.asarray(result['xyz'],dtype=np.float32),xyz)


@pytest.mark.parametrize('change',['registry','conflicting_actor','missing_actor'])
def test_native_actor_identity_cannot_change_within_original_scan(change):
    parts=[decode(data) for data in ray_packets('epoch-a',1,10,0,[0.,0.,0.],
        [[1.,0.,0.]]*2500,actor_ids=[1]*2500,actor_registry_sha256='a'*64)]
    assembler=RayAssembler();assembler.add(parts[0],now=0.)
    altered=dict(parts[1])
    if change=='registry':altered['actor_registry_sha256']='b'*64
    if change=='missing_actor':altered.pop('actor_ids')
    if change=='conflicting_actor':
        altered=dict(parts[0]);altered['actor_ids']=[0]*len(altered['xyz'])
    with pytest.raises(ValueError):assembler.add(altered,now=.01)
    assert not assembler.pending


def packets(count=2500, sequence=1):
    return [decode(data) for data in ray_packets('epoch-a', sequence, 100000000,
        0, [.42, 0., .15], [[i*.01, 1., -.15] for i in range(count)])]


def test_reordered_complete_scan_preserves_original_source_time_and_origin():
    assembler = RayAssembler()
    parts = packets()
    assert assembler.add(parts[2], now=0.) is None
    assert assembler.add(parts[0], now=.01) is None
    result = assembler.add(parts[1], now=.02)
    assert result['sim_time_ns'] == 100000000
    assert result['origin'] == (.42, 0., .15)
    assert len(result['xyz']) == 2500
    assert result['xyz'][0] == pytest.approx((0., 1., -.15),abs=1e-6)
    assert result['xyz'][-1] == pytest.approx((24.99, 1., -.15),abs=2e-6)
    assert assembler.add(parts[0], now=.03) is None


def test_missing_scan_chunk_expires_without_inventing_free_space():
    assembler = RayAssembler(timeout_s=.15)
    old = packets()
    assert assembler.add(old[0], now=0.) is None
    new = packets(sequence=2)
    assert assembler.add(new[0], now=.2) is None
    assert assembler.dropped == 1
    assert all(key[2] != 1 for key in assembler.pending)


def test_chunk_origin_or_source_clock_cannot_change_within_scan():
    assembler = RayAssembler()
    parts = packets()
    assert assembler.add(parts[0], now=0.) is None
    parts[1]['sim_time_ns'] += 1
    with pytest.raises(ValueError, match='inconsistent_scan_chunks'):
        assembler.add(parts[1], now=.01)
    assert not assembler.pending


def test_invalid_measurements_and_command_lifetime_fail_before_wire_send():
    with pytest.raises(ValueError, match='nonfinite_vector'):
        state_packet('epoch-a', 1, 10, [0, 0, 0, 0, 0, 0, 1], [math.nan, 0, 0], [0, 0, 0])
    with pytest.raises(ValueError, match='invalid_command_lifetime'):
        command_packet('epoch-a', 1, 10, .3, 0, valid_for_s=.5)


def test_native_imu_keeps_its_independent_measurement_time_and_si_vectors():
    measured=decode(imu_packet('epoch-a',3,12500000,[0,0,0,1],[0,0,.5],[0,0,9.81]))
    body=decode(state_packet('epoch-a',1,16666667,[0,0,.35,0,0,0,1],[0,0,0],[0,0,.5]))
    assert measured['sim_time_ns']!=body['sim_time_ns']
    assert measured['sim_time_ns']==12500000
    assert measured['sensor_frame']=='body'
    assert measured['linear_acceleration']==[0.,0.,9.81]
    assert measured['angular_velocity']==[0.,0.,.5]


def test_static_geometry_attestation_is_optional_and_never_changes_body_time():
    arguments=('epoch-a',1,16666667,[0,0,.35,0,0,0,1],[0,0,0],[0,0,0])
    plain=decode(state_packet(*arguments))
    assert 'static_prior_geometry_valid' not in plain
    metadata=dict(static_prior_geometry_sha256='a'*64,static_prior_geometry_valid=True,
        static_prior_geometry_checked_sim_time_ns=16666667,static_prior_geometry_fault='')
    state=decode(state_packet(*arguments,metadata=metadata))
    assert state['sim_time_ns']==16666667
    assert state['static_prior_geometry_checked_sim_time_ns']==16666667
    assert state['static_prior_geometry_valid'] is True
    metadata.update(static_prior_geometry_sha256='',static_prior_geometry_valid=False,
                    static_prior_geometry_fault='unlisted_static_collider:/World/NewObstacle')
    assert decode(state_packet(*arguments,metadata=metadata))['static_prior_geometry_valid'] is False
    for invalid in ({**metadata,'sim_time_ns':99999999},
                    {**metadata,'static_prior_geometry_checked_sim_time_ns':16666668},
                    {**metadata,'static_prior_geometry_valid':True},
                    {**metadata,'static_prior_geometry_valid':1}):
        with pytest.raises(ValueError,match='invalid_geometry_metadata'):
            state_packet(*arguments,metadata=invalid)


def test_native_vertical_ring_labels_follow_filtered_points_across_chunks():
    xyz=[[i*.01,1.,.2] for i in range(2500)]
    rings=[i%33 for i in range(2500)]
    parts=[decode(packet) for packet in ray_packets('epoch-a',1,10000000,1,[-.42,0,.15],xyz,rings=rings)]
    assembler=RayAssembler()
    result=None
    for index,part in enumerate(reversed(parts)):
        result=assembler.add(part,now=index*.01)
    assert result['rings']==rings
    for actual,expected in zip(result['xyz'],xyz):
        assert actual==pytest.approx(expected,abs=1e-6)
    with pytest.raises(ValueError,match='invalid_native_ring_indices'):
        list(ray_packets('epoch-a',2,20000000,1,[-.42,0,.15],xyz,rings=rings[:-1]))


def test_binary_clouds_fit_capped_loopback_buffer_and_reject_partial_payload():
    xyz=[[i*.0001,1.,-.15] for i in range(9600)]
    rings=[i%33 for i in range(9600)]
    burst=[packet for sensor in (0,1) for packet in ray_packets('epoch-a',1,10,sensor,
        [.42 if sensor==0 else -.42,0,.15],xyz,rings=rings)]
    assert sum(map(len,burst)) < 300000  # dual real-size burst, XYZ f32/ring u16
    with pytest.raises(ValueError,match='invalid_binary_ray_payload'):
        decode(burst[0][:-1])


def test_native_ndarray_fast_path_is_wire_identical_and_retains_invalid_data_checks():
    import numpy as np
    xyz=np.arange(7500,dtype=np.float32).reshape(2500,3)*.001
    rings=np.arange(2500,dtype=np.uint16)%33
    scalar=list(ray_packets('epoch-a',1,10,0,[.42,0,.15],xyz.tolist(),rings=rings.tolist()))
    native=list(ray_packets('epoch-a',1,10,0,[.42,0,.15],xyz,rings=rings))
    assert scalar==native
    xyz[5,1]=np.nan
    with pytest.raises(ValueError,match='nonfinite_vector'):
        list(ray_packets('epoch-a',1,10,0,[.42,0,.15],xyz,rings=rings))
    xyz[5,1]=0.
    rings_invalid=rings.astype(np.int32);rings_invalid[4]=-1
    with pytest.raises(ValueError,match='invalid_native_ring_indices'):
        list(ray_packets('epoch-a',1,10,0,[.42,0,.15],xyz,rings=rings_invalid))


def phase_packets(original_native_time_s=None):
    original = math.nextafter(.020, math.inf) if original_native_time_s is None else original_native_time_s
    metadata = dict(native_frame_time_ns=round(original*1e9), native_frame_time_s=original,
        phase=NATIVE_RAY_PHASE, native_frame_physics_step=10, capture_physics_step=9)
    return [decode(data) for data in ray_packets('epoch-a', 1, 18_000_000, 0,
        [.2, 0., .2], [[i*.01, 1., -.2] for i in range(2500)], **metadata)]


def test_native_end_metadata_reassembles_exact_float_without_changing_begin():
    parts = phase_packets()
    assembler = RayAssembler()
    result = None
    for index, part in enumerate(reversed(parts)):
        result = assembler.add(part, now=index*.01)
    assert result['sim_time_ns'] == 18_000_000
    assert result['native_frame_time_ns'] == 20_000_000
    assert result['native_frame_time_s'].hex() == math.nextafter(.020, math.inf).hex()
    assert result['phase'] == NATIVE_RAY_PHASE
    assert result['capture_physics_step'] == 9 and result['native_frame_physics_step'] == 10
    assert 'phase' not in packets(count=1)[0]  # legacy wire stays optional


@pytest.mark.parametrize('update', [
    dict(native_frame_time_s=None), dict(native_frame_time_s=math.nan),
    dict(native_frame_time_s=1e300), dict(native_frame_time_ns=20_000_001),
    dict(native_frame_time_ns=True), dict(phase='postphysics'),
    dict(native_frame_physics_step=9), dict(capture_physics_step=True),
    dict(capture_physics_step=None)])
def test_incomplete_or_invalid_native_phase_metadata_is_rejected(update):
    packet = dict(phase_packets()[0], **update)
    with pytest.raises(ValueError, match='invalid_native_ray_phase_metadata'):
        native_ray_metadata(packet)
    # Test hostile decoded metadata as well as the sender's kwargs path.
    if not any(isinstance(v,float) and not math.isfinite(v) for v in update.values()):
        with pytest.raises(ValueError, match='invalid_native_ray_phase_metadata'):
            decode(encode(packet))


def test_native_phase_metadata_must_match_on_every_chunk():
    for mutation in ('native_float', 'native_ns', 'physics_step', 'missing'):
        parts = phase_packets()
        assembler = RayAssembler()
        assert assembler.add(parts[0], now=0.) is None
        if mutation == 'native_float':
            parts[1]['native_frame_time_s'] = math.nextafter(parts[1]['native_frame_time_s'], math.inf)
        elif mutation == 'native_ns':
            parts[1].update(native_frame_time_s=.020000001, native_frame_time_ns=20_000_001)
        elif mutation == 'physics_step':
            parts[1].update(capture_physics_step=10, native_frame_physics_step=11)
        else:
            parts[1].pop('phase')
        with pytest.raises(ValueError, match='inconsistent_scan_chunks|invalid_native_ray_phase_metadata'):
            assembler.add(parts[1], now=.01)
        assert not assembler.pending
