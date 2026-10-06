import pytest
from protocol import decode, dynamic_packet


def test_dynamic_measurements_keep_original_source_and_separate_type():
    sample = {'walker': {'present': True, 'position': [1, 2, .8],
                         'linear_velocity': [.4, 0, 0]}}
    packet = decode(dynamic_packet('source-epoch', 7, 123456789, 'a' * 64, sample))
    assert packet['sim_time_ns'] == 123456789
    assert packet['type'] == 'dynamic'
    assert packet['samples']['walker']['position'] == [1., 2., .8]
    assert 'xyz' not in packet


@pytest.mark.parametrize('sample', [{'walker': {'present': False}},
    {'walker': {'present': True, 'position': [0, 0, float('nan')], 'linear_velocity': [0, 0, 0]}}])
def test_missing_actor_or_invalid_measurement_rejected(sample):
    with pytest.raises(ValueError):
        dynamic_packet('source-epoch', 1, 1, 'a' * 64, sample)


def test_empty_registry_heartbeat_is_explicit_and_cannot_omit_registered_actor():
    from dynamic_collision import registry_digest, oracle_payload
    digest = registry_digest([])
    packet = decode(dynamic_packet('empty-epoch', 1, 20_000_000, digest, {}))
    proof = oracle_payload([], packet['samples'], session_id='s', epoch=1, seed_id='isaac-empty',
        context_sequence=1, sequence=1, source_stamp_ns=100_000_000,
        reachable_horizon_ns=6_000_000_000)
    assert proof['complete'] and proof['actors'] == [] and proof['registry_sha256'] == digest
    with pytest.raises(ValueError, match='missing_or_unregistered'):
        oracle_payload([{'id':'registered'}], {}, session_id='s', epoch=1, seed_id='isaac-empty',
            context_sequence=1, sequence=1, source_stamp_ns=100_000_000)
