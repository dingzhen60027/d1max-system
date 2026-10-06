import pytest
from bridge import dynamic_measurement_payload
from dynamic_collision import actor_registry, registry_digest


def registry():
    return actor_registry({'dynamic_actors': [{'id': 'walker', 'enabled': True,
        'collision_shapes': [{'id': 'body', 'type': 'box', 'collision': True,
                              'center': [0, 0, 0], 'size': [.3, .3, 1.7]}],
        'max_linear_speed_mps': 1., 'max_linear_acceleration_mps2': 2.}]})


def test_bridge_preserves_integer_source_and_original_context():
    reg = registry()
    anchor = 1734000000000000001
    packet = dict(sequence=2, sim_time_ns=10000001, registry_sha256=registry_digest(reg),
                  samples={'walker': dict(present=True, position=[1, 2, 0], linear_velocity=[.5, 0, 0])})
    context = dict(session_id='test', epoch=1, seed_id='isaac-original', sequence=7)
    value = dynamic_measurement_payload(packet, reg, context, anchor)
    assert value['source_stamp_ns'] == anchor+10000001
    assert value['context_sequence'] == 7
    assert value['seed_id'] == 'isaac-original'
    assert value['actors'][0]['regions'][0]['state'] == 2
    assert value['valid_until_ns'] == value['source_stamp_ns']+300000000


def test_foreign_dynamic_registry_is_rejected():
    with pytest.raises(ValueError, match='registry_changed'):
        dynamic_measurement_payload({'registry_sha256': 'f'*64}, registry(), {}, 1)
