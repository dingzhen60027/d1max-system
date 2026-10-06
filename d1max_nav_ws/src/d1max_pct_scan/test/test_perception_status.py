import pytest
from d1max_pct_scan.perception_status import integrated_ray_stamp


CONTEXT = dict(session_id='session', epoch=1, seed_id='seed', sequence=2, barrier_ns=99000000000)


def receipt(**changes):
    return dict(CONTEXT, schema=1, received_at_unix=100., source_stamp_ns=99900000000,
                valid=True, reason='integrated', **changes)


def test_integrated_receipt_uses_measurement_not_arrival():
    assert integrated_ray_stamp(receipt(), CONTEXT, now=100., timeout=.5, barrier=99.) == pytest.approx(99.9)


def test_simulated_ros_clock_receipt_keeps_wall_time_only_as_diagnostics():
    value=receipt()
    value['callback_wall_time']=1791080000.
    assert integrated_ray_stamp(value,CONTEXT,now=100.,timeout=.5,barrier=99.)==pytest.approx(99.9)
    value['received_at_unix']=value['callback_wall_time']
    with pytest.raises(ValueError,match='ray_receipt_stale'):
        integrated_ray_stamp(value,CONTEXT,now=100.,timeout=.5,barrier=99.)


@pytest.mark.parametrize('key,value', [('session_id','old'), ('epoch',2), ('epoch',True),
    ('seed_id','new'), ('sequence',1), ('barrier_ns',0), ('valid',False),
    ('valid',1), ('schema',True), ('source_stamp_ns',99400000000),
    ('source_stamp_ns',99900000000.), ('received_at_unix',99.),
    ('received_at_unix',float('nan')), ('received_at_unix',float('inf'))])
def test_invalid_or_stale_receipts_rejected(key, value):
    v=receipt();v[key]=value
    with pytest.raises(ValueError):
        integrated_ray_stamp(v, CONTEXT, now=100., timeout=.5, barrier=99.)


def test_missing_context_and_later_reset_barrier_rejected():
    for context, barrier in ((None,99.),(CONTEXT,100.)):
        with pytest.raises(ValueError):
            integrated_ray_stamp(receipt(),context,now=100.,timeout=.5,barrier=barrier)
