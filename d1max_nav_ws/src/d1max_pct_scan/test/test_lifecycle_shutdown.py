from copy import deepcopy
import pytest
from d1max_pct_scan.lifecycle_shutdown import drained_status


def status():
    return dict(schema=2, session_id='s', stamp=10.2, lifecycle_active=False,
                lifecycle_draining=False, lifecycle_quarantined=False,
                physical_stop_confirmed=False)


def check(value):
    return drained_status(value, session_id='s', requested_at=10., now=10.3)


def test_software_retirement_does_not_require_or_invent_physical_stop():
    value = status()
    assert check(value)
    assert value['physical_stop_confirmed'] is False


@pytest.mark.parametrize('key,value', [('schema',1), ('session_id','old'),
    ('stamp',9.9), ('stamp',10.9), ('stamp',float('nan')),
    ('lifecycle_active',True), ('lifecycle_draining',True),
    ('lifecycle_quarantined',True)])
def test_stale_foreign_active_or_quarantined_is_not_drained(key, value):
    data = deepcopy(status())
    data[key] = value
    assert not check(data)
