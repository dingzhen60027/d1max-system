import pytest

from d1max_pct_scan.single_floor_session import scan_map_cadence


def test_template_is_the_only_cadence_owner():
    parameters={'grid_map.integration_rate_hz':8.,'grid_map.visualization_rate_hz':2.}
    assert scan_map_cadence(parameters)==parameters
    assert parameters=={'grid_map.integration_rate_hz':8.,'grid_map.visualization_rate_hz':2.}


@pytest.mark.parametrize('name,value',[
    ('integration_rate_hz',0.),('integration_rate_hz',20.1),
    ('integration_rate_hz',float('nan')),('integration_rate_hz',float('inf')),
    ('visualization_rate_hz',0.),('visualization_rate_hz',5.1)])
def test_invalid_cadence_cannot_silently_fall_back(name,value):
    parameters={'grid_map.integration_rate_hz':10.,'grid_map.visualization_rate_hz':3.}
    parameters['grid_map.'+name]=value
    with pytest.raises(ValueError,match='invalid_scan_'+name):
        scan_map_cadence(parameters)


def test_missing_cadence_is_not_a_hidden_runtime_default():
    with pytest.raises(KeyError):
        scan_map_cadence({'grid_map.visualization_rate_hz':3.})
