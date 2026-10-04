import json
import pytest
from d1max_pct_scan.isolated_zenoh import environment, validate_environment


def test_production_environment_not_inherited_or_mutated(tmp_path):
    original=dict(ROS_DOMAIN_ID='24',ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.168.100:7447"]')
    env=environment(tmp_path,19384,original)
    assert original['ROS_DOMAIN_ID']=='24'
    assert 'ZENOH_CONFIG_OVERRIDE' not in env
    assert validate_environment(env)=='tcp/127.0.0.1:19384'


@pytest.mark.parametrize('key,value',[('ROS_DOMAIN_ID','24'),('RMW_IMPLEMENTATION','rmw_fastrtps_cpp'),
    ('ZENOH_CONFIG_OVERRIDE','{}'),('D1MAX_NAV_TRANSPORT','live'),('D1MAX_NAV_ISOLATION_TOKEN','')])
def test_reject_unisolated_env(tmp_path,key,value):
    env=environment(tmp_path,19384,{})
    env[key]=value
    with pytest.raises(ValueError): validate_environment(env)


@pytest.mark.parametrize('kind',['uplink','discovery','listen','different_port'])
def test_configs_are_checked_not_just_boolean(tmp_path,kind):
    env=environment(tmp_path,19384,{})
    path=tmp_path/'router.json5'; config=json.loads(path.read_text())
    if kind=='uplink': config['connect']['endpoints']=['tcp/192.168.168.100:7447']
    elif kind=='discovery': config['scouting']['gossip']['enabled']=True
    elif kind=='listen': config['listen']['endpoints']=['tcp/0.0.0.0:19384']
    else: config['listen']['endpoints']=['tcp/127.0.0.1:19385']
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError): validate_environment(env)
