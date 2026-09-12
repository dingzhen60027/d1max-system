import math
import pytest
from d1max_localization.sensor_policy import finite_vector, valid_stamp, tracking_velocity, GyroBias, health_state

@pytest.mark.parametrize('value',[[0,0,float('nan')],[True,0,0],[0,0,4],[1,2],['1',0,0]])
def test_bad_sensor_values(value):
    with pytest.raises(ValueError):finite_vector(value,3)

def test_stamp_rejects_frozen_future_nan():
    assert valid_stamp(99.9,100,.3)
    assert not valid_stamp(99,100,.3)
    assert not valid_stamp(100.2,100,.3)
    assert not valid_stamp(float('nan'),100,.3)

def test_lever_arm_and_axis_rotation():
    assert tracking_velocity((1,0,0),math.pi/2,(.4,0,0))==pytest.approx((0,1,0))
    assert tracking_velocity((0,0,1),0,(.4,0,0))==pytest.approx((0,.4,1))

def test_stationary_calibration_does_not_zero_real_turns():
    bias=GyroBias(samples=3)
    assert bias.update(.01,None) is None
    assert bias.update(.01,0) is None
    assert bias.update(.01,.2) is None
    assert bias.count==0
    assert bias.update(.01,0) is None
    assert bias.update(.01,0) is None
    assert bias.update(.01,0)==0
    assert bias.update(.31,0)==pytest.approx(.3)
    assert bias.update(.015,.2)==pytest.approx(.005)

def test_readiness_is_not_process_status():
    assert health_state(False,True,True,True,0,True,True)=='waiting_sensors'
    assert health_state(True,False,False,False,None,True,True)=='calibrating'
    assert health_state(True,True,False,False,None,True,True)=='waiting_initial_pose'
    assert health_state(True,True,True,False,0,True,True)=='acquiring'
    assert health_state(True,True,True,True,1.1,True,True)=='degraded'
    assert health_state(True,True,True,True,0,True,True)=='tracking'


def test_full_3d_lever_arm_including_pitch_and_vertical_velocity():
    from d1max_localization.sensor_policy import tracking_twist
    v,w=tracking_twist((1.,2.,3.),(.2,.3,.4),0.,(.4,.1,-.04))
    assert v==pytest.approx((1-.012-.04,2+.16+.008,3+.02-.12))
    assert w==pytest.approx((.2,.3,.4))


def test_three_axis_bias_requires_stationary_all_axes():
    from d1max_localization.sensor_policy import GyroBias3
    bias=GyroBias3(samples=2)
    assert bias.update((.01,.02,.01),None) is None
    assert bias.update((.01,.02,.01),0.) is None
    assert bias.update((.3,0.,0.),0.) is None
    assert bias.count==0
    assert bias.update((.01,.02,.01),0.) is None
    assert bias.update((.01,.02,.01),0.)==(0.,0.,0.)
    assert bias.update((.21,-.08,.31),.2)==pytest.approx((.2,-.1,.3))
