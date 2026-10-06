"""Runtime attestation failure must be detected before usable body publication."""
import numpy as np
import pytest
from bridge import quaternion_matrix, validate_static_prior_state


def packet():
    return dict(sim_time_ns=200000000,pose=[0.,0.,.35,0.,0.,0.,1.],static_prior_geometry_checked_sim_time_ns=100000000,
        static_prior_geometry_valid=True,static_prior_geometry_sha256='a'*64)


CONTRACT=dict(static_prior_geometry_sha256='a'*64,max_body_tilt_rad=.01,
    expected_body_world_z=.35,max_body_height_error_m=.005)


def test_attestation_has_bounded_actual_source_age_and_exact_geometry():
    p=packet();validate_static_prior_state(p,CONTRACT,np.eye(3))
    for updates in ({'static_prior_geometry_valid':False},
            {'static_prior_geometry_sha256':'b'*64},
            {'static_prior_geometry_checked_sim_time_ns':99999999},
            {'static_prior_geometry_checked_sim_time_ns':200000001}):
        with pytest.raises(ValueError,match='geometry_revoked'):
            validate_static_prior_state(dict(p,**updates),CONTRACT,np.eye(3))


def test_yaw_is_allowed_but_uncertified_tilt_revokes_prior():
    validate_static_prior_state(packet(),CONTRACT,quaternion_matrix([0.,0.,.5,3**.5/2]))
    with pytest.raises(ValueError,match='body_tilt'):
        validate_static_prior_state(packet(),CONTRACT,quaternion_matrix([.02,0.,0.,(1-.02**2)**.5]))


def test_original_no_prior_fixture_does_not_require_truth_attestation():
    validate_static_prior_state({},None,np.eye(3))


def test_flat_ground_prior_requires_actual_body_height_and_rejects_unsupported_low_solids():
    from nav_prepare import validate_prior_body_domain
    p=packet();p['pose'][2]=.37
    with pytest.raises(ValueError,match='body_height'):
        validate_static_prior_state(p,CONTRACT,np.eye(3))
    for top in (.03,.04,.07,.10,.12):
        spec=dict(floor=dict(z=0.),static_boxes=[dict(name='low',center=[0.,0.,top/2],size=[1.,1.,top])])
        with pytest.raises(ValueError,match='unsupported_low_collider'):
            validate_prior_body_domain(spec,.05)
    validate_prior_body_domain(dict(floor=dict(z=0.),static_boxes=[
        dict(name='supported',center=[0.,0.,.12],size=[1.,1.,.24])]),.05)
