"""Passive startup-contract checks against the real formal session generator.

The recorded source map, PCT package and production parameter templates are
used here.  No ROS graph, SDK session or navigation child may be started.
"""
from copy import deepcopy
import json
import os
from pathlib import Path

import pytest
import yaml

from d1max_pct_planner import paths
from d1max_pct_scan import navigation_contract
from d1max_pct_scan.single_floor_session import prepare


def recorded_map():
    explicit = os.environ.get('D1MAX_TEST_MAP_DIRECTORY')
    if explicit:
        return Path(explicit).resolve()
    try:
        return paths.default_map_directory()
    except (ValueError, OSError):
        return None


@pytest.fixture(scope='module')
def formal_coordinates(tmp_path_factory):
    """Generate both actual purposes once; each test receives an isolated copy."""
    map_directory = recorded_map()
    if map_directory is None or not map_directory.is_dir():
        pytest.skip('requires the selected release map package')
    directory = tmp_path_factory.mktemp('mainline-coordinate-contract')
    patch = pytest.MonkeyPatch()

    def forbidden(*args, **kwargs):
        raise AssertionError('coordinate preparation must not start any process or ROS graph')

    patch.setattr('subprocess.Popen', forbidden)
    patch.setattr('subprocess.run', forbidden)
    # Module fixtures execute before the function-scoped no-ROS test fixture.
    # Keep preparation passive even if that execution order changes.
    import rclpy
    patch.setattr(rclpy, 'init', forbidden)
    result = {}
    try:
        for purpose in ('planning_only', 'execution'):
            target = directory / purpose
            options = dict(map_directory=map_directory,
                           session_id='coordinates-' + purpose)
            if purpose == 'planning_only':
                options.update(purpose=purpose, transport_mode='live',
                               expected_sdk_session='coordinate-read-only-existing-session')
            session = prepare(target, **options)
            names = ('bt', 'bt_adapter', 'global', 'reference', 'scan')
            if purpose == 'execution':
                names += ('tracker',)
            parameters = {
                name: yaml.safe_load((target / (name + '.yaml')).read_text())
                ['/**']['ros__parameters'] for name in names
            }
            result[purpose] = dict(
                session=session, parameters=parameters,
                localization=yaml.safe_load((target / 'localization.yaml').read_text()),
                planning_manifest=json.loads(Path(session['planning_manifest']).read_text()))
    finally:
        patch.undo()
    return result


def inspect(values):
    return navigation_contract.mainline_coordinate_contract(
        values['localization'], values['session'], values['parameters'],
        values['planning_manifest'])


@pytest.mark.parametrize('purpose', ['planning_only', 'execution'])
def test_real_formal_coordinate_chain_and_cad_uncertainty_are_reported(formal_coordinates, purpose):
    values = deepcopy(formal_coordinates[purpose])
    before = deepcopy(values)
    report = inspect(values)
    assert values == before, 'startup diagnosis must not rewrite geometry or acceptance'
    assert report['frames'] == navigation_contract.FRAMES
    assert report['public_tf_edges']
    assert report['sensor_origins']
    assert report['physical_open_items'], 'an engineering CAD estimate is not a calibrated physical fact'
    # The rear CAD/measurement discrepancy is an open measurement, not a
    # software frame mismatch that should make a valid preview unstartable.
    assert report['sensor_origins']['1']['cad_residual_m'] == pytest.approx(.07645, abs=1e-4)
    assert values['parameters']['scan']['grid_map.frame_id'] == 'd1max_loc_odom'
    assert values['parameters']['scan']['fsm.reference_path_z_offset'] == 0.
    assert values['parameters']['reference']['body_height_m'] == values['session']['body_height']
    manifest=values['planning_manifest']
    capabilities=values['session']['map_capabilities']
    if capabilities['geometry_kind']=='source_identity':
        assert manifest['geometry_operation']=='source_identity'
        assert manifest['xyz_modified'] is False
        assert manifest['frame_id']==navigation_contract.FRAMES['map_frame']
    else:
        # The existing selected 10-02 map is a conditioned multi-floor search
        # artifact. Do not rewrite it or assume the earlier floor-one subset.
        # Its reference must return to the same original localization source.
        from d1max_pct_scan.navigation_map_package import CONDITIONED_GEOMETRY
        assert capabilities['geometry_kind']=='conditioned_multifloor_ground'
        assert manifest['geometry_operation']==CONDITIONED_GEOMETRY
        assert manifest['frame_id']==capabilities['planning_frame']=='d1max_multifloor_planning'
        assert manifest['planning_only'] is True
        assert capabilities['execution_geometry_eligible'] is False
        assert values['session']['physical_acceptance'] is False
    assert values['planning_manifest']['source_path'] == values['session']['map_pcd']
    assert manifest['source_sha256']==values['session']['input_hashes'][values['session']['map_pcd']]


@pytest.mark.parametrize('purpose', ['planning_only', 'execution'])
@pytest.mark.parametrize('fault', [
    'scan_in_map', 'second_height_offset', 'wrong_local_body',
    'wrong_source_map', 'foreign_global_manifest', 'lio_second_tf',
    'reference_height_mismatch',
])
def test_actual_frame_and_handoff_mistakes_fail_before_children_start(formal_coordinates, purpose, fault):
    values = deepcopy(formal_coordinates[purpose])
    parameters = values['parameters']
    session = values['session']
    localization = values['localization']
    if fault == 'scan_in_map':
        parameters['scan']['grid_map.frame_id'] = 'd1max_loc_map'
    elif fault == 'second_height_offset':
        parameters['scan']['fsm.reference_path_z_offset'] = session['body_height']
    elif fault == 'wrong_local_body':
        localization['navigation_estimation']['ros__parameters']['body_frame'] = 'base_link'
    elif fault == 'wrong_source_map':
        # This is an existing planning subset, not the full localization source.
        session['map_pcd'] = str(Path(session['planning_manifest']).parent /
                                  values['planning_manifest']['output_file'])
    elif fault == 'foreign_global_manifest':
        parameters['global']['planning_manifest'] = str(Path(session['planning_manifest']).with_name('foreign.json'))
    elif fault == 'lio_second_tf':
        localization['/d1max/localization/lio/laserMapping']['ros__parameters']['publish.tf_enabled'] = True
    elif fault == 'reference_height_mismatch':
        parameters['reference']['body_height_m'] = session['body_height'] + .05
    with pytest.raises(ValueError):
        inspect(values)


@pytest.mark.parametrize('field, wrong', [
    ('planning_frame', 'd1max_loc_map'),
    ('base_frame', 'd1max_loc_tracking'),
])
def test_execution_tracker_cannot_follow_a_different_frame_or_body_origin(formal_coordinates, field, wrong):
    values = deepcopy(formal_coordinates['execution'])
    values['parameters']['tracker'][field] = wrong
    with pytest.raises(ValueError):
        inspect(values)
