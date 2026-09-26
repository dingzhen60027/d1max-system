"""Pose-handle contract and real Humble auto-completed visual geometry.

No ROS graph or robot is started. The native visual test calls the installed
interactive_markers library, not a Python imitation of its geometry.
"""
import json
import math
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

pytest.importorskip('visualization_msgs.msg', reason='Source ROS Humble for marker tests')
from visualization_msgs.msg import InteractiveMarkerControl, Marker

from d1max_pct_planner.preview_markers import pose_marker


def rotated_x(q):
    x, y, z, w = q
    return np.array([1 - 2 * (y*y + z*z), 2 * (x*y + w*z), 2 * (x*z - w*y)])


def q_values(q):
    return [q.x, q.y, q.z, q.w]


def build(**overrides):
    args = dict(role='start', xyz=[-4.1, 6.9, -0.8], quaternion_xyzw=[0, 0, 0, 1],
                frame='d1max_loc_map', scale=2.0, valid=True)
    args.update(overrides)
    return pose_marker(**args)


def test_true_pose_is_not_lifted_and_orientation_is_preserved_normalized():
    marker = build(quaternion_xyzw=[0, 0, 2, 2])
    assert [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z] == [-4.1, 6.9, -0.8]
    np.testing.assert_allclose(q_values(marker.pose.orientation), [0, 0, math.sqrt(.5), math.sqrt(.5)])
    assert marker.header.frame_id == 'd1max_loc_map'
    assert marker.header.stamp.sec == marker.header.stamp.nanosec == 0


@pytest.mark.parametrize('role', ['start', 'goal'])
def test_moveit_style_center_has_free_translation_rotation_and_visible_role(role):
    marker = build(role=role)
    assert marker.name == role and marker.description == role.upper()
    assert len(marker.controls) == 7
    center = marker.controls[0]
    assert center.name == 'move_rotate_3d'
    assert center.interaction_mode == InteractiveMarkerControl.MOVE_ROTATE_3D
    assert center.orientation_mode == InteractiveMarkerControl.VIEW_FACING
    assert center.independent_marker_orientation and center.always_visible
    sphere, arrow = center.markers
    assert sphere.type == Marker.SPHERE and arrow.type == Marker.ARROW
    assert sphere.color.a == arrow.color.a == 1
    assert sphere.scale.x == pytest.approx(.26 * marker.scale)
    assert arrow.scale.x < .5 * marker.scale
    assert sphere.header.frame_id == arrow.header.frame_id == ''


def test_rotating_pose_does_not_rotate_world_translation_or_rotation_axes():
    original = build()
    rotated = build(quaternion_xyzw=[.1, .2, .3, .4])
    for axis_index, axis in enumerate('xyz'):
        for action, mode in [('move', InteractiveMarkerControl.MOVE_AXIS),
                             ('rotate', InteractiveMarkerControl.ROTATE_AXIS)]:
            a = next(control for control in original.controls if control.name == f'{action}_{axis}')
            b = next(control for control in rotated.controls if control.name == f'{action}_{axis}')
            assert a == b
            assert b.orientation_mode == InteractiveMarkerControl.FIXED
            assert b.interaction_mode == mode and b.always_visible
            np.testing.assert_allclose(rotated_x(q_values(b.orientation)), np.eye(3)[axis_index], atol=1e-12)
            assert not b.markers, 'RViz must generate native arrow/ring geometry'


def test_invalid_position_stays_editable_and_changes_role_center_not_axis_contract():
    valid, invalid = build(), build(valid=False)
    assert invalid.description == 'START · INVALID'
    assert invalid.pose == valid.pose and invalid.controls[1:] == valid.controls[1:]
    color = invalid.controls[0].markers[0].color
    assert color.r == 1 and color.g < .2 and color.b < .2 and color.a == 1
    assert invalid.controls[0].interaction_mode == InteractiveMarkerControl.MOVE_ROTATE_3D
    assert valid.controls[0].markers[0].color != build(role='goal').controls[0].markers[0].color


def test_ground_mode_is_world_xy_plane_with_no_height_or_tilt_controls():
    marker = build(ground_follow=True, quaternion_xyzw=[.1, .2, .3, .4])
    center = marker.controls[0]
    assert center.interaction_mode == InteractiveMarkerControl.MOVE_PLANE
    assert center.orientation_mode == InteractiveMarkerControl.FIXED
    np.testing.assert_allclose(rotated_x(q_values(center.orientation)), [0, 0, 1], atol=1e-12)
    assert {c.name for c in marker.controls[1:]} == {'move_x', 'move_y', 'rotate_z'}
    assert len(center.markers) == 2 and center.always_visible


@pytest.mark.parametrize('overrides', [
    {'role': 'robot'}, {'xyz': [1, 2]}, {'xyz': [0, float('nan'), 1]},
    {'xyz': None}, {'quaternion_xyzw': [0, 0, 0, 0]},
    {'quaternion_xyzw': [1, 0, 0]}, {'quaternion_xyzw': [0, 0, 0, float('inf')]},
    {'frame': ''}, {'frame': ' map'}, {'frame': None}, {'scale': 0},
    {'scale': -1}, {'scale': float('nan')}, {'scale': True}, {'valid': 'false'},
])
def test_bad_marker_inputs_fail_before_building_ros_message(overrides):
    with pytest.raises(ValueError):
        build(**overrides)


NATIVE_PROBE = r'''
#include <interactive_markers/tools.hpp>
#include <iostream>
#include <iomanip>
#include <string>
int main() {
  visualization_msgs::msg::InteractiveMarker msg;
  int n;
  std::cin >> msg.scale >> n;
  msg.pose.orientation.w = 1;
  for (int i=0; i<n; ++i) {
    visualization_msgs::msg::InteractiveMarkerControl c;
    int mode, orientation_mode;
    std::cin >> c.name >> mode >> orientation_mode >> c.orientation.x >> c.orientation.y
             >> c.orientation.z >> c.orientation.w;
    c.interaction_mode = mode;
    c.orientation_mode = orientation_mode;
    msg.controls.push_back(c);
  }
  interactive_markers::autoComplete(msg, true);
  std::cout << std::setprecision(12) << "[";
  bool first_control=true;
  for (const auto& c: msg.controls) {
    if (!first_control) std::cout << ",";
    first_control=false;
    std::cout << "{\"name\":\"" << c.name << "\",\"markers\":[";
    bool first_marker=true;
    for (const auto& m: c.markers) {
      if (!first_marker) std::cout << ",";
      first_marker=false;
      const auto& q=m.pose.orientation;
      const auto& s=m.scale;
      const auto& color=m.color;
      std::cout << "{\"type\":" << m.type << ",\"q\":[" << q.x << "," << q.y << "," << q.z << "," << q.w
                << "],\"scale\":[" << s.x << "," << s.y << "," << s.z
                << "],\"color\":[" << color.r << "," << color.g << "," << color.b << "," << color.a
                << "],\"points\":[";
      bool first_point=true;
      for (const auto& p: m.points) {
        if (!first_point) std::cout << ",";
        first_point=false;
        std::cout << "[" << p.x << "," << p.y << "," << p.z << "]";
      }
      std::cout << "]}";
    }
    std::cout << "]}";
  }
  std::cout << "]";
}
'''


@pytest.fixture(scope='module')
def native_visual_probe(tmp_path_factory):
    prefix = Path('/opt/ros/humble')
    compiler = shutil.which('g++')
    if compiler is None or not (prefix / 'lib/libinteractive_markers.so').is_file():
        pytest.skip('Native Humble library/C++ compiler not available')
    binary = tmp_path_factory.mktemp('marker_native_probe') / 'autocomplete'
    include_packages = ['interactive_markers', 'visualization_msgs', 'geometry_msgs', 'sensor_msgs',
                        'std_msgs', 'builtin_interfaces', 'rosidl_runtime_cpp', 'rosidl_runtime_c',
                        'rosidl_typesupport_interface']
    command = [compiler, '-std=c++17', '-x', 'c++', '-', '-o', str(binary)]
    command += [f'-I{prefix / "include" / package}' for package in include_packages]
    command += [f'-L{prefix / "lib"}', f'-Wl,-rpath,{prefix / "lib"}', '-linteractive_markers']
    compiled = subprocess.run(command, input=NATIVE_PROBE, text=True, capture_output=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr

    def complete(marker):
        controls = marker.controls[1:]
        payload = [f'{marker.scale} {len(controls)}']
        for c in controls:
            payload.append(' '.join(map(str, [c.name, c.interaction_mode, c.orientation_mode,
                                              *q_values(c.orientation)])))
        result = subprocess.run([str(binary)], input='\n'.join(payload), text=True,
                                capture_output=True, check=True, timeout=5)
        return {item['name']: item['markers'] for item in json.loads(result.stdout)}
    return complete


@pytest.mark.parametrize('scale', [1.2, 2.5, 4.0])
def test_installed_humble_generates_large_axis_correct_arrows_and_rings(native_visual_probe, scale):
    visuals = native_visual_probe(build(scale=scale))
    for index, axis in enumerate('xyz'):
        arrows = visuals[f'move_{axis}']
        assert len(arrows) == 2
        signs = []
        for arrow in arrows:
            assert arrow['type'] == Marker.ARROW
            np.testing.assert_allclose(arrow['color'][:3], np.eye(3)[index], atol=1e-6)
            assert arrow['color'][3] == pytest.approx(.5)
            direction = rotated_x(arrow['q'])
            np.testing.assert_allclose(direction, np.eye(3)[index], atol=1e-6)
            points = np.array(arrow['points'])
            assert points.shape == (2, 3)
            sign = math.copysign(1, points[0, 0])
            signs.append(sign)
            np.testing.assert_allclose(points[:, 0] / scale, sign * np.array([.5, .9]), atol=1e-6)
            np.testing.assert_allclose(points[:, 1:], 0)
            np.testing.assert_allclose(np.array(arrow['scale']) / scale, [.15, .25, .2], atol=1e-6)
        assert sorted(signs) == [-1., 1.]
        ring, = visuals[f'rotate_{axis}']
        assert ring['type'] == Marker.TRIANGLE_LIST
        np.testing.assert_allclose(rotated_x(ring['q']), np.eye(3)[index], atol=1e-6)
        np.testing.assert_allclose(ring['color'][:3], np.eye(3)[index], atol=1e-6)
        np.testing.assert_allclose(ring['scale'], scale)
        points = np.array(ring['points'])
        assert len(points) >= 100  # Filled pickable annulus, not a thin line.
        np.testing.assert_allclose(points[:, 0], 0)
        radius = np.linalg.norm(points[:, 1:], axis=1)
        assert radius.min() == pytest.approx(.5)
        assert radius.max() == pytest.approx(.65)
