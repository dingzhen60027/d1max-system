"""Native actor identity follows original rays through the same projection."""
import numpy as np
import pytest
from d1max_pct_scan.ray_projection import (FIELDS, ISAAC_FIELDS, ISAAC_RAY_DTYPE,
    RAY_DTYPE, RayProjectorCore, ProjectionError)
from test_ray_projection import CTX, EPOCH, IDENTITY, raw_points, decode, feed, project


def tagged_points():
    original = raw_points()
    points = np.zeros(len(original), dtype=ISAAC_RAY_DTYPE)
    for field in RAY_DTYPE.names:
        points[field] = original[field]
    points['isaac_actor_id'] = np.arange(len(points)) % 3
    return points


def tagged_decode(points, **changes):
    return decode(points, fields=ISAAC_FIELDS, point_step=72, row_step=len(points)*72,
        maximum_isaac_actor_id=2, **changes)


def test_default_schema_cannot_accept_an_isolated_actor_tag():
    points = tagged_points()
    with pytest.raises(ProjectionError, match='schema'):
        decode(points, fields=ISAAC_FIELDS, point_step=72, row_step=len(points)*72)
    for fields in (ISAAC_FIELDS[:-1], ISAAC_FIELDS + (ISAAC_FIELDS[-1],),
                   FIELDS + (('isaac_actor_id', 66, 4, 1),)):
        with pytest.raises(ProjectionError, match='schema'):
            decode(points, fields=fields, point_step=72, row_step=len(points)*72, maximum_isaac_actor_id=2)
    points['isaac_actor_id'][5] = 3
    with pytest.raises(ProjectionError, match='outside_registry'):
        tagged_decode(points)


def test_whole_original_ray_fields_and_actor_ids_survive_snapshot_projection():
    points = tagged_points()
    raw = tagged_decode(points)
    value = RayProjectorCore(projection_frame='odom', allow_simulation_snapshot=True, maximum_isaac_actor_id=2)
    value.reset(CTX);value.set_extrinsics(body_to_tracking=IDENTITY, ray_to_tracking=IDENTITY)
    feed(value)
    snapshot = value.projection_snapshot()
    result = project(snapshot, raw)
    assert result.points.dtype == ISAAC_RAY_DTYPE
    assert result.start_ns == EPOCH and result.end_ns == EPOCH + 100_000_000
    for name in RAY_DTYPE.names + ('isaac_actor_id',):
        if name not in ('x','y','z','origin_x','origin_y','origin_z'):
            np.testing.assert_array_equal(result.points[name], points[name])
    assert snapshot.maximum_isaac_actor_id == value.maximum_isaac_actor_id == 2
    with pytest.raises(ProjectionError, match='invalid_projection_input'):
        ordinary = RayProjectorCore(projection_frame='odom')
        ordinary.reset(CTX);ordinary.set_extrinsics(body_to_tracking=IDENTITY, ray_to_tracking=IDENTITY)
        feed(ordinary);project(ordinary, raw)
    assert np.any(result.points['x'] != points['x'])
