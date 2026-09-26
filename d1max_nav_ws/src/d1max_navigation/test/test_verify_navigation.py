import math
from types import SimpleNamespace as NS
import pytest

from d1max_navigation.verify_navigation import raw_costmap_lethal_cells_near, collision_stop_evidence


def costmap(yaw=0.):
    values = [0] * 100
    values[5*10+5] = 254
    values[0] = 255  # Unknown is not proof of a dynamic obstacle.
    values[1] = 253  # Inscribed/inflation cost is also not a lethal return.
    return NS(metadata=NS(resolution=.1, size_x=10, size_y=10,
                          origin=NS(position=NS(x=2., y=3.),
                                    orientation=NS(z=math.sin(yaw/2), w=math.cos(yaw/2)))),
              data=values)


def test_raw_lethal_count_excludes_unknown_inflation_and_outside():
    grid = costmap()
    assert raw_costmap_lethal_cells_near(grid, 2.55, 3.55, .05) == 1
    assert raw_costmap_lethal_cells_near(grid, 2.05, 3.05, .1) == 0
    assert raw_costmap_lethal_cells_near(grid, -10., -10., .1) == 0


def test_raw_lethal_count_respects_rotated_grid_origin():
    grid = costmap(math.pi/2)
    assert raw_costmap_lethal_cells_near(grid, 1.45, 3.55, .05) == 1


def safe_zeros():
    return [(1+i*.05, 0., 0., 0.) for i in range(10)]


def test_documented_stopped_silence_requires_affirmative_stop_and_safe_zeros():
    assert collision_stop_evidence([], safe_zeros(), [(1., 'polygon stop')], 45, 0.) == 'documented_zero_publication_timeout'
    assert collision_stop_evidence([(1.,0.,0.,0.)], safe_zeros(), [(1.,'polygon stop')], 45, 0.) == 'zero_twists'


@pytest.mark.parametrize('checked,safe,events,points,drift', [
    ([], safe_zeros(), [], 45, 0.),
    ([], [], [(1.,'stop')], 45, 0.),
    ([(1., .1, 0., 0.)], safe_zeros(), [(1.,'stop')], 45, 0.),
    ([], [(1., .1, 0., 0.)]*10, [(1.,'stop')], 45, 0.),
    ([], [(1+i*.3,0.,0.,0.) for i in range(10)], [(1.,'stop')], 45, 0.),
    ([], safe_zeros(), [(1.,'stop')], 3, 0.),
    ([], safe_zeros(), [(1.,'stop')], 45, .1),
])
def test_collision_stop_never_accepts_missing_evidence_or_nonzero_motion(checked,safe,events,points,drift):
    with pytest.raises(RuntimeError):
        collision_stop_evidence(checked,safe,events,points,drift)
