"""Planning-only XYZ selection model; independent of RViz, ROS and robot control."""
import numpy as np


def finite_xyz(values):
    xyz = np.asarray(values, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError('Select a finite X, Y, Z point')
    if np.any(np.abs(xyz) > 10000):
        raise ValueError('Point exceeds the editor coordinate range')
    return xyz.copy()


def unit_quaternion(values):
    """Validate editor orientation; q and -q denote the same spatial attitude."""
    quat = np.asarray(values, dtype=float)
    if quat.shape != (4,) or not np.isfinite(quat).all():
        raise ValueError('Orientation must be a finite XYZW quaternion')
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Orientation quaternion must not be zero')
    quat = quat / norm
    # Canonical sign, including the w == 0 (180-degree) case.
    for index in (3, 0, 1, 2):
        if abs(quat[index]) > 1e-12:
            if quat[index] < 0:
                quat = -quat
            break
    return quat


class PreviewSelection:
    def __init__(self, grid, height_tolerance=0.08):
        if not np.isfinite(height_tolerance) or not 0 < height_tolerance <= 0.15:
            raise ValueError('Selection height tolerance must be in (0, 0.15] m')
        self.grid = grid
        self.height_tolerance = float(height_tolerance)
        self.points = {'start': None, 'goal': None}
        # The native planner constrains XYZ, not endpoint orientation. Keep
        # pose-editor orientation and its revision separate from route validity.
        self.orientations = {role: np.array([0., 0., 0., 1.]) for role in self.points}
        self.orientation_revision = 0
        self.revision = 0

    def set_orientation(self, role, quaternion_xyzw):
        if role not in self.points or self.points[role] is None:
            raise ValueError('Place the endpoint before editing orientation')
        value = unit_quaternion(quaternion_xyzw)
        if np.allclose(value, self.orientations[role], atol=1e-10, rtol=0):
            return False
        self.orientations[role] = value
        self.orientation_revision += 1
        return True

    def orientation_snapshot(self):
        return {
            'orientation_semantics': 'editor_preview_only_xyz_planner',
            'orientation_revision': self.orientation_revision,
            **{f'{role}_orientation_xyzw': None if self.points[role] is None else quat.tolist()
               for role, quat in self.orientations.items()},
        }

    def set_point(self, role, xyz):
        if role not in self.points:
            raise ValueError('Unknown endpoint')
        value = finite_xyz(xyz)
        if self.points[role] is not None and np.array_equal(value, self.points[role]):
            return False
        self.points[role] = value
        self.revision += 1
        return True

    def clear(self):
        self.points = {'start': None, 'goal': None}
        self.orientations = {role: np.array([0., 0., 0., 1.]) for role in self.points}
        self.orientation_revision += 1
        self.revision += 1

    def activate(self, role, anchor_xy):
        """Create a movable map position, never require a rendered LiDAR hit.

        A measured free cell is just the initial seed. Re-activation preserves
        arbitrary user-edited XYZ, including positions not valid for planning.
        """
        if role not in self.points:
            raise ValueError('Unknown endpoint')
        if self.points[role] is not None:
            return False
        target = np.asarray(anchor_xy, dtype=float)
        if target.shape != (2,) or not np.isfinite(target).all():
            raise ValueError('Invalid placement anchor')
        if role == 'goal':
            base = self.points['start']
            target = (base[:2] if base is not None else target) + [0., 2.]
        indices = np.argwhere(self.grid.planner_free)
        if not len(indices):
            raise ValueError('Map has no supported placement seed')
        xy = self.grid.origin + (indices + .5) * self.grid.resolution
        selected = int(np.argmin(np.sum((xy - target) ** 2, axis=1)))
        cell = tuple(indices[selected])
        return self.set_point(role, [*xy[selected], self.grid.height[cell]])

    def ground(self, role):
        point = self.points[role]
        if point is None:
            raise ValueError(f'Select {role} first')
        return self.grid.validate_point(point[:2])

    def snap_ground(self, role):
        height = self.ground(role)
        point = self.points[role].copy()
        point[2] = height
        return self.set_point(role, point)

    def validate(self, role):
        ground = self.ground(role)
        point = self.points[role]
        error = float(point[2] - ground)
        if abs(error) > self.height_tolerance:
            raise ValueError(
                f'{role}: Z differs from ground by {error:+.3f} m; '
                'move the Z handle or explicitly click Ground')
        index = tuple(self.grid.index(point[:2]))
        if not self.grid.planner_free[index]:
            raise ValueError(f'{role}: insufficient clearance for the PCT planning margin')
        return point.copy()

    def request(self):
        start, goal = self.validate('start'), self.validate('goal')
        if np.linalg.norm(goal[:2] - start[:2]) < self.grid.resolution:
            raise ValueError('Start and goal must be at least one grid cell apart in XY')
        return {'generation': self.revision, 'start_xy': start[:2].tolist(),
                'goal_xy': goal[:2].tolist(), 'start_xyz': start.tolist(),
                'goal_xyz': goal.tolist()}

    def checked_result(self, result, request, max_ground_step=0.15):
        if result.get('generation') != self.revision or request['generation'] != self.revision:
            return None
        path = np.array(result['path'], dtype=float, copy=True)
        if path.ndim != 2 or path.shape[1] != 3 or len(path) < 2 or not np.isfinite(path).all():
            raise ValueError('Native planner returned an invalid path')
        start, goal = self.validate('start'), self.validate('goal')
        if not np.allclose(path[0, :2], start[:2], atol=1e-6, rtol=0) or not np.allclose(
                path[-1, :2], goal[:2], atol=1e-6, rtol=0):
            raise ValueError('Planner endpoint contract mismatch')
        # Never silently change the picked/edited endpoint Z. Small permitted
        # differences reflect point-vs-grid sampling, not permission to fly.
        path[0], path[-1] = start, goal
        if abs(path[1, 2] - start[2]) > max_ground_step or abs(
                path[-2, 2] - goal[2]) > max_ground_step:
            raise ValueError('Selected endpoint creates too large a ground-height transition')
        self.grid.validate_path(path, max_ground_step)
        return path

    def snapshot(self):
        return {f'{role}_xyz': None if point is None else point.tolist()
                for role, point in self.points.items()}
