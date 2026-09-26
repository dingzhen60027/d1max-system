"""Layer-aware endpoint editor over the actual PCT tomogram.

The ground-follow editor and the native planner share one surface authority.
No XY snapping, mask dilation or guessed ceiling is performed here.
"""
import numpy as np

from .preview_core import PreviewSelection, finite_xyz


class TomogramSelection(PreviewSelection):
    def __init__(self, tomogram, height_tolerance=0.08, mode='ground',
                 anchor_z=-0.6375, max_follow_height_change=0.3):
        super().__init__(tomogram, height_tolerance)
        self.layers = {'start': None, 'goal': None}
        self.mode = 'ground'
        self.active_layer = -1
        self.anchor_z = float(anchor_z)
        self.max_follow_height_change = float(max_follow_height_change)
        if not np.isfinite(self.anchor_z) or not np.isfinite(self.max_follow_height_change) or self.max_follow_height_change <= 0:
            raise ValueError('Invalid ground-follow height settings')
        self.set_mode(mode)
        self.edit_errors = {'start': None, 'goal': None}

    def set_mode(self, mode):
        if mode not in ('ground', 'free'):
            raise ValueError('Selection mode must be ground or free')
        self.mode = mode

    def set_active_layer(self, layer):
        if isinstance(layer, bool) or not isinstance(layer, int) or not -1 <= layer < self.grid.ground.shape[0]:
            raise ValueError('Invalid tomogram editing layer')
        # This selects the layer for subsequent edits, never teleports existing
        # endpoints and never claims that a tomographic slice is a floor number.
        self.active_layer = layer

    def _store(self, role, xyz, layer, error=None):
        previous_layer = self.layers[role]
        changed = super().set_point(role, xyz)
        self.layers[role] = layer
        self.edit_errors[role] = error
        if previous_layer != layer and not changed:
            self.revision += 1
            changed = True
        return changed

    def set_point(self, role, xyz):
        if role not in self.points:
            raise ValueError('Unknown endpoint')
        desired = finite_xyz(xyz)
        hint = desired.copy()
        if self.mode == 'ground' and self.points[role] is not None:
            hint[2] = self.points[role][2]
        try:
            surface = self.grid.select(
                hint, mode='ground_follow' if self.mode == 'ground' else 'free_xyz',
                preferred_layer=self.layers[role],
                layer_lock=self.active_layer if self.active_layer >= 0 else None,
                height_tolerance_m=self.height_tolerance,
                max_follow_height_change_m=self.max_follow_height_change)
            return self._store(role, surface['xyz'], surface['layer_id'])
        except ValueError as exc:
            # Keep the requested XY visible, even over unsupported/blocked
            # space, so the user can correct it. Ground mode does not derive
            # arbitrary Z from camera movement when no supported surface exists.
            if self.mode == 'ground':
                desired[2] = hint[2]
            return self._store(role, desired, self.layers[role], exc)

    def activate(self, role, anchor_xy):
        if role not in self.points:
            raise ValueError('Unknown endpoint')
        if self.points[role] is not None:
            return False
        anchor = np.r_[np.asarray(anchor_xy, dtype=float), self.anchor_z]
        if role == 'goal' and self.points['start'] is not None:
            anchor = self.points['start'] + [0., 2., 0.]
        surface = self.grid.initial_seed(
            anchor, layer_lock=self.active_layer if self.active_layer >= 0 else None)
        return self._store(role, surface['xyz'], surface['layer_id'])

    def validate(self, role):
        point = self.points[role]
        if point is None:
            raise ValueError(f'Select {role} first')
        if self.edit_errors[role]:
            raise self.edit_errors[role]
        self.grid.validate_endpoint(point, layer_id=self.layers[role],
                                    height_tolerance_m=self.height_tolerance)
        return point.copy()

    def snap_ground(self, role):
        if self.points[role] is None:
            raise ValueError(f'Select {role} first')
        lock = self.active_layer if self.active_layer >= 0 else self.layers[role]
        surface = self.grid.select(
            self.points[role], mode='ground_follow', preferred_layer=self.layers[role],
            layer_lock=lock, height_tolerance_m=self.height_tolerance,
            max_follow_height_change_m=10000.)
        return self._store(role, surface['xyz'], surface['layer_id'])

    def validation(self, role):
        try:
            self.validate(role)
            return {'valid': True, 'reason': 'traversable', 'reason_code': 'traversable',
                    'layer_id': self.layers[role]}
        except ValueError as exc:
            return {'valid': False, 'reason': getattr(exc, 'code', str(exc)),
                    'reason_code': getattr(exc, 'code', 'invalid_endpoint'),
                    'detail': str(exc), 'layer_id': self.layers[role]}

    def request(self):
        start, goal = self.validate('start'), self.validate('goal')
        if np.linalg.norm(start - goal) < 1e-5:
            raise ValueError('Start and goal must be distinct')
        return {'generation': self.revision,
                'start_xyz': start.tolist(), 'goal_xyz': goal.tolist(),
                'start_xy': start[:2].tolist(), 'goal_xy': goal[:2].tolist(),
                'start_layer': self.layers['start'], 'goal_layer': self.layers['goal']}

    def checked_result(self, result, request, max_ground_step=0.15):
        if result.get('generation') != self.revision or request['generation'] != self.revision:
            return None
        points = np.asarray(result['path'], dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('Native PCT returned an invalid layered route')
        if not np.allclose(points[[0, -1]], [self.validate('start'), self.validate('goal')],
                           rtol=0, atol=1e-6):
            raise ValueError('PCT endpoint XYZ contract mismatch')
        self.grid.validate_path(points, result['layer_ids'], max_ground_step_m=max_ground_step)
        return points.copy()

    def clear(self):
        super().clear()
        self.layers = {'start': None, 'goal': None}
        self.edit_errors = {'start': None, 'goal': None}
