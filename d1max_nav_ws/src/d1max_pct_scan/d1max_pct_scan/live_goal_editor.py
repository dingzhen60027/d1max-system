"""Pure, display-only placement. Actual endpoint admission stays in PCT."""
import math
import numpy as np


def goal_position(*, existing=None, body=None, surfaces=None, fallback=(0., 0., 0.), body_height=.55):
    if existing is not None:
        point = np.asarray(existing, dtype=float)
    elif body is not None:
        x, y, z, yaw = body
        point = np.array([x+1.5*math.cos(yaw), y+1.5*math.sin(yaw), z-body_height])
        if surfaces is not None and len(surfaces):
            candidates = np.asarray(surfaces, dtype=float)
            candidates = candidates[np.abs(candidates[:, 2]-point[2]) < 1.]
            if len(candidates):
                distance = np.linalg.norm(candidates[:, :2]-point[:2], axis=1)
                nearest = int(np.argmin(distance))
                if distance[nearest] < 3.:
                    point = candidates[nearest]
    else:
        point = np.asarray(fallback, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or np.max(np.abs(point)) >= 10000:
        raise ValueError('Invalid editable goal coordinates')
    return tuple(map(float, point))
