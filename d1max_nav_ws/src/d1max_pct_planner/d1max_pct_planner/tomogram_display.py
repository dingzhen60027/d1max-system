"""Render the same measured surfaces that endpoint and route checks use."""
import numpy as np


def surface_clouds(tomogram, layers):
    """Return allowed XYZ/cost and blocked XYZ, without overlapping duplicates.

    Adjacent retained slices can represent the same physical ground. A valid
    representation wins over a blocked duplicate only for visualization; no
    layer's planning mask or costs are changed.
    """
    visible, costs, blocked = [], [], []
    for layer in sorted(set(layers)):
        layer = tomogram.layer(layer)
        data = tomogram.display_points(layer_id=layer)
        visible.append(data['xyz'])
        costs.append(data['cost'])
        indices = np.argwhere(tomogram.ground_known[layer] & ~tomogram.allowed[layer])
        xy = tomogram.center + (indices - tomogram.offset) * tomogram.resolution
        z = tomogram.ground[layer, indices[:, 0], indices[:, 1]]
        blocked.append(np.column_stack((xy, z)))
    xyz = np.concatenate(visible) if visible else np.empty((0, 3))
    cost = np.concatenate(costs) if costs else np.empty(0)
    if len(xyz):
        # Keep the lowest-cost valid representation of the same XYZ surface.
        order = np.argsort(cost, kind='stable')
        _, keep = np.unique(np.round(xyz[order], 4), axis=0, return_index=True)
        keep = order[keep]
        xyz, cost = xyz[keep], cost[keep]
    red = np.concatenate(blocked) if blocked else np.empty((0, 3))
    if len(red):
        _, keep = np.unique(np.round(red, 4), axis=0, return_index=True)
        red = red[keep]
        green_keys = {tuple(point) for point in np.round(xyz, 4)}
        red = red[[tuple(point) not in green_keys for point in np.round(red, 4)]]
    return xyz, cost, red
