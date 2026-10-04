"""Bounded PointCloud2 XYZ decoder, including endian/organized-row padding."""
import numpy as np


def pointcloud_xyz(message, max_points):
    if (message.width <= 0 or message.height <= 0 or message.width * message.height > max_points
            or message.point_step < 12 or message.row_step < message.width * message.point_step
            or len(message.data) != message.row_step * message.height):
        raise ValueError('invalid_or_oversized_pointcloud_layout')
    fields = {f.name: f for f in message.fields}
    if any(k not in fields for k in ('x', 'y', 'z')):
        raise ValueError('pointcloud_missing_xyz')
    endian = '>' if message.is_bigendian else '<'
    types = {7: 'f4', 8: 'f8'}
    columns = []
    for key in ('x', 'y', 'z'):
        f = fields[key]
        if f.datatype not in types or f.count != 1:
            raise ValueError('invalid_xyz_field')
        dtype = np.dtype(endian + types[f.datatype])
        if f.offset < 0 or f.offset + dtype.itemsize > message.point_step:
            raise ValueError('xyz_field_exceeds_point_step')
        array = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
            offset=f.offset, strides=(message.row_step, message.point_step))
        columns.append(array.reshape(-1))
    points = np.column_stack(columns).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]
