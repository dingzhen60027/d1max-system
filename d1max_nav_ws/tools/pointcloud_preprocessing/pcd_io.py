"""Strict binary PCD IO that filters original records without rewriting fields.

Only uncompressed ``DATA binary`` is supported. ``write_subset`` never changes
records; ``write_derived_z`` explicitly creates a planning-only Z derivative.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
from typing import Any

import numpy as np


class PcdError(ValueError):
    """Malformed/unsupported PCD or invalid subset request."""


@dataclass(frozen=True)
class PcdCloud:
    xyz: np.ndarray
    records: np.ndarray
    fields: list[str]
    metadata: dict[str, Any]

    @property
    def finite_xyz_mask(self) -> np.ndarray:
        """Return a mask without dropping or altering any original record."""
        return np.isfinite(self.xyz).all(axis=1)


_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_INTEGER = re.compile(r"^[0-9]+$")
_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", **{
    (kind, size): f"<{numpy_kind}{size}"
    for kind, numpy_kind in (("I", "i"), ("U", "u"))
    for size in (1, 2, 4, 8)
}}


def _integers(header: dict[str, list[str]], key: str, length: int,
              *, minimum: int = 0) -> list[int]:
    values = header.get(key, [])
    if len(values) != length or any(not _INTEGER.fullmatch(value) for value in values):
        raise PcdError(f"{key} must contain exactly {length} non-negative integer(s)")
    result = list(map(int, values))
    if any(value < minimum for value in result):
        raise PcdError(f"{key} values must be >= {minimum}")
    return result


def read_pcd(path: str | Path) -> PcdCloud:
    """Read a strictly checked binary PCD; preserve arbitrary numeric fields.

    ``records`` is a read-only structured view of the original bytes, including
    COUNT > 1 fields and padding fields. ``xyz`` is a separate float64 matrix.
    A non-finite XYZ record is retained and exposed by ``finite_xyz_mask``.
    """
    source = Path(path).resolve(strict=True)
    header: dict[str, list[str]] = {}
    with source.open("rb") as stream:
        for _ in range(4096):
            line = stream.readline(65537)
            if not line or len(line) > 65536 or stream.tell() > 65536:
                raise PcdError("PCD header is missing DATA or exceeds 64 KiB")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise PcdError("PCD header must be ASCII") from exc
            if not text or text.startswith("#"):
                continue
            tokens = text.split()
            key = tokens[0].upper()
            if key == "FIELD":
                key = "FIELDS"
            if key in header:
                raise PcdError(f"Duplicate PCD header entry: {key}")
            if len(tokens) < 2:
                raise PcdError(f"Empty PCD header entry: {key}")
            header[key] = tokens[1:]
            if key == "DATA":
                if not line.endswith(b"\n"):
                    raise PcdError("DATA header line must end with a newline")
                break
        else:
            raise PcdError("PCD header has too many lines")
        offset = stream.tell()
        if header.get("DATA") != ["binary"]:
            raise PcdError("Only uncompressed DATA binary PCD is supported")
        fields = header.get("FIELDS", [])
        if not fields or len(set(fields)) != len(fields) or any(not _NAME.fullmatch(name) for name in fields):
            raise PcdError("FIELDS must contain unique valid field names")
        if not {"x", "y", "z"}.issubset(fields):
            raise PcdError("PCD requires x, y and z fields")
        number = len(fields)
        sizes = _integers(header, "SIZE", number, minimum=1)
        types = header.get("TYPE", [])
        if len(types) != number:
            raise PcdError("TYPE count does not match FIELDS")
        header.setdefault("COUNT", ["1"] * number)
        counts = _integers(header, "COUNT", number, minimum=1)
        width = _integers(header, "WIDTH", 1)[0]
        height = _integers(header, "HEIGHT", 1, minimum=1)[0]
        points = _integers(header, "POINTS", 1)[0]
        if width * height != points:
            raise PcdError("POINTS must equal WIDTH * HEIGHT")
        header.setdefault("VERSION", ["0.7"])
        if header["VERSION"] not in (["0.7"], [".7"]):
            raise PcdError("Only PCD VERSION 0.7 is supported")
        header.setdefault("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"])
        try:
            viewpoint = [float(value) for value in header["VIEWPOINT"]]
        except ValueError as exc:
            raise PcdError("VIEWPOINT must contain seven finite numbers") from exc
        if len(viewpoint) != 7 or not np.isfinite(viewpoint).all():
            raise PcdError("VIEWPOINT must contain seven finite numbers")

        layout = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            dtype = _TYPES.get((kind, size))
            if dtype is None:
                raise PcdError(f"Unsupported SIZE/TYPE for field {name}: {size}/{kind}")
            if name in ("x", "y", "z") and count != 1:
                raise PcdError(f"Coordinate field {name} requires COUNT 1")
            layout.append((name, dtype) if count == 1 else (name, dtype, (count,)))
        try:
            dtype = np.dtype(layout, align=False)
        except (ValueError, TypeError, OverflowError) as exc:
            raise PcdError("PCD record layout is invalid or too large") from exc
        expected = points * dtype.itemsize
        actual = os.fstat(stream.fileno()).st_size - offset
        if actual != expected:
            raise PcdError(f"PCD binary payload length mismatch: expected {expected}, got {actual}")
        payload = stream.read()
        if len(payload) != expected:
            raise PcdError("PCD changed or was truncated while reading")

    records = np.frombuffer(payload, dtype=dtype, count=points)
    xyz = np.column_stack([records[axis].astype(np.float64) for axis in ("x", "y", "z")])
    return PcdCloud(xyz=xyz, records=records, fields=list(fields), metadata={
        "source_path": str(source), "header": header, "record_size": dtype.itemsize,
        "data_offset": offset, "points": points, "width": width, "height": height,
        "viewpoint": viewpoint,
    })


def _subset_indices(indices: Any, count: int) -> np.ndarray:
    values = np.asarray(indices)
    if values.ndim != 1:
        raise PcdError("Subset indices must be a one-dimensional integer list or boolean mask")
    if values.dtype.kind == "b":
        if len(values) != count:
            raise PcdError("Boolean subset mask length must equal the input point count")
        return np.flatnonzero(values)
    # An empty Python list has float dtype; no values are being converted.
    if not len(values):
        return np.empty(0, dtype=np.intp)
    if values.dtype.kind not in "iu":
        raise PcdError("Subset indices must be integers, not rounded coordinates/floats")
    if (values < 0).any() or (values >= count).any():
        raise PcdError("Subset index is outside the input record range")
    values = values.astype(np.intp, copy=False)
    if len(np.unique(values)) != len(values):
        raise PcdError("Subset indices must not repeat records")
    return values


def write_subset(path: str | Path, cloud: PcdCloud, indices: Any) -> list[str]:
    """Exclusively create a binary PCD containing byte-exact selected records.

    Existing files/symlinks and the source path are never overwritten. Input
    ordering follows the supplied indices. Empty subsets are supported. The
    destination parent must already exist. An incomplete file on write failure
    is left for its owning job to report/recover; it is never marked complete.
    """
    destination = Path(path)
    if destination.resolve() == Path(cloud.metadata["source_path"]).resolve():
        raise PcdError("Output path must differ from the input PCD")
    selected = _subset_indices(indices, len(cloud.records))
    if tuple(cloud.fields) != cloud.records.dtype.names:
        raise PcdError("Cloud fields no longer match the original record layout")
    return _write_records(destination, cloud, cloud.records[selected],
                          "original-record subset; no resampling")


def write_derived_z(path: str | Path, cloud: PcdCloud, indices: Any,
                    z_values: Any) -> list[str]:
    """Create an explicitly non-rigid, planning-only derivative, modifying Z only.

    ``z_values`` corresponds to selected indices, not the full cloud. All other
    fields retain their original bytes. A non-rigid map has no single sensor
    VIEWPOINT, so this metadata is reset to identity (not silently preserved).
    Source/existing outputs are never overwritten. No points are synthesized.
    """
    destination = Path(path)
    if destination.resolve() == Path(cloud.metadata['source_path']).resolve():
        raise PcdError('Output path must differ from the input PCD')
    selected = _subset_indices(indices, len(cloud.records))
    values = np.asarray(z_values, dtype=np.float64)
    dtype = cloud.records.dtype.fields['z'][0]
    if values.shape != (len(selected),) or not np.isfinite(values).all():
        raise PcdError('Derived Z must be one finite value per selected point')
    if dtype.kind != 'f' or np.any(np.abs(values) > np.finfo(dtype).max):
        raise PcdError('Derived Z requires a representable floating-point Z field')
    if tuple(cloud.fields) != cloud.records.dtype.names:
        raise PcdError('Cloud fields no longer match the original record layout')
    records = cloud.records[selected].copy()
    records['z'] = values
    return _write_records(destination, cloud, records,
                          'planning-only derived Z; original XY and non-Z fields',
                          reset_viewpoint=True)


def _write_records(destination, cloud, records, description, *, reset_viewpoint=False):
    header = {key: list(values) for key, values in cloud.metadata["header"].items()}
    if reset_viewpoint:
        header['VIEWPOINT'] = ['0', '0', '0', '1', '0', '0', '0']
    header.update(WIDTH=[str(len(records))], HEIGHT=["1"], POINTS=[str(len(records))], DATA=["binary"])
    order = ("VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS")
    lines = ["# .PCD v0.7 - " + description]
    lines.extend(f"{key} {' '.join(header[key])}" for key in order)
    lines.extend(f"{key} {' '.join(value)}" for key, value in header.items() if key not in {*order, "DATA"})
    lines.append("DATA binary")
    header_bytes = ("\n".join(lines) + "\n").encode("ascii")
    payload = records.tobytes(order="C")
    with destination.open("xb") as stream:
        stream.write(header_bytes)
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    return list(cloud.fields)
