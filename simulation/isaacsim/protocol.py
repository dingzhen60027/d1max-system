"""Bounded, loopback-only wire format between Isaac Python and Humble Python.

Each measurement retains its original simulation timestamp. There is no ROS or
navigation implementation in this module. A partial ray scan is discarded.
"""
from array import array
import json
import math
import struct
import sys
import time

SCHEMA = 1
STATE_PORT = 18741
COMMAND_PORT = 18742
MAX_DATAGRAM = 60000
MAX_POINTS = 100000
POINTS_PER_CHUNK = 1024
MAX_PENDING_SCANS = 4
RAY_BINARY_MAGIC = b'D1R1'
NATIVE_RAY_PHASE = 'physx_prephysics_capture_v1'
NATIVE_RAY_FIELDS = ('native_frame_time_ns', 'native_frame_time_s', 'phase',
                     'native_frame_physics_step', 'capture_physics_step')


def encode(value):
    data = json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_DATAGRAM:
        raise ValueError("datagram_too_large")
    return data


def decode(data):
    if len(data) > MAX_DATAGRAM:
        raise ValueError("datagram_too_large")
    payload = None
    if data.startswith(RAY_BINARY_MAGIC):
        if len(data) < 6:
            raise ValueError('invalid_binary_ray_header')
        size = struct.unpack('!H',data[4:6])[0]
        if not 0 < size <= 1024 or len(data) < 6+size:
            raise ValueError('invalid_binary_ray_header')
        value = json.loads(data[6:6+size])
        payload = data[6+size:]
    else:
        value = json.loads(data)
    if not isinstance(value, dict) or value.get("schema") != SCHEMA:
        raise ValueError("unsupported_wire_schema")
    if value.get("type") not in ("state", "imu", "rays", "command", "status", "dynamic"):
        raise ValueError("unsupported_packet_type")
    epoch = value.get("epoch")
    if not isinstance(epoch, str) or not 1 <= len(epoch) <= 128:
        raise ValueError("invalid_epoch")
    if type(value.get("sim_time_ns")) is not int or not 0 <= value["sim_time_ns"] < 2**62:
        raise ValueError("invalid_source_time")
    if payload is not None:
        count, rings_present = value.get('point_count'),value.get('rings_present')
        if (value['type'] != 'rays' or type(count) is not int or not 1 <= count <= POINTS_PER_CHUNK
                or type(rings_present) is not bool or len(payload) != count*(14 if rings_present else 12)):
            raise ValueError('invalid_binary_ray_payload')
        xyz = array('f');xyz.frombytes(payload[:count*12])
        if sys.byteorder != 'little':xyz.byteswap()
        value['xyz'] = [list(xyz[i:i+3]) for i in range(0,len(xyz),3)]
        if rings_present:
            rings = array('H');rings.frombytes(payload[count*12:])
            if sys.byteorder != 'little':rings.byteswap()
            value['rings'] = list(rings)
    if value['type'] == 'rays':
        native_ray_metadata(value)
    return value


def finite_vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("invalid_vector_shape")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError("nonfinite_vector")
    return tuple(float(x) for x in value)


def native_ray_metadata(value):
    """Retain original wrapper END separately from actual capture BEGIN.

    The native float in seconds is serialized directly, never reconstructed
    from an integer tick. Optional physics-step identities travel together.
    The session's sealed physical dt is enforced by the receiving bridge.
    """
    metadata = {key: value[key] for key in NATIVE_RAY_FIELDS if key in value}
    if not metadata:
        return {}
    end_ns, end_s = value.get('native_frame_time_ns'), value.get('native_frame_time_s')
    begin_ns = value.get('sim_time_ns')
    if (value.get('phase') != NATIVE_RAY_PHASE
            or type(begin_ns) is not int or not 0 <= begin_ns < 2**62
            or type(end_ns) is not int or not 0 <= end_ns < 2**62
            or type(end_s) is not float or not math.isfinite(end_s) or not 0 <= end_s < 2**62 / 1e9
            or round(end_s * 1e9) != end_ns):
        raise ValueError('invalid_native_ray_phase_metadata')
    step_fields = ('native_frame_physics_step', 'capture_physics_step')
    if any(key in value for key in step_fields):
        native_step, capture_step = (value.get(key) for key in step_fields)
        if (type(native_step) is not int or type(capture_step) is not int
                or not 0 <= capture_step < native_step < 2**62
                or native_step != capture_step + 1):
            raise ValueError('invalid_native_ray_phase_metadata')
    return metadata


def state_packet(epoch, sequence, sim_time_ns, pose, linear_velocity_world, angular_velocity_world, metadata=None):
    value = dict(schema=SCHEMA, type="state", epoch=epoch,
        sequence=int(sequence), sim_time_ns=int(sim_time_ns),
        pose=finite_vector(pose, 7),
        linear_velocity_world=finite_vector(linear_velocity_world, 3),
        angular_velocity_world=finite_vector(angular_velocity_world, 3))
    if metadata is not None:
        allowed = {"static_prior_geometry_sha256", "static_prior_geometry_valid",
                   "static_prior_geometry_checked_sim_time_ns", "static_prior_geometry_fault",
                   "body_envelope_valid", "body_envelope_checked_sim_time_ns",
                   "body_envelope_registry_sha256", "body_envelope_fault"}
        if not isinstance(metadata, dict) or set(metadata) - allowed:
            raise ValueError("invalid_geometry_metadata")
        digest = metadata.get("static_prior_geometry_sha256")
        valid = metadata.get("static_prior_geometry_valid")
        checked = metadata.get("static_prior_geometry_checked_sim_time_ns")
        fault = metadata.get("static_prior_geometry_fault", "")
        if (type(valid) is not bool or not isinstance(digest, str)
                or (digest and (len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)))
                or (valid and not digest) or type(checked) is not int or not 0 <= checked <= sim_time_ns
                or not isinstance(fault, str) or len(fault) > 512):
            raise ValueError("invalid_geometry_metadata")
        if 'body_envelope_valid' in metadata:
            body_digest = metadata.get('body_envelope_registry_sha256', '')
            if (type(metadata['body_envelope_valid']) is not bool
                    or not isinstance(body_digest, str) or len(body_digest) != 64
                    or any(c not in '0123456789abcdef' for c in body_digest)
                    or metadata.get('body_envelope_checked_sim_time_ns') != sim_time_ns
                    or not isinstance(metadata.get('body_envelope_fault', ''), str)
                    or len(metadata.get('body_envelope_fault', '')) > 512):
                raise ValueError('invalid_body_envelope_metadata')
        value.update(metadata)
    return encode(value)


def imu_packet(epoch, sequence, sim_time_ns, orientation, angular_velocity, linear_acceleration):
    """Native PhysX IMU sample, body axes, SI angular rate and specific force."""
    return encode(dict(schema=SCHEMA, type="imu", epoch=epoch,
        sequence=int(sequence), sim_time_ns=int(sim_time_ns), sensor_frame="body",
        orientation=finite_vector(orientation, 4),
        angular_velocity=finite_vector(angular_velocity, 3),
        linear_acceleration=finite_vector(linear_acceleration, 3)))


def dynamic_packet(epoch, sequence, sim_time_ns, registry_sha256, samples):
    """Same-step kinematic actor measurements; never encoded as laser rays."""
    if (not isinstance(registry_sha256, str) or len(registry_sha256) != 64
            or any(c not in '0123456789abcdef' for c in registry_sha256)
            or not isinstance(samples, dict) or not 0 <= len(samples) <= 64):
        raise ValueError('invalid_dynamic_measurement_registry')
    checked = {}
    for actor_id, sample in samples.items():
        if (not isinstance(actor_id, str) or not 1 <= len(actor_id) <= 128
                or sample.get('present') is not True):
            raise ValueError('incomplete_dynamic_measurement')
        checked[actor_id] = dict(present=True,
            position=finite_vector(sample['position'], 3),
            linear_velocity=finite_vector(sample['linear_velocity'], 3),
            orientation_xyzw=finite_vector(sample.get('orientation_xyzw', [0., 0., 0., 1.]), 4))
    return encode(dict(schema=SCHEMA, type='dynamic', epoch=epoch,
        sequence=int(sequence), sim_time_ns=int(sim_time_ns),
        registry_sha256=registry_sha256, samples=checked))


def ray_packets(epoch, scan_sequence, sim_time_ns, sensor_id, origin, xyz, rings=None,
                *, native_frame_time_ns=None, native_frame_time_s=None, phase=None,
                native_frame_physics_step=None, capture_physics_step=None):
    """Emit finite actual hits; optional native wrapper END leaves BEGIN intact.

    With explicit prephysics phase, provide the original native time float and
    its rounded nanoseconds together. Physics-step identities are an optional
    pair; native END must be the next step after the actual capture BEGIN.
    """
    if sensor_id not in (0, 1):
        raise ValueError("invalid_sensor")
    origin = finite_vector(origin, 3)
    metadata = native_ray_metadata(dict(sim_time_ns=sim_time_ns, **{
        key: value for key, value in (
            ('native_frame_time_ns', native_frame_time_ns), ('native_frame_time_s', native_frame_time_s),
            ('phase', phase), ('native_frame_physics_step', native_frame_physics_step),
            ('capture_physics_step', capture_physics_step)) if value is not None}))
    array_input = hasattr(xyz,'dtype') and hasattr(xyz,'shape')
    if array_input:
        # The native sensor already provides an ndarray. Validate the complete
        # acquisition once and serialize contiguous measured f32 bytes directly.
        import numpy as np
        if (xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.dtype.kind not in 'fiu'):
            raise ValueError('invalid_ray_array')
        points = np.asarray(xyz,dtype='<f4',order='C')
        if not np.isfinite(points).all():
            raise ValueError('nonfinite_vector')
    else:
        points = [finite_vector(row, 3) for row in xyz]
    if not 0 < len(points) <= MAX_POINTS:
        raise ValueError("invalid_acquisition_size")
    if rings is not None:
        if array_input:
            rings = np.asarray(rings)
            if (rings.ndim!=1 or len(rings)!=len(points) or rings.dtype.kind not in 'iu'
                    or rings.min()<0 or rings.max()>=65536):
                raise ValueError('invalid_native_ring_indices')
            rings=np.asarray(rings,dtype='<u2',order='C')
        else:
            rings = list(rings)
            if len(rings)!=len(points) or any(type(r) is not int or not 0 <= r < 65536 for r in rings):
                raise ValueError("invalid_native_ring_indices")
    count = (len(points) + POINTS_PER_CHUNK - 1) // POINTS_PER_CHUNK
    for index in range(count):
        chunk=points[index*POINTS_PER_CHUNK:(index+1)*POINTS_PER_CHUNK]
        packet=dict(schema=SCHEMA, type="rays", epoch=epoch,
            scan_sequence=int(scan_sequence), sim_time_ns=int(sim_time_ns),
            sensor_id=sensor_id, origin=origin, chunk_index=index, chunk_count=count,
            point_count=len(chunk),rings_present=rings is not None)
        packet.update(metadata)
        # Match the eventual PointCloud2 XYZ precision without a JSON float
        # burst exceeding the host's capped UDP receive buffer. Acquisition
        # identity, source time and origin remain explicit JSON metadata.
        if array_input:
            payload=chunk.tobytes(order='C')
        else:
            xyz_data=array('f',(coordinate for point in chunk for coordinate in point))
            if sys.byteorder != 'little':xyz_data.byteswap()
            payload=xyz_data.tobytes()
        if rings is not None:
            ring_chunk=rings[index*POINTS_PER_CHUNK:(index+1)*POINTS_PER_CHUNK]
            if array_input:
                payload+=ring_chunk.tobytes(order='C')
            else:
                ring_data=array('H',ring_chunk)
                if sys.byteorder != 'little':ring_data.byteswap()
                payload+=ring_data.tobytes()
        header=encode(packet)
        if len(header)>1024:
            raise ValueError('binary_ray_header_too_large')
        yield RAY_BINARY_MAGIC+struct.pack('!H',len(header))+header+payload


def command_packet(epoch, sequence, sim_time_ns, vx, wz, valid_for_s=.15):
    vx, wz = finite_vector((vx, wz), 2)
    if not 0 < valid_for_s <= .25:
        raise ValueError("invalid_command_lifetime")
    return encode(dict(schema=SCHEMA, type="command", epoch=epoch,
        sequence=int(sequence), sim_time_ns=int(sim_time_ns),
        vx=vx, wz=wz, valid_for_s=valid_for_s))


class RayAssembler:
    """Depth-bounded datagram reassembly; expired/incomplete scans never publish."""
    def __init__(self, timeout_s=.15):
        self.timeout_s = timeout_s
        self.pending = {}
        self.completed = {}
        self.dropped = 0

    def add(self, packet, now=None):
        now = time.monotonic() if now is None else now
        for key, scan in list(self.pending.items()):
            if now-scan["began"] > self.timeout_s:
                del self.pending[key]
                self.dropped += 1
        sensor = packet["sensor_id"]
        sequence = packet["scan_sequence"]
        index, count = packet["chunk_index"], packet["chunk_count"]
        if (sensor not in (0, 1) or type(sequence) is not int or sequence < 0
                or type(index) is not int or type(count) is not int
                or not 0 <= index < count <= math.ceil(MAX_POINTS/POINTS_PER_CHUNK)):
            raise ValueError("invalid_ray_chunk")
        key = packet["epoch"], sensor, sequence
        last = self.completed.get(key[:2], -1)
        if sequence <= last:
            return None
        origin = finite_vector(packet["origin"], 3)
        rows = packet["xyz"]
        if not isinstance(rows, list) or not 0 < len(rows) <= POINTS_PER_CHUNK:
            raise ValueError("invalid_ray_payload")
        rows = [finite_vector(row, 3) for row in rows]
        rings = packet.get('rings')
        if rings is not None and (not isinstance(rings,list) or len(rings)!=len(rows)
                or any(type(r) is not int or not 0 <= r < 65536 for r in rings)):
            raise ValueError('invalid_native_ring_indices')
        try:
            metadata = native_ray_metadata(packet)
        except ValueError:
            if key in self.pending:
                del self.pending[key]
                self.dropped += 1
            raise
        native_identity = tuple((key, metadata[key]) for key in NATIVE_RAY_FIELDS if key in metadata)
        identity = (count, packet["sim_time_ns"], origin, rings is not None, native_identity)
        scan = self.pending.get(key)
        if scan is None:
            if len(self.pending) >= MAX_PENDING_SCANS:
                oldest = min(self.pending, key=lambda k: self.pending[k]["began"])
                del self.pending[oldest]
                self.dropped += 1
            scan = dict(began=now, identity=identity, chunks={}, ring_chunks={})
            self.pending[key] = scan
        if identity != scan["identity"]:
            del self.pending[key]
            self.dropped += 1
            raise ValueError("inconsistent_scan_chunks")
        if index in scan["chunks"] and (scan["chunks"][index] != rows
                or scan['ring_chunks'][index] != rings):
            del self.pending[key]
            self.dropped += 1
            raise ValueError("conflicting_scan_chunk")
        scan["chunks"][index] = rows
        scan['ring_chunks'][index] = rings
        if len(scan["chunks"]) != count:
            return None
        result = [row for i in range(count) for row in scan["chunks"][i]]
        del self.pending[key]
        if len(result) > MAX_POINTS:
            raise ValueError("acquisition_budget_exceeded")
        self.completed[key[:2]] = sequence
        return dict(epoch=key[0], sensor_id=sensor, scan_sequence=sequence,
            sim_time_ns=identity[1], origin=origin, xyz=result,
            rings=[ring for i in range(count) for ring in scan['ring_chunks'][i]]
                if identity[3] else None, **metadata)
