"""Executed in-memory on NX: 20 seconds of IMU-only capture, no remote files."""
import ctypes
import json
import os
from pathlib import Path
import socket
import struct
import threading
import time

# Per-probe client configuration, not a modification of the installed router.
os.environ['ZENOH_CONFIG_OVERRIDE'] = ('mode="client";connect/endpoints=["tcp/127.0.0.1:7447"];'
    'scouting/multicast/enabled=false;scouting/gossip/enabled=false')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def udp_counters():
    lines = [x.split()[1:] for x in Path('/proc/net/snmp').read_text().splitlines() if x.startswith('Udp:')]
    return dict(zip(lines[0], map(int, lines[1])))


sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
sock.bind(('enx546c503e29e1', 0))  # Passive copy; never promiscuous, never binds UDP driver port.
sock.settimeout(.1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2*1024*1024)
sock.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS, kernel receive time.
class Filter(ctypes.Structure):
    _fields_ = [('code', ctypes.c_ushort), ('jt', ctypes.c_ubyte), ('jf', ctypes.c_ubyte), ('k', ctypes.c_uint)]
class Program(ctypes.Structure):
    _fields_ = [('len', ctypes.c_ushort), ('filter', ctypes.POINTER(Filter))]
# Ethernet IPv4 UDP, destination 6688. Drop all image/point-cloud/other traffic in kernel.
code = (Filter*10)(Filter(0x28,0,0,12), Filter(0x15,0,7,0x0800),
    Filter(0x30,0,0,23), Filter(0x15,0,5,17), Filter(0xb1,0,0,14),
    Filter(0x48,0,0,16), Filter(0x15,0,2,6688), Filter(0x06,0,0,65535),
    Filter(0x06,0,0,0), Filter(0x06,0,0,0))
program = Program(10, code)
libc = ctypes.CDLL(None, use_errno=True)
if libc.setsockopt(sock.fileno(), socket.SOL_SOCKET, 26, ctypes.byref(program), ctypes.sizeof(program)):
    raise OSError(ctypes.get_errno(), 'attach passive packet filter')

rclpy.init(args=[])
node = Node('d1max_nx_readonly_imu_probe', enable_rosout=False)
rows, packets = [], []
def imu(msg):
    if len(rows) < 10000:
        rows.append([msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec, time.time_ns()])
node.create_subscription(Imu, '/front_lidar/imu', imu,
    QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE))
started = time.monotonic()
before = udp_counters()
errors = []
def capture():
    try:
        while time.monotonic()-started < 20:
            try:
                data, anc, flags, addr = sock.recvmsg(2048, 256)
            except socket.timeout:
                continue
            iplen = (data[14]&15)*4
            offset = 14+iplen
            size = struct.unpack_from('!H', data, offset+4)[0]
            payload = data[offset+8:offset+size]
            if data[26:30] != socket.inet_aton('192.168.1.200') or len(payload) != 51:
                continue
            kernel = next((struct.unpack('ll', value) for level,kind,value in anc
                           if level == socket.SOL_SOCKET and kind == 35), None)
            stamp = kernel[0]*1000000000+kernel[1] if kernel else time.time_ns()
            if len(packets) < 10000:
                packets.append([stamp, payload.hex()])
    except Exception as exc:
        errors.append(type(exc).__name__+': '+str(exc))
worker = threading.Thread(target=capture)
worker.start()
try:
    while time.monotonic()-started < 20:
        rclpy.spin_once(node, timeout_sec=.01)
finally:
    worker.join(timeout=1.)
    stats = struct.unpack('II', sock.getsockopt(263, 6, 8))  # SOL_PACKET, PACKET_STATISTICS
    sock.close()
    node.destroy_node()
    rclpy.try_shutdown()
after = udp_counters()
print(json.dumps(dict(kind='READ_ONLY_NX_IMU_CAPTURE', duration=time.monotonic()-started,
    packet_capture_stats=dict(received=stats[0], dropped=stats[1]), errors=errors,
    udp_delta={k:after[k]-before[k] for k in before}, ros=rows, packets=packets)))
