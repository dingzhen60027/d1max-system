"""No ROS: partial IPC frames cannot occupy the navigation owner lane."""
import multiprocessing
from multiprocessing.reduction import ForkingPickler
import os
import struct
import threading
import time

import pytest

from d1max_pct_scan.bounded_worker_channel import BoundedWorkerChannel
from d1max_pct_scan.live_global_worker_cleanup import RetiredChildren


def wait_until(predicate):
    deadline = time.monotonic() + 2.
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.001)
    assert predicate()


def test_partial_header_has_no_complete_packet_and_cancel_does_not_wait():
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent)
    encoded = ForkingPickler.dumps({'kind': 'result', 'path': list(range(10000))})
    os.write(peer.fileno(), struct.pack('!i', len(encoded)))
    assert not channel.poll(0)
    with pytest.raises(BlockingIOError):
        channel.recv()
    channel.close()  # no complete payload, and still returns
    peer.close()
    wait_until(lambda: channel.retirement_ready)


def test_wire_protocol_ordering_and_eof_do_not_drop_complete_result():
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent)
    channel.send({'kind': 'plan', 'generation': 3})
    assert peer.recv() == {'kind': 'plan', 'generation': 3}
    peer.send({'kind': 'progress', 'generation': 3})
    peer.send({'kind': 'result', 'generation': 3, 'xyz': [[0., 1., 2.]]})
    peer.close()
    wait_until(channel.poll)
    wait_until(lambda: not channel._reader.is_alive())
    assert channel.recv()['kind'] == 'progress'
    assert channel.recv()['kind'] == 'result'
    with pytest.raises(EOFError):
        channel.recv()
    channel.close()
    wait_until(lambda: channel.retirement_ready)


class GatedConnection:
    def __init__(self):
        self.read_gate = threading.Event()
        self.write_gate = threading.Event()
        self.writing = threading.Event()
        self.sent = []

    def recv_bytes(self, maxlength):
        self.read_gate.wait()
        raise EOFError()

    def send_bytes(self, value):
        self.writing.set()
        self.write_gate.wait()
        self.sent.append(ForkingPickler.loads(value))

    def interrupt(self):
        self.read_gate.set()
        self.write_gate.set()

    def close(self):
        pass


def test_blocked_peer_has_one_pending_request_and_no_unbounded_sender_queue():
    peer = GatedConnection()
    channel = BoundedWorkerChannel(peer)
    channel.send({'generation': 1})
    assert peer.writing.wait(1.)
    channel.send({'generation': 2})  # one active write, one queued
    with pytest.raises(OSError, match='queue_full'):
        channel.send({'generation': 3})
    channel.close()
    wait_until(lambda: channel.retirement_ready)
    assert all(packet['generation'] != 2 for packet in peer.sent)


def test_oversize_outgoing_and_reply_overflow_fail_closed():
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent)
    with pytest.raises(OSError, match='exceeds_bound'):
        channel.send({'data': 'x' * 5000})
    for index in range(5):
        peer.send({'kind': 'result', 'generation': index})
    wait_until(lambda: not channel._reader.is_alive())
    with pytest.raises(OSError, match='overflow'):
        channel.recv()
    channel.close()
    peer.close()
    wait_until(lambda: channel.retirement_ready)


def test_oversize_reply_header_is_rejected_before_receiving_its_payload():
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent)
    os.write(peer.fileno(), struct.pack('!i', channel.MAX_REPLY_BYTES + 1))
    wait_until(channel.poll)
    with pytest.raises(OSError, match='frame_invalid'):
        channel.recv()
    channel.close()
    peer.close()
    wait_until(lambda: channel.retirement_ready)


@pytest.mark.parametrize('packet', [b'invalid pickle', struct.pack('!i', 0)])
def test_malformed_reply_does_not_deliver_partial_data(packet):
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent)
    peer.send_bytes(packet)
    wait_until(channel.poll)
    with pytest.raises(OSError, match='frame_invalid'):
        channel.recv()
    channel.close()
    peer.close()
    wait_until(lambda: channel.retirement_ready)


def test_retirement_prevents_new_worker_until_io_threads_have_retired():
    from test_live_global_worker_cleanup import FakeChild
    class Channel:
        retirement_ready = False
        def close(self): pass
    child, channel = FakeChild(alive=False), Channel()
    retired = RetiredChildren()
    retired.retire(child, channel)
    assert retired.reap() == 1
    channel.retirement_ready = True
    assert retired.reap() == 0
    assert not retired._channels


def test_child_exit_waits_for_complete_reply_decode_without_waiting_in_owner(monkeypatch):
    parent, peer = multiprocessing.Pipe()
    decoding, release = threading.Event(), threading.Event()
    original = ForkingPickler.loads
    def gated_loads(value):
        decoding.set()
        release.wait(1.)
        return original(value)
    monkeypatch.setattr(ForkingPickler, 'loads', gated_loads)
    channel = BoundedWorkerChannel(parent)
    peer.send({'kind': 'planned'})
    peer.close()
    assert decoding.wait(1.)
    assert not channel.receive_finished and not channel.poll(0)
    release.set()
    wait_until(lambda: channel.receive_finished)
    assert channel.recv() == {'kind': 'planned'}
    channel.close()
    wait_until(lambda: channel.retirement_ready)


def test_partial_thread_start_failure_is_closed_and_owned_until_reader_retires(monkeypatch):
    from test_live_global_worker_cleanup import FakeChild
    parent, peer = multiprocessing.Pipe()
    channel = BoundedWorkerChannel(parent, autostart=False)
    monkeypatch.setattr(channel._writer, 'start', lambda: (_ for _ in ()).throw(RuntimeError('no thread')))
    with pytest.raises(RuntimeError, match='no thread'):
        channel.start()
    assert channel.failed
    cleanup = RetiredChildren()
    child = FakeChild(alive=False)
    cleanup.retire(child, channel)
    peer.close()
    wait_until(lambda: channel.retirement_ready)
    assert cleanup.reap() == 0


def test_close_keeps_fd_owned_until_decoder_has_left(monkeypatch):
    parent, peer = multiprocessing.Pipe()
    decoding, release = threading.Event(), threading.Event()
    original = ForkingPickler.loads
    def gated_loads(value):
        decoding.set()
        release.wait(1.)
        return original(value)
    monkeypatch.setattr(ForkingPickler, 'loads', gated_loads)
    channel = BoundedWorkerChannel(parent)
    peer.send({'kind': 'planned'})
    assert decoding.wait(1.)
    channel.close()
    assert not parent.closed  # descriptor cannot be reused mid-read/decode
    release.set()
    wait_until(lambda: channel.retirement_ready)
    assert parent.closed and not channel._replies
    peer.close()


def test_interrupt_after_actual_thread_start_cannot_forge_retirement(monkeypatch):
    parent, peer = multiprocessing.Pipe()
    decoding, release = threading.Event(), threading.Event()
    original_loads = ForkingPickler.loads
    def gated_loads(value):
        decoding.set()
        release.wait(1.)
        return original_loads(value)
    monkeypatch.setattr(ForkingPickler, 'loads', gated_loads)
    channel = BoundedWorkerChannel(parent, autostart=False)
    original_start = channel._reader.start
    def start_then_interrupt():
        original_start()
        assert decoding.wait(1.)
        raise KeyboardInterrupt()
    monkeypatch.setattr(channel._reader, 'start', start_then_interrupt)
    peer.send({'kind': 'planned'})
    with pytest.raises(KeyboardInterrupt):
        channel.start()
    assert not channel._reader_done.is_set()
    assert not parent.closed
    release.set()
    wait_until(lambda: channel.retirement_ready)
    assert parent.closed
    peer.close()


def test_repeated_partial_frame_cancel_does_not_accumulate_fds_or_io_threads():
    before = len(os.listdir('/proc/self/fd'))
    for _ in range(100):
        parent, peer = multiprocessing.Pipe()
        channel = BoundedWorkerChannel(parent)
        os.write(peer.fileno(), struct.pack('!i', 1000))
        channel.close()
        peer.close()
        wait_until(lambda: channel.retirement_ready)
        assert parent.closed
    assert len(os.listdir('/proc/self/fd')) == before
