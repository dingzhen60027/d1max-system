"""Complete-frame IPC outside the serialized navigation owner.

Connection.poll() means bytes are available, NOT that recv() cannot block.
Only the two dedicated I/O threads access the pipe. The owner sees bounded,
complete packets and never waits for the child to read/write a partial frame.
Wire encoding remains multiprocessing.Connection's existing pickle protocol;
the peer is our owned native child, never an untrusted network endpoint.
"""
from collections import deque
from multiprocessing.reduction import ForkingPickler
import socket
import threading


class BoundedWorkerChannel:
    MAX_REQUEST_BYTES = 4096
    MAX_REPLY_BYTES = 16 * 1024 * 1024
    REPLY_CAPACITY = 4  # warmup <=2, plan <=3; overflow is a protocol fault

    def __init__(self, connection, *, autostart=True):
        self.connection = connection
        self._condition = threading.Condition()
        self._replies = deque()
        self._outgoing = None
        self._closed = False
        self._error = None
        self._started = False
        self._fd_closed = False
        self._reader_done = threading.Event()
        self._writer_done = threading.Event()
        self._reader_done.set()
        self._writer_done.set()
        self._reader = threading.Thread(target=self._read, name='pct-ipc-reader', daemon=True)
        self._writer = threading.Thread(target=self._write, name='pct-ipc-writer', daemon=True)
        if autostart:
            self.start()

    def start(self):
        with self._condition:
            if self._started or self._closed:
                raise RuntimeError('native_channel_may_only_start_once')
            self._started = True
        try:
            self._reader_done.clear()
            try:
                self._reader.start()
            except Exception:
                self._reader_done.set()
                raise
            self._writer_done.clear()
            try:
                self._writer.start()
            except Exception:
                self._writer_done.set()
                raise
        except BaseException:
            self.close()
            raise

    def _fail(self, error, *, discard=False):
        with self._condition:
            if self._closed:
                return
            if self._error is None:
                self._error = error
            if discard:
                self._replies.clear()
            self._outgoing = None
            self._condition.notify_all()

    def _read(self):
        try:
            while True:
                encoded = self.connection.recv_bytes(self.MAX_REPLY_BYTES)
                packet = ForkingPickler.loads(encoded)
                with self._condition:
                    if self._closed or self._error is not None:
                        return
                    if len(self._replies) >= self.REPLY_CAPACITY:
                        self._fail(OSError('native_reply_queue_overflow'), discard=True)
                        return
                    self._replies.append(packet)
        except EOFError as error:
            # A terminal result can precede EOF. Deliver it once before EOF;
            # do not invent a successful result when no packet was received.
            self._fail(error)
        except Exception as error:
            self._fail(OSError('native_reply_frame_invalid: ' + str(error)[:160]), discard=True)
        finally:
            self._reader_done.set()
            self._close_if_retired()

    def _write(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._error is not None
                                             or self._outgoing is not None)
                    if self._closed or self._error is not None:
                        return
                    encoded, self._outgoing = self._outgoing, None
                self.connection.send_bytes(encoded)
        except Exception as error:
            self._fail(OSError('native_request_send_failed: ' + str(error)[:160]), discard=True)
        finally:
            self._writer_done.set()
            self._close_if_retired()

    def send(self, packet):
        encoded = ForkingPickler.dumps(packet)
        if len(encoded) > self.MAX_REQUEST_BYTES:
            raise OSError('native_request_frame_exceeds_bound')
        with self._condition:
            if self._closed:
                raise OSError('native_channel_closed')
            if self._error is not None:
                raise self._error
            if self._outgoing is not None:
                raise OSError('native_request_queue_full')
            self._outgoing = encoded
            self._condition.notify_all()

    def poll(self, timeout=0):
        if timeout != 0:
            raise ValueError('owner_channel_poll_must_not_wait')
        with self._condition:
            return bool(self._replies) or self._error is not None or self._closed

    def recv(self):
        with self._condition:
            if self._replies:
                return self._replies.popleft()
            if self._error is not None:
                raise self._error
            if self._closed:
                raise EOFError('native_channel_closed')
            raise BlockingIOError('complete_native_packet_not_ready')

    def close(self):
        # Linux duplex Pipe is a socket. Shutdown interrupts in-flight I/O
        # without releasing its FD for reuse by an unrelated resource. Actual
        # Connection.close happens only after both I/O owners leave. Process
        # retirement also fences replacement, without joining in callbacks.
        endpoint = None
        with self._condition:
            self._closed = True
            self._replies.clear()
            self._outgoing = None
            self._condition.notify_all()
            # Duplicate under the same lock used by final FD close: the integer
            # handle cannot be closed/reused between fileno() and fromfd().
            if not hasattr(self.connection, 'interrupt') and not self._fd_closed:
                try:
                    endpoint = socket.fromfd(self.connection.fileno(), socket.AF_UNIX, socket.SOCK_STREAM)
                except (OSError, ValueError):
                    pass
        if hasattr(self.connection, 'interrupt'):
            self.connection.interrupt()  # isolated test connection, no FD
        elif endpoint is not None:
            try:
                with endpoint:
                    endpoint.shutdown(socket.SHUT_RDWR)
            except OSError:
                # Child retirement still provides EOF. Never close a possibly
                # in-use descriptor as a fallback for shutdown failure.
                pass
        self._close_if_retired()

    def _close_if_retired(self):
        with self._condition:
            if (self._closed and not self._fd_closed
                    and self._reader_done.is_set() and self._writer_done.is_set()):
                self.connection.close()
                self._fd_closed = True

    @property
    def retirement_ready(self):
        return not self._reader.is_alive() and not self._writer.is_alive()

    @property
    def receive_finished(self):
        # Child exit can race with decoding its final complete packet. The
        # owner's existing goal/warmup deadline still bounds this drain.
        return not self._reader.is_alive()

    @property
    def failed(self):
        with self._condition:
            return self._closed or self._error is not None
