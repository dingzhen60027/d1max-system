"""Single bounded, killable native worker. Imports never start processes/ROS."""
import multiprocessing
import os
import time


def _run(connection, map_path, cache_dir, parameters, expected_map_sha256):
    # Must precede native imports in this spawn child; never change ROS thread
    # configuration or the user's middleware in the parent process.
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[name] = str(parameters['threads'])
    try:
        # NumPy can have been imported by spawn's __main__. Debian's BLAS
        # thread setter handles that case without a new optional dependency.
        import ctypes
        import ctypes.util
        for library in ('openblas', 'blas'):
            path = ctypes.util.find_library(library)
            if path:
                handle = ctypes.CDLL(path)
                setter = getattr(handle, 'openblas_set_num_threads', None)
                if setter is not None:
                    setter.argtypes = [ctypes.c_int]
                    setter(parameters['threads'])
                    break
        from .global_registration import GlobalMapIndex, RegistrationConfig
        index = GlobalMapIndex(map_path, cache_dir, RegistrationConfig(**parameters))
        if expected_map_sha256 and index.map_sha256!=expected_map_sha256:
            raise ValueError('localization_map_identity_changed')
        connection.send(dict(kind='index_ready', map_sha256=index.map_sha256,
            config_sha256=index.config_sha256, cache_hit=index.cache_hit, tiles=len(index.centers)))
        while True:
            request = connection.recv()
            if request is None:
                return
            result = index.search(request['points'], request['request_id'])
            # All failures (including early insufficient-geometry/deadline
            # exits) must carry identity so the owner can retire the request.
            result.update(request_id=request['request_id'],map_sha256=index.map_sha256,
                config_sha256=index.config_sha256)
            connection.send(dict(kind='result', **result))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            connection.send(dict(kind='error', reason=type(error).__name__ + ':' + str(error)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        connection.close()


class RegistrationWorker:
    def __init__(self, map_path, cache_dir, config, expected_map_sha256=None):
        from dataclasses import asdict
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_run, args=(child, str(map_path), str(cache_dir), asdict(config),expected_map_sha256),
            name='global-relocalization', daemon=True)
        self.config = config
        self.state = 'indexing'
        self.deadline = time.monotonic() + config.index_timeout_s
        self.request_id = None
        self.process.start()
        child.close()

    def submit(self, points, request_id):
        if self.state != 'ready':
            return False
        self.connection.send(dict(points=points, request_id=request_id))
        self.request_id = request_id
        self.state = 'searching'
        self.deadline = time.monotonic() + self.config.search_timeout_s
        return True

    def poll(self):
        if self.state in ('closed', 'failed'):
            return None
        try:
            if self.state in ('indexing', 'searching') and time.monotonic() >= self.deadline:
                reason = 'global_' + self.state + '_deadline'
                self.close()
                self.state = 'failed'
                return dict(kind='error', reason=reason)
            if self.connection.poll():
                result = self.connection.recv()
                if result['kind'] == 'index_ready':
                    self.state = 'ready'
                elif result['kind'] == 'result':
                    if result.get('request_id') != self.request_id:
                        return dict(kind='error', reason='worker_request_identity_mismatch')
                    self.state = 'ready'
                else:
                    self.close()
                    self.state = 'failed'
                return result
            if not self.process.is_alive():
                self.close()
                self.state = 'failed'
                return dict(kind='error', reason='global_worker_exited')
        except (EOFError, BrokenPipeError, OSError):
            self.close()
            self.state = 'failed'
            return dict(kind='error', reason='global_worker_channel_closed')
        return None

    def close(self):
        if self.state == 'closed':
            return
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=.1)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=.1)
        else:
            self.process.join(timeout=.1)
        self.connection.close()
        self.state = 'closed'
