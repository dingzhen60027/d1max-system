"""Causal input admission around the pinned inertial prediction core.

The release-pinned core remains intact. This version is selected explicitly
by the new startup-localization package and its isolated replay. The numerical
integration, original stamps and all age/gap/rotation bounds are unchanged.
"""
from collections import deque
import math
from .contracts import epoch_header
from .prediction import InertialPredictor


class CausalInertialPredictor(InertialPredictor):
    # InputClock accepts the same bounded sensor/ROS receipt phase difference.
    # This is only a queue-admission window, not an enlarged age, propagation
    # or collision lease. evaluate() below still removes every sample whose
    # ORIGINAL acquisition time is later than the requested output target.
    _receive_future_sec = 0.05

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pending_notices = 0
        self.last_source_notice = {}

    def accept(self, value, now):
        epoch, valid = epoch_header(value, now)
        reason=value.get('reason','waiting_lio')
        if not isinstance(reason,str) or len(reason)>256:
            if epoch>=self.epoch and value['fault']:
                self.fault='invalid_lio_fault_envelope'
            raise ValueError('invalid_lio_reason')
        scan_end = value.get('imu_quality_scan_end_sec')
        finite_scan_end=type(scan_end) in (float,int) and math.isfinite(scan_end)
        self.last_source_notice = dict(epoch=epoch, valid=valid, fault=value['fault'],
            reason=reason, received_at_unix=value['received_at_unix'],
            imu_quality_scan_end_sec=scan_end if finite_scan_end else None)
        # Native RobustStatus emits a non-fault, invalid "tracking" envelope
        # when its scan-end deadline expires. It is NOT a new posterior or a
        # reset. A independently bounded predictor may still propagate its
        # last genuine scan posterior. Admit only this exact native notice,
        # with the retained scan identity; never generalize to weak geometry,
        # sensor faults, initialization, a new epoch or an unknown producer.
        same_scan = (finite_scan_end
            and self.snapshot is not None and abs(scan_end-self.snapshot[0]) <= 1e-6)
        if (not valid and not value['fault'] and value.get('reason') == 'tracking'
                and value.get('imu_quality_scope') == 'last_admitted_scan'
                and epoch == self.epoch and self.valid and not self.fault and same_scan):
            self.pending_notices += 1
            # Leave source/publication/receipt, biases and every deadline
            # unchanged. evaluate() still expires the original posterior and
            # IMU on their own bounds, even with an endless stream of notices.
            return True
        return super().accept(value, now)

    def evaluate(self, now):
        # Receipt tolerates a small sensor/ROS-clock phase difference. A later
        # received sample still cannot support an earlier output target. Keep
        # it in the real bounded buffer for the next tick, without restamping.
        # Like the underlying core, this object has one serialized owner.
        received = self.imu
        self.imu = deque((s for s in received if s[0] <= now), maxlen=received.maxlen)
        try:
            return super().evaluate(now)
        finally:
            self.imu = received
