"""Explicit new-package predictor; the sealed production entry is untouched."""
import signal
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from .lio_predictor import LioPredictor
from .estimation.causal_prediction import CausalInertialPredictor


class CausalLioPredictor(LioPredictor):
    def __init__(self):
        super().__init__()
        # No callbacks run before construction completes. Preserve the same
        # ROS subscriptions, parameters, timer and downstream state contract.
        self.core = CausalInertialPredictor(self.core.limits, self.core.odom, self.core.tracking)


def main(args=None):
    rclpy.init(args=args)
    node = CausalLioPredictor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError as error:
            if rclpy.ok() or 'rcl_shutdown already called' not in str(error):
                raise
