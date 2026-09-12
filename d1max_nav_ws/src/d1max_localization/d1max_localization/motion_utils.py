from math import sqrt
from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]


class StationaryGyroCorrector:
    """Estimate startup gyro bias and suppress residual stationary yaw rate."""

    def __init__(
        self,
        required_samples: int,
        initialization_max_gyro: float,
        stationary_linear_speed: float,
        stationary_gyro_deadband: float,
    ):
        self.required_samples = max(1, int(required_samples))
        self.initialization_max_gyro = max(
            0.0, float(initialization_max_gyro)
        )
        self.stationary_linear_speed = max(
            0.0, float(stationary_linear_speed)
        )
        self.stationary_gyro_deadband = max(
            0.0, float(stationary_gyro_deadband)
        )
        self.bias: Vector3 = (0.0, 0.0, 0.0)
        self.sample_count = 0
        self.stationary_clamps = 0
        self._sum = [0.0, 0.0, 0.0]
        self.ready = False

    @staticmethod
    def _vector(values: Sequence[float]) -> Vector3:
        if len(values) != 3:
            raise ValueError("expected a 3-vector")
        return (float(values[0]), float(values[1]), float(values[2]))

    @staticmethod
    def _norm(vector: Vector3) -> float:
        return sqrt(sum(value * value for value in vector))

    def reset_initialization(self):
        if self.ready:
            return
        self.sample_count = 0
        self._sum = [0.0, 0.0, 0.0]

    def correct(
        self, velocity_values: Sequence[float], gyro_values: Sequence[float]
    ) -> Tuple[Vector3, Vector3, bool]:
        velocity = self._vector(velocity_values)
        gyro = self._vector(gyro_values)
        planar_speed = sqrt(
            velocity[0] * velocity[0] + velocity[1] * velocity[1]
        )

        if not self.ready:
            initialization_stationary = (
                planar_speed <= self.stationary_linear_speed
                and self._norm(gyro) <= self.initialization_max_gyro
            )
            if initialization_stationary:
                for index in range(3):
                    self._sum[index] += gyro[index]
                self.sample_count += 1
                if self.sample_count >= self.required_samples:
                    self.bias = tuple(
                        value / self.sample_count for value in self._sum
                    )  # type: ignore[assignment]
                    self.ready = True
            else:
                self.reset_initialization()

        corrected_gyro = tuple(
            gyro[index] - self.bias[index] for index in range(3)
        )
        stationary = (
            self.ready
            and planar_speed <= self.stationary_linear_speed
            and abs(corrected_gyro[2]) <= self.stationary_gyro_deadband
        )
        if stationary:
            corrected_gyro = (
                corrected_gyro[0],
                corrected_gyro[1],
                0.0,
            )
            velocity = (0.0, 0.0, velocity[2])
            self.stationary_clamps += 1
        return velocity, corrected_gyro, stationary
