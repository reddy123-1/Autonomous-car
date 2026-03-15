"""
PID controller to correct motor speeds for straight-line driving.
Uses heading error (e.g. from gyro vs target waypoint) to adjust left/right motor difference.
"""

import time
from typing import Optional, Tuple

from config import (
    PID_KP,
    PID_KI,
    PID_KD,
    PID_OUTPUT_LIMIT,
    PID_INTEGRAL_LIMIT,
)
from utils.math_utils import clamp


class PIDController:
    """
    PID controller for heading correction. Output is a correction value:
    apply (base - correction) to left motor and (base + correction) to right
    when error is positive (need to turn left).
    """

    def __init__(
        self,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        output_limit: Optional[float] = None,
        integral_limit: Optional[float] = None,
    ) -> None:
        self.kp = kp if kp is not None else PID_KP
        self.ki = ki if ki is not None else PID_KI
        self.kd = kd if kd is not None else PID_KD
        self.output_limit = output_limit if output_limit is not None else PID_OUTPUT_LIMIT
        self.integral_limit = integral_limit if integral_limit is not None else PID_INTEGRAL_LIMIT
        self._integral = 0.0
        self._last_error: Optional[float] = None
        self._last_time: Optional[float] = None
        self._last_output: float = 0.0

    def reset(self) -> None:
        """Reset integral and last error/time. Call when starting a new segment."""
        self._integral = 0.0
        self._last_error = None
        self._last_time = None
        self._last_output = 0.0

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Compute PID correction from current error.

        Args:
            error: Current error (e.g. heading error in degrees; positive = turn left).
            dt: Time step in seconds. If None, uses elapsed time since last update or 0.02.

        Returns:
            Correction value: add to one motor, subtract from other (symmetric).
        """
        now = time.time()
        if dt is None:
            dt = (now - self._last_time) if self._last_time else 0.02
        self._last_time = now

        self._integral += error * dt
        self._integral = clamp(self._integral, -self.integral_limit, self.integral_limit)

        derivative = 0.0
        if self._last_error is not None:
            derivative = (error - self._last_error) / max(dt, 1e-6)
        self._last_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = clamp(output, -self.output_limit, self.output_limit)
        self._last_output = output
        return output

    def get_corrected_speeds(
        self,
        base_speed: float,
        heading_error_deg: float,
        dt: Optional[float] = None,
    ) -> Tuple[float, float]:
        """
        Return (left_speed, right_speed) to drive toward target heading.
        Positive heading_error = should turn left -> slow left, speed right.

        Args:
            base_speed: Base duty 0-100.
            heading_error_deg: Error in degrees (positive = turn left).
            dt: Optional time step for PID.

        Returns:
            (left_speed, right_speed) in [0, 100].
        """
        correction = self.update(heading_error_deg, dt)
        left = clamp(base_speed - correction, 0.0, 100.0)
        right = clamp(base_speed + correction, 0.0, 100.0)
        return (left, right)

    def get_last_output(self) -> float:
        """Last PID output (for logging/debugging)."""
        return self._last_output
