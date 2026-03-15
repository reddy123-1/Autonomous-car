"""
High-level motor controller for differential drive: forward, reverse, turn, stop.
Uses TB6612FNG driver with PWM speed control. All speeds in 0-100 percent.
"""

from typing import Tuple, Optional

from .tb6612fng_driver import TB6612FNGDriver
from config import DEFAULT_MOTOR_SPEED
from utils.logger import log


class MotorController:
    """
    Differential drive motor controller. Left and right motors are controlled
    independently. Supports forward, reverse, in-place left/right turn, and stop.
    """

    def __init__(self) -> None:
        self._driver = TB6612FNGDriver()
        self._base_speed = float(DEFAULT_MOTOR_SPEED)
        self._left_speed = 0.0
        self._right_speed = 0.0

    def setup(self) -> None:
        """Initialize the motor driver. Call once before use."""
        self._driver.setup()

    def set_base_speed(self, speed: float) -> None:
        """Set base speed 0-100 for forward/reverse/turn when speed arg is None."""
        self._base_speed = max(0.0, min(100.0, speed))

    def set_motor_speeds(self, left: float, right: float) -> None:
        """
        Set left and right motor speeds directly.
        Sign: positive = forward, negative = reverse. Magnitude 0-100.

        Args:
            left: Left motor speed in [-100, 100].
            right: Right motor speed in [-100, 100].
        """
        left = max(-100.0, min(100.0, float(left)))
        right = max(-100.0, min(100.0, float(right)))
        self._driver.set_left_speed(abs(left), forward=(left >= 0))
        self._driver.set_right_speed(abs(right), forward=(right >= 0))
        self._left_speed = left
        self._right_speed = right

    def forward(self, speed: Optional[float] = None) -> None:
        """Drive forward. Speed 0-100; uses base speed if None."""
        s = speed if speed is not None else self._base_speed
        s = max(0.0, min(100.0, s))
        self._driver.set_both_speeds(s, s, True, True)
        self._left_speed = s
        self._right_speed = s
        log.debug("Forward at %.0f%%", s)

    def reverse(self, speed: Optional[float] = None) -> None:
        """Drive in reverse."""
        s = speed if speed is not None else self._base_speed
        s = max(0.0, min(100.0, s))
        self._driver.set_both_speeds(s, s, False, False)
        self._left_speed = -s
        self._right_speed = -s
        log.debug("Reverse at %.0f%%", s)

    def turn_left(self, speed: Optional[float] = None) -> None:
        """Turn in place to the left (left wheel reverse, right wheel forward)."""
        s = speed if speed is not None else self._base_speed
        s = max(0.0, min(100.0, s))
        self._driver.set_left_speed(s, False)
        self._driver.set_right_speed(s, True)
        self._left_speed = -s
        self._right_speed = s
        log.debug("Turn left at %.0f%%", s)

    def turn_right(self, speed: Optional[float] = None) -> None:
        """Turn in place to the right."""
        s = speed if speed is not None else self._base_speed
        s = max(0.0, min(100.0, s))
        self._driver.set_left_speed(s, True)
        self._driver.set_right_speed(s, False)
        self._left_speed = s
        self._right_speed = -s
        log.debug("Turn right at %.0f%%", s)

    def stop(self) -> None:
        """Stop both motors (coast)."""
        self._driver.stop_all()
        self._left_speed = 0.0
        self._right_speed = 0.0
        log.debug("Motors stop")

    def get_last_speeds(self) -> Tuple[float, float]:
        """Return last set (left_speed, right_speed)."""
        return (self._left_speed, self._right_speed)

    def is_stopped(self) -> bool:
        """True if both motors are at zero speed."""
        return self._left_speed == 0.0 and self._right_speed == 0.0

    def shutdown(self) -> None:
        """Stop motors and release driver resources."""
        self.stop()
        self._driver.shutdown()
