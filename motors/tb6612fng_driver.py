"""
Low-level driver for TB6612FNG dual DC motor driver.
Controls left and right motors via GPIO (direction) and PWM (speed).
Compatible with Raspberry Pi Zero 2 W; uses BCM pin numbering.
"""

import RPi.GPIO as GPIO
from typing import Optional

from config import (
    MOTOR_LEFT_PWM,
    MOTOR_LEFT_IN1,
    MOTOR_LEFT_IN2,
    MOTOR_RIGHT_PWM,
    MOTOR_RIGHT_IN1,
    MOTOR_RIGHT_IN2,
    MOTOR_STBY,
    PWM_FREQUENCY,
)
from utils.logger import log


class TB6612FNGDriver:
    """
    TB6612FNG motor driver: Channel A = left motor, Channel B = right motor.
    IN1/IN2 control direction; PWM duty cycle controls speed.
    STBY must be HIGH for motors to run.
    """

    def __init__(self) -> None:
        self._left_pwm: Optional[GPIO.PWM] = None
        self._right_pwm: Optional[GPIO.PWM] = None
        self._initialized: bool = False

    def setup(self) -> None:
        """Initialize GPIO and PWM for both motors. Idempotent."""
        if self._initialized:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Left motor (Channel A)
        GPIO.setup(MOTOR_LEFT_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_LEFT_IN2, GPIO.OUT)
        GPIO.setup(MOTOR_LEFT_PWM, GPIO.OUT)
        self._left_pwm = GPIO.PWM(MOTOR_LEFT_PWM, PWM_FREQUENCY)
        self._left_pwm.start(0)

        # Right motor (Channel B)
        GPIO.setup(MOTOR_RIGHT_IN1, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_IN2, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_PWM, GPIO.OUT)
        self._right_pwm = GPIO.PWM(MOTOR_RIGHT_PWM, PWM_FREQUENCY)
        self._right_pwm.start(0)

        # Standby: HIGH = motors enabled, LOW = high-impedance (coast)
        GPIO.setup(MOTOR_STBY, GPIO.OUT)
        GPIO.output(MOTOR_STBY, GPIO.HIGH)

        self._initialized = True
        log.info("TB6612FNG driver initialized (L: PWM=%s IN1=%s IN2=%s, R: PWM=%s IN1=%s IN2=%s, STBY=%s)",
                 MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2,
                 MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_STBY)

    def _set_left_direction(self, forward: bool) -> None:
        """Set left motor direction. True = forward, False = reverse."""
        if forward:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.LOW)
        else:
            GPIO.output(MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_LEFT_IN2, GPIO.HIGH)

    def _set_right_direction(self, forward: bool) -> None:
        """Set right motor direction. True = forward, False = reverse."""
        if forward:
            GPIO.output(MOTOR_RIGHT_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_RIGHT_IN2, GPIO.LOW)
        else:
            GPIO.output(MOTOR_RIGHT_IN1, GPIO.LOW)
            GPIO.output(MOTOR_RIGHT_IN2, GPIO.HIGH)

    def set_left_speed(self, speed_percent: float, forward: bool = True) -> None:
        """
        Set left motor speed and direction.

        Args:
            speed_percent: Duty cycle 0-100. Clamped to [0, 100].
            forward: True = forward, False = reverse.
        """
        if not self._initialized:
            return
        speed_percent = max(0.0, min(100.0, float(speed_percent)))
        self._set_left_direction(forward)
        if self._left_pwm:
            self._left_pwm.ChangeDutyCycle(speed_percent)

    def set_right_speed(self, speed_percent: float, forward: bool = True) -> None:
        """
        Set right motor speed and direction.

        Args:
            speed_percent: Duty cycle 0-100.
            forward: True = forward, False = reverse.
        """
        if not self._initialized:
            return
        speed_percent = max(0.0, min(100.0, float(speed_percent)))
        self._set_right_direction(forward)
        if self._right_pwm:
            self._right_pwm.ChangeDutyCycle(speed_percent)

    def set_both_speeds(
        self,
        left_speed: float,
        right_speed: float,
        left_forward: bool = True,
        right_forward: bool = True,
    ) -> None:
        """Set both motors. Speeds 0-100; direction per motor."""
        self.set_left_speed(left_speed, left_forward)
        self.set_right_speed(right_speed, right_forward)

    def stop_left(self) -> None:
        """Stop left motor (duty 0 = coast)."""
        if self._initialized and self._left_pwm:
            self._left_pwm.ChangeDutyCycle(0)

    def stop_right(self) -> None:
        """Stop right motor (duty 0 = coast)."""
        if self._initialized and self._right_pwm:
            self._right_pwm.ChangeDutyCycle(0)

    def stop_all(self) -> None:
        """Stop both motors."""
        self.stop_left()
        self.stop_right()
        log.debug("Motors stopped")

    def shutdown(self) -> None:
        """Stop motors, stop PWM, mark uninitialized. Does not GPIO.cleanup()."""
        self.stop_all()
        if self._left_pwm:
            try:
                self._left_pwm.stop()
            except Exception:
                pass
            self._left_pwm = None
        if self._right_pwm:
            try:
                self._right_pwm.stop()
            except Exception:
                pass
            self._right_pwm = None
        self._initialized = False
        log.info("TB6612FNG driver shutdown")

    @property
    def is_initialized(self) -> bool:
        """True if setup() has been called successfully."""
        return self._initialized
