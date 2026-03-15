"""
Wheel encoder support for both motors.
Quadrature decoding: uses A and B channels to count ticks and infer direction.
Distance is derived from ticks per revolution and wheel circumference.
"""

import threading
from typing import Tuple

import RPi.GPIO as GPIO

from config import (
    ENCODER_LEFT_A,
    ENCODER_LEFT_B,
    ENCODER_RIGHT_A,
    ENCODER_RIGHT_B,
    TICKS_PER_REVOLUTION,
    WHEEL_CIRCUMFERENCE_CM,
)
from utils.logger import log


class WheelEncoder:
    """
    Single wheel quadrature encoder. Counts ticks; positive = forward (by convention).
    On rising edge of A: if B is low, count +1; if B is high, count -1.
    """

    def __init__(self, pin_a: int, pin_b: int, name: str = "") -> None:
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name or "encoder"
        self._ticks = 0
        self._lock = threading.Lock()
        self._last_a: int = -1  # 0 or 1 for last A state

    def _on_rising_a(self, channel: int) -> None:
        """Callback on rising edge of A: B low -> +1, B high -> -1 (quadrature)."""
        with self._lock:
            try:
                b_val = GPIO.input(self.pin_b)
                if b_val == GPIO.LOW:
                    self._ticks += 1
                else:
                    self._ticks -= 1
            except Exception:
                self._ticks += 1  # Fallback: count up only

    def setup(self) -> None:
        """Setup GPIO and edge detection on channel A. B used only in callback."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            self.pin_a,
            GPIO.RISING,
            callback=self._on_rising_a,
            bouncetime=2,
        )
        log.debug("Encoder %s setup (A=%s, B=%s)", self.name, self.pin_a, self.pin_b)

    def get_ticks(self) -> int:
        """Return current net tick count (signed)."""
        with self._lock:
            return self._ticks

    def reset(self) -> None:
        """Reset tick count to zero."""
        with self._lock:
            self._ticks = 0

    def get_revolutions(self) -> float:
        """Revolutions based on TICKS_PER_REVOLUTION (uses absolute tick count for distance)."""
        return abs(self._ticks) / max(1, TICKS_PER_REVOLUTION)

    def get_distance_cm(self) -> float:
        """Distance traveled in cm (absolute; sign of ticks not used for distance)."""
        rev = abs(self.get_ticks()) / max(1, TICKS_PER_REVOLUTION)
        return rev * WHEEL_CIRCUMFERENCE_CM

    def get_signed_distance_cm(self) -> float:
        """Signed distance in cm: positive = forward, negative = backward (by encoder direction)."""
        rev = self.get_ticks() / max(1, TICKS_PER_REVOLUTION)
        return rev * WHEEL_CIRCUMFERENCE_CM


class EncoderSystem:
    """
    Both wheel encoders: left and right. Provides combined ticks and distances.
    """

    def __init__(self) -> None:
        self.left = WheelEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B, "left")
        self.right = WheelEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B, "right")

    def setup(self) -> None:
        """Initialize both encoders."""
        self.left.setup()
        self.right.setup()
        log.info("Encoder system initialized (L: A=%s B=%s, R: A=%s B=%s)",
                 ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B)

    def get_ticks(self) -> Tuple[int, int]:
        """Return (left_ticks, right_ticks)."""
        return (self.left.get_ticks(), self.right.get_ticks())

    def get_distances_cm(self) -> Tuple[float, float]:
        """Return (left_cm, right_cm) as absolute distance traveled."""
        return (self.left.get_distance_cm(), self.right.get_distance_cm())

    def get_signed_distances_cm(self) -> Tuple[float, float]:
        """Return (left_cm, right_cm) signed by encoder direction."""
        return (self.left.get_signed_distance_cm(), self.right.get_signed_distance_cm())

    def reset_both(self) -> None:
        """Reset both encoder counts to zero."""
        self.left.reset()
        self.right.reset()
        log.debug("Encoders reset")
