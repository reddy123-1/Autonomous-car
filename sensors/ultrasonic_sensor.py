"""
HC-SR04 ultrasonic distance sensor support.
Reads three sensors (front-left, front-center, front-right) with optional
inter-sensor delay to reduce crosstalk. Returns distance in cm or -1 if invalid.
"""

import time
from typing import List, Tuple

import RPi.GPIO as GPIO

from config import (
    ULTRASONIC_LEFT_TRIG,
    ULTRASONIC_LEFT_ECHO,
    ULTRASONIC_CENTER_TRIG,
    ULTRASONIC_CENTER_ECHO,
    ULTRASONIC_RIGHT_TRIG,
    ULTRASONIC_RIGHT_ECHO,
    ULTRASONIC_MAX_CM,
    ULTRASONIC_MIN_CM,
    OBSTACLE_THRESHOLD_CM,
    ULTRASONIC_INTER_SENSOR_DELAY,
)
from utils.logger import log

# Speed of sound ~343 m/s at 20°C → 34300 cm/s; round-trip so 1 cm ≈ 58.2 µs → 17150 cm/s
CM_PER_SEC = 17150.0
TRIGGER_PULSE_SEC = 0.00001   # 10 µs minimum trigger
ECHO_TIMEOUT_SEC = 0.02      # ~3.4 m max; timeout if no echo


def _read_one(trigger_pin: int, echo_pin: int) -> float:
    """
    Read one HC-SR04: trigger pulse, measure echo duration, convert to cm.
    Returns distance in cm, or -1.0 on timeout/out-of-range/error.
    """
    try:
        GPIO.output(trigger_pin, GPIO.LOW)
        time.sleep(0.00005)
        GPIO.output(trigger_pin, GPIO.HIGH)
        time.sleep(TRIGGER_PULSE_SEC)
        GPIO.output(trigger_pin, GPIO.LOW)

        start = time.time()
        while GPIO.input(echo_pin) == GPIO.LOW and (time.time() - start) < ECHO_TIMEOUT_SEC:
            pass
        rise = time.time()
        while GPIO.input(echo_pin) == GPIO.HIGH and (time.time() - rise) < ECHO_TIMEOUT_SEC:
            pass
        fall = time.time()
        duration = fall - rise
        distance_cm = duration * CM_PER_SEC
        if ULTRASONIC_MIN_CM <= distance_cm <= ULTRASONIC_MAX_CM:
            return round(distance_cm, 2)
        return -1.0
    except Exception as e:
        log.debug("Ultrasonic read error on trig=%s echo=%s: %s", trigger_pin, echo_pin, e)
        return -1.0


class UltrasonicSensors:
    """
    Manages three HC-SR04 sensors: front-left, front-center, front-right.
    read_all() optionally inserts a delay between sensors to reduce crosstalk.
    """

    def __init__(self) -> None:
        self._initialized = False
        self._pins: List[Tuple[int, int]] = [
            (ULTRASONIC_LEFT_TRIG, ULTRASONIC_LEFT_ECHO),
            (ULTRASONIC_CENTER_TRIG, ULTRASONIC_CENTER_ECHO),
            (ULTRASONIC_RIGHT_TRIG, ULTRASONIC_RIGHT_ECHO),
        ]

    def setup(self) -> None:
        """Initialize GPIO for all three sensors. Idempotent."""
        if self._initialized:
            return
        try:
            GPIO.setmode(GPIO.BCM)
            for trig, echo in self._pins:
                GPIO.setup(trig, GPIO.OUT)
                GPIO.setup(echo, GPIO.IN)
            self._initialized = True
            log.info("Ultrasonic sensors initialized (L=%s/%s, C=%s/%s, R=%s/%s)",
                     ULTRASONIC_LEFT_TRIG, ULTRASONIC_LEFT_ECHO,
                     ULTRASONIC_CENTER_TRIG, ULTRASONIC_CENTER_ECHO,
                     ULTRASONIC_RIGHT_TRIG, ULTRASONIC_RIGHT_ECHO)
        except Exception as e:
            log.error("Ultrasonic setup failed: %s", e)

    def read_left(self) -> float:
        """Distance in cm from front-left sensor. -1 if invalid."""
        return _read_one(self._pins[0][0], self._pins[0][1])

    def read_center(self) -> float:
        """Distance in cm from front-center sensor. -1 if invalid."""
        return _read_one(self._pins[1][0], self._pins[1][1])

    def read_right(self) -> float:
        """Distance in cm from front-right sensor. -1 if invalid."""
        return _read_one(self._pins[2][0], self._pins[2][1])

    def read_all(self, inter_sensor_delay: float = ULTRASONIC_INTER_SENSOR_DELAY) -> Tuple[float, float, float]:
        """
        Read all three sensors. Returns (left_cm, center_cm, right_cm). -1 for invalid.
        If inter_sensor_delay > 0, sleeps between each read to reduce crosstalk.
        """
        left = _read_one(self._pins[0][0], self._pins[0][1])
        if inter_sensor_delay > 0:
            time.sleep(inter_sensor_delay)
        center = _read_one(self._pins[1][0], self._pins[1][1])
        if inter_sensor_delay > 0:
            time.sleep(inter_sensor_delay)
        right = _read_one(self._pins[2][0], self._pins[2][1])
        return (left, center, right)

    def obstacle_detected(self, threshold_cm: float = None) -> bool:
        """
        True if any sensor sees an obstacle closer than threshold.
        Uses config OBSTACLE_THRESHOLD_CM if threshold_cm is None.
        Invalid readings (-1) are ignored (no obstacle assumed for that sensor).
        """
        thresh = threshold_cm if threshold_cm is not None else OBSTACLE_THRESHOLD_CM
        left, center, right = self.read_all()
        for d in (left, center, right):
            if d >= 0 and d < thresh:
                return True
        return False

    def get_min_distance(self) -> float:
        """Minimum valid distance across all three sensors. Returns -1.0 if all invalid."""
        left, center, right = self.read_all()
        valid = [d for d in (left, center, right) if d >= 0]
        return min(valid) if valid else -1.0

    def get_distances_dict(self) -> dict:
        """Return dict with keys 'left', 'center', 'right' and values in cm (-1 if invalid)."""
        left, center, right = self.read_all()
        return {"left": left, "center": center, "right": right}
