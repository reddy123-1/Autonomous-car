"""
Odometry: track robot position (x, y) and heading using encoder deltas and gyro heading.
Differential drive: pose updated by (delta_left + delta_right)/2 in heading direction.
"""

import math
import threading
from typing import Tuple

from utils.math_utils import normalize_angle_deg


class Odometry:
    """
    Robot pose (x_cm, y_cm, heading_deg). Thread-safe.
    Update with encoder deltas and current gyro heading; or set pose explicitly.
    """

    def __init__(self) -> None:
        self._x_cm = 0.0
        self._y_cm = 0.0
        self._heading_deg = 0.0
        self._lock = threading.Lock()

    def update(
        self,
        left_cm: float,
        right_cm: float,
        heading_deg: float,
    ) -> None:
        """
        Update pose from left/right wheel distances (cumulative or since last update) and current heading.
        Treats (left_cm + right_cm)/2 as forward distance in heading direction.
        """
        with self._lock:
            self._heading_deg = normalize_angle_deg(heading_deg)
            dist_cm = (left_cm + right_cm) / 2.0
            rad = math.radians(self._heading_deg)
            self._x_cm += dist_cm * math.cos(rad)
            self._y_cm += dist_cm * math.sin(rad)

    def update_from_deltas(
        self,
        delta_left_cm: float,
        delta_right_cm: float,
        heading_deg: float,
    ) -> None:
        """
        Update pose from incremental left/right distances and current heading.
        Heading is taken from gyro; displacement is average of deltas in heading direction.
        """
        with self._lock:
            self._heading_deg = normalize_angle_deg(heading_deg)
            dist_cm = (delta_left_cm + delta_right_cm) / 2.0
            rad = math.radians(self._heading_deg)
            self._x_cm += dist_cm * math.cos(rad)
            self._y_cm += dist_cm * math.sin(rad)

    def set_pose(self, x_cm: float, y_cm: float, heading_deg: float) -> None:
        """Set pose explicitly (e.g. at start or after avoidance)."""
        with self._lock:
            self._x_cm = float(x_cm)
            self._y_cm = float(y_cm)
            self._heading_deg = normalize_angle_deg(heading_deg)

    def get_pose(self) -> Tuple[float, float, float]:
        """Return (x_cm, y_cm, heading_deg)."""
        with self._lock:
            return (self._x_cm, self._y_cm, self._heading_deg)

    def get_position(self) -> Tuple[float, float]:
        """Return (x_cm, y_cm)."""
        with self._lock:
            return (self._x_cm, self._y_cm)

    def get_heading_deg(self) -> float:
        """Current heading in degrees [-180, 180]."""
        with self._lock:
            return self._heading_deg
