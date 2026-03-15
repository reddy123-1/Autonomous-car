"""
Math utilities for robot navigation and control.
Angles, distances, and clamping used across navigation and PID.
"""

import math
from typing import Tuple


def normalize_angle_deg(angle: float) -> float:
    """
    Normalize angle to [-180, 180] degrees.

    Args:
        angle: Angle in degrees (any range).

    Returns:
        Normalized angle in [-180, 180].
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def normalize_angle_rad(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] radians.

    Args:
        angle: Angle in radians.

    Returns:
        Normalized angle in [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def deg2rad(angle_deg: float) -> float:
    """Convert degrees to radians."""
    return math.radians(angle_deg)


def rad2deg(angle_rad: float) -> float:
    """Convert radians to degrees."""
    return math.degrees(angle_rad)


def angle_diff_deg(from_angle: float, to_angle: float) -> float:
    """
    Shortest angular difference from one angle to another (degrees).
    Positive = turn left (counter-clockwise), Negative = turn right (clockwise).

    Args:
        from_angle: Current heading in degrees.
        to_angle: Target heading in degrees.

    Returns:
        Difference in degrees, in [-180, 180].
    """
    return normalize_angle_deg(to_angle - from_angle)


def angle_diff_rad(from_angle: float, to_angle: float) -> float:
    """
    Shortest angular difference (radians). Returns value in [-pi, pi].
    """
    return normalize_angle_rad(to_angle - from_angle)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Euclidean distance between two 2D points.

    Args:
        x1, y1: First point (cm or any unit).
        x2, y2: Second point.

    Returns:
        Distance in same units.
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value to [min_val, max_val]. Handles min_val > max_val by swapping."""
    lo, hi = min(min_val, max_val), max(min_val, max_val)
    return max(lo, min(hi, value))


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation: (1-t)*a + t*b. t=0 -> a, t=1 -> b."""
    return (1.0 - t) * a + t * b


def heading_to_point_deg(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """
    Heading in degrees from (from_x, from_y) toward (to_x, to_y).
    Convention: 0 = +x, 90 = +y. Returns angle in [-180, 180].
    """
    dx = to_x - from_x
    dy = to_y - from_y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return normalize_angle_deg(rad2deg(math.atan2(dy, dx)))
