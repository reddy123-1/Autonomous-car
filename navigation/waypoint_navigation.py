"""
Waypoint following: heading toward next waypoint, check if reached, next index.
"""

import math
from typing import List, Tuple, Optional

from utils.math_utils import angle_diff_deg, distance_2d, heading_to_point_deg
from config import WAYPOINT_REACHED_THRESHOLD_CM, HEADING_TOLERANCE_DEG


def get_heading_to_point(
    from_x: float,
    from_y: float,
    to_x: float,
    to_y: float,
) -> float:
    """
    Heading in degrees from (from_x, from_y) toward (to_x, to_y).
    Returns angle in [-180, 180]. 0 = +x, 90 = +y.
    """
    return heading_to_point_deg(from_x, from_y, to_x, to_y)


def waypoint_reached(
    x_cm: float,
    y_cm: float,
    wx_cm: float,
    wy_cm: float,
    threshold_cm: Optional[float] = None,
) -> bool:
    """True if robot is within threshold of waypoint."""
    thresh = threshold_cm if threshold_cm is not None else WAYPOINT_REACHED_THRESHOLD_CM
    return distance_2d(x_cm, y_cm, wx_cm, wy_cm) <= thresh


def distance_to_waypoint(
    x_cm: float,
    y_cm: float,
    wx_cm: float,
    wy_cm: float,
) -> float:
    """Euclidean distance from robot to waypoint in cm."""
    return distance_2d(x_cm, y_cm, wx_cm, wy_cm)


def heading_error_deg(
    current_heading_deg: float,
    from_x: float,
    from_y: float,
    to_x: float,
    to_y: float,
) -> float:
    """
    Shortest angle to turn so current heading points toward (to_x, to_y).
    Positive = turn left, negative = turn right. In [-180, 180].
    """
    target_heading = get_heading_to_point(from_x, from_y, to_x, to_y)
    return angle_diff_deg(current_heading_deg, target_heading)


def next_waypoint_index(
    x_cm: float,
    y_cm: float,
    waypoints: List[Tuple[float, float]],
    start_index: int = 0,
    threshold_cm: Optional[float] = None,
) -> int:
    """
    Index of the next waypoint to aim for. Skips waypoints already reached.
    Returns len(waypoints) when all have been reached (caller should treat as goal reached).
    """
    thresh = threshold_cm if threshold_cm is not None else WAYPOINT_REACHED_THRESHOLD_CM
    i = start_index
    while i < len(waypoints):
        wx, wy = waypoints[i]
        if not waypoint_reached(x_cm, y_cm, wx, wy, thresh):
            return i
        i += 1
    return i


def remaining_waypoints(
    waypoints: List[Tuple[float, float]],
    current_index: int,
) -> List[Tuple[float, float]]:
    """Return waypoints from current_index to end (for logging/display)."""
    if current_index >= len(waypoints):
        return []
    return waypoints[current_index:]


def heading_within_tolerance(
    current_heading_deg: float,
    target_heading_deg: float,
    tolerance_deg: Optional[float] = None,
) -> bool:
    """True if current heading is within tolerance of target."""
    tol = tolerance_deg if tolerance_deg is not None else HEADING_TOLERANCE_DEG
    err = abs(angle_diff_deg(current_heading_deg, target_heading_deg))
    return err <= tol
