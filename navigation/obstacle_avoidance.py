"""
Obstacle avoidance behavior:
- Stop if obstacle detected within threshold
- Wait 3–5 seconds (configurable), recheck sensors
- If obstacle moved -> continue
- If still present -> choose left or right by clearance, go around, return to path
"""

import random
import time
from typing import Dict, Any

from config import (
    OBSTACLE_WAIT_MIN_SEC,
    OBSTACLE_WAIT_MAX_SEC,
    OBSTACLE_THRESHOLD_CM,
    OBSTACLE_SIDE_THRESHOLD_CM,
)
from utils.logger import log


def wait_random_seconds() -> float:
    """
    Wait a random time between OBSTACLE_WAIT_MIN_SEC and OBSTACLE_WAIT_MAX_SEC.
    Returns actual wait duration in seconds.
    """
    duration = random.uniform(OBSTACLE_WAIT_MIN_SEC, OBSTACLE_WAIT_MAX_SEC)
    log.info("Obstacle wait %.1f s", duration)
    time.sleep(duration)
    return duration


def choose_avoidance_direction(left_cm: float, right_cm: float) -> str:
    """
    Choose 'left' or 'right' to go around obstacle.
    Prefer the side with more space (larger distance). Invalid readings (< 0) treated as blocked.
    If both sides below OBSTACLE_SIDE_THRESHOLD_CM, default to 'right'.
    """
    left_ok = left_cm < 0 or left_cm >= OBSTACLE_SIDE_THRESHOLD_CM
    right_ok = right_cm < 0 or right_cm >= OBSTACLE_SIDE_THRESHOLD_CM
    if left_ok and right_ok:
        # Prefer side with more space; treat -1 as very large
        left_val = left_cm if left_cm >= 0 else 999.0
        right_val = right_cm if right_cm >= 0 else 999.0
        return "right" if right_val >= left_val else "left"
    if left_ok:
        return "left"
    if right_ok:
        return "right"
    return "right"


def get_avoidance_plan(
    left_cm: float,
    center_cm: float,
    right_cm: float,
    threshold_cm: float = None,
) -> Dict[str, Any]:
    """
    Decide avoidance action from sensor readings.

    Args:
        left_cm, center_cm, right_cm: Distances in cm (-1 if invalid).
        threshold_cm: Obstacle threshold; uses config if None.

    Returns:
        Dict with keys: need_avoidance (bool), direction ('left'|'right'|None).
    """
    thresh = threshold_cm if threshold_cm is not None else OBSTACLE_THRESHOLD_CM
    obstacle = (
        (left_cm >= 0 and left_cm < thresh)
        or (center_cm >= 0 and center_cm < thresh)
        or (right_cm >= 0 and right_cm < thresh)
    )
    if not obstacle:
        return {"need_avoidance": False, "direction": None}
    direction = choose_avoidance_direction(left_cm, right_cm)
    return {"need_avoidance": True, "direction": direction}


def should_stop_for_obstacle(
    left_cm: float,
    center_cm: float,
    right_cm: float,
    threshold_cm: float = None,
) -> bool:
    """Convenience: True if any sensor reports distance below threshold."""
    plan = get_avoidance_plan(left_cm, center_cm, right_cm, threshold_cm)
    return plan["need_avoidance"]
