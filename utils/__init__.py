"""Utility modules for the robot: logging and math helpers."""

from .logger import setup_logger, log, log_exception
from .math_utils import (
    normalize_angle_deg,
    normalize_angle_rad,
    angle_diff_deg,
    angle_diff_rad,
    distance_2d,
    clamp,
    deg2rad,
    rad2deg,
    lerp,
    heading_to_point_deg,
)

__all__ = [
    "setup_logger",
    "log",
    "log_exception",
    "normalize_angle_deg",
    "normalize_angle_rad",
    "angle_diff_deg",
    "angle_diff_rad",
    "distance_2d",
    "clamp",
    "deg2rad",
    "rad2deg",
    "lerp",
    "heading_to_point_deg",
]
