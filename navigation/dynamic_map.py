"""
Dynamic mapping for the robot: update grid from ultrasonic readings, then replan with A*.
When the robot detects an obstacle, mark that cell on the grid (world position from pose + sensor angles + distance).
"""

import math
from typing import Tuple

from config import ULTRASONIC_ANGLES_DEG, CELL_SIZE_CM, ULTRASONIC_MAX_CM
from utils.math_utils import normalize_angle_deg
from .grid_map import GridMap


def update_grid_from_sensors(
    grid: GridMap,
    robot_x_cm: float,
    robot_y_cm: float,
    robot_heading_deg: float,
    left_cm: float,
    center_cm: float,
    right_cm: float,
    max_valid_cm: float = None,
) -> int:
    """
    Mark grid cells as obstacle where sensors detected something.
    Uses (robot pose + sensor angle + distance) to get world position, then grid cell.
    Returns number of new obstacle cells added.
    """
    max_valid_cm = max_valid_cm if max_valid_cm is not None else ULTRASONIC_MAX_CM
    angles = [
        normalize_angle_deg(robot_heading_deg + ULTRASONIC_ANGLES_DEG[0]),
        normalize_angle_deg(robot_heading_deg + ULTRASONIC_ANGLES_DEG[1]),
        normalize_angle_deg(robot_heading_deg + ULTRASONIC_ANGLES_DEG[2]),
    ]
    readings = [left_cm, center_cm, right_cm]
    robot_gx = int(round(robot_x_cm / grid.cell_size_cm))
    robot_gy = int(round(robot_y_cm / grid.cell_size_cm))
    added = 0
    for angle_deg, dist_cm in zip(angles, readings):
        if dist_cm < 0 or dist_cm >= max_valid_cm:
            continue
        rad = math.radians(angle_deg)
        ox = robot_x_cm + dist_cm * math.cos(rad)
        oy = robot_y_cm + dist_cm * math.sin(rad)
        gx = int(round(ox / grid.cell_size_cm))
        gy = int(round(oy / grid.cell_size_cm))
        if (gx, gy) == (robot_gx, robot_gy):
            continue
        if grid.in_bounds(gx, gy) and grid.is_free(gx, gy):
            grid.set_obstacle(gx, gy)
            added += 1
        gx2 = int(round((robot_x_cm + (dist_cm - CELL_SIZE_CM * 0.5) * math.cos(rad)) / grid.cell_size_cm))
        gy2 = int(round((robot_y_cm + (dist_cm - CELL_SIZE_CM * 0.5) * math.sin(rad)) / grid.cell_size_cm))
        if (gx2, gy2) != (gx, gy) and (gx2, gy2) != (robot_gx, robot_gy):
            if grid.in_bounds(gx2, gy2) and grid.is_free(gx2, gy2):
                grid.set_obstacle(gx2, gy2)
                added += 1
    return added
