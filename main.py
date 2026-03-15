"""
Main entry point for the autonomous robot car.
Raspberry Pi Zero 2 W | TB6612FNG | 2x BO motors | 3x HC-SR04 | MPU6050 | Encoders.

Run from the robot/ directory: python main.py
"""

import os
import sys
import time
import signal
import threading
from typing import List, Tuple, Optional

# Ensure robot directory is on path when run from project root
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import RPi.GPIO as GPIO

from config import (
    OBSTACLE_THRESHOLD_CM,
    OBSTACLE_AVOIDANCE_TURN_CM,
    OBSTACLE_AVOIDANCE_FORWARD_CM,
    DEFAULT_MOTOR_SPEED,
    WAYPOINT_REACHED_THRESHOLD_CM,
    TURN_TO_HEADING_TIMEOUT_SEC,
    TURN_TO_HEADING_SPEED,
    NAV_REPLAN_COOLDOWN_STEPS,
)
from utils.logger import log
from utils.math_utils import angle_diff_deg
from motors.motor_controller import MotorController
from sensors.ultrasonic_sensor import UltrasonicSensors
from sensors.encoder import EncoderSystem
from sensors.gyro_mpu6050 import GyroMPU6050
from navigation.grid_map import GridMap
from navigation.astar import astar, path_to_waypoints_cm
from navigation.odometry import Odometry
from navigation.dynamic_map import update_grid_from_sensors
from navigation.waypoint_navigation import (
    waypoint_reached,
    heading_error_deg,
    next_waypoint_index,
)
from navigation.obstacle_avoidance import wait_random_seconds, get_avoidance_plan
from control.pid_controller import PIDController

# Goal in grid cells for A* replan
GOAL_GRID = (12, 12)


# -----------------------------------------------------------------------------
# Grid and path planning
# -----------------------------------------------------------------------------
def make_example_grid() -> GridMap:
    """Build a 15x15 grid with obstacles. More obstacles added dynamically when detected."""
    grid = GridMap()
    for gy in range(5, 10):
        grid.set_obstacle(7, gy)
        grid.set_obstacle(8, gy)
    for gx in range(3, 6):
        grid.set_obstacle(gx, 12)
    return grid


def plan_path(grid: GridMap, start_gx: int, start_gy: int) -> List[Tuple[float, float]]:
    """A* from (start_gx, start_gy) to GOAL_GRID; return waypoints in cm or empty list."""
    path = astar(grid, (start_gx, start_gy), GOAL_GRID, allow_diagonal=False)
    if not path:
        return []
    return path_to_waypoints_cm(grid, path)


# -----------------------------------------------------------------------------
# Robot application
# -----------------------------------------------------------------------------
class Robot:
    """Main robot: hardware init, sensor threads, waypoint following, obstacle handling."""

    def __init__(self) -> None:
        self.motors = MotorController()
        self.ultrasonic = UltrasonicSensors()
        self.encoders = EncoderSystem()
        self.gyro = GyroMPU6050()
        self.odometry = Odometry()
        self.pid = PIDController()
        self._shutdown_requested = False
        self._gyro_thread: Optional[threading.Thread] = None

    def setup(self) -> None:
        """Initialize all hardware. Call once before start_sensor_threads and follow_waypoints."""
        log.info("Initializing hardware...")
        self.motors.setup()
        self.ultrasonic.setup()
        self.encoders.setup()
        self.gyro.setup()
        self.odometry.set_pose(0.0, 0.0, self.gyro.get_heading_deg())
        log.info("Hardware initialized")

    def _gyro_update_loop(self) -> None:
        """Background thread: integrate gyro for heading at ~50 Hz."""
        while not self._shutdown_requested:
            self.gyro.update_heading()
            time.sleep(0.02)

    def start_sensor_threads(self) -> None:
        """Start background threads for sensors (e.g. gyro integration)."""
        self._gyro_thread = threading.Thread(target=self._gyro_update_loop, daemon=True)
        self._gyro_thread.start()
        log.info("Sensor threads started")

    def stop(self) -> None:
        """Stop motors and request shutdown."""
        self._shutdown_requested = True
        self.motors.stop()
        time.sleep(0.1)

    def shutdown(self) -> None:
        """Full cleanup: stop, join threads, motors, gyro, GPIO."""
        self.stop()
        if self._gyro_thread:
            self._gyro_thread.join(timeout=1.0)
        self.motors.shutdown()
        self.gyro.shutdown()
        GPIO.cleanup()
        log.info("Robot shutdown complete")

    def turn_to_heading_deg(
        self,
        target_heading_deg: float,
        speed: float = TURN_TO_HEADING_SPEED,
        tolerance_deg: float = 3.0,
        timeout_sec: float = TURN_TO_HEADING_TIMEOUT_SEC,
    ) -> bool:
        """
        Turn in place until current heading is within tolerance_deg of target.
        Returns True if reached, False if timeout or shutdown.
        """
        start = time.time()
        while not self._shutdown_requested and (time.time() - start) < timeout_sec:
            current = self.gyro.get_heading_deg()
            err = angle_diff_deg(current, target_heading_deg)
            if abs(err) <= tolerance_deg:
                self.motors.stop()
                return True
            if err > 0:
                self.motors.turn_left(speed)
            else:
                self.motors.turn_right(speed)
            self.gyro.update_heading()
            time.sleep(0.02)
        self.motors.stop()
        return False

    def check_obstacle_and_react(self) -> bool:
        """
        If obstacle detected: stop, wait 3–5 s, recheck.
        Returns True if still blocked (caller should run avoidance); False to continue.
        """
        left, center, right = self.ultrasonic.read_all()
        plan = get_avoidance_plan(left, center, right)
        if not plan["need_avoidance"]:
            return False
        log.warning("Obstacle detected L=%.1f C=%.1f R=%.1f cm", left, center, right)
        self.motors.stop()
        wait_random_seconds()
        left2, center2, right2 = self.ultrasonic.read_all()
        plan2 = get_avoidance_plan(left2, center2, right2)
        if not plan2["need_avoidance"]:
            log.info("Obstacle cleared, continuing")
            return False
        log.info("Obstacle still present, will go around (%s)", plan2["direction"])
        return True

    def run_avoidance(self, direction: str) -> None:
        """
        Execute go-around: turn 90° in direction, drive forward, turn 90° back toward path.
        Uses OBSTACLE_AVOIDANCE_TURN_CM and OBSTACLE_AVOIDANCE_FORWARD_CM from config.
        """
        self.encoders.reset_both()
        base = DEFAULT_MOTOR_SPEED * 0.6
        # Turn ~90 in chosen direction
        if direction == "left":
            self.motors.turn_left(base)
        else:
            self.motors.turn_right(base)
        self._drive_until_distance_cm(OBSTACLE_AVOIDANCE_TURN_CM)
        self.motors.stop()
        time.sleep(0.2)
        # Drive forward to pass obstacle
        self.encoders.reset_both()
        self.motors.forward(base)
        self._drive_until_distance_cm(OBSTACLE_AVOIDANCE_FORWARD_CM)
        self.motors.stop()
        time.sleep(0.2)
        # Turn back toward original path direction
        if direction == "left":
            self.motors.turn_right(base)
        else:
            self.motors.turn_left(base)
        self._drive_until_distance_cm(OBSTACLE_AVOIDANCE_TURN_CM)
        self.motors.stop()
        log.info("Avoidance maneuver complete")

    def _drive_until_distance_cm(self, target_cm: float) -> None:
        """Drive forward until either wheel has traveled at least target_cm (encoder-based)."""
        while not self._shutdown_requested:
            left_cm, right_cm = self.encoders.get_distances_cm()
            if left_cm >= target_cm or right_cm >= target_cm:
                break
            time.sleep(0.05)

    def follow_waypoints(
        self,
        waypoints_cm: List[Tuple[float, float]],
        grid: Optional[GridMap] = None,
    ) -> None:
        """
        Follow list of (x_cm, y_cm) waypoints using gyro heading and PID.
        If grid is provided: dynamic mapping – on obstacle, update grid, run A* from current
        position to goal, follow new path (with replan cooldown so the robot keeps moving).
        If grid is None: on obstacle, wait 3–5 s, recheck, then run_avoidance if still blocked.
        """
        if not waypoints_cm:
            log.warning("No waypoints to follow")
            return
        idx = 0
        base_speed = DEFAULT_MOTOR_SPEED
        last_encoder_left = 0.0
        last_encoder_right = 0.0
        replan_cooldown = 0
        self.pid.reset()
        self.encoders.reset_both()
        self.odometry.set_pose(0.0, 0.0, self.gyro.get_heading_deg())

        while idx < len(waypoints_cm) and not self._shutdown_requested:
            x, y, heading = self.odometry.get_pose()
            idx = next_waypoint_index(x, y, waypoints_cm, idx)
            if idx >= len(waypoints_cm):
                log.info("All waypoints reached")
                break
            wx, wy = waypoints_cm[idx]
            if waypoint_reached(x, y, wx, wy):
                idx += 1
                continue

            # After replanning, drive for a while before re-checking obstacle
            if replan_cooldown > 0:
                replan_cooldown -= 1
            elif self.ultrasonic.obstacle_detected(OBSTACLE_THRESHOLD_CM):
                if grid is not None:
                    # Dynamic mapping: update grid from sensors, replan A*, follow new path
                    self.motors.stop()
                    left_cm, center_cm, right_cm = self.ultrasonic.read_all()
                    added = update_grid_from_sensors(
                        grid, x, y, heading, left_cm, center_cm, right_cm,
                    )
                    gx, gy = grid.world_to_grid(x, y)
                    gx = max(0, min(grid.width - 1, gx))
                    gy = max(0, min(grid.height - 1, gy))
                    if not grid.is_free(gx, gy):
                        for (nx, ny) in grid.get_neighbors(gx, gy, allow_diagonal=True):
                            if grid.is_free(nx, ny):
                                gx, gy = nx, ny
                                break
                    new_waypoints = plan_path(grid, gx, gy)
                    if new_waypoints:
                        waypoints_cm = new_waypoints
                        idx = 0
                        replan_cooldown = NAV_REPLAN_COOLDOWN_STEPS
                        log.info("Obstacle: map updated (+%d cells), replanned %d waypoints", added, len(waypoints_cm))
                    else:
                        log.warning("Obstacle: no path to goal after update; trying avoidance")
                        if self.check_obstacle_and_react():
                            left_cm, center_cm, right_cm = self.ultrasonic.read_all()
                            plan = get_avoidance_plan(left_cm, center_cm, right_cm)
                            self.run_avoidance(plan["direction"])
                    continue
                # No grid: legacy behavior
                if self.check_obstacle_and_react():
                    left_cm, center_cm, right_cm = self.ultrasonic.read_all()
                    plan = get_avoidance_plan(left_cm, center_cm, right_cm)
                    self.run_avoidance(plan["direction"])
                continue

            # Heading correction and drive
            err_deg = heading_error_deg(heading, x, y, wx, wy)
            left_speed, right_speed = self.pid.get_corrected_speeds(base_speed, err_deg)
            self.motors.set_motor_speeds(left_speed, right_speed)
            left_cm, right_cm = self.encoders.get_distances_cm()
            delta_l = left_cm - last_encoder_left
            delta_r = right_cm - last_encoder_right
            last_encoder_left = left_cm
            last_encoder_right = right_cm
            self.gyro.update_heading()
            self.odometry.update_from_deltas(delta_l, delta_r, self.gyro.get_heading_deg())
            time.sleep(0.05)
        self.motors.stop()


# -----------------------------------------------------------------------------
# Entry point and safe shutdown
# -----------------------------------------------------------------------------
def main() -> None:
    robot = Robot()
    signal.signal(signal.SIGINT, lambda sig, frame: robot.stop())
    signal.signal(signal.SIGTERM, lambda sig, frame: robot.stop())

    try:
        robot.setup()
        robot.start_sensor_threads()
        grid = make_example_grid()
        waypoints = plan_path(grid, 0, 0)
        if not waypoints:
            log.error("No path from start to goal")
        else:
            log.info(
                "Dynamic mapping: %d waypoints, goal %s; start (%.0f, %.0f) -> end (%.0f, %.0f)",
                len(waypoints), GOAL_GRID, waypoints[0][0], waypoints[0][1], waypoints[-1][0], waypoints[-1][1],
            )
            robot.follow_waypoints(waypoints, grid=grid)
    except Exception as e:
        log.exception("Error: %s", e)
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()
