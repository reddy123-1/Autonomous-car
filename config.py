"""
Configuration for the autonomous robot car.
Raspberry Pi Zero 2 W - GPIO pins (BCM numbering).

All pin numbers use BCM (Broadcom) numbering. Ensure no conflicts with I2C
(GPIO 2, 3) when adding hardware.
"""

from typing import Tuple

# =============================================================================
# TB6612FNG Motor Driver Pins
# =============================================================================
# Left motor (Channel A): IN1/IN2 = direction, PWM = speed
MOTOR_LEFT_PWM = 12   # PWMA - GPIO 12
MOTOR_LEFT_IN1 = 5    # AIN1 - GPIO 5
MOTOR_LEFT_IN2 = 6    # AIN2 - GPIO 6

# Right motor (Channel B)
MOTOR_RIGHT_PWM = 13  # PWMB - GPIO 13
MOTOR_RIGHT_IN1 = 20  # BIN1 - GPIO 20
MOTOR_RIGHT_IN2 = 21  # BIN2 - GPIO 21

# Standby: drive HIGH to enable both channels; LOW = high-impedance (coast)
MOTOR_STBY = 16       # STBY - GPIO 16

# PWM frequency (Hz). 1 kHz is typical for TB6612FNG.
PWM_FREQUENCY = 1000

# Default motor speed (0-100 duty cycle) for forward/reverse/turn.
DEFAULT_MOTOR_SPEED = 60


# =============================================================================
# HC-SR04 Ultrasonic Sensors (front-left, front-center, front-right)
# =============================================================================
ULTRASONIC_LEFT_TRIG = 23   # Front-left trigger
ULTRASONIC_LEFT_ECHO = 24   # Front-left echo

ULTRASONIC_CENTER_TRIG = 25  # Front-center trigger
ULTRASONIC_CENTER_ECHO = 26  # Front-center echo

ULTRASONIC_RIGHT_TRIG = 27   # Front-right trigger
ULTRASONIC_RIGHT_ECHO = 22   # Front-right echo

# Obstacle detection: robot stops if any sensor reports distance < this (cm).
OBSTACLE_THRESHOLD_CM = 25.0

# Valid range for HC-SR04 (~2 cm to 400 cm). Readings outside are treated as invalid.
ULTRASONIC_MAX_CM = 400.0
ULTRASONIC_MIN_CM = 2.0

# Delay (seconds) between reading each sensor to reduce crosstalk.
ULTRASONIC_INTER_SENSOR_DELAY = 0.02

# Ultrasonic ray angles relative to robot heading (degrees): left, center, right (for dynamic mapping).
ULTRASONIC_ANGLES_DEG = (-25.0, 0.0, 25.0)


# =============================================================================
# Wheel Encoders (quadrature - 2 pins per encoder)
# =============================================================================
ENCODER_LEFT_A = 17   # Left encoder channel A
ENCODER_LEFT_B = 18   # Left encoder channel B

ENCODER_RIGHT_A = 19  # Right encoder channel A
ENCODER_RIGHT_B = 10  # Right encoder channel B

# Encoder parameters: adjust for your BO motor/encoder specs.
TICKS_PER_REVOLUTION = 20   # Total ticks per full wheel revolution (A+B edges)
WHEEL_DIAMETER_CM = 6.5     # Wheel diameter in cm
WHEEL_CIRCUMFERENCE_CM = 3.14159265359 * WHEEL_DIAMETER_CM
WHEEL_BASE_CM = 14.0        # Distance between left and right wheel centers (cm)


# =============================================================================
# MPU6050 Gyroscope (I2C)
# =============================================================================
MPU6050_I2C_ADDR = 0x68   # Default I2C address (AD0 pin low). Use 0x69 if AD0 high.
MPU6050_I2C_BUS = 1       # Raspberry Pi I2C bus 1 (GPIO 2 = SDA, GPIO 3 = SCL)

# Gyro sensitivity: LSB per (deg/s) for ±250 deg/s full scale = 131.0
GYRO_SCALE = 131.0

# Calibration: number of samples to average for zero-rate offset at startup.
GYRO_CALIBRATION_SAMPLES = 100


# =============================================================================
# PID Controller (for straight-line driving)
# =============================================================================
PID_KP = 0.5
PID_KI = 0.01
PID_KD = 0.05
PID_OUTPUT_LIMIT = 30  # Max correction (duty delta) applied to one motor

# Integral anti-windup: clamp integral term to avoid huge corrections after long error.
PID_INTEGRAL_LIMIT = 50.0


# =============================================================================
# Obstacle Avoidance
# =============================================================================
OBSTACLE_WAIT_MIN_SEC = 3.0
OBSTACLE_WAIT_MAX_SEC = 5.0
OBSTACLE_AVOIDANCE_TURN_CM = 30.0   # Approx distance (cm) for 90° turn in place
OBSTACLE_AVOIDANCE_FORWARD_CM = 45.0 # Forward distance after turning to pass obstacle
OBSTACLE_SIDE_THRESHOLD_CM = 40.0    # Min clearance (cm) on side to prefer that direction


# =============================================================================
# Grid Map & Navigation
# =============================================================================
CELL_SIZE_CM = 20.0   # Each grid cell represents CELL_SIZE_CM × CELL_SIZE_CM in world.
GRID_WIDTH = 15       # Grid width in cells
GRID_HEIGHT = 15      # Grid height in cells

WAYPOINT_REACHED_THRESHOLD_CM = 15.0  # Robot considered at waypoint within this distance (cm).
HEADING_TOLERANCE_DEG = 5.0           # Degrees: ignore small heading corrections below this.

# Turn-to-heading: max time (s) to spend turning before giving up.
TURN_TO_HEADING_TIMEOUT_SEC = 10.0
TURN_TO_HEADING_SPEED = 50.0

# Dynamic mapping: after replan, drive this many loop cycles before checking obstacle again.
NAV_REPLAN_COOLDOWN_STEPS = 25


# =============================================================================
# Logging
# =============================================================================
LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR
LOG_FILE = None     # Set to a path string to also log to file; None = stdout only.


def get_motor_pins() -> Tuple[Tuple[int, int, int], Tuple[int, int, int], int]:
    """
    Return motor pin configuration for debugging/documentation.
    Returns ((left_pwm, left_in1, left_in2), (right_pwm, right_in1, right_in2), stby).
    """
    return (
        (MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2),
        (MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2),
        MOTOR_STBY,
    )
