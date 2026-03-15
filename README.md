# Autonomous-car

Python codebase for a differential-drive autonomous robot running on **Raspberry Pi Zero 2 W**, with grid navigation, waypoint following, odometry, and obstacle avoidance.

## Hardware

| Component | Model |
|-----------|--------|
| Controller | Raspberry Pi Zero 2 W |
| Motor driver | TB6612FNG |
| Motors | 2× BO geared motors (differential drive) |
| Sensors | 3× HC-SR04 (front-left, front-center, front-right), MPU6050 gyro (I2C), wheel encoders |
| Power | Pi from power bank; motors from separate battery pack |

## Project structure

```
robot/
├── main.py              # Entry point: init, sensor threads, waypoint following
├── config.py            # GPIO pins (BCM), thresholds, PID, grid size
├── motors/
│   ├── tb6612fng_driver.py   # Low-level TB6612FNG PWM/direction
│   └── motor_controller.py   # Forward, reverse, turn left/right, stop
├── sensors/
│   ├── ultrasonic_sensor.py  # 3× HC-SR04, obstacle detection
│   ├── encoder.py            # Wheel encoders, distance per wheel
│   └── gyro_mpu6050.py       # MPU6050 yaw (heading)
├── navigation/
│   ├── grid_map.py           # 2D grid, obstacles, world↔grid
│   ├── astar.py              # A* pathfinding, path → waypoints (cm)
│   ├── odometry.py           # (x, y, heading) from encoders + gyro
│   ├── waypoint_navigation.py # Heading to waypoint, waypoint reached
│   └── obstacle_avoidance.py  # Wait 3–5 s, choose left/right, rejoin
├── control/
│   └── pid_controller.py     # PID to keep robot straight
└── utils/
    ├── logger.py             # Logging
    └── math_utils.py         # Angle normalize, distance, clamp
```

## GPIO (BCM) – Raspberry Pi Zero 2 W

- **TB6612FNG:** Left PWM=12, IN1=5, IN2=6; Right PWM=13, IN1=20, IN2=21; STBY=16  
- **Ultrasonic:** Left 23/24, Center 25/26, Right 27/22 (Trig/Echo)  
- **Encoders:** Left A/B=17/18, Right A/B=19/10  
- **MPU6050:** I2C bus 1 (SDA=GPIO2, SCL=GPIO3)

## Run

On the Pi, from the `robot/` directory:

```bash
cd robot
python main.py
```

Install dependencies first (on the Pi):

```bash
pip install -r requirements.txt
```

## Behaviour

1. **Grid & path:** Builds an example 15×15 grid with obstacles, runs A* from (0,0) to (12,12), converts path to waypoints in cm.  
2. **Waypoint following:** Drives toward the next waypoint using gyro heading and PID-corrected motor speeds; odometry updated from encoders.  
3. **Obstacle:** If any ultrasonic sees distance &lt; threshold: **stop** → **wait 3–5 s** → **recheck**. If clear, continue; if still blocked, **go around** (left or right by clearance) then **return to path**.  
4. **Shutdown:** Ctrl+C or SIGTERM stops motors and cleans up GPIO.

## Configuration

Edit `config.py` for:

- GPIO pins, PWM frequency, default speed  
- Ultrasonic obstacle threshold, encoder ticks per revolution, wheel size, wheel base  
- PID gains, obstacle wait range, waypoint threshold  

Example grid and goal are defined in `main.py` (`make_example_grid()`, `get_example_waypoints_cm()`).
