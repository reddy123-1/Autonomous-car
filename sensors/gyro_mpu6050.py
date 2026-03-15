"""
MPU6050 gyroscope over I2C (smbus2).
Reads Z-axis angular velocity and integrates to estimate yaw (heading) in degrees.
Optional calibration at startup to remove zero-rate offset.
"""

import threading
import time
from typing import Optional

from smbus2 import SMBus

from config import (
    MPU6050_I2C_ADDR,
    MPU6050_I2C_BUS,
    GYRO_SCALE,
    GYRO_CALIBRATION_SAMPLES,
)
from utils.logger import log
from utils.math_utils import normalize_angle_deg

# MPU6050 register addresses
REG_PWR_MGMT_1 = 0x6B
REG_GYRO_CONFIG = 0x1B
REG_GYRO_XOUT_H = 0x43
REG_GYRO_YOUT_H = 0x45
REG_GYRO_ZOUT_H = 0x47
REG_WHO_AM_I = 0x75

# ±250 deg/s full scale -> 131 LSB/(deg/s)
GYRO_FS_250 = 0x00


class GyroMPU6050:
    """
    MPU6050 gyroscope: read raw Z rate, optionally calibrate zero offset,
    then integrate to get yaw (heading) in degrees [-180, 180].
    """

    def __init__(self) -> None:
        self._bus: Optional[SMBus] = None
        self._heading_deg = 0.0
        self._last_time: Optional[float] = None
        self._lock = threading.Lock()
        self._initialized = False
        self._zero_offset_dps = 0.0  # Calibrated Z zero-rate offset in deg/s

    def setup(self, calibrate: bool = True) -> None:
        """
        Initialize I2C, wake MPU6050, set gyro ±250 deg/s.
        If calibrate=True, average GYRO_CALIBRATION_SAMPLES for zero offset (robot should be still).
        """
        if self._initialized:
            return
        try:
            self._bus = SMBus(MPU6050_I2C_BUS)
            # Wake up (clear sleep bit)
            self._bus.write_byte_data(MPU6050_I2C_ADDR, REG_PWR_MGMT_1, 0)
            time.sleep(0.1)
            # Gyro full scale ±250 deg/s
            self._bus.write_byte_data(MPU6050_I2C_ADDR, REG_GYRO_CONFIG, (GYRO_FS_250 << 3) & 0xFF)
            time.sleep(0.05)
            self._last_time = time.time()

            if calibrate and GYRO_CALIBRATION_SAMPLES > 0:
                self._calibrate_zero_offset()

            self._initialized = True
            log.info("MPU6050 gyro initialized on I2C bus %s (addr=0x%02X), zero_offset=%.4f deg/s",
                     MPU6050_I2C_BUS, MPU6050_I2C_ADDR, self._zero_offset_dps)
        except Exception as e:
            log.error("MPU6050 setup failed: %s", e)

    def _calibrate_zero_offset(self) -> None:
        """Average Z gyro readings while robot is still to get zero-rate offset."""
        samples = []
        for _ in range(GYRO_CALIBRATION_SAMPLES):
            try:
                raw = self._read_raw_gyro_z()
                if raw is not None:
                    samples.append(raw / GYRO_SCALE)
            except Exception:
                pass
            time.sleep(0.01)
        if samples:
            self._zero_offset_dps = sum(samples) / len(samples)
            log.info("MPU6050 Z gyro calibration: %d samples, offset=%.4f deg/s", len(samples), self._zero_offset_dps)
        else:
            self._zero_offset_dps = 0.0

    def _read_raw_gyro_z(self) -> Optional[int]:
        """Read raw 16-bit signed Z gyro value. Returns None on error."""
        if not self._bus:
            return None
        try:
            high = self._bus.read_byte_data(MPU6050_I2C_ADDR, REG_GYRO_ZOUT_H)
            low = self._bus.read_byte_data(MPU6050_I2C_ADDR, REG_GYRO_ZOUT_H + 1)
            raw = (high << 8) | low
            if raw >= 0x8000:
                raw -= 0x10000
            return raw
        except Exception:
            return None

    def _read_gyro_z(self) -> float:
        """Read Z gyro in deg/s (minus calibrated offset)."""
        raw = self._read_raw_gyro_z()
        if raw is None:
            return 0.0
        return (raw / GYRO_SCALE) - self._zero_offset_dps

    def update_heading(self) -> float:
        """
        Read gyro Z, integrate, update internal heading.
        Returns current heading in degrees [-180, 180].
        """
        with self._lock:
            now = time.time()
            dt = (now - self._last_time) if self._last_time else 0.02
            self._last_time = now
            rate_z = self._read_gyro_z()
            self._heading_deg += rate_z * dt
            self._heading_deg = normalize_angle_deg(self._heading_deg)
            return self._heading_deg

    def get_heading_deg(self) -> float:
        """Current heading in degrees [-180, 180]."""
        with self._lock:
            return self._heading_deg

    def set_heading_deg(self, heading: float) -> None:
        """Set current heading (e.g. after calibration or known pose)."""
        with self._lock:
            self._heading_deg = normalize_angle_deg(heading)

    def reset_heading(self) -> None:
        """Set heading to 0."""
        self.set_heading_deg(0.0)

    def get_angular_velocity_z_dps(self) -> float:
        """Current Z angular velocity in deg/s (for debugging)."""
        return self._read_gyro_z()

    def shutdown(self) -> None:
        """Close I2C bus and mark uninitialized."""
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        self._initialized = False
        log.info("MPU6050 gyro shutdown")
