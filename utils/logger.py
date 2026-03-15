"""
Logging utility for the robot.
Provides consistent logging with timestamps, configurable level, and optional file output.
"""

import logging
import sys
from typing import Optional

try:
    from config import LOG_LEVEL, LOG_FILE
except ImportError:
    LOG_LEVEL = "INFO"
    LOG_FILE = None


def setup_logger(
    name: str = "robot",
    level: Optional[str] = None,
    log_file: Optional[str] = None,
) -> logging.Logger:
    """
    Create and configure a logger for the robot.

    Args:
        name: Logger name (e.g. "robot", "motors").
        level: Log level (DEBUG, INFO, WARNING, ERROR). Uses config.LOG_LEVEL if None.
        log_file: If set, also write logs to this file path. Uses config.LOG_FILE if None.

    Returns:
        Configured logger instance.
    """
    level_str = level or LOG_LEVEL
    log_path = log_file if log_file is not None else LOG_FILE
    level_value = getattr(logging, level_str.upper(), logging.INFO)

    logger = logging.getLogger(name)
    logger.setLevel(level_value)

    if logger.handlers:
        return logger

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Console handler
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level_value)
    console.setFormatter(formatter)
    logger.addHandler(console)

    # Optional file handler
    if log_path:
        try:
            fh = logging.FileHandler(log_path, encoding="utf-8")
            fh.setLevel(level_value)
            fh.setFormatter(formatter)
            logger.addHandler(fh)
        except OSError:
            logger.warning("Could not open log file %s", log_path)

    return logger


def log_exception(logger: logging.Logger, message: str = "Exception") -> None:
    """Log the current exception with traceback at ERROR level."""
    logger.exception(message)


# Module-level logger for convenience (used by other modules).
log = setup_logger("robot")
