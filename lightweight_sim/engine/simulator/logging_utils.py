"""Run logging helpers for the lightweight simulator."""

from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional


_LOGGER: Optional[logging.Logger] = None
_LOG_PATH: Optional[Path] = None


def get_run_logger() -> logging.Logger:
    """Return the process-wide run logger, writing under ``temp/logs``."""

    global _LOGGER, _LOG_PATH
    if _LOGGER is not None:
        return _LOGGER

    root = Path(__file__).resolve().parents[2]
    log_dir = root / "temp" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    _LOG_PATH = log_dir / f"run_{datetime.now():%Y%m%d_%H%M%S_%f}.log"

    logger = logging.getLogger("lightweight_sim")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    handler = logging.FileHandler(_LOG_PATH, encoding="utf-8")
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    logger.addHandler(handler)
    _LOGGER = logger
    logger.info("run logger initialized; log_file=%s", _LOG_PATH)
    return logger


def get_log_path() -> Optional[Path]:
    """Return the current log path, initializing the logger if necessary."""

    get_run_logger()
    return _LOG_PATH
