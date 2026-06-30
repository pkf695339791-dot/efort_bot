from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .config import RobotControlConfig
from .sdk_backend import RobotBackend, RobotBackendError


@dataclass
class ExecutionMonitor:
    config: RobotControlConfig
    backend: RobotBackend
    logger: logging.Logger

    def assert_ready(self) -> None:
        status = self.backend.read_status()
        if status.get("alarm"):
            raise RobotBackendError("robot has active alarm")
        if status.get("emergency_stop"):
            raise RobotBackendError("emergency stop is active")
        if self.config.require_servo_on and not status.get("servo_on"):
            raise RobotBackendError("servo is not on")

    def wait_until_idle(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.backend.read_status()
            if status.get("alarm") or status.get("emergency_stop"):
                raise RobotBackendError("robot entered unsafe state during execution")
            if not status.get("moving"):
                return
            time.sleep(self.config.monitor_poll_s)
        raise TimeoutError("robot did not become idle before timeout")
