from __future__ import annotations

import ctypes
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from .config import RobotControlConfig
from .models import QueuePoint


class RobotBackend(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def prepare(self) -> None: ...
    def set_int(self, index: int, value: int) -> None: ...
    def get_int(self, index: int) -> int: ...
    def set_bool(self, index: int, value: bool) -> None: ...
    def get_bool(self, index: int) -> bool: ...
    def set_pointc_vector(self, points: list[QueuePoint]) -> None: ...
    def read_status(self) -> dict[str, object]: ...


class RobotBackendError(RuntimeError):
    pass


@dataclass
class DryRunBackend:
    config: RobotControlConfig
    logger: logging.Logger
    bools: dict[int, bool] = field(default_factory=dict)
    ints: dict[int, int] = field(default_factory=dict)
    last_vector: list[QueuePoint] = field(default_factory=list)
    connected: bool = False

    def connect(self) -> None:
        self.connected = True
        self.logger.info("dry-run robot connected ip=%s", self.config.controller_ip)

    def disconnect(self) -> None:
        self.connected = False
        self.logger.info("dry-run robot disconnected")

    def prepare(self) -> None:
        self._ensure_connected()
        self.logger.info(
            "dry-run prepare tool=%s workobject=%s",
            self.config.tool_name,
            self.config.workobject_name,
        )

    def set_int(self, index: int, value: int) -> None:
        self._ensure_connected()
        self.ints[index] = value
        self.logger.info("dry-run set PC_INT[%s]=%s", index, value)

    def get_int(self, index: int) -> int:
        self._ensure_connected()
        return self.ints.get(index, 0)

    def set_bool(self, index: int, value: bool) -> None:
        self._ensure_connected()
        self.bools[index] = value
        self.logger.info("dry-run set PC_BOOL[%s]=%s", index, value)

    def get_bool(self, index: int) -> bool:
        self._ensure_connected()
        return self.bools.get(index, False)

    def set_pointc_vector(self, points: list[QueuePoint]) -> None:
        self._ensure_connected()
        self.last_vector = list(points)
        self.logger.info("dry-run accepted PointC vector length=%s", len(points))

    def read_status(self) -> dict[str, object]:
        self._ensure_connected()
        return {
            "connected": self.connected,
            "alarm": False,
            "emergency_stop": False,
            "servo_on": True,
            "moving": False,
        }

    def _ensure_connected(self) -> None:
        if not self.connected:
            raise RobotBackendError("dry-run backend is not connected")


@dataclass
class EfortSdkBackend:
    config: RobotControlConfig
    logger: logging.Logger
    api: object | None = None
    dev_id: ctypes.c_uint = field(default_factory=lambda: ctypes.c_uint(0))

    def connect(self) -> None:
        self._load_sdk()
        assert self.api is not None
        result = self.api.connect_robot(
            self.config.controller_ip.encode("utf-8"),
            ctypes.byref(self.dev_id),
            {"reCntFlag": True, "reCntMaxNum": 2, "verCheck": False},
        )
        self._check(result, "connect_robot")
        self.logger.info("connected robot ip=%s id=%s", self.config.controller_ip, self.dev_id.value)

    def disconnect(self) -> None:
        if self.api is not None and self.dev_id.value != 0:
            self.api.dis_connect_robot(ctypes.byref(self.dev_id))
            self.logger.info("disconnected robot")

    def prepare(self) -> None:
        assert self.api is not None
        self._check(self.api.enable_api_control(True, self.dev_id), "enable_api_control")
        self._check(
            self.api.set_current_tool_byname(
                self.config.tool_name.encode("utf-8"),
                self.dev_id,
            ),
            "set_current_tool_byname",
        )
        self._check(
            self.api.set_current_uframe_byname(
                self.config.workobject_name.encode("utf-8"),
                self.dev_id,
            ),
            "set_current_uframe_byname",
        )

        status = self.read_status()
        if status["alarm"]:
            raise RobotBackendError("robot has active alarm")
        if status["emergency_stop"]:
            raise RobotBackendError("robot emergency stop is active")
        if self.config.require_servo_on and not status["servo_on"]:
            raise RobotBackendError("servo is not on")

    def set_int(self, index: int, value: int) -> None:
        assert self.api is not None
        self._check(self.api.set_int_variable(index, int(value), self.dev_id), f"set_int_variable[{index}]")

    def get_int(self, index: int) -> int:
        assert self.api is not None
        value = ctypes.c_int(0)
        self._check(self.api.get_int_variable(index, ctypes.byref(value), self.dev_id), f"get_int_variable[{index}]")
        return int(value.value)

    def set_bool(self, index: int, value: bool) -> None:
        assert self.api is not None
        self._check(self.api.set_bool_variable(index, bool(value), self.dev_id), f"set_bool_variable[{index}]")

    def get_bool(self, index: int) -> bool:
        assert self.api is not None
        value = ctypes.c_bool(False)
        self._check(self.api.get_bool_variable(index, ctypes.byref(value), self.dev_id), f"get_bool_variable[{index}]")
        return bool(value.value)

    def set_pointc_vector(self, points: list[QueuePoint]) -> None:
        assert self.api is not None
        point_type = self._pointc_type()
        vector = (point_type * len(points))()
        for idx, item in enumerate(points):
            vector[idx] = self._to_pointc(item, idx)
        self._check(self.api.set_pointc_vector(vector, len(points), False, self.dev_id), "set_pointc_vector")

    def read_status(self) -> dict[str, object]:
        assert self.api is not None
        alarm = ctypes.c_bool(False)
        emg = ctypes.c_bool(False)
        servo = ctypes.c_bool(False)
        moving = ctypes.c_bool(False)
        self._check(self.api.get_current_alarm_status(ctypes.byref(alarm), self.dev_id), "get_current_alarm_status")
        self._check(self.api.get_current_emg_status(ctypes.byref(emg), self.dev_id), "get_current_emg_status")
        self._check(self.api.get_current_servo_status(ctypes.byref(servo), self.dev_id), "get_current_servo_status")
        self._check(self.api.get_move_state(ctypes.byref(moving), self.dev_id), "get_move_state")
        return {
            "connected": True,
            "alarm": bool(alarm.value),
            "emergency_stop": bool(emg.value),
            "servo_on": bool(servo.value),
            "moving": bool(moving.value),
        }

    def _load_sdk(self) -> None:
        sdk_py = self._resolve_path(self.config.sdk_python_path)
        if str(sdk_py) not in sys.path:
            sys.path.insert(0, str(sdk_py))
        from efort_api import RobotAPI  # type: ignore

        dll_path = self._resolve_path(self.config.sdk_dll_path)
        self.api = RobotAPI(str(dll_path))

    def _pointc_type(self):
        sdk_py = self._resolve_path(self.config.sdk_python_path)
        if str(sdk_py) not in sys.path:
            sys.path.insert(0, str(sdk_py))
        from efort_types import PointC  # type: ignore

        return PointC

    def _to_pointc(self, item: QueuePoint, index: int):
        point = self._pointc_type()()
        point.index = index % 50
        point.x = item.pose.x
        point.y = item.pose.y
        point.z = item.pose.z
        point.a = item.pose.a
        point.b = item.pose.b
        point.c = item.pose.c
        point.cfgx = item.pose.cfgx
        point.cfg1 = item.pose.cfg1
        point.cfg4 = item.pose.cfg4
        point.cfg6 = item.pose.cfg6
        return point

    def _resolve_path(self, path: Path) -> Path:
        if path.is_absolute():
            return path
        return Path.cwd() / path

    @staticmethod
    def _check(result: int, operation: str) -> None:
        if int(result) != 0:
            raise RobotBackendError(f"{operation} failed with code {result}")


def create_backend(config: RobotControlConfig, logger: logging.Logger) -> RobotBackend:
    if config.dry_run:
        return DryRunBackend(config=config, logger=logger)
    return EfortSdkBackend(config=config, logger=logger)
