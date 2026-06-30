from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class WorkspaceLimits:
    x: tuple[float, float] = (-1000.0, 1000.0)
    y: tuple[float, float] = (-1000.0, 1000.0)
    z: tuple[float, float] = (-500.0, 1500.0)
    a: tuple[float, float] = (-180.0, 180.0)
    b: tuple[float, float] = (-180.0, 180.0)
    c: tuple[float, float] = (-180.0, 180.0)


@dataclass(frozen=True)
class RplVariableMap:
    total_points_int: int = 0
    current_point_int: int = 1
    error_code_int: int = 2
    request_buffer_a_bool: int = 1
    request_buffer_b_bool: int = 2
    start_bool: int = 3
    stop_bool: int = 4
    batch_done_bool: int = 5


@dataclass(frozen=True)
class RobotControlConfig:
    controller_ip: str = "192.168.1.12"
    sdk_dll_path: Path = Path(
        "埃弗顿机器人/SDK V2.8/V2.8.0/libs/x64/EftSdk.dll"
    )
    sdk_python_path: Path = Path(
        "埃弗顿机器人/SDK V2.8/V2.8.0/test/PySamples"
    )
    tool_name: str = "tool_tie"
    workobject_name: str = "wobj_rebar"
    batch_size: int = 25
    speed: int = 10
    zone: float = -1.0
    dry_run: bool = True
    require_servo_on: bool = False
    min_confidence: float = 0.55
    duplicate_distance_mm: float = 1.0
    minimum_point_spacing_mm: float = 0.5
    approach_offset_z_mm: float = 50.0
    retreat_offset_z_mm: float = 50.0
    tie_dwell_s: float = 0.25
    handshake_poll_s: float = 0.05
    handshake_timeout_s: float = 10.0
    monitor_poll_s: float = 0.1
    workspace: WorkspaceLimits = field(default_factory=WorkspaceLimits)
    rpl: RplVariableMap = field(default_factory=RplVariableMap)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotControlConfig":
        values = dict(data)
        if "workspace" in values and isinstance(values["workspace"], dict):
            values["workspace"] = WorkspaceLimits(**values["workspace"])
        if "rpl" in values and isinstance(values["rpl"], dict):
            values["rpl"] = RplVariableMap(**values["rpl"])
        if "sdk_dll_path" in values:
            values["sdk_dll_path"] = Path(values["sdk_dll_path"])
        if "sdk_python_path" in values:
            values["sdk_python_path"] = Path(values["sdk_python_path"])
        return cls(**values)
