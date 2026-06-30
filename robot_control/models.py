from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class QueuePointKind(str, Enum):
    APPROACH = "approach"
    TIE = "tie"
    RETREAT = "retreat"


@dataclass(frozen=True)
class CartesianPose:
    x: float
    y: float
    z: float
    a: float
    b: float
    c: float
    cfgx: int = 0
    cfg1: int = 0
    cfg4: int = 0
    cfg6: int = 0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CartesianPose":
        return cls(
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
            a=float(data.get("a", 180.0)),
            b=float(data.get("b", 0.0)),
            c=float(data.get("c", 180.0)),
            cfgx=int(data.get("cfgx", 0)),
            cfg1=int(data.get("cfg1", 0)),
            cfg4=int(data.get("cfg4", 0)),
            cfg6=int(data.get("cfg6", 0)),
        )

    def shifted(self, *, dz: float = 0.0) -> "CartesianPose":
        return CartesianPose(
            x=self.x,
            y=self.y,
            z=self.z + dz,
            a=self.a,
            b=self.b,
            c=self.c,
            cfgx=self.cfgx,
            cfg1=self.cfg1,
            cfg4=self.cfg4,
            cfg6=self.cfg6,
        )


@dataclass(frozen=True)
class TiePoint:
    point_id: str
    pose: CartesianPose
    confidence: float = 1.0
    source: str = "offline"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TiePoint":
        pose_data = data.get("pose", data)
        return cls(
            point_id=str(data.get("point_id", data.get("id", ""))),
            pose=CartesianPose.from_dict(pose_data),
            confidence=float(data.get("confidence", 1.0)),
            source=str(data.get("source", "offline")),
        )


@dataclass(frozen=True)
class QueuePoint:
    queue_index: int
    tie_point_id: str
    kind: QueuePointKind
    pose: CartesianPose
    confidence: float


@dataclass(frozen=True)
class SafetyIssue:
    point_id: str
    reason: str
    severity: str = "error"
