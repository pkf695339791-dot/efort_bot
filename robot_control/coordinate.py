from __future__ import annotations

from dataclasses import dataclass

from .models import CartesianPose, TiePoint


@dataclass(frozen=True)
class CoordinateManager:
    """Transforms vision/workpiece coordinates into robot workobject poses.

    The first implementation uses an identity transform because hand-eye
    calibration is site-specific. Keeping this boundary explicit lets the
    calibration matrix replace the identity transform without touching queue
    or SDK code.
    """

    tool_name: str = "tool_tie"
    workobject_name: str = "wobj_rebar"

    def to_workobject_pose(self, point: TiePoint) -> CartesianPose:
        return point.pose
