from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .config import RobotControlConfig
from .models import QueuePoint, QueuePointKind, SafetyIssue, TiePoint


@dataclass
class SafetyChecker:
    config: RobotControlConfig

    def validate_tie_points(self, points: Iterable[TiePoint]) -> list[SafetyIssue]:
        issues: list[SafetyIssue] = []
        seen: list[TiePoint] = []
        for point in points:
            pose = point.pose
            if not point.point_id:
                issues.append(SafetyIssue("<missing>", "point_id is required"))
            if point.confidence < self.config.min_confidence:
                issues.append(
                    SafetyIssue(
                        point.point_id,
                        f"confidence {point.confidence:.3f} below {self.config.min_confidence:.3f}",
                    )
                )
            issues.extend(self._check_pose_ranges(point.point_id, pose))
            for prev in seen:
                if self._distance(point.pose, prev.pose) < self.config.duplicate_distance_mm:
                    issues.append(
                        SafetyIssue(
                            point.point_id,
                            f"duplicate or near-duplicate of {prev.point_id}",
                        )
                    )
                    break
            seen.append(point)
        return issues

    def validate_queue(self, queue_points: list[QueuePoint]) -> list[SafetyIssue]:
        issues: list[SafetyIssue] = []
        if not queue_points:
            return [SafetyIssue("<queue>", "queue is empty")]

        last_tie_pose = None
        for item in queue_points:
            issues.extend(self._check_pose_ranges(item.tie_point_id, item.pose))
            if item.kind == QueuePointKind.TIE:
                if last_tie_pose is not None:
                    distance = self._distance(item.pose, last_tie_pose)
                    if distance < self.config.minimum_point_spacing_mm:
                        issues.append(
                            SafetyIssue(
                                item.tie_point_id,
                                f"tie point spacing {distance:.3f} mm below minimum",
                            )
                        )
                last_tie_pose = item.pose
        return issues

    def _check_pose_ranges(self, point_id: str, pose) -> list[SafetyIssue]:
        limits = self.config.workspace
        checks = {
            "x": (pose.x, limits.x),
            "y": (pose.y, limits.y),
            "z": (pose.z, limits.z),
            "a": (pose.a, limits.a),
            "b": (pose.b, limits.b),
            "c": (pose.c, limits.c),
        }
        issues: list[SafetyIssue] = []
        for axis, (value, (lower, upper)) in checks.items():
            if value < lower or value > upper:
                issues.append(
                    SafetyIssue(point_id, f"{axis}={value:.3f} outside [{lower:.3f}, {upper:.3f}]")
                )
        return issues

    @staticmethod
    def _distance(a, b) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
