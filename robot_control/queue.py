from __future__ import annotations

from dataclasses import dataclass, field

from .config import RobotControlConfig
from .coordinate import CoordinateManager
from .models import QueuePoint, QueuePointKind, TiePoint
from .safety import SafetyChecker


@dataclass
class TiePointQueue:
    config: RobotControlConfig
    coordinate_manager: CoordinateManager
    safety_checker: SafetyChecker
    skipped: list[str] = field(default_factory=list)

    def build(self, tie_points: list[TiePoint]) -> list[QueuePoint]:
        issues = self.safety_checker.validate_tie_points(tie_points)
        blocked_ids = {issue.point_id for issue in issues if issue.severity == "error"}
        self.skipped = [f"{issue.point_id}: {issue.reason}" for issue in issues]

        usable = [point for point in tie_points if point.point_id not in blocked_ids]
        usable.sort(key=lambda p: (p.pose.x, p.pose.y, p.pose.z, p.point_id))

        queue: list[QueuePoint] = []
        for point in usable:
            base_pose = self.coordinate_manager.to_workobject_pose(point)
            for kind, pose in (
                (QueuePointKind.APPROACH, base_pose.shifted(dz=self.config.approach_offset_z_mm)),
                (QueuePointKind.TIE, base_pose),
                (QueuePointKind.RETREAT, base_pose.shifted(dz=self.config.retreat_offset_z_mm)),
            ):
                queue.append(
                    QueuePoint(
                        queue_index=len(queue),
                        tie_point_id=point.point_id,
                        kind=kind,
                        pose=pose,
                        confidence=point.confidence,
                    )
                )

        queue_issues = self.safety_checker.validate_queue(queue)
        if queue_issues:
            reasons = "; ".join(f"{issue.point_id}: {issue.reason}" for issue in queue_issues)
            raise ValueError(f"queue failed safety validation: {reasons}")
        return queue

    def batches(self, queue_points: list[QueuePoint]) -> list[list[QueuePoint]]:
        size = self.config.batch_size
        if size <= 0:
            raise ValueError("batch_size must be positive")
        return [queue_points[i : i + size] for i in range(0, len(queue_points), size)]
