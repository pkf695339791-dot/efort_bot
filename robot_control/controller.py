from __future__ import annotations

import logging
from dataclasses import dataclass

from .config import RobotControlConfig
from .coordinate import CoordinateManager
from .models import QueuePoint, QueuePointKind, TiePoint
from .monitor import ExecutionMonitor
from .queue import TiePointQueue
from .safety import SafetyChecker
from .sdk_backend import RobotBackend, create_backend
from .sender import RplBatchSender
from .tool import TieToolInterface


@dataclass
class RobotControlSystem:
    config: RobotControlConfig
    logger: logging.Logger
    backend: RobotBackend | None = None

    def __post_init__(self) -> None:
        if self.backend is None:
            self.backend = create_backend(self.config, self.logger)
        self.coordinate_manager = CoordinateManager(
            tool_name=self.config.tool_name,
            workobject_name=self.config.workobject_name,
        )
        self.safety_checker = SafetyChecker(self.config)
        self.queue_builder = TiePointQueue(
            self.config,
            self.coordinate_manager,
            self.safety_checker,
        )
        self.sender = RplBatchSender(self.config, self.backend, self.logger)
        self.monitor = ExecutionMonitor(self.config, self.backend, self.logger)
        self.tie_tool = TieToolInterface(self.config, self.logger)

    def run(self, tie_points: list[TiePoint]) -> list[QueuePoint]:
        queue_points = self.queue_builder.build(tie_points)
        self.logger.info(
            "built queue tie_points=%s queue_points=%s skipped=%s",
            len(tie_points),
            len(queue_points),
            len(self.queue_builder.skipped),
        )
        for skipped in self.queue_builder.skipped:
            self.logger.warning("skipped point %s", skipped)

        assert self.backend is not None
        self.backend.connect()
        try:
            self.backend.prepare()
            self.monitor.assert_ready()
            self.sender.send_queue(queue_points)
            if self.config.dry_run:
                self._simulate_tie_actions(queue_points)
            return queue_points
        except Exception:
            self.logger.exception("execution failed; requesting robot stop")
            try:
                self.sender.request_stop()
            except Exception:
                self.logger.exception("failed to request stop")
            raise
        finally:
            self.backend.disconnect()

    def _simulate_tie_actions(self, queue_points: list[QueuePoint]) -> None:
        for item in queue_points:
            if item.kind == QueuePointKind.TIE:
                self.tie_tool.tie(item)
