from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .config import RobotControlConfig
from .models import QueuePoint


@dataclass
class TieToolInterface:
    config: RobotControlConfig
    logger: logging.Logger

    def tie(self, point: QueuePoint) -> None:
        self.logger.info(
            "simulated tie action point=%s queue_index=%s dwell=%.3fs",
            point.tie_point_id,
            point.queue_index,
            self.config.tie_dwell_s,
        )
        time.sleep(max(0.0, self.config.tie_dwell_s))
