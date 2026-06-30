from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .config import RobotControlConfig
from .models import QueuePoint
from .sdk_backend import RobotBackend, RobotBackendError


@dataclass
class RplBatchSender:
    config: RobotControlConfig
    backend: RobotBackend
    logger: logging.Logger

    def send_queue(self, queue_points: list[QueuePoint]) -> None:
        if not queue_points:
            raise ValueError("cannot send an empty queue")

        batches = self._batches(queue_points)
        self.backend.set_int(self.config.rpl.total_points_int, len(queue_points))
        self._send_batch(batches[0], batch_index=0)
        self.backend.set_bool(self.config.rpl.start_bool, True)

        if self.config.dry_run:
            for idx, batch in enumerate(batches[1:], start=1):
                self._send_batch(batch, batch_index=idx)
            return

        for idx, batch in enumerate(batches[1:], start=1):
            request_var = (
                self.config.rpl.request_buffer_a_bool
                if idx % 2 == 1
                else self.config.rpl.request_buffer_b_bool
            )
            self._wait_for_request(request_var)
            self._send_batch(batch, batch_index=idx)
            self.backend.set_bool(request_var, False)

    def request_stop(self) -> None:
        self.backend.set_bool(self.config.rpl.stop_bool, True)

    def _send_batch(self, batch: list[QueuePoint], batch_index: int) -> None:
        self.backend.set_pointc_vector(batch)
        first = batch[0].queue_index
        last = batch[-1].queue_index
        self.logger.info(
            "sent batch=%s size=%s queue_range=%s-%s",
            batch_index,
            len(batch),
            first,
            last,
        )

    def _wait_for_request(self, bool_index: int) -> None:
        deadline = time.monotonic() + self.config.handshake_timeout_s
        while time.monotonic() < deadline:
            status = self.backend.read_status()
            if status.get("alarm") or status.get("emergency_stop"):
                raise RobotBackendError(f"robot unsafe while waiting for PC_BOOL[{bool_index}]")
            if self.backend.get_bool(bool_index):
                return
            time.sleep(self.config.handshake_poll_s)
        raise TimeoutError(f"timed out waiting for PC_BOOL[{bool_index}]")

    def _batches(self, queue_points: list[QueuePoint]) -> list[list[QueuePoint]]:
        size = self.config.batch_size
        return [queue_points[i : i + size] for i in range(0, len(queue_points), size)]
