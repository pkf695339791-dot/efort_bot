from __future__ import annotations

import argparse
import json
from pathlib import Path

from robot_control.config import RobotControlConfig
from robot_control.controller import RobotControlSystem
from robot_control.logging_utils import configure_logger
from robot_control.models import TiePoint


def load_points(path: Path) -> list[TiePoint]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("points", [])
    return [TiePoint.from_dict(item) for item in data]


def load_config(path: Path | None) -> RobotControlConfig:
    if path is None:
        return RobotControlConfig()
    data = json.loads(path.read_text(encoding="utf-8"))
    return RobotControlConfig.from_dict(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a rebar tying PointC queue.")
    parser.add_argument(
        "--points",
        type=Path,
        default=Path("samples/tie_points.json"),
        help="JSON file containing detected/offline tie points.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Optional JSON config override.",
    )
    parser.add_argument(
        "--real-robot",
        action="store_true",
        help="Use the real Efort SDK backend instead of dry-run.",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    if args.real_robot:
        config = RobotControlConfig.from_dict({**config.__dict__, "dry_run": False})

    logger = configure_logger(Path("logs/tie_queue.log"))
    points = load_points(args.points)
    system = RobotControlSystem(config=config, logger=logger)
    queue_points = system.run(points)

    print(f"Generated and sent {len(queue_points)} queue points from {len(points)} tie points.")
    if system.queue_builder.skipped:
        print("Skipped points:")
        for item in system.queue_builder.skipped:
            print(f"  - {item}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
