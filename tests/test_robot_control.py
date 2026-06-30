import unittest

from robot_control.config import RobotControlConfig
from robot_control.coordinate import CoordinateManager
from robot_control.models import CartesianPose, TiePoint
from robot_control.queue import TiePointQueue
from robot_control.safety import SafetyChecker
from robot_control.sdk_backend import DryRunBackend
from robot_control.sender import RplBatchSender
from robot_control.logging_utils import configure_logger


def make_points(count: int) -> list[TiePoint]:
    return [
        TiePoint(
            point_id=f"P{i:03d}",
            pose=CartesianPose(
                x=100.0 + i * 10.0,
                y=-50.0,
                z=250.0,
                a=180.0,
                b=0.0,
                c=180.0,
                cfgx=1,
            ),
            confidence=0.9,
        )
        for i in range(count)
    ]


class RobotControlTests(unittest.TestCase):
    def setUp(self):
        self.config = RobotControlConfig(dry_run=True, tie_dwell_s=0.0)
        self.logger = configure_logger()

    def test_queue_builds_approach_tie_retreat_segments(self):
        queue = TiePointQueue(
            self.config,
            CoordinateManager(),
            SafetyChecker(self.config),
        ).build(make_points(2))

        self.assertEqual(len(queue), 6)
        self.assertEqual([item.kind.value for item in queue[:3]], ["approach", "tie", "retreat"])
        self.assertEqual(queue[0].pose.z, 300.0)
        self.assertEqual(queue[1].pose.z, 250.0)

    def test_batches_handle_boundary_sizes(self):
        builder = TiePointQueue(self.config, CoordinateManager(), SafetyChecker(self.config))
        self.assertEqual([len(batch) for batch in builder.batches(builder.build(make_points(8)))], [24])
        self.assertEqual([len(batch) for batch in builder.batches(builder.build(make_points(9)))], [25, 2])

    def test_safety_rejects_low_confidence_and_out_of_range(self):
        bad = [
            TiePoint("LOW", CartesianPose(0, 0, 0, 0, 0, 0), confidence=0.1),
            TiePoint("FAR", CartesianPose(9999, 0, 0, 0, 0, 0), confidence=0.9),
        ]
        issues = SafetyChecker(self.config).validate_tie_points(bad)
        reasons = " ".join(issue.reason for issue in issues)
        self.assertIn("confidence", reasons)
        self.assertIn("outside", reasons)

    def test_dry_run_sender_sets_total_and_start(self):
        backend = DryRunBackend(self.config, self.logger)
        backend.connect()
        queue = TiePointQueue(
            self.config,
            CoordinateManager(),
            SafetyChecker(self.config),
        ).build(make_points(10))

        RplBatchSender(self.config, backend, self.logger).send_queue(queue)

        self.assertEqual(backend.ints[self.config.rpl.total_points_int], 30)
        self.assertTrue(backend.bools[self.config.rpl.start_bool])
        self.assertEqual(len(backend.last_vector), 5)


if __name__ == "__main__":
    unittest.main()
