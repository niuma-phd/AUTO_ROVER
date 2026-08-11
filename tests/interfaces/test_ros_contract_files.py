import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INTERFACES = ROOT / "src/interfaces/auto_rover_interfaces"


class RosContractFileTests(unittest.TestCase):
    def read_lines(self, relative_path):
        return [
            line.strip()
            for line in (INTERFACES / relative_path)
            .read_text(encoding="utf-8")
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]

    def test_public_message_set_is_explicit(self):
        messages = sorted(path.name for path in (INTERFACES / "msg").glob("*.msg"))
        self.assertEqual(
            [
                "ChassisState.msg",
                "EgoState.msg",
                "EmergencyStop.msg",
                "MotionReference.msg",
                "RoutePlan.msg",
                "RouteWaypoint.msg",
                "SafetyState.msg",
                "Trajectory.msg",
                "TrajectoryPoint.msg",
            ],
            messages,
        )

    def test_semantic_identifiers_do_not_depend_on_header_sequence(self):
        expected_identifiers = {
            "msg/EgoState.msg": "uint64 state_id",
            "msg/RoutePlan.msg": "uint64 plan_version",
            "msg/Trajectory.msg": "string trajectory_id",
            "msg/MotionReference.msg": "uint64 command_id",
            "msg/ChassisState.msg": "uint64 state_id",
            "msg/SafetyState.msg": "uint64 state_id",
            "msg/EmergencyStop.msg": "string request_id",
        }
        for path, field in expected_identifiers.items():
            with self.subTest(path=path):
                self.assertIn(field, self.read_lines(path))

    def test_time_and_validity_semantics_are_represented(self):
        ego = self.read_lines("msg/EgoState.msg")
        chassis = self.read_lines("msg/ChassisState.msg")
        trajectory = self.read_lines("msg/Trajectory.msg")
        motion = self.read_lines("msg/MotionReference.msg")
        safety = self.read_lines("msg/SafetyState.msg")

        self.assertIn("uint8 time_source", ego)
        self.assertIn("uint8 time_source", chassis)
        self.assertIn("duration valid_for", trajectory)
        self.assertIn("duration valid_for", motion)
        self.assertIn("string producer_generation_id", motion)
        self.assertIn("duration valid_for", safety)
        self.assertIn("uint32 valid_mask", ego)
        self.assertIn("uint32 valid_mask", chassis)
        self.assertIn("bool valid", trajectory)
        self.assertIn("bool valid", motion)

    def test_estop_reset_and_arm_are_separate_service_boundaries(self):
        services = sorted(path.name for path in (INTERFACES / "srv").glob("*.srv"))
        self.assertEqual(
            [
                "ArmVehicle.srv",
                "AssertEmergencyStop.srv",
                "ReloadRoute.srv",
                "ResetEmergencyStop.srv",
            ],
            services,
        )
        assertion = self.read_lines("srv/AssertEmergencyStop.srv")
        self.assertIn("uint32 schema_version", assertion)
        self.assertIn("time source_stamp", assertion)
        self.assertIn("string request_id", assertion)
        self.assertIn("uint64 latch_generation", assertion)
        reset = self.read_lines("srv/ResetEmergencyStop.srv")
        self.assertIn("uint64 latch_generation", reset)
        self.assertIn("string operator_id", reset)
        arm = self.read_lines("srv/ArmVehicle.srv")
        self.assertIn("bool arm", arm)
        self.assertIn("uint64 safety_generation", arm)


if __name__ == "__main__":
    unittest.main()
