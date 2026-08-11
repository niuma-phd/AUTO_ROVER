import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
NODE = (
    ROOT
    / "src"
    / "vehicle"
    / "auto_rover_vehicle"
    / "src"
    / "vehicle_execution_node.cpp"
)


class VehicleExecutionNodeContractTest(unittest.TestCase):
    def test_graph_boundary_is_relative_queue_one_and_fake_only(self):
        text = NODE.read_text(encoding="utf-8")
        for topic in (
            "ego_state",
            "trajectory",
            "motion_reference",
        ):
            self.assertRegex(
                text,
                rf'subscribe\s*\(\s*"{topic}"\s*,\s*1\s*,',
            )
        self.assertRegex(
            text,
            r'subscribe\s*\(\s*"emergency_stop"\s*,\s*10\s*,',
        )
        for topic in ("chassis_state", "safety_state"):
            self.assertRegex(text, rf'\(\s*"{topic}"\s*,\s*1\s*\)')
        for service in (
            "arm_vehicle",
            "assert_emergency_stop",
            "reset_emergency_stop",
        ):
            self.assertRegex(text, rf'advertiseService\s*\(\s*"{service}"')
        for service in (
            "fake_connected",
            "fake_faulted",
            "fake_drop_feedback",
        ):
            self.assertRegex(text, rf'advertiseService\s*\(\s*"{service}"')
        self.assertNotIn("/cmd_vel", text)
        self.assertNotIn("/dev/", text)
        self.assertNotIn("serial", text.lower())

    def test_watchdogs_use_steady_time_and_actuation_defaults_disabled(self):
        text = NODE.read_text(encoding="utf-8")
        self.assertIn("ros::SteadyTime::now()", text)
        self.assertIn("createSteadyTimer", text)
        self.assertIn("safety_supervisor.actuation_enabled = false", text)
        self.assertIn('"safety_supervisor/actuation_enabled"', text)

    def test_fake_feedback_identity_has_runtime_process_generation(self):
        text = NODE.read_text(encoding="utf-8")
        self.assertIn("makeProcessGenerationId", text)
        self.assertIn("ros::WallTime::now()", text)
        self.assertIn("ros::SteadyTime::now()", text)
        self.assertIn("getpid()", text)
        self.assertIn("fake_vcu.process_generation_id =", text)
        self.assertNotIn('"fake_vcu/process_generation_id"', text)

    def test_fake_wrapper_explicitly_stops_and_reports_stop_delivery(self):
        text = NODE.read_text(encoding="utf-8")
        self.assertIn("~VehicleExecutionNode()", text)
        self.assertIn("runtime_.shutdown(monotonicNow())", text)
        self.assertGreaterEqual(
            text.count("result.stop_delivery.delivery_unconfirmed"), 5
        )
        self.assertIn("Fake backend shutdown stop delivery is unconfirmed", text)


if __name__ == "__main__":
    unittest.main()
