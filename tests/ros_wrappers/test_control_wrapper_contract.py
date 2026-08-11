import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = "src/control/auto_rover_control"


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class ControlWrapperContractTests(unittest.TestCase):
    def test_wrapper_records_four_receipts_and_always_publishes(self):
        source = read(PACKAGE + "/src/pure_pursuit_node.cpp")
        self.assertGreaterEqual(source.count("ros::SteadyTime::now()"), 4)
        self.assertRegex(source, r"subscribe\([^\n]+,\s*1\s*,")
        self.assertEqual(len(re.findall(r"subscribe\(", source)), 4)
        self.assertRegex(source, r"advertise<[^>]+>\([^\n]+,\s*1\s*\)")
        self.assertIn("tracker_.update", source)
        self.assertIn("publisher_.publish", source)
        self.assertIn("auto_rover_ros1::toCore", source)
        self.assertIn("auto_rover_ros1::toRos", source)
        self.assertIn("makeProducerGenerationId", source)
        self.assertIn("config.tracker.producer_generation_id", source)
        self.assertIn("auto_rover_interfaces/SafetyState.h", source)
        self.assertIn('subscribe("safety_state", 1', source)
        self.assertIn("input.safety = safety_", source)

    def test_wrapper_requires_profile_tracker_and_watchdog_parameters(self):
        source = read(PACKAGE + "/src/pure_pursuit_node.cpp")
        required_names = (
            "world_frame",
            "control_frame",
            "lookahead_min_m",
            "lookahead_max_m",
            "lookahead_speed_gain_s",
            "goal_position_tolerance_m",
            "standstill_speed_threshold_mps",
            "localization_freshness_ns",
            "trajectory_freshness_ns",
            "chassis_freshness_ns",
            "safety_freshness_ns",
            "motion_valid_for_ns",
            "publish_period_s",
            "vehicle_profile/schema_version",
            "vehicle_profile/profile_id",
            "vehicle_profile/kinematic_model",
            "vehicle_profile/direction_capability",
            "vehicle_profile/reference_frame",
            "vehicle_profile/wheelbase_m",
            "vehicle_profile/max_forward_speed_mps",
            "vehicle_profile/max_longitudinal_accel_mps2",
            "vehicle_profile/min_turning_radius_m",
            "vehicle_profile/reverse_supported",
        )
        for name in required_names:
            self.assertIn('"{}"'.format(name), source)
        self.assertNotIn("private_node.param(", source)

    def test_core_target_does_not_link_the_ros_aggregate(self):
        cmake = read(PACKAGE + "/CMakeLists.txt")
        match = re.search(
            r"target_link_libraries\(\s*auto_rover_control_core\b(.*?)\n\)",
            cmake,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        self.assertNotIn("${catkin_LIBRARIES}", match.group(1))
        self.assertIn("${auto_rover_core_LIBRARIES}", match.group(1))

    def test_package_has_no_cmake_policy_version_override(self):
        cmake = read(PACKAGE + "/CMakeLists.txt")
        self.assertIn("cmake_minimum_required(VERSION 3.0.2)", cmake)
        self.assertNotIn("CMAKE_POLICY_VERSION_MINIMUM", cmake)

    def test_wrapper_target_and_manifest_have_ros_edge_dependencies(self):
        cmake = read(PACKAGE + "/CMakeLists.txt")
        manifest = read(PACKAGE + "/package.xml")
        self.assertIn("add_executable(pure_pursuit_node", cmake)
        self.assertIn("${catkin_LIBRARIES}", cmake)
        for dependency in (
            "auto_rover_interfaces",
            "auto_rover_ros1_conversions",
            "roscpp",
        ):
            self.assertIn("<depend>{}</depend>".format(dependency), manifest)


if __name__ == "__main__":
    unittest.main()
