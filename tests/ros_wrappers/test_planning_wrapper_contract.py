import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = "src/planning/auto_rover_planning"


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class PlanningWrapperContractTests(unittest.TestCase):
    def test_wrapper_reloads_and_refreshes_without_new_identity(self):
        source = read(PACKAGE + "/src/route_planner_node.cpp")
        self.assertIn("auto_rover_interfaces::ReloadRoute", source)
        self.assertIn("reloadFromFile", source)
        self.assertIn("active_trajectory_", source)
        self.assertIn("published.stamp_ns = now_ros_ns", source)
        self.assertNotIn("published.trajectory_id =", source)
        self.assertRegex(source, r"advertise<[^>]+>\([^\n]+,\s*1\s*\)")
        self.assertIn("makeInvalidTrajectory", source)

    def test_wrapper_requires_profile_and_generator_parameters(self):
        source = read(PACKAGE + "/src/route_planner_node.cpp")
        required_names = (
            "waypoints_file",
            "expected_frame_id",
            "sampling_resolution_m",
            "trajectory_valid_for_ns",
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

    def test_core_target_does_not_link_the_ros_aggregate(self):
        cmake = read(PACKAGE + "/CMakeLists.txt")
        match = re.search(
            r"target_link_libraries\(\s*auto_rover_planning\b(.*?)\n\)",
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
        self.assertIn("add_executable(route_planner_node", cmake)
        self.assertIn("${catkin_LIBRARIES}", cmake)
        for dependency in (
            "auto_rover_interfaces",
            "auto_rover_ros1_conversions",
            "roscpp",
        ):
            self.assertIn("<depend>{}</depend>".format(dependency), manifest)


if __name__ == "__main__":
    unittest.main()
