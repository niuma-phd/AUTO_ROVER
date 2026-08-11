import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class RosWrapperContractTests(unittest.TestCase):
    def test_localization_wrapper_is_latest_value_and_has_no_tf(self):
        source = read(
            "src/perception/auto_rover_localization/"
            "src/fast_livo_odometry_node.cpp"
        )
        self.assertIn("ros::SteadyTime::now()", source)
        self.assertRegex(source, r"advertise<[^>]+>\([^\n]+,\s*1\s*\)")
        self.assertRegex(source, r"subscribe\([^\n]+,\s*1\s*,")
        self.assertIn("auto_rover_ros1::toRos", source)
        for prohibited in (
            "tf2",
            "TransformBroadcaster",
            "lookupTransform",
            "sendTransform",
        ):
            self.assertNotIn(prohibited, source)

    def test_localization_requires_audited_parameters(self):
        source = read(
            "src/perception/auto_rover_localization/"
            "src/fast_livo_odometry_node.cpp"
        )
        required_names = (
            "source/topic",
            "source/expected_frame_id",
            "source/expected_child_frame_id",
            "source/revision",
            "source/timestamp_semantics",
            "source/freshness_limit_s",
            "extrinsic/known",
            "extrinsic/translation/x",
            "extrinsic/translation/y",
            "extrinsic/translation/z",
            "extrinsic/rotation/x",
            "extrinsic/rotation/y",
            "extrinsic/rotation/z",
            "extrinsic/rotation/w",
        )
        for name in required_names:
            self.assertIn('"{}"'.format(name), source)
        self.assertIn('timestamp_semantics != "PUBLISH_TIME"', source)
        self.assertNotIn("private_node.param(", source)

    def test_route_planner_reloads_and_refreshes_without_new_identity(self):
        source = read(
            "src/planning/auto_rover_planning/src/route_planner_node.cpp"
        )
        self.assertIn("auto_rover_interfaces::ReloadRoute", source)
        self.assertIn("reloadFromFile", source)
        self.assertIn("active_trajectory_", source)
        self.assertIn("published.stamp_ns = now_ros_ns", source)
        self.assertNotIn("published.trajectory_id =", source)
        self.assertRegex(source, r"advertise<[^>]+>\([^\n]+,\s*1\s*\)")
        self.assertIn("makeInvalidTrajectory", source)

    def test_route_planner_requires_profile_and_generator_parameters(self):
        source = read(
            "src/planning/auto_rover_planning/src/route_planner_node.cpp"
        )
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

    def test_control_wrapper_records_three_receipts_and_always_publishes(self):
        source = read(
            "src/control/auto_rover_control/src/pure_pursuit_node.cpp"
        )
        self.assertGreaterEqual(source.count("ros::SteadyTime::now()"), 4)
        self.assertRegex(source, r"subscribe\([^\n]+,\s*1\s*,")
        self.assertEqual(len(re.findall(r"subscribe\(", source)), 3)
        self.assertRegex(source, r"advertise<[^>]+>\([^\n]+,\s*1\s*\)")
        self.assertIn("tracker_.update", source)
        self.assertIn("publisher_.publish", source)
        self.assertIn("auto_rover_ros1::toCore", source)
        self.assertIn("auto_rover_ros1::toRos", source)
        self.assertIn("makeProducerGenerationId", source)
        self.assertIn("config.tracker.producer_generation_id", source)

    def test_control_requires_profile_tracker_and_watchdog_parameters(self):
        source = read(
            "src/control/auto_rover_control/src/pure_pursuit_node.cpp"
        )
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

    def test_core_targets_do_not_link_the_ros_aggregate(self):
        packages = (
            (
                "src/perception/auto_rover_localization/CMakeLists.txt",
                "auto_rover_localization_core",
            ),
            (
                "src/planning/auto_rover_planning/CMakeLists.txt",
                "auto_rover_planning",
            ),
            (
                "src/control/auto_rover_control/CMakeLists.txt",
                "auto_rover_control_core",
            ),
        )
        for path, target in packages:
            cmake = read(path)
            match = re.search(
                r"target_link_libraries\(\s*{}\b(.*?)\n\)".format(target),
                cmake,
                flags=re.DOTALL,
            )
            self.assertIsNotNone(match, path)
            self.assertNotIn("${catkin_LIBRARIES}", match.group(1), path)
            self.assertIn("${auto_rover_core_LIBRARIES}", match.group(1), path)

    def test_no_cmake_policy_version_override(self):
        for path in (
            "src/perception/auto_rover_localization/CMakeLists.txt",
            "src/planning/auto_rover_planning/CMakeLists.txt",
            "src/control/auto_rover_control/CMakeLists.txt",
        ):
            cmake = read(path)
            self.assertIn("cmake_minimum_required(VERSION 3.0.2)", cmake)
            self.assertNotIn("CMAKE_POLICY_VERSION_MINIMUM", cmake)

    def test_wrapper_targets_and_manifests_have_ros_edge_dependencies(self):
        packages = (
            (
                "src/perception/auto_rover_localization",
                "fast_livo_odometry_node",
                "nav_msgs",
            ),
            (
                "src/planning/auto_rover_planning",
                "route_planner_node",
                "auto_rover_interfaces",
            ),
            (
                "src/control/auto_rover_control",
                "pure_pursuit_node",
                "auto_rover_interfaces",
            ),
        )
        for package_path, target, extra_dependency in packages:
            cmake = read(package_path + "/CMakeLists.txt")
            manifest = read(package_path + "/package.xml")
            self.assertIn("add_executable({}".format(target), cmake)
            self.assertIn("${catkin_LIBRARIES}", cmake)
            for dependency in (
                "auto_rover_interfaces",
                "auto_rover_ros1_conversions",
                "roscpp",
                extra_dependency,
            ):
                self.assertIn("<depend>{}</depend>".format(dependency), manifest)


if __name__ == "__main__":
    unittest.main()
