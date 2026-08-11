import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = "src/perception/auto_rover_localization"


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class LocalizationWrapperContractTests(unittest.TestCase):
    def test_wrapper_is_latest_value_and_has_no_tf(self):
        source = read(PACKAGE + "/src/fast_livo_odometry_node.cpp")
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

    def test_wrapper_requires_audited_parameters(self):
        source = read(PACKAGE + "/src/fast_livo_odometry_node.cpp")
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

    def test_core_target_does_not_link_the_ros_aggregate(self):
        cmake = read(PACKAGE + "/CMakeLists.txt")
        match = re.search(
            r"target_link_libraries\(\s*auto_rover_localization_core\b(.*?)\n\)",
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
        self.assertIn("add_executable(fast_livo_odometry_node", cmake)
        self.assertIn("${catkin_LIBRARIES}", cmake)
        for dependency in (
            "auto_rover_interfaces",
            "auto_rover_ros1_conversions",
            "roscpp",
            "nav_msgs",
        ):
            self.assertIn("<depend>{}</depend>".format(dependency), manifest)


if __name__ == "__main__":
    unittest.main()
