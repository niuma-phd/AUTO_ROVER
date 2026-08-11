import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class Phase1ImplementationPolicyTests(unittest.TestCase):
    expected_packages = {
        "src/core/auto_rover_core": "auto_rover_core",
        "src/interfaces/auto_rover_interfaces": "auto_rover_interfaces",
        "src/interfaces/auto_rover_ros1_conversions":
            "auto_rover_ros1_conversions",
        "src/perception/auto_rover_localization": "auto_rover_localization",
        "src/planning/auto_rover_planning": "auto_rover_planning",
        "src/control/auto_rover_control": "auto_rover_control",
        "src/safety/auto_rover_safety": "auto_rover_safety",
        "src/vehicle/auto_rover_vehicle": "auto_rover_vehicle",
        "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial":
            "auto_rover_vcu_wheeltec_serial",
        "src/apps/known_map_navigation/auto_rover_known_map_bringup":
            "auto_rover_known_map_bringup",
    }

    def source_text(self, directory):
        suffixes = {".cpp", ".h", ".hpp", ".c", ".cc"}
        return "\n".join(
            path.read_text(encoding="utf-8")
            for path in sorted(directory.rglob("*"))
            if path.is_file() and path.suffix in suffixes
        )

    def dependencies(self, relative_package):
        root = ET.parse(ROOT / relative_package / "package.xml").getroot()
        tags = {
            "depend",
            "build_depend",
            "build_export_depend",
            "exec_depend",
            "test_depend",
        }
        return {
            (element.text or "").strip()
            for element in root
            if element.tag in tags and (element.text or "").strip()
        }

    def test_exact_create_on_demand_package_set_and_names(self):
        actual = {
            path.parent.relative_to(ROOT).as_posix():
                (ET.parse(path).getroot().findtext("name") or "").strip()
            for path in ROOT.glob("src/**/package.xml")
        }
        self.assertEqual(self.expected_packages, actual)

    def test_every_package_has_a_substantive_delivery_and_test(self):
        special = {
            "auto_rover_interfaces": ["msg/EgoState.msg", "srv/ArmVehicle.srv"],
            "auto_rover_known_map_bringup": [
                "config/waypoints.yaml",
                "launch/known_map_fake.launch",
                "test/fake_closed_loop.test",
            ],
        }
        test_directories = {
            "auto_rover_core": "tests/core",
            "auto_rover_ros1_conversions": "tests/interfaces",
            "auto_rover_localization": "tests/localization",
            "auto_rover_planning": "tests/planning",
            "auto_rover_control": "tests/control",
            "auto_rover_safety": "tests/safety",
            "auto_rover_vehicle": "tests/vehicle",
            "auto_rover_vcu_wheeltec_serial": "tests/wheeltec",
        }
        for relative, package_name in self.expected_packages.items():
            package = ROOT / relative
            with self.subTest(package=package_name):
                if package_name in special:
                    for required in special[package_name]:
                        self.assertTrue((package / required).is_file(), required)
                else:
                    self.assertTrue(list(package.rglob("*.cpp")))
                    test_root = ROOT / test_directories[package_name]
                    self.assertTrue(test_root.is_dir())
                    self.assertTrue(
                        list(test_root.rglob("test_*.cpp"))
                        or list(test_root.rglob("test_*.py"))
                    )

    def test_dependency_direction(self):
        self.assertEqual(set(), self.dependencies("src/core/auto_rover_core"))

        planning = self.dependencies("src/planning/auto_rover_planning")
        self.assertFalse(
            planning
            & {
                "auto_rover_localization",
                "auto_rover_control",
                "auto_rover_vehicle",
                "auto_rover_vcu_wheeltec_serial",
                "auto_rover_known_map_bringup",
            }
        )
        control = self.dependencies("src/control/auto_rover_control")
        self.assertFalse(
            control
            & {
                "auto_rover_vehicle",
                "auto_rover_vcu_wheeltec_serial",
                "auto_rover_known_map_bringup",
            }
        )
        adapter = self.dependencies(
            "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial"
        )
        self.assertFalse(
            adapter
            & {
                "auto_rover_planning",
                "auto_rover_control",
                "auto_rover_localization",
                "auto_rover_known_map_bringup",
            }
        )
        for relative, package_name in self.expected_packages.items():
            if package_name == "auto_rover_known_map_bringup":
                continue
            self.assertNotIn(
                "auto_rover_known_map_bringup",
                self.dependencies(relative),
            )

    def test_algorithm_core_files_have_no_ros_api(self):
        pure_package_roots = [
            ROOT / "src/core/auto_rover_core",
            ROOT / "src/perception/auto_rover_localization",
            ROOT / "src/planning/auto_rover_planning",
            ROOT / "src/control/auto_rover_control",
            ROOT / "src/safety/auto_rover_safety",
            ROOT / "src/vehicle/auto_rover_vehicle",
            ROOT / "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial",
        ]
        forbidden = re.compile(
            r"#include\s*[<\"](?:ros|tf|tf2|std_msgs|geometry_msgs|nav_msgs)/"
            r"|\bros::|\bNodeHandle\b|\bROS_(?:INFO|WARN|ERROR|FATAL)\b"
        )
        suffixes = {".cpp", ".h", ".hpp", ".c", ".cc"}
        for package_root in pure_package_roots:
            for path in sorted(package_root.rglob("*")):
                if (
                    not path.is_file()
                    or path.suffix not in suffixes
                    or path.name.endswith("_node.cpp")
                ):
                    continue
                with self.subTest(path=path.relative_to(ROOT).as_posix()):
                    self.assertIsNone(
                        forbidden.search(path.read_text(encoding="utf-8"))
                    )

    def test_phase1_scope_and_command_boundary(self):
        source_paths = [
            path.relative_to(ROOT).as_posix().lower()
            for path in ROOT.glob("src/**/*")
            if path.is_file()
        ]
        for token in ("ros2", "world_model", "obstacle", "exploration", "coverage"):
            self.assertFalse([path for path in source_paths if token in path])

        for path in ROOT.glob("src/**/*"):
            if not path.is_file() or path.suffix not in {
                ".cpp",
                ".h",
                ".hpp",
                ".launch",
                ".yaml",
                ".xml",
            }:
                continue
            text = path.read_text(encoding="utf-8")
            if "/cmd_vel" not in text:
                continue
            relative = path.relative_to(ROOT).as_posix()
            self.assertTrue(
                relative.startswith(
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                ),
                relative,
            )

    def test_safety_critical_defaults(self):
        bringup = (
            ROOT
            / "src/apps/known_map_navigation/auto_rover_known_map_bringup"
        )
        vehicle = (bringup / "config/vehicle_profile.yaml").read_text(
            encoding="utf-8"
        )
        self.assertRegex(vehicle, r"initial_max_forward_speed_mps:\s*0\.50\b")
        self.assertRegex(vehicle, r"operational_min_turning_radius_m:\s*0\.95\b")
        self.assertRegex(vehicle, r"reverse_supported:\s*false\b")
        self.assertRegex(
            vehicle,
            r"direction_capability:\s*signed_speed_direction\b",
        )
        localization = (bringup / "config/localization.yaml").read_text(
            encoding="utf-8"
        )
        self.assertRegex(localization, r"(?m)^\s*known:\s*false\b")
        serial = (bringup / "config/wheeltec_serial_unverified.yaml").read_text(
            encoding="utf-8"
        )
        self.assertRegex(serial, r"real_device_enabled:\s*false\b")
        self.assertRegex(serial, r"actuation_opt_in:\s*false\b")
        self.assertRegex(serial, r"feedback_rate_hz:\s*20\.0\b")
        self.assertRegex(serial, r"standstill_threshold_mps:\s*unknown\b")
        self.assertRegex(serial, r"readiness_gate_passed:\s*false\b")
        for launch in bringup.glob("launch/*.launch"):
            text = launch.read_text(encoding="utf-8")
            if "actuation_enabled" in text:
                self.assertNotRegex(
                    text,
                    r'name="actuation_enabled"\s+default="true"',
                    launch.name,
                )

    def test_wheeltec_vehicle_execution_is_separate_and_default_disabled(self):
        bringup = (
            ROOT
            / "src/apps/known_map_navigation/auto_rover_known_map_bringup"
        )
        launch_path = (
            bringup / "launch/vehicle_execution_wheeltec_unverified.launch"
        )
        safety_path = bringup / "config/safety_wheeltec_unverified.yaml"
        backend_path = bringup / "config/wheeltec_serial_unverified.yaml"
        self.assertTrue(launch_path.is_file())
        self.assertTrue(safety_path.is_file())
        self.assertTrue(backend_path.is_file())

        launch_text = launch_path.read_text(encoding="utf-8")
        launch = ET.parse(launch_path).getroot()
        defaults = {
            element.attrib["name"]: element.attrib.get("default")
            for element in launch.findall("arg")
        }
        for gate in (
            "real_device_enabled",
            "actuation_enabled",
            "physical_device_opt_in",
            "actuation_opt_in",
            "readiness_gate_passed",
            "external_or_durable_estop_strategy_approved",
            "unverified_protocol_acknowledged",
        ):
            self.assertEqual("false", defaults.get(gate), gate)
        self.assertEqual("", defaults.get("device_path"))
        self.assertEqual("0.50", defaults.get("max_forward_speed_mps"))

        nodes = launch.findall("node")
        self.assertEqual(1, len(nodes))
        self.assertEqual(
            "auto_rover_vcu_wheeltec_serial", nodes[0].attrib.get("pkg")
        )
        self.assertEqual(
            "wheeltec_vehicle_execution_node", nodes[0].attrib.get("type")
        )
        parameter_names = {
            element.attrib.get("name") for element in nodes[0].findall("param")
        }
        for gate in (
            "real_device_enabled",
            "actuation_enabled",
            "physical_device_opt_in",
            "actuation_opt_in",
            "readiness_gate_passed",
            "external_or_durable_estop_strategy_approved",
            "unverified_protocol_acknowledged",
        ):
            self.assertIn(gate, parameter_names)
            self.assertNotIn(f"physical_backend/{gate}", parameter_names)
        self.assertIn(
            "auto_rover_vcu_wheeltec_serial",
            self.dependencies(
                "src/apps/known_map_navigation/auto_rover_known_map_bringup"
            ),
        )
        self.assertNotIn("/cmd_vel", launch_text)
        self.assertNotIn("VehicleExecutionCommand", launch_text)
        self.assertFalse(
            (
                ROOT
                / "src/interfaces/auto_rover_interfaces/msg/VehicleExecutionCommand.msg"
            ).exists()
        )

        backend = backend_path.read_text(encoding="utf-8")
        for gate in (
            "real_device_enabled",
            "actuation_enabled",
            "physical_device_opt_in",
            "actuation_opt_in",
            "readiness_gate_passed",
            "external_or_durable_estop_strategy_approved",
            "unverified_protocol_acknowledged",
        ):
            self.assertRegex(backend, rf"(?m)^\s*{gate}:\s*false\s*$")
            self.assertEqual(
                1, len(re.findall(rf"(?m)^{gate}:\s*false\s*$", backend))
            )
        self.assertRegex(backend, r'(?m)^\s*device_path:\s*""\s*$')
        for top_level_gate in (
            "physical_device_opt_in",
            "actuation_opt_in",
            "unverified_protocol_acknowledged",
        ):
            self.assertRegex(
                backend, rf"(?m)^{top_level_gate}:\s*false\s*$"
            )
        self.assertRegex(
            backend, r"(?m)^\s*max_forward_speed_mps:\s*0\.50\s*$"
        )
        self.assertRegex(
            backend, r"(?m)^\s*maximum_feedback_age_ns:\s*150000000\s*$"
        )
        for required_runtime in (
            r"cycle_period_s:\s*0\.02",
            r"max_command_age_ns:\s*100000000",
            r"fresh_commands_required:\s*3",
            r"zero_retry_interval_ns:\s*20000000",
            r"max_zero_write_attempts:\s*3",
            r"drain_read_timeout_ns:\s*1000000",
            r"maximum_drain_reads:\s*64",
        ):
            self.assertRegex(backend, required_runtime)
        self.assertNotRegex(backend, r"(?m):\s*null\s*$")

        safety = safety_path.read_text(encoding="utf-8")
        self.assertRegex(
            safety,
            r"(?ms)^safety_supervisor:\s+.*?^\s+actuation_enabled:\s*false\s*$",
        )
        self.assertIn("UNVERIFIED", safety)

    def test_wheeltec_udev_rule_is_identity_pinned_and_not_world_writable(self):
        rule = (
            ROOT / "deploy/system/udev/wheeltec_controller3.rules"
        ).read_text(encoding="utf-8")
        for required in (
            'ATTRS{idVendor}=="1a86"',
            'ATTRS{idProduct}=="55d4"',
            'ATTRS{serial}=="0002"',
            'MODE:="0660"',
            'OWNER:="root"',
            'GROUP:="dialout"',
        ):
            self.assertIn(required, rule)
        self.assertNotIn("0777", rule)

    def test_wheeltec_codec_speed_limit_is_explicit_and_phase1_stays_at_half(self):
        adapter = (
            ROOT / "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial"
        )
        header = (
            adapter
            / "include/auto_rover_vcu_wheeltec_serial/codec.hpp"
        ).read_text(encoding="utf-8")
        implementation = (adapter / "src/codec.cpp").read_text(
            encoding="utf-8"
        )
        configuration = (
            adapter / "config/wheeltec_serial_unverified.yaml"
        ).read_text(encoding="utf-8")

        self.assertRegex(
            header,
            r"kMaximumConfigurableForwardSpeedMps\s*=\s*6\.0\s*;",
        )
        self.assertRegex(
            header,
            r"double\s+max_forward_speed_mps\s*\{\s*0\.0\s*\}\s*;",
        )
        self.assertNotIn("kPhase1MaximumForwardSpeedMps", header)
        self.assertRegex(
            implementation,
            r"std::isfinite\(limits\.max_forward_speed_mps\)\s*&&\s*"
            r"limits\.max_forward_speed_mps\s*>\s*0\.0\s*&&\s*"
            r"limits\.max_forward_speed_mps\s*<\s*"
            r"kMaximumConfigurableForwardSpeedMps",
        )
        self.assertRegex(
            configuration,
            r"(?m)^\s*max_forward_speed_mps:\s*0\.50\s*$",
        )

    def test_fixed_waypoint_entry_and_unverified_adapter_notice(self):
        waypoint = (
            ROOT
            / "src/apps/known_map_navigation/auto_rover_known_map_bringup"
            / "config/waypoints.yaml"
        )
        self.assertTrue(waypoint.is_file())
        self.assertIn("speed_mps: 0.50", waypoint.read_text(encoding="utf-8"))
        adapter_readme = (
            ROOT
            / "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/README.md"
        ).read_text(encoding="utf-8")
        self.assertIn("UNVERIFIED", adapter_readme)
        self.assertIn("disabled by default", adapter_readme.lower())


if __name__ == "__main__":
    unittest.main()
