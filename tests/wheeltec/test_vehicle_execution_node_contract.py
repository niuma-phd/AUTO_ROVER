import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
NODE = (
    ROOT
    / "src"
    / "vehicle"
    / "adapters"
    / "auto_rover_vcu_wheeltec_serial"
    / "src"
    / "wheeltec_vehicle_execution_node.cpp"
)
CMAKE = NODE.parents[1] / "CMakeLists.txt"
PACKAGE = NODE.parents[1] / "package.xml"
BENCH = NODE.parent / "bench_main.cpp"
RAW_PROFILE = NODE.parent / "raw_profile_main.cpp"
FEEDBACK_CAPTURE = NODE.parent / "feedback_capture_main.cpp"
ORDERED_MAILBOX = NODE.parent / "ordered_cycle_publication_mailbox.hpp"


class WheeltecVehicleExecutionNodeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = NODE.read_text(encoding="utf-8")
        cls.cmake = CMAKE.read_text(encoding="utf-8")
        cls.package = PACKAGE.read_text(encoding="utf-8")
        cls.bench = BENCH.read_text(encoding="utf-8")
        cls.raw_profile = RAW_PROFILE.read_text(encoding="utf-8")
        cls.feedback_capture = FEEDBACK_CAPTURE.read_text(encoding="utf-8")
        cls.ordered_mailbox = ORDERED_MAILBOX.read_text(encoding="utf-8")

    def test_graph_has_no_guard_bypass_or_fake_services(self):
        for topic in ("ego_state", "trajectory", "motion_reference"):
            self.assertRegex(
                self.text,
                rf'subscribe\s*\(\s*"{topic}"\s*,\s*1\s*,',
            )
        self.assertRegex(
            self.text,
            r'subscribe\s*\(\s*"emergency_stop"\s*,\s*10\s*,',
        )
        for topic in ("chassis_state", "safety_state"):
            self.assertRegex(self.text, rf'\(\s*"{topic}"\s*,\s*1\s*\)')
        for service in (
            "arm_vehicle",
            "assert_emergency_stop",
            "reset_emergency_stop",
        ):
            self.assertRegex(
                self.text, rf'advertiseService\s*\(\s*"{service}"'
            )
        for forbidden in (
            "/cmd_vel",
            "fake_connected",
            "fake_faulted",
            "fake_drop_feedback",
            "advertise<auto_rover_interfaces::VehicleExecutionCommand>",
        ):
            self.assertNotIn(forbidden, self.text)

    def test_device_open_is_after_full_config_validation_and_explicit_gate(self):
        load_index = self.text.index(
            "loadWheeltecConfig(private_node, &config, &reason)"
        )
        generation_index = self.text.index("makeProcessGenerationId()", load_index)
        wrapper_index = self.text.index(
            "WheeltecVehicleExecutionNode wrapper", generation_index
        )
        clock_index = self.text.index("preopen_monotonic_ns", generation_index)
        prepare_index = self.text.index(
            "PreparedPhysicalSerialOpen prepared_open", clock_index
        )
        open_index = self.text.index("prepared_open.open()", prepare_index)
        activation_index = self.text.index(
            "wrapper.activatePhysical", open_index
        )
        self.assertLess(load_index, generation_index)
        self.assertLess(generation_index, wrapper_index)
        self.assertLess(wrapper_index, clock_index)
        self.assertLess(clock_index, prepare_index)
        self.assertLess(prepare_index, open_index)
        self.assertLess(open_index, activation_index)
        self.assertRegex(
            self.text,
            r"if\s*\(config\.real_device_enabled\)\s*\{",
        )
        self.assertIn("PreparedPhysicalSerialOpen", self.text[clock_index:])
        release_gate = self.text.index("kPhysicalActuationReleaseEnabled")
        self.assertLess(release_gate, wrapper_index)
        self.assertIn(
            "physical actuation is release-frozen before device preparation/open",
            self.text[:wrapper_index],
        )
        self.assertIn(
            "partial physical-backend enablement is rejected before device open",
            self.text,
        )
        self.assertIn("physicalDeviceOptionsAreComplete", self.text)
        self.assertIn(
            "physical serial open rejected because startup clocks ",
            self.text,
        )

    def test_post_open_boundary_is_zero_only_recovery_then_runtime(self):
        constructor = self.text[
            self.text.index("WheeltecVehicleExecutionNode(") :
            self.text.index("~WheeltecVehicleExecutionNode() noexcept")
        ]
        for token in (
            "PreparedPhysicalActivation",
            "VehicleExecutionCore",
            "createWallTimer",
        ):
            self.assertIn(token, constructor)
        self.assertRegex(
            constructor,
            r"createWallTimer\([\s\S]*?this,\s*false,\s*false\s*\)",
        )
        self.assertIn("config.backend, nullptr, operations", constructor)
        self.assertNotIn("openPhysicalSerial", constructor)

        activation = self.text[
            self.text.index("bool activatePhysical(") :
            self.text.index("bool activateDisabled(")
        ]
        guard = activation.index("physical_activation_->activate")
        stage = activation.index("setTransportForInitialization", guard)
        initialize = activation.index("initializeAndStart", stage)
        self.assertLess(guard, stage)
        self.assertLess(stage, initialize)
        self.assertIn("catch (...)", activation)
        self.assertIn("stopAfterActivationFailure", activation)
        self.assertIn("write_stream_poisoned", activation)

    def test_disabled_activation_never_opens_or_touches_transport(self):
        main = self.text[self.text.index("int main(") :]
        enabled_branch = main.index("if (config.real_device_enabled)")
        open_index = main.index("prepared_open.open()")
        disabled_index = main.index("wrapper.activateDisabled", open_index)
        self.assertLess(enabled_branch, open_index)
        self.assertLess(open_index, disabled_index)
        disabled = self.text[
            self.text.index("bool activateDisabled(") :
            self.text.index("private:", self.text.index("bool activateDisabled("))
        ]
        self.assertNotIn("prepared_open.open()", disabled)
        self.assertNotIn("physical_activation_->activate", disabled)

    def test_installed_bench_uses_same_no_record_zero_only_guard(self):
        prepare = self.bench.index("PreparedPhysicalActivation activation")
        prepare_open = self.bench.index(
            "PreparedPhysicalSerialOpen prepared_open", prepare
        )
        open_index = self.bench.index("prepared_open.open()", prepare_open)
        activate = self.bench.index("activation.activate", open_index)
        record = self.bench.index("openResultRecord(opened)", activate)
        session = self.bench.index("session.run", activate)
        self.assertLess(prepare, open_index)
        self.assertLess(prepare_open, open_index)
        self.assertLess(open_index, activate)
        self.assertLess(activate, record)
        self.assertLess(activate, session)
        self.assertLess(
            self.bench.index("kPhysicalActuationReleaseEnabled"),
            open_index,
        )

    def test_every_physical_open_caller_has_audited_access_semantics(self):
        callers = {
            path.relative_to(ROOT).as_posix()
            for path in ROOT.glob("src/**/*.cpp")
            if re.search(
                r"\b(?:PreparedPhysicalSerialOpen|openPhysicalSerial)\b",
                path.read_text(encoding="utf-8"),
            )
        }
        self.assertEqual(
            callers,
            {
                (
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                    "src/bench_main.cpp"
                ),
                (
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                    "src/feedback_capture_main.cpp"
                ),
                (
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                    "src/raw_profile_main.cpp"
                ),
                (
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                    "src/transport.cpp"
                ),
                (
                    "src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/"
                    "src/wheeltec_vehicle_execution_node.cpp"
                ),
            },
        )
        self.assertIn(
            "PhysicalAccessMode::kFeedbackOnly", self.feedback_capture
        )
        self.assertNotIn(
            "PhysicalAccessMode::kActuation", self.feedback_capture
        )

    def test_raw_profile_uses_parser_recovery_before_any_session_io(self):
        self.assertIn(
            '"auto_rover_vcu_wheeltec_serial/physical_activation.hpp"',
            self.raw_profile,
        )
        prepare = self.raw_profile.index(
            "PreparedPhysicalActivation activation"
        )
        prepare_open = self.raw_profile.index(
            "PreparedPhysicalSerialOpen prepared_open", prepare
        )
        open_index = self.raw_profile.index(
            "prepared_open.open()", prepare_open
        )
        activate = self.raw_profile.index("activation.activate", open_index)
        record = self.raw_profile.index("openResultRecord(opened", activate)
        session = self.raw_profile.index("session.run", activate)
        self.assertLess(prepare, prepare_open)
        self.assertLess(prepare_open, open_index)
        self.assertLess(open_index, activate)
        self.assertLess(activate, record)
        self.assertLess(activate, session)
        self.assertNotIn("openPhysicalSerial(physical)", self.raw_profile)
        self.assertNotIn(
            "writeRawProfileImmediateZeroNoRecord", self.raw_profile
        )

    def test_all_independent_actuation_gates_are_loaded(self):
        for parameter in (
            "real_device_enabled",
            "actuation_enabled",
            "unverified_protocol_acknowledged",
            "physical_device_opt_in",
            "actuation_opt_in",
            "readiness_gate_passed",
            "external_or_durable_estop_strategy_approved",
        ):
            self.assertIn(f'"{parameter}"', self.text)
        self.assertIn("runtimeActuationGatesAreSatisfied", self.text)
        self.assertIn("PhysicalAccessMode::kActuation", self.text)
        self.assertNotIn("PhysicalAccessMode::kFeedbackOnly", self.text)

    def test_feedback_identity_is_per_process_and_not_yaml_supplied(self):
        for token in (
            "ros::WallTime::now()",
            "ros::SteadyTime::now()",
            "getpid()",
            '"wheeltec_serial_candidate_v1#process=" + generation',
        ):
            self.assertIn(token, self.text)
        self.assertNotIn('"source_id"', self.text)
        self.assertNotIn('"process_generation_id"', self.text)

    def test_shutdown_attempts_stop_while_transport_is_alive(self):
        self.assertIn("runtime_->shutdown(monotonicNow())", self.text)
        self.assertIn("shutdown stop delivery is unconfirmed", self.text)
        transport_field = self.text.index(
            "std::unique_ptr<auto_rover::wheeltec_serial::ByteTransport> "
            "transport_"
        )
        runtime_field = self.text.index(
            "std::unique_ptr<auto_rover_vehicle::VehicleExecutionCore> runtime_"
        )
        self.assertLess(transport_field, runtime_field)

    def test_monotonic_worker_is_independent_of_ros_callback_queue(self):
        for token in (
            "std::thread(&WheeltecVehicleExecutionNode::workerLoop, this)",
            "std::chrono::steady_clock",
            "worker_wakeup_.wait_until",
            "WorkerTerminalReason::kDeadlineMiss",
            "WorkerTerminalReason::kCycleOverrun",
            "terminalWorkerStop",
            "tryStoreCycleResult",
            "publication_timer_",
            "two execution-worker periods must fit within the normal write gap",
            "runtime_->shutdown(monotonicNow())",
            "worker_.join()",
            "std::lock_guard<std::mutex> lock(core_mutex_)",
        ):
            self.assertIn(token, self.text)
        self.assertIn("void workerLoop() noexcept", self.text)
        self.assertRegex(
            self.text,
            r"void terminalWorkerStop\(WorkerTerminalReason reason\) noexcept",
        )
        self.assertIn("~WheeltecVehicleExecutionNode() noexcept", self.text)
        worker_body = self.text[
            self.text.index("void workerLoop() noexcept") :
            self.text.index("void publicationCallback")
        ]
        for forbidden in ("publishCycleResult", "ROS_", ".publish("):
            self.assertNotIn(forbidden, worker_body)
        self.assertIn("publishCycleResult(result)", self.text)
        self.assertIn("accepting_ros_callbacks_.store(false)", self.text)
        self.assertIn("ros::requestShutdown()", self.text)
        lock_index = worker_body.index(
            "std::lock_guard<std::mutex> lock(core_mutex_)"
        )
        locked_deadline_index = worker_body.index(
            "if (Clock::now() >= next_deadline + worker_period_)",
            lock_index,
        )
        cycle_index = worker_body.index("runtime_->cycle", lock_index)
        self.assertLess(lock_index, locked_deadline_index)
        self.assertLess(locked_deadline_index, cycle_index)
        self.assertNotIn("createSteadyTimer", self.text)
        self.assertNotIn("ros::SteadyTimer", self.text)

    def test_worker_mailbox_cannot_block_the_physical_watchdog(self):
        worker_body = self.text[
            self.text.index("void workerLoop() noexcept") :
            self.text.index("void publicationCallback")
        ]
        self.assertIn("std::try_to_lock", self.ordered_mailbox)
        self.assertIn("lock.owns_lock()", self.ordered_mailbox)
        self.assertIn("cycle_result_pending_ = false", self.ordered_mailbox)
        self.assertIn("OrderedCycleStoreResult::kLate", self.ordered_mailbox)
        self.assertIn(
            "OrderedCycleStoreResult::kStaleDropped",
            self.ordered_mailbox,
        )
        worker_mailbox = self.ordered_mailbox[
            self.ordered_mailbox.index("tryStoreCycleResult") :
            self.ordered_mailbox.index(
                "bool prepareDirectSafetyPublication"
            )
        ]
        self.assertIn("pending_cycle_result_.safety", worker_mailbox)
        self.assertNotIn("highWaterOf(", worker_mailbox)
        core_lock = worker_body.index(
            "std::lock_guard<std::mutex> lock(core_mutex_)"
        )
        mailbox = worker_body.index(
            "publication_mailbox_.tryStoreCycleResult", core_lock
        )
        late_stop = worker_body.index(
            "OrderedCycleStoreResult::kLate", mailbox
        )
        self.assertLess(core_lock, mailbox)
        self.assertLess(mailbox, late_stop)
        self.assertIn(
            "terminalWorkerStopLocked(WorkerTerminalReason::kMailboxFailure)",
            worker_body,
        )

    def test_direct_safety_transitions_order_before_ros_publication(self):
        self.assertEqual(
            self.text.count(
                "publication_mailbox_.prepareDirectSafetyPublication"
            ),
            4,
        )
        callback_names = (
            "void emergencyStopCallback",
            "bool armCallback",
            "bool assertEmergencyStopCallback",
            "bool resetCallback",
        )
        for index, callback_name in enumerate(callback_names):
            begin = self.text.index(callback_name)
            end = (
                self.text.index(callback_names[index + 1], begin)
                if index + 1 < len(callback_names)
                else self.text.index("ros::NodeHandle node_", begin)
            )
            callback = self.text[begin:end]
            core_lock = callback.index(
                "std::lock_guard<std::mutex> lock(core_mutex_)"
            )
            barrier = callback.index(
                "publication_mailbox_.prepareDirectSafetyPublication",
                core_lock,
            )
            core_unlock = callback.index("}", barrier)
            publish = callback.index(
                "publishPreparedDirectSafetyState", barrier
            )
            self.assertLess(core_lock, barrier)
            self.assertLess(barrier, core_unlock)
            self.assertLess(core_unlock, publish)

        direct_publisher = self.text[
            self.text.index("void publishPreparedDirectSafetyState") :
            self.text.index("ros::NodeHandle node_")
        ]
        direct_mutex = direct_publisher.index(
            "std::lock_guard<std::mutex> lock(safety_publication_mutex_)"
        )
        direct_claim = direct_publisher.index(
            "claimPreparedDirectSafetyPublication", direct_mutex
        )
        direct_publish = direct_publisher.index(
            "safety_publisher_.publish", direct_claim
        )
        self.assertLess(direct_mutex, direct_claim)
        self.assertLess(direct_claim, direct_publish)

        cycle_publisher = self.text[
            self.text.index("void publishCycleResult") :
            self.text.index("bool armCallback")
        ]
        cycle_mutex = cycle_publisher.index(
            "std::lock_guard<std::mutex> lock(safety_publication_mutex_)"
        )
        cycle_claim = cycle_publisher.index(
            "claimCycleSafetyPublication", cycle_mutex
        )
        cycle_publish = cycle_publisher.index(
            "safety_publisher_.publish", cycle_claim
        )
        self.assertLess(cycle_mutex, cycle_claim)
        self.assertLess(cycle_claim, cycle_publish)

        worker_body = self.text[
            self.text.index("void workerLoop() noexcept") :
            self.text.index("void publicationCallback")
        ]
        self.assertNotIn("safety_publication_mutex_", worker_body)
        self.assertIn(
            "claimPreparedDirectSafetyPublication", self.ordered_mailbox
        )
        self.assertIn("claimCycleSafetyPublication", self.ordered_mailbox)
        self.assertEqual(self.text.count("safety_publisher_.publish"), 2)

        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn(
            "auto_rover_wheeltec_ordered_publication_tests", cmake
        )

    def test_ros_inputs_are_bounded_before_conversion_or_core_lock(self):
        for token in (
            "kMaximumExecutionTrajectoryPoints",
            "kMaximumExecutionIdentifierBytes",
            "WorkerTerminalReason::kRosCallbackException",
        ):
            self.assertIn(token, self.text)
        callback = self.text.index("void trajectoryCallback")
        bound = self.text.index("executionInputIsBounded(*message)", callback)
        conversion = self.text.index(
            "auto_rover_ros1::toCore(*message)", callback
        )
        core_lock = self.text.index(
            "std::lock_guard<std::mutex> lock(core_mutex_)", callback
        )
        self.assertLess(bound, conversion)
        self.assertLess(conversion, core_lock)
        self.assertGreaterEqual(
            self.text.count("WorkerTerminalReason::kRosCallbackException"),
            8,
        )

    def test_destructor_attempts_physical_stop_before_ros_teardown(self):
        destructor_start = self.text.index(
            "~WheeltecVehicleExecutionNode() noexcept"
        )
        destructor = self.text[
            destructor_start : self.text.index("private:", destructor_start)
        ]
        self.assertIn("try {", destructor)
        self.assertLess(
            destructor.index("runtime_->shutdown(monotonicNow())"),
            destructor.index("ego_subscriber_.shutdown()"),
        )

    def test_stop_delivery_is_observable_at_every_ros_boundary(self):
        self.assertGreaterEqual(
            self.text.count("result.stop_delivery.delivery_unconfirmed"), 5
        )
        self.assertIn(
            "Emergency-stop latch ",
            self.text,
        )
        self.assertIn(
            "Wheeltec physical zero delivery is unconfirmed",
            self.text,
        )

    def test_node_is_built_installed_and_has_direct_ros_dependencies(self):
        self.assertIn("add_executable(wheeltec_vehicle_execution_node", self.cmake)
        self.assertRegex(
            self.cmake,
            r"install\(TARGETS[\s\S]*wheeltec_vehicle_execution_node",
        )
        self.assertIn("find_package(Threads REQUIRED)", self.cmake)
        self.assertIn("Threads::Threads", self.cmake)
        core_link = re.search(
            r"target_link_libraries\(auto_rover_vcu_wheeltec_serial\n"
            r"([\s\S]*?)\n\)",
            self.cmake,
        )
        self.assertIsNotNone(core_link)
        self.assertNotIn("${catkin_LIBRARIES}", core_link.group(1))
        node_link = re.search(
            r"target_link_libraries\(wheeltec_vehicle_execution_node\n"
            r"([\s\S]*?)\n\)",
            self.cmake,
        )
        self.assertIsNotNone(node_link)
        self.assertIn("${catkin_LIBRARIES}", node_link.group(1))
        for dependency in (
            "auto_rover_interfaces",
            "auto_rover_ros1_conversions",
            "roscpp",
        ):
            self.assertIn(f"<build_depend>{dependency}</build_depend>", self.package)
            self.assertIn(
                f"<build_export_depend>{dependency}</build_export_depend>",
                self.package,
            )
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", self.package)

    def test_limits_are_parameters_with_strict_phase1_composition(self):
        self.assertIn('"codec/max_forward_speed_mps"', self.text)
        self.assertIn("runtimeConfigIsStructurallyValid", self.text)
        self.assertIn(
            "codec speed limit must equal the selected vehicle-profile limit",
            self.text,
        )
        self.assertNotRegex(
            self.text,
            re.compile(r"max_forward_speed_mps\s*=\s*0\.5"),
        )


if __name__ == "__main__":
    unittest.main()
