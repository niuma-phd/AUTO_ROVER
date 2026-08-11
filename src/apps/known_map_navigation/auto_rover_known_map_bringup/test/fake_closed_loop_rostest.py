#!/usr/bin/env python3
"""Bounded, hardware-free rostest for the Phase-1 fake navigation loop."""

import threading
import time
import unittest

import rospy
import rostest
from auto_rover_interfaces.msg import (
    ChassisState,
    EgoState,
    MotionReference,
    SafetyState,
)
from auto_rover_interfaces.srv import (
    ArmVehicle,
    ArmVehicleRequest,
    AssertEmergencyStop,
    AssertEmergencyStopRequest,
    ResetEmergencyStop,
    ResetEmergencyStopRequest,
)
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest


class FakeClosedLoopRostest(unittest.TestCase):
    MAX_FORWARD_SPEED_MPS = 0.50
    MAX_ACCEL_MPS2 = 0.20
    STOPPED_SPEED_MPS = 0.005

    def setUp(self):
        self.disabled_namespace = rospy.get_param(
            "~disabled_namespace", "/default_disabled"
        ).rstrip("/")
        self.enabled_namespace = rospy.get_param(
            "~enabled_namespace", "/test_enabled"
        ).rstrip("/")
        self._lock = threading.Lock()
        self._history = {}
        self._subscribers = []
        for namespace in (self.disabled_namespace, self.enabled_namespace):
            self._subscribe(namespace, "chassis_state", ChassisState)
            self._subscribe(namespace, "ego_state", EgoState)
            self._subscribe(namespace, "motion_reference", MotionReference)
            self._subscribe(namespace, "safety_state", SafetyState)
            self._subscribe(namespace, "aft_mapped_to_init", Odometry)

    def tearDown(self):
        for subscriber in self._subscribers:
            subscriber.unregister()

    @staticmethod
    def _join(namespace, name):
        return "{}/{}".format(namespace, name)

    def _subscribe(self, namespace, name, message_type):
        key = self._join(namespace, name)
        with self._lock:
            self._history[key] = []

        def callback(message, history_key=key):
            with self._lock:
                history = self._history[history_key]
                history.append((time.monotonic(), message))
                if len(history) > 2000:
                    del history[:-1000]

        self._subscribers.append(
            rospy.Subscriber(key, message_type, callback, queue_size=1)
        )

    def _snapshot(self, namespace, name, since=None):
        key = self._join(namespace, name)
        with self._lock:
            messages = list(self._history.get(key, ()))
        if since is None:
            return messages
        return [(stamp, message) for stamp, message in messages if stamp >= since]

    def _latest(self, namespace, name):
        messages = self._snapshot(namespace, name)
        return None if not messages else messages[-1][1]

    def _wait_until(self, predicate, timeout_s, description):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            result = predicate()
            if result:
                return result
            time.sleep(0.02)
        self.fail("timed out waiting for {} after {:.1f} s".format(
            description, timeout_s
        ))

    def _wait_for_mode(self, namespace, mode, timeout_s=8.0):
        return self._wait_until(
            lambda: (
                message
                if (message := self._latest(namespace, "safety_state"))
                is not None
                and message.valid
                and message.mode == mode
                else None
            ),
            timeout_s,
            "{} safety mode {}".format(namespace, mode),
        )

    def _wait_for_service(self, service_name, service_type, timeout_s=6.0):
        try:
            rospy.wait_for_service(service_name, timeout=timeout_s)
        except rospy.ROSException as error:
            self.fail("service {} was unavailable: {}".format(
                service_name, error
            ))
        return rospy.ServiceProxy(service_name, service_type)

    def _arm(self, namespace, safety_state, operator_id):
        service = self._wait_for_service(
            self._join(namespace, "arm_vehicle"), ArmVehicle
        )
        request = ArmVehicleRequest()
        request.operator_id = operator_id
        request.safety_generation = safety_state.state_id
        request.arm = True
        return service(request)

    def _assert_stationary_window(self, namespace, duration_s):
        start = time.monotonic()
        deadline = start + duration_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(0.02)
        samples = self._snapshot(namespace, "chassis_state", since=start)
        self.assertGreaterEqual(
            len(samples), 5, "expected fresh fake chassis feedback"
        )
        maximum = max(abs(message.measured_speed_mps) for _, message in samples)
        self.assertLessEqual(maximum, 1.0e-9)

    def _first_positive(self, namespace, name, since, timeout_s=7.0):
        def find_sample():
            for stamp, message in self._snapshot(namespace, name, since=since):
                if name == "chassis_state":
                    value = message.measured_speed_mps
                else:
                    value = message.target_speed_mps
                if value > 1.0e-5:
                    return stamp, message
            return None

        return self._wait_until(
            find_sample,
            timeout_s,
            "a positive {} in {}".format(name, namespace),
        )

    def _assert_chassis_acceleration_bound(self, namespace, since):
        samples = self._snapshot(namespace, "chassis_state", since=since)
        increasing_pairs = 0
        for (_, previous), (_, current) in zip(samples, samples[1:]):
            elapsed = (current.header.stamp - previous.header.stamp).to_sec()
            speed_change = (
                current.measured_speed_mps - previous.measured_speed_mps
            )
            if elapsed <= 0.01 or speed_change <= 1.0e-6:
                continue
            increasing_pairs += 1
            observed_acceleration = speed_change / elapsed
            self.assertLessEqual(
                observed_acceleration,
                self.MAX_ACCEL_MPS2 + 0.05,
                "fake feedback exceeded the configured acceleration ramp",
            )
        self.assertGreaterEqual(
            increasing_pairs, 2, "expected multiple samples on the speed ramp"
        )

    def test_default_safe_arm_fault_recovery_and_estop_latch(self):
        disabled_ready = self._wait_for_mode(
            self.disabled_namespace, SafetyState.MODE_DISARMED
        )
        self._wait_until(
            lambda: self._latest(self.disabled_namespace, "ego_state"),
            5.0,
            "default-disabled localization output",
        )
        self._assert_stationary_window(self.disabled_namespace, 1.0)

        raw_pose = self._latest(self.disabled_namespace, "aft_mapped_to_init")
        rear_pose = self._latest(self.disabled_namespace, "ego_state")
        self.assertIsNotNone(raw_pose)
        self.assertIsNotNone(rear_pose)
        self.assertAlmostEqual(
            0.20,
            raw_pose.pose.pose.position.x - rear_pose.pose.pose.position.x,
            delta=1.0e-6,
            msg="fake localization must exercise the explicit nonidentity extrinsic",
        )

        rejected = self._arm(
            self.disabled_namespace, disabled_ready, "rostest_disabled"
        )
        self.assertFalse(rejected.success)
        self.assertIn("actuation is disabled", rejected.reason)
        self._assert_stationary_window(self.disabled_namespace, 0.75)

        # The enabled test stack has deliberately remained disarmed throughout
        # the default-safety checks. This catches regressions where a tracker
        # silently ramps to its target while control is not enabled.
        enabled_ready = self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_DISARMED
        )
        pre_arm_motion = self._latest(
            self.enabled_namespace, "motion_reference"
        )
        self.assertIsNotNone(pre_arm_motion)
        self.assertTrue(
            pre_arm_motion.producer_generation_id,
            "the tracker node must publish an explicit producer generation",
        )
        enabled_tracker_generation = pre_arm_motion.producer_generation_id
        self.assertAlmostEqual(0.0, pre_arm_motion.target_speed_mps, delta=1.0e-9)
        self._assert_stationary_window(self.enabled_namespace, 0.50)

        arm_started = time.monotonic()
        armed = self._arm(
            self.enabled_namespace, enabled_ready, "rostest_operator"
        )
        self.assertTrue(armed.success, armed.reason)
        self.assertEqual(SafetyState.MODE_ARMED, armed.safety_mode)

        _, first_motion = self._first_positive(
            self.enabled_namespace,
            "motion_reference",
            since=arm_started,
        )
        self.assertGreater(first_motion.target_speed_mps, 0.0)
        self.assertEqual(
            enabled_tracker_generation,
            first_motion.producer_generation_id,
            "one running tracker must retain its producer generation while command IDs advance",
        )
        self.assertGreater(first_motion.command_id, pre_arm_motion.command_id)
        self.assertLess(
            first_motion.target_speed_mps,
            self.MAX_FORWARD_SPEED_MPS,
            "the first nonzero command must be a ramp value, not a 0.50 m/s jump",
        )

        _, first_feedback = self._first_positive(
            self.enabled_namespace, "chassis_state", since=arm_started
        )
        self.assertGreater(first_feedback.measured_speed_mps, 0.0)
        self.assertLess(
            first_feedback.measured_speed_mps, self.MAX_FORWARD_SPEED_MPS
        )

        initial_ego = self._latest(self.enabled_namespace, "ego_state")
        self.assertIsNotNone(initial_ego)
        initial_x = initial_ego.pose.pose.position.x
        self._wait_until(
            lambda: (
                ego
                if (ego := self._latest(self.enabled_namespace, "ego_state"))
                is not None
                and ego.pose.pose.position.x >= initial_x + 0.03
                else None
            ),
            6.0,
            "closed-loop forward pose progress",
        )
        self._assert_chassis_acceleration_bound(
            self.enabled_namespace, arm_started
        )

        fault_service = self._wait_for_service(
            self._join(
                self.enabled_namespace, "vehicle_execution/fake_faulted"
            ),
            SetBool,
        )
        fault_request = SetBoolRequest(data=True)
        faulted = fault_service(fault_request)
        self.assertTrue(faulted.success, faulted.message)
        self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_FAULT_INHIBITED
        )
        self._wait_until(
            lambda: (
                message
                if (message := self._latest(
                    self.enabled_namespace, "chassis_state"
                ))
                is not None
                and abs(message.measured_speed_mps) <= self.STOPPED_SPEED_MPS
                else None
            ),
            5.0,
            "fake VCU ramped stop after fault injection",
        )

        fault_state = self._latest(self.enabled_namespace, "safety_state")
        arm_while_faulted = self._arm(
            self.enabled_namespace, fault_state, "rostest_operator"
        )
        self.assertFalse(arm_while_faulted.success)

        clear_request = SetBoolRequest(data=False)
        cleared = fault_service(clear_request)
        self.assertTrue(cleared.success, cleared.message)
        recovered = self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_DISARMED
        )
        rearm_started = time.monotonic()
        rearmed = self._arm(
            self.enabled_namespace, recovered, "rostest_operator"
        )
        self.assertTrue(rearmed.success, rearmed.reason)
        _, restarted_feedback = self._first_positive(
            self.enabled_namespace, "chassis_state", since=rearm_started
        )
        self.assertLess(
            restarted_feedback.measured_speed_mps,
            self.MAX_FORWARD_SPEED_MPS,
        )

        rearmed_state = self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_ARMED
        )
        previous_latch_generation = rearmed_state.latch_generation
        estop_service = self._wait_for_service(
            self._join(self.enabled_namespace, "assert_emergency_stop"),
            AssertEmergencyStop,
        )
        estop_request = AssertEmergencyStopRequest()
        estop_request.schema_version = 1
        estop_request.source_stamp = rospy.Time.now()
        estop_request.request_id = "rostest-estop-1"
        estop_request.source_id = "fake_closed_loop_rostest"
        self.assertFalse(estop_request.source_stamp.is_zero())
        asserted = estop_service(estop_request)
        self.assertTrue(asserted.success, asserted.reason)
        self.assertEqual(
            SafetyState.MODE_ESTOP_LATCHED, asserted.safety_mode
        )
        self.assertGreater(
            asserted.latch_generation, previous_latch_generation
        )

        latched = self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_ESTOP_LATCHED
        )
        self.assertEqual(asserted.latch_generation, latched.latch_generation)
        self._wait_until(
            lambda: (
                message
                if (message := self._latest(
                    self.enabled_namespace, "chassis_state"
                ))
                is not None
                and abs(message.measured_speed_mps) <= self.STOPPED_SPEED_MPS
                else None
            ),
            5.0,
            "fake VCU ramped stop after acknowledged emergency stop",
        )

        arm_while_latched = self._arm(
            self.enabled_namespace, latched, "rostest_operator"
        )
        self.assertFalse(arm_while_latched.success)
        self.assertEqual(
            SafetyState.MODE_ESTOP_LATCHED,
            arm_while_latched.safety_mode,
        )

        reset_service = self._wait_for_service(
            self._join(self.enabled_namespace, "reset_emergency_stop"),
            ResetEmergencyStop,
        )
        reset_request = ResetEmergencyStopRequest()
        reset_request.operator_id = "rostest_operator"
        reset_request.latch_generation = latched.latch_generation
        reset_request.conditions_cleared_acknowledged = True
        reset = reset_service(reset_request)
        self.assertFalse(reset.success)
        self.assertIn("authorization boundary rejected", reset.reason)
        self.assertEqual(SafetyState.MODE_ESTOP_LATCHED, reset.safety_mode)
        self.assertEqual(
            latched.latch_generation, reset.current_latch_generation
        )
        still_latched = self._wait_for_mode(
            self.enabled_namespace, SafetyState.MODE_ESTOP_LATCHED
        )
        self.assertEqual(
            latched.latch_generation, still_latched.latch_generation
        )


if __name__ == "__main__":
    rospy.init_node("fake_closed_loop_rostest")
    rostest.rosrun(
        "auto_rover_known_map_bringup",
        "fake_closed_loop_rostest",
        FakeClosedLoopRostest,
    )
