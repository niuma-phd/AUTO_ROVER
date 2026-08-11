#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <string>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_safety/command_guard.hpp"
#include "auto_rover_safety/safety_supervisor.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

auto_rover::VehicleProfile profile() {
  auto_rover::VehicleProfile value;
  value.schema_version = 1U;
  value.profile_id = "nuc_senior_akm_v1";
  value.kinematic_model = auto_rover::KinematicModel::kAckermannBicycle;
  value.direction_capability =
      auto_rover::DirectionCapability::kSignedSpeedDirection;
  value.reference_frame = "rear_axle_center";
  value.wheelbase_m = 0.3187;
  value.max_forward_speed_mps = 0.50;
  value.max_longitudinal_accel_mps2 = 0.20;
  value.min_turning_radius_m = 0.95;
  value.reverse_supported = false;
  return value;
}

auto_rover_safety::SafetySupervisorConfig supervisorConfig(
    bool actuation_enabled) {
  auto_rover_safety::SafetySupervisorConfig value;
  value.actuation_enabled = actuation_enabled;
  value.reset_service_enabled = true;
  value.fresh_recovery_count = 3U;
  value.state_valid_for_ns = 100000000LL;
  return value;
}

void prepareFreshDisarmed(auto_rover_safety::SafetySupervisor* supervisor) {
  supervisor->completeBoot(1000000000LL);
  supervisor->observeConditions(true, auto_rover::StopReason::kNone,
                                1010000000LL);
  supervisor->observeConditions(true, auto_rover::StopReason::kNone,
                                1020000000LL);
  supervisor->observeConditions(true, auto_rover::StopReason::kNone,
                                1030000000LL);
}

void testDefaultDisabledAndFreshRecovery() {
  auto_rover_safety::SafetySupervisor disabled(supervisorConfig(false));
  prepareFreshDisarmed(&disabled);
  const auto state = disabled.currentState(1040000000LL);
  expect(state.mode == auto_rover::SafetyMode::kDisarmed,
         "fresh inputs reach disarmed state without arming");
  const auto rejected =
      disabled.requestArm("operator", state.state_id, 1050000000LL);
  expect(!rejected.accepted, "actuation-disabled configuration cannot arm");

  auto_rover_safety::SafetySupervisor enabled(supervisorConfig(true));
  enabled.completeBoot(1000000000LL);
  enabled.observeConditions(true, auto_rover::StopReason::kNone,
                            1010000000LL);
  enabled.observeConditions(true, auto_rover::StopReason::kNone,
                            1020000000LL);
  expect(enabled.currentState(1020000000LL).mode ==
             auto_rover::SafetyMode::kRecoveryInhibited,
         "one or two fresh observations cannot recover");
  enabled.observeConditions(true, auto_rover::StopReason::kNone,
                            1030000000LL);
  const auto ready = enabled.currentState(1030000000LL);
  const auto armed =
      enabled.requestArm("operator", ready.state_id, 1040000000LL);
  expect(armed.accepted, "explicit arm succeeds after full fresh run");
  expect(armed.state.mode == auto_rover::SafetyMode::kArmed,
         "successful request enters armed state");

  const auto fault = enabled.observeConditions(
      false, auto_rover::StopReason::kStaleLocalization, 1050000000LL);
  expect(fault.mode == auto_rover::SafetyMode::kFaultInhibited,
         "input failure disarms and inhibits");
  enabled.observeConditions(true, auto_rover::StopReason::kNone,
                            1060000000LL);
  expect(enabled.currentState(1060000000LL).mode ==
             auto_rover::SafetyMode::kRecoveryInhibited,
         "ordinary recovery requires a new consecutive run");
}

void testLatchedEmergencyStopAndAuthorizedReset() {
  auto_rover_safety::SafetySupervisor supervisor(supervisorConfig(true));
  prepareFreshDisarmed(&supervisor);
  auto ready = supervisor.currentState(1040000000LL);
  expect(supervisor.requestArm("operator", ready.state_id, 1050000000LL)
             .accepted,
         "precondition arm succeeds");

  auto_rover::EmergencyStop request;
  request.stamp_ns = 1060000000LL;
  request.request_id = "panel:1";
  request.source_id = "panel";
  request.asserted = true;
  const auto latched = supervisor.assertEmergencyStop(request, 1060000000LL);
  expect(latched.accepted, "valid emergency stop assertion is accepted");
  expect(latched.state.mode == auto_rover::SafetyMode::kEmergencyStopLatched,
         "emergency stop latches independently");
  const std::uint64_t generation = latched.state.latch_generation;

  request.asserted = false;
  const auto false_message =
      supervisor.assertEmergencyStop(request, 1070000000LL);
  expect(!false_message.accepted,
         "false assertion cannot auto-clear emergency stop");
  expect(false_message.state.mode ==
             auto_rover::SafetyMode::kEmergencyStopLatched,
         "latch survives a false message");

  auto_rover_safety::ResetEmergencyStopRequest reset;
  reset.operator_id = "operator";
  reset.latch_generation = generation;
  reset.conditions_cleared_acknowledged = true;
  reset.authorization_granted = false;
  expect(!supervisor.resetEmergencyStop(reset, 1080000000LL).accepted,
         "unauthorized reset has no effect");
  reset.authorization_granted = true;
  reset.latch_generation = generation + 1U;
  expect(!supervisor.resetEmergencyStop(reset, 1090000000LL).accepted,
         "generation mismatch prevents stale reset replay");

  reset.latch_generation = generation;
  supervisor.observeConditions(false, auto_rover::StopReason::kInvalidInput,
                               1100000000LL);
  expect(!supervisor.resetEmergencyStop(reset, 1110000000LL).accepted,
         "reset fails while trigger conditions remain");
  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1120000000LL);
  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1130000000LL);
  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1140000000LL);
  const auto cleared = supervisor.resetEmergencyStop(reset, 1150000000LL);
  expect(cleared.accepted, "authorized reset succeeds after conditions clear");
  expect(cleared.state.mode == auto_rover::SafetyMode::kDisarmed,
         "reset returns to disarmed and never directly to motion");
}

void testInvalidTimeAndRepeatedBootCannotBypassEmergencyStop() {
  auto_rover_safety::SafetySupervisor arm_supervisor(supervisorConfig(true));
  prepareFreshDisarmed(&arm_supervisor);
  const auto ready = arm_supervisor.currentState(1040000000LL);
  const auto invalid_time_arm =
      arm_supervisor.requestArm("operator", ready.state_id, 0LL);
  expect(!invalid_time_arm.accepted,
         "an arm request with invalid ROS time is rejected");
  expect(!invalid_time_arm.state.valid,
         "an invalid-time arm response cannot advertise valid state");
  const auto after_invalid_arm = arm_supervisor.currentState(1050000000LL);
  expect(after_invalid_arm.mode == auto_rover::SafetyMode::kDisarmed &&
             after_invalid_arm.state_id == ready.state_id,
         "an invalid-time arm request cannot arm later when time recovers");

  auto_rover_safety::SafetySupervisor supervisor(supervisorConfig(true));
  prepareFreshDisarmed(&supervisor);
  auto_rover::EmergencyStop stop;
  stop.stamp_ns = 1060000000LL;
  stop.request_id = "panel:invalid-time-regression";
  stop.source_id = "panel";
  stop.asserted = true;
  const auto latched = supervisor.assertEmergencyStop(stop, 1060000000LL);
  expect(latched.accepted, "emergency-stop regression precondition latches");
  const std::uint64_t latched_state_id = latched.state.state_id;
  const std::uint64_t generation = latched.state.latch_generation;

  const auto repeated_boot = supervisor.completeBoot(1070000000LL);
  expect(repeated_boot.mode == auto_rover::SafetyMode::kEmergencyStopLatched &&
             repeated_boot.state_id == latched_state_id &&
             repeated_boot.latch_generation == generation,
         "repeating boot completion cannot clear or replace an emergency-stop latch");
  const auto invalid_time_boot = supervisor.completeBoot(0LL);
  expect(invalid_time_boot.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             invalid_time_boot.state_id == latched_state_id &&
             invalid_time_boot.latch_generation == generation,
         "invalid-time boot completion cannot clear an emergency-stop latch");

  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1080000000LL);
  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1090000000LL);
  supervisor.observeConditions(true, auto_rover::StopReason::kNone,
                               1100000000LL);
  auto_rover_safety::ResetEmergencyStopRequest reset;
  reset.operator_id = "operator";
  reset.latch_generation = generation;
  reset.conditions_cleared_acknowledged = true;
  reset.authorization_granted = true;
  const auto invalid_time_reset = supervisor.resetEmergencyStop(reset, 0LL);
  expect(!invalid_time_reset.accepted,
         "an emergency-stop reset with invalid ROS time is rejected");
  expect(invalid_time_reset.state.mode ==
             auto_rover::SafetyMode::kEmergencyStopLatched,
         "invalid-time reset leaves the emergency stop latched");
  expect(supervisor.currentState(1110000000LL).mode ==
             auto_rover::SafetyMode::kEmergencyStopLatched,
         "invalid-time reset cannot take effect after time recovers");

  const auto invalid_observation = supervisor.observeConditions(
      true, auto_rover::StopReason::kNone, 0LL);
  expect(invalid_observation.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             invalid_observation.latch_generation == generation,
         "invalid-time condition observation preserves latch priority");
  expect(!supervisor.resetEmergencyStop(reset, 1120000000LL).accepted,
         "invalid-time condition observation invalidates reset recovery evidence");
}

auto_rover::EgoState ego() {
  auto_rover::EgoState value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "camera_init";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kPublishTime;
  value.reference_frame = "rear_axle_center";
  value.pose.orientation.w = 1.0;
  value.source_id = "synthetic";
  value.valid = true;
  return value;
}

auto_rover::Trajectory trajectory() {
  auto_rover::Trajectory value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "camera_init";
  value.trajectory_id = "route:1:1";
  value.route_id = "route";
  value.plan_version = 1U;
  value.vehicle_profile_id = "nuc_senior_akm_v1";
  value.valid_for_ns = 1000000000LL;
  value.completion_behavior = auto_rover::CompletionBehavior::kStopAndHold;
  value.valid = true;
  value.points.push_back(
      {0.0, 0.0, 0.0, 0.0, 0.5, 0.0, auto_rover::Direction::kForward});
  value.points.push_back(
      {2.0, 0.0, 0.0, 0.0, 0.5, 2.0, auto_rover::Direction::kForward});
  return value;
}

auto_rover::MotionReference motion() {
  auto_rover::MotionReference value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "rear_axle_center";
  value.command_id = 1U;
  value.producer_generation_id = "safety-test-tracker-generation-1";
  value.trajectory_id = "route:1:1";
  value.direction = auto_rover::Direction::kForward;
  value.target_speed_mps = 0.02;
  value.valid_for_ns = 200000000LL;
  value.valid = true;
  return value;
}

auto_rover::ChassisState chassis() {
  auto_rover::ChassisState value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "rear_axle_center";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kReceiptTime;
  value.valid_mask = auto_rover::ChassisState::kMeasuredSpeedValid;
  value.source_id = "fake_vcu";
  value.valid = true;
  return value;
}

auto_rover::SafetyState armedSafety() {
  auto_rover::SafetyState value;
  value.stamp_ns = 1000000000LL;
  value.state_id = 9U;
  value.mode = auto_rover::SafetyMode::kArmed;
  value.valid_for_ns = 200000000LL;
  value.valid = true;
  return value;
}

auto_rover_safety::GuardConfig guardConfig() {
  auto_rover_safety::GuardConfig value;
  value.world_frame = "camera_init";
  value.localization_freshness_ns = 100000000LL;
  value.trajectory_freshness_ns = 100000000LL;
  value.motion_freshness_ns = 100000000LL;
  value.chassis_freshness_ns = 100000000LL;
  value.safety_freshness_ns = 100000000LL;
  value.fresh_recovery_count = 3U;
  return value;
}

auto_rover_safety::GuardInput guardInput(std::int64_t now_mono,
                                         std::int64_t now_ros) {
  const std::int64_t receipt = now_mono - 10000000LL;
  auto_rover_safety::GuardInput input;
  input.ego = {ego(), receipt};
  input.trajectory = {trajectory(), receipt};
  input.motion = {motion(), receipt};
  input.chassis = {chassis(), receipt};
  input.safety = {armedSafety(), receipt};
  input.now_monotonic_ns = now_mono;
  input.now_ros_ns = now_ros;
  return input;
}

void testGuardIndependentFreshnessAndRecovery() {
  auto_rover_safety::CommandGuard guard(guardConfig(), profile());
  for (int count = 0; count < 2; ++count) {
    auto input = guardInput(1050000000LL + count * 10000000LL,
                            1050000000LL + count * 10000000LL);
    input.motion.value.command_id =
        static_cast<std::uint64_t>(count + 98);
    const auto result = guard.evaluate(input);
    expect(!result.execution_allowed,
           "guard requires configured consecutive fresh run");
    expect(result.reason == auto_rover::StopReason::kRecoveryPending,
           "recovery stop reason is explicit");
  }
  auto third = guardInput(1070000000LL, 1070000000LL);
  third.motion.value.command_id = 100U;
  const auto ready = guard.evaluate(third);
  expect(ready.execution_allowed,
         "third consecutive fresh set enables guarded execution");
  expect(ready.reference.target_speed_mps == 0.02,
         "guard preserves a valid bounded reference");

  auto same_receipt_restart = third;
  same_receipt_restart.motion.value.producer_generation_id =
      "safety-test-tracker-generation-2";
  same_receipt_restart.motion.value.command_id = 1U;
  const auto same_receipt_restart_result =
      guard.evaluate(same_receipt_restart);
  expect(!same_receipt_restart_result.execution_allowed &&
             same_receipt_restart_result.reason ==
                 auto_rover::StopReason::kInvalidInput,
         "a producer generation cannot change within one received snapshot");

  auto restarted = guardInput(1080000000LL, 1080000000LL);
  restarted.motion.value.producer_generation_id =
      "safety-test-tracker-generation-2";
  restarted.motion.value.command_id = 1U;
  auto restarted_result = guard.evaluate(restarted);
  expect(!restarted_result.execution_allowed &&
             restarted_result.reason ==
                 auto_rover::StopReason::kRecoveryPending &&
             guard.consecutiveFreshCount() == 1U,
         "a newer tracker generation may restart command identity at one but must rebuild recovery");
  for (std::uint64_t command_id = 2U; command_id <= 3U; ++command_id) {
    const std::int64_t now =
        1080000000LL + static_cast<std::int64_t>(command_id - 1U) *
                             10000000LL;
    restarted = guardInput(now, now);
    restarted.motion.value.producer_generation_id =
        "safety-test-tracker-generation-2";
    restarted.motion.value.command_id = command_id;
    restarted_result = guard.evaluate(restarted);
  }
  expect(restarted_result.execution_allowed,
         "fresh commands from the restarted tracker recover without the old generation high-water mark");

  auto retired_replay = guardInput(1110000000LL, 1110000000LL);
  retired_replay.motion.value.producer_generation_id =
      "safety-test-tracker-generation-1";
  retired_replay.motion.value.command_id = 101U;
  const auto retired_result = guard.evaluate(retired_replay);
  expect(!retired_result.execution_allowed &&
             retired_result.reason == auto_rover::StopReason::kInvalidInput &&
             retired_result.diagnostic ==
                 "motion producer generation is retired",
         "a retired tracker generation replay fails closed even with a higher command identity");

  auto_rover_safety::CommandGuard duplicate_guard(guardConfig(), profile());
  auto duplicate = guardInput(1080000000LL, 1080000000LL);
  duplicate.motion.value.command_id = 20U;
  expect(!duplicate_guard.evaluate(duplicate).execution_allowed,
         "first fresh command begins recovery");
  const auto same_snapshot = duplicate_guard.evaluate(duplicate);
  expect(!same_snapshot.execution_allowed &&
             duplicate_guard.consecutiveFreshCount() == 1U,
         "re-evaluating one Received snapshot cannot advance recovery");
  duplicate.motion.receipt_monotonic_ns += 1000000LL;
  const auto replayed_identity = duplicate_guard.evaluate(duplicate);
  expect(!replayed_identity.execution_allowed &&
             replayed_identity.reason ==
                 auto_rover::StopReason::kInvalidInput,
         "a republished motion command identity fails closed");

  struct Case {
    enum class Field { kEgo, kTrajectory, kMotion, kChassis, kSafety } field;
    auto_rover::StopReason expected;
  };
  const Case cases[] = {
      {Case::Field::kEgo, auto_rover::StopReason::kStaleLocalization},
      {Case::Field::kTrajectory, auto_rover::StopReason::kStaleTrajectory},
      {Case::Field::kMotion, auto_rover::StopReason::kStaleMotionReference},
      {Case::Field::kChassis, auto_rover::StopReason::kStaleChassisState},
      {Case::Field::kSafety, auto_rover::StopReason::kInvalidInput},
  };
  for (const auto& test_case : cases) {
    auto_rover_safety::CommandGuard local_guard(guardConfig(), profile());
    auto input = guardInput(1200000000LL, 1100000000LL);
    const std::int64_t stale = 1000000000LL;
    if (test_case.field == Case::Field::kEgo) input.ego.receipt_monotonic_ns = stale;
    if (test_case.field == Case::Field::kTrajectory)
      input.trajectory.receipt_monotonic_ns = stale;
    if (test_case.field == Case::Field::kMotion)
      input.motion.receipt_monotonic_ns = stale;
    if (test_case.field == Case::Field::kChassis)
      input.chassis.receipt_monotonic_ns = stale;
    if (test_case.field == Case::Field::kSafety)
      input.safety.receipt_monotonic_ns = stale;
    const auto result = local_guard.evaluate(input);
    expect(!result.execution_allowed, "each stale required input fails closed");
    expect(result.reason == test_case.expected,
           "stale input reports its guard boundary reason");
    expect(result.reference.target_speed_mps == 0.0,
           "stale input produces explicit zero reference");
  }

  auto mismatch = guardInput(1080000000LL, 1080000000LL);
  mismatch.motion.value.command_id = 4U;
  mismatch.motion.value.trajectory_id = "different";
  const auto mismatch_result = guard.evaluate(mismatch);
  expect(!mismatch_result.execution_allowed,
         "trajectory identity mismatch fails closed");
  expect(guard.consecutiveFreshCount() == 0U,
         "any failure resets the fresh recovery run");
}

void testMotionProducerGenerationHistoryIsBounded() {
  auto_rover_safety::CommandGuard guard(guardConfig(), profile());
  const std::int64_t fixed_ros_ns = 1050000000LL;
  auto_rover_safety::GuardResult result;
  for (std::size_t generation = 0U;
       generation <=
           auto_rover_safety::kMaximumMotionProducerGenerationHistoryEntries;
       ++generation) {
    const std::int64_t now_monotonic_ns =
        1050000000LL + static_cast<std::int64_t>(generation) * 1000000LL;
    auto input = guardInput(now_monotonic_ns, fixed_ros_ns);
    input.motion.value.producer_generation_id =
        "bounded-motion-generation-" + std::to_string(generation);
    input.motion.value.command_id = 1U;
    result = guard.evaluate(input);
    expect(result.reason == auto_rover::StopReason::kRecoveryPending,
           "each generation through the fixed history capacity remains bounded recovery work");
  }

  const std::size_t rejected_generation =
      auto_rover_safety::kMaximumMotionProducerGenerationHistoryEntries + 1U;
  auto over_limit = guardInput(
      1050000000LL +
          static_cast<std::int64_t>(rejected_generation) * 1000000LL,
      fixed_ros_ns);
  over_limit.motion.value.producer_generation_id =
      "bounded-motion-generation-" + std::to_string(rejected_generation);
  over_limit.motion.value.command_id = 1U;
  result = guard.evaluate(over_limit);
  expect(!result.execution_allowed &&
             result.reason == auto_rover::StopReason::kInvalidInput &&
             result.diagnostic ==
                 "motion producer generation history reached its limit" &&
             guard.consecutiveFreshCount() == 0U,
         "a producer generation beyond the fixed history capacity fails closed");
}

}  // namespace

int main() {
  testDefaultDisabledAndFreshRecovery();
  testLatchedEmergencyStopAndAuthorizedReset();
  testInvalidTimeAndRepeatedBootCannotBypassEmergencyStop();
  testGuardIndependentFreshnessAndRecovery();
  testMotionProducerGenerationHistoryIsBounded();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "safety tests passed\n";
  return EXIT_SUCCESS;
}
