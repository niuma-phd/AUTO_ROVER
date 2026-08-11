#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "auto_rover_control/pure_pursuit.hpp"
#include "auto_rover_core/geometry.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

bool contains(const std::string& value, const std::string& fragment) {
  return value.find(fragment) != std::string::npos;
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

auto_rover_control::PurePursuitConfig config() {
  auto_rover_control::PurePursuitConfig value;
  value.world_frame = "camera_init";
  value.control_frame = "rear_axle_center";
  value.producer_generation_id = "pure-pursuit-test-generation-1";
  value.lookahead_min_m = 0.40;
  value.lookahead_max_m = 0.80;
  value.lookahead_speed_gain_s = 0.50;
  value.goal_position_tolerance_m = 0.08;
  value.standstill_speed_threshold_mps = 0.01;
  value.localization_freshness_ns = 200000000LL;
  value.trajectory_freshness_ns = 500000000LL;
  value.chassis_freshness_ns = 200000000LL;
  value.safety_freshness_ns = 200000000LL;
  value.motion_valid_for_ns = 100000000LL;
  return value;
}

auto_rover::Trajectory straightTrajectory() {
  auto_rover::Trajectory value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "camera_init";
  value.trajectory_id = "straight:1:1";
  value.route_id = "straight";
  value.plan_version = 1U;
  value.vehicle_profile_id = "nuc_senior_akm_v1";
  value.valid_for_ns = 10000000000LL;
  value.completion_behavior = auto_rover::CompletionBehavior::kStopAndHold;
  value.valid = true;
  value.points.push_back(
      {0.0, 0.0, 0.0, 0.0, 0.50, 0.0, auto_rover::Direction::kForward});
  value.points.push_back(
      {0.5, 0.0, 0.0, 0.0, 0.50, 0.5, auto_rover::Direction::kForward});
  value.points.push_back(
      {1.0, 0.0, 0.0, 0.0, 0.50, 1.0, auto_rover::Direction::kForward});
  value.points.push_back(
      {2.0, 0.0, 0.0, 0.0, 0.50, 2.0, auto_rover::Direction::kForward});
  return value;
}

auto_rover::EgoState ego(double x, double y, double yaw) {
  auto_rover::EgoState value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "camera_init";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kPublishTime;
  value.reference_frame = "rear_axle_center";
  value.pose.position.x = x;
  value.pose.position.y = y;
  value.pose.orientation = auto_rover::quaternionFromYaw(yaw);
  value.source_id = "synthetic_localization";
  value.valid = true;
  return value;
}

auto_rover::ChassisState chassis(double speed, bool control_enabled = true,
                                 bool control_enable_valid = true) {
  auto_rover::ChassisState value;
  value.stamp_ns = 1000000000LL;
  value.frame_id = "rear_axle_center";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kReceiptTime;
  value.measured_speed_mps = speed;
  value.control_enabled = control_enabled;
  value.valid_mask = auto_rover::ChassisState::kMeasuredSpeedValid;
  if (control_enable_valid) {
    value.valid_mask |= auto_rover::ChassisState::kControlEnabledValid;
  }
  value.source_id = "fake_vcu";
  value.valid = true;
  return value;
}

auto_rover::SafetyState safety(
    auto_rover::SafetyMode mode = auto_rover::SafetyMode::kArmed) {
  auto_rover::SafetyState value;
  value.stamp_ns = 1000000000LL;
  value.state_id = 1U;
  value.mode = mode;
  value.latch_generation = 0U;
  value.valid_for_ns = 200000000LL;
  value.valid = true;
  return value;
}

auto_rover_control::TrackingInput input(const auto_rover::Trajectory& trajectory,
                                        const auto_rover::EgoState& ego_state,
                                        const auto_rover::ChassisState& state,
                                        std::int64_t monotonic_now,
                                        std::int64_t ros_now) {
  auto_rover_control::TrackingInput value;
  const std::int64_t receipt = monotonic_now - 50000000LL;
  value.trajectory = {trajectory, receipt};
  value.ego = {ego_state, receipt};
  value.chassis = {state, receipt};
  value.safety = {safety(), receipt};
  // The common fixtures model a fresh producer callback on every helper call.
  // Keep their semantic identities advancing with the new local receipt. Tests
  // that exercise ordering use explicit, non-default identities below.
  if (value.ego.value.state_id == 1U &&
      value.ego.value.stamp_ns == 1000000000LL) {
    value.ego.value.state_id = static_cast<std::uint64_t>(receipt);
    value.ego.value.stamp_ns = ros_now;
  }
  if (value.chassis.value.state_id == 1U &&
      value.chassis.value.stamp_ns == 1000000000LL) {
    value.chassis.value.state_id = static_cast<std::uint64_t>(receipt);
    value.chassis.value.stamp_ns = ros_now;
  }
  value.safety.value.state_id = static_cast<std::uint64_t>(receipt);
  value.safety.value.stamp_ns = ros_now;
  value.now_monotonic_ns = monotonic_now;
  value.now_ros_ns = ros_now;
  return value;
}

void testConfiguration() {
  expect(auto_rover_control::validatePurePursuitConfig(config()).ok,
         "test control configuration validates");
  auto invalid = config();
  invalid.lookahead_min_m = invalid.lookahead_max_m + 0.1;
  expect(!auto_rover_control::validatePurePursuitConfig(invalid).ok,
         "inverted lookahead bounds fail closed");
  invalid = config();
  invalid.producer_generation_id.clear();
  expect(!auto_rover_control::validatePurePursuitConfig(invalid).ok,
         "missing producer generation fails closed");
  invalid = config();
  invalid.safety_freshness_ns = 0;
  expect(!auto_rover_control::validatePurePursuitConfig(invalid).ok,
         "missing safety freshness limit fails closed");
}

void testStraightRampAndStop() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  const auto state = chassis(0.0);

  auto first = tracker.update(
      input(trajectory, ego(0.0, 0.0, 0.0), state, 1050000000LL,
            1050000000LL));
  expect(first.reference.valid, "first straight update is a valid stop command");
  expect(first.reference.command_id == 1U &&
             first.reference.producer_generation_id ==
                 "pure-pursuit-test-generation-1",
         "tracker output binds its per-process command sequence to the configured generation");
  expectNear(first.reference.target_speed_mps, 0.0, 1e-12,
             "first update establishes the ramp at zero");
  expectNear(first.reference.target_curvature_inv_m, 0.0, 1e-12,
             "straight lookahead requests zero curvature");

  auto second = tracker.update(
      input(trajectory, ego(0.0, 0.0, 0.0), state, 1150000000LL,
            1150000000LL));
  expect(second.reference.valid, "second straight update remains valid");
  expect(second.reference.command_id == 2U &&
             second.reference.producer_generation_id ==
                 first.reference.producer_generation_id,
         "one tracker process keeps its generation while command identity advances");
  expectNear(second.reference.target_speed_mps, 0.02, 1e-12,
             "0.20 m/s2 bounds the first non-zero ramp command");
  expect(second.reference.target_speed_mps < 0.50,
         "commissioning target is not the first non-zero command");

  auto at_goal = tracker.update(
      input(trajectory, ego(1.99, 0.0, 0.0), chassis(0.0), 1250000000LL,
            1250000000LL));
  expect(at_goal.reference.valid, "terminal hold is a valid reference");
  expect(at_goal.reference.route_complete, "standstill near goal completes route");
  expectNear(at_goal.reference.target_speed_mps, 0.0, 1e-12,
             "completion produces explicit zero speed");
}

void testIndependentTrackerProcessesOwnIndependentCommandSequences() {
  auto first_config = config();
  auto second_config = config();
  second_config.producer_generation_id = "pure-pursuit-test-generation-2";
  auto_rover_control::PurePursuit first(first_config, profile());
  auto_rover_control::PurePursuit second(second_config, profile());
  const auto trajectory = straightTrajectory();
  const auto state = chassis(0.0);
  const auto first_output = first.update(input(
      trajectory, ego(0.0, 0.0, 0.0), state, 1050000000LL,
      1050000000LL));
  const auto second_output = second.update(input(
      trajectory, ego(0.0, 0.0, 0.0), state, 1050000000LL,
      1050000000LL));
  expect(first_output.reference.command_id == 1U &&
             second_output.reference.command_id == 1U,
         "each tracker process begins its own command sequence at one");
  expect(first_output.reference.producer_generation_id !=
             second_output.reference.producer_generation_id,
         "independent tracker processes expose distinct generation identities");
}

void testDisabledControlHoldsAndRestartsRampFromZero() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();

  for (std::int64_t time = 1050000000LL; time <= 2050000000LL;
       time += 100000000LL) {
    const auto disabled = tracker.update(input(
        trajectory, ego(0.0, 0.0, 0.0), chassis(0.0, false), time, time));
    expect(disabled.reference.valid,
           "substantiated disabled control produces a valid hold");
    expectNear(disabled.reference.target_speed_mps, 0.0, 1e-12,
               "waiting while disarmed cannot accumulate a speed ramp");
  }

  const auto first_enabled = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), chassis(0.0, true), 2150000000LL,
      2150000000LL));
  expect(first_enabled.reference.valid,
         "first enabled control sample remains valid");
  expectNear(first_enabled.reference.target_speed_mps, 0.0, 1e-12,
             "first enabled sample re-establishes a zero ramp origin");

  const auto ramp = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), chassis(0.0, true), 2250000000LL,
      2250000000LL));
  expectNear(ramp.reference.target_speed_mps, 0.02, 1e-12,
             "re-enabled control ramps from zero at 0.20 m/s2");

  const auto unavailable = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), chassis(0.0, false, false),
      2350000000LL, 2350000000LL));
  expect(!unavailable.reference.valid,
         "a previously observed control-enable capability cannot disappear");
}

void testSafetyAuthorizationModesHoldAndRestartRampFromZero() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  std::int64_t now = 1050000000LL;

  expectNear(tracker.update(input(trajectory, ego(0.0, 0.0, 0.0),
                                  chassis(0.0), now, now))
                 .reference.target_speed_mps,
             0.0, 1e-12,
             "first armed safety sample establishes a zero ramp origin");
  now += 100000000LL;
  expectNear(tracker.update(input(trajectory, ego(0.0, 0.0, 0.0),
                                  chassis(0.0), now, now))
                 .reference.target_speed_mps,
             0.02, 1e-12, "armed safety permits the bounded reference ramp");

  const auto_rover::SafetyMode inhibited_modes[] = {
      auto_rover::SafetyMode::kBootInhibited,
      auto_rover::SafetyMode::kDisarmed,
      auto_rover::SafetyMode::kRecoveryInhibited,
      auto_rover::SafetyMode::kFaultInhibited,
      auto_rover::SafetyMode::kEmergencyStopLatched,
  };
  for (const auto mode : inhibited_modes) {
    now += 100000000LL;
    auto inhibited =
        input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0), now, now);
    inhibited.safety.value.mode = mode;
    const auto hold = tracker.update(inhibited);
    expect(hold.reference.valid,
           "every non-armed valid safety mode produces a valid hold");
    expectNear(hold.reference.target_speed_mps, 0.0, 1e-12,
               "non-armed safety cannot accumulate a speed ramp");

    now += 100000000LL;
    const auto first_armed = tracker.update(
        input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0), now, now));
    expect(first_armed.reference.valid,
           "a fresh armed safety transition remains a valid reference");
    expectNear(first_armed.reference.target_speed_mps, 0.0, 1e-12,
               "each armed transition re-establishes a zero ramp origin");

    now += 100000000LL;
    const auto ramp = tracker.update(
        input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0), now, now));
    expectNear(ramp.reference.target_speed_mps, 0.02, 1e-12,
               "the post-arm ramp remains bounded at 0.20 m/s2");
  }
}

void testUnavailableControlEnableCapabilityUsesOnlySoftwareSafetyState() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  auto unavailable = chassis(0.0, false, false);
  unavailable.source_id =
      "backend_without_enable_capability#process=generation-1";

  const auto first = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), unavailable, 1050000000LL,
      1050000000LL));
  expect(first.reference.valid,
         "an armed software safety state permits a backend whose contract never provides control-enable feedback");
  expectNear(first.reference.target_speed_mps, 0.0, 1e-12,
             "missing optional VCU enable starts from an explicit zero");

  const auto ramp = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), unavailable, 1150000000LL,
      1150000000LL));
  expect(ramp.reference.valid, "the unavailable-capability path stays valid");
  expectNear(ramp.reference.target_speed_mps, 0.02, 1e-12,
             "software authorization still uses the Phase-1 acceleration limit");

  auto capability_appears = chassis(0.0, true, true);
  capability_appears.source_id = unavailable.source_id;
  const auto newly_substantiated = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), capability_appears, 1250000000LL,
      1250000000LL));
  expect(newly_substantiated.reference.valid,
         "newly substantiated true VCU enable is accepted");
  expectNear(newly_substantiated.reference.target_speed_mps, 0.0, 1e-12,
             "new VCU enable evidence re-establishes the zero ramp origin");

  const auto substantiated_ramp = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), capability_appears, 1350000000LL,
      1350000000LL));
  expectNear(substantiated_ramp.reference.target_speed_mps, 0.02, 1e-12,
             "substantiated VCU enable permits the bounded ramp");

  const auto disappeared = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), unavailable, 1450000000LL,
      1450000000LL));
  expect(!disappeared.reference.valid &&
             contains(disappeared.reason, "capability disappeared"),
         "control-enable validity disappearing within one chassis source fails closed");

  auto new_source = unavailable;
  new_source.source_id =
      "backend_without_enable_capability#process=generation-2";
  const auto restarted = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), new_source, 1550000000LL,
      1550000000LL));
  expect(restarted.reference.valid,
         "a new chassis source may establish its own unavailable capability contract");
  expectNear(restarted.reference.target_speed_mps, 0.0, 1e-12,
             "a new chassis source always restarts the reference ramp at zero");
}

void testSafetyStateFreshnessValidityAndOrderingFailClosed() {
  const auto trajectory = straightTrajectory();

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto missing = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1050000000LL, 1050000000LL);
    missing.safety.receipt_monotonic_ns = 0;
    const auto result = tracker.update(missing);
    expect(!result.reference.valid && contains(result.reason, "safety"),
           "missing safety state fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto stale = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                       1300000000LL, 1300000000LL);
    stale.safety.receipt_monotonic_ns = 1000000000LL;
    const auto result = tracker.update(stale);
    expect(!result.reference.valid && contains(result.reason, "safety"),
           "stale safety receiver state fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto invalid = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1050000000LL, 1050000000LL);
    invalid.safety.value.valid = false;
    const auto result = tracker.update(invalid);
    expect(!result.reference.valid && contains(result.reason, "safety"),
           "invalid safety producer state fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto expired = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 2000000000LL);
    expired.safety.value.stamp_ns = 1000000000LL;
    expired.safety.value.valid_for_ns = 500000000LL;
    const auto result = tracker.update(expired);
    expect(!result.reference.valid && contains(result.reason, "safety"),
           "expired safety producer validity fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto initial = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 1150000000LL);
    initial.safety.value.state_id = 10U;
    initial.safety.value.stamp_ns = 1100000000LL;
    initial.safety.receipt_monotonic_ns = 1100000000LL;
    expect(tracker.update(initial).reference.valid,
           "initial ordered safety sample is accepted");

    auto rollback = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                          1250000000LL, 1250000000LL);
    rollback.safety.value.state_id = 9U;
    rollback.safety.value.stamp_ns = 1200000000LL;
    rollback.safety.receipt_monotonic_ns = 1200000000LL;
    const auto result = tracker.update(rollback);
    expect(!result.reference.valid &&
               contains(result.reason, "safety state identity rolled back"),
           "a later-received safety identity rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto initial = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 1150000000LL);
    initial.safety.value.state_id = 10U;
    initial.safety.value.stamp_ns = 1100000000LL;
    initial.safety.value.valid_for_ns = 1000000000LL;
    initial.safety.receipt_monotonic_ns = 1100000000LL;
    tracker.update(initial);

    auto rollback = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                          1250000000LL, 1250000000LL);
    rollback.safety.value.state_id = 11U;
    rollback.safety.value.stamp_ns = 1050000000LL;
    rollback.safety.value.valid_for_ns = 1000000000LL;
    rollback.safety.receipt_monotonic_ns = 1200000000LL;
    const auto result = tracker.update(rollback);
    expect(!result.reference.valid &&
               contains(result.reason, "safety state source stamp rolled back"),
           "safety source stamp rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto initial = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 1150000000LL);
    initial.safety.value.state_id = 10U;
    initial.safety.value.stamp_ns = 1100000000LL;
    initial.safety.value.latch_generation = 2U;
    initial.safety.receipt_monotonic_ns = 1100000000LL;
    tracker.update(initial);

    auto rollback = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                          1250000000LL, 1250000000LL);
    rollback.safety.value.state_id = 11U;
    rollback.safety.value.stamp_ns = 1200000000LL;
    rollback.safety.value.latch_generation = 1U;
    rollback.safety.receipt_monotonic_ns = 1200000000LL;
    const auto result = tracker.update(rollback);
    expect(!result.reference.valid &&
               contains(result.reason, "safety latch generation rolled back"),
           "safety latch-generation rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto initial = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 1150000000LL);
    initial.safety.value.state_id = 10U;
    initial.safety.value.stamp_ns = 1100000000LL;
    initial.safety.receipt_monotonic_ns = 1100000000LL;
    tracker.update(initial);

    auto rollback = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                          1160000000LL, 1160000000LL);
    rollback.safety.value.state_id = 11U;
    rollback.safety.value.stamp_ns = 1150000000LL;
    rollback.safety.receipt_monotonic_ns = 1090000000LL;
    const auto result = tracker.update(rollback);
    expect(!result.reference.valid &&
               contains(result.reason, "safety state receipt time rolled back"),
           "safety receipt-time rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto initial = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                         1150000000LL, 1150000000LL);
    initial.safety.value.state_id = 10U;
    initial.safety.value.stamp_ns = 1100000000LL;
    initial.safety.receipt_monotonic_ns = 1100000000LL;
    tracker.update(initial);

    auto replay = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                        1250000000LL, 1250000000LL);
    replay.safety.value.state_id = initial.safety.value.state_id;
    replay.safety.value.stamp_ns = initial.safety.value.stamp_ns;
    replay.safety.receipt_monotonic_ns = 1200000000LL;
    const auto result = tracker.update(replay);
    expect(!result.reference.valid &&
               contains(result.reason,
                        "safety semantic sample was replayed"),
           "a retransmitted safety sample cannot refresh authorization");
  }
}

void testGoalApproachRemainsValidWhileDecelerating() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  tracker.update(input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                       1050000000LL, 1050000000LL));
  const auto ramp = tracker.update(input(
      trajectory, ego(0.0, 0.0, 0.0), chassis(0.02), 1150000000LL,
      1150000000LL));
  expectNear(ramp.reference.target_speed_mps, 0.02, 1e-12,
             "goal approach fixture establishes a nonzero ramp command");

  const auto decelerating = tracker.update(input(
      trajectory, ego(1.99, 0.0, 0.0), chassis(0.02), 1200000000LL,
      1200000000LL));
  expect(decelerating.reference.valid &&
             !decelerating.reference.route_complete,
         "goal position with residual chassis speed remains a valid reference");
  expectNear(decelerating.reference.target_speed_mps, 0.01, 1e-12,
             "goal-position deceleration preserves the 0.20 m/s2 limit");
  expectNear(decelerating.reference.target_curvature_inv_m, 0.0, 1e-12,
             "goal-position deceleration does not chase a target behind the vehicle");

  const auto stopped = tracker.update(input(
      trajectory, ego(1.99, 0.0, 0.0), chassis(0.0), 1250000000LL,
      1250000000LL));
  expect(stopped.reference.valid && stopped.reference.route_complete,
         "confirmed standstill after goal deceleration enters terminal hold");
}

void testTurnSigns() {
  auto left = straightTrajectory();
  left.trajectory_id = "left:1:1";
  left.route_id = "left";
  left.points[1].y_m = 0.10;
  left.points[2].y_m = 0.20;
  left.points[3].y_m = 0.30;
  for (std::size_t index = 1U; index < left.points.size(); ++index) {
    left.points[index].arc_length_m =
        left.points[index - 1U].arc_length_m +
        std::hypot(left.points[index].x_m - left.points[index - 1U].x_m,
                   left.points[index].y_m - left.points[index - 1U].y_m);
  }
  auto_rover_control::PurePursuit left_tracker(config(), profile());
  left_tracker.update(
      input(left, ego(0.0, 0.0, 0.0), chassis(0.0), 1040000000LL,
            1040000000LL));
  const auto left_output = left_tracker.update(
      input(left, ego(0.0, 0.0, 0.0), chassis(0.0), 1050000000LL,
            1050000000LL));
  expect(left_output.reference.valid, "gentle left trajectory validates");
  expect(left_output.reference.target_curvature_inv_m > 0.0,
         "positive curvature means left turn");

  auto right = left;
  right.trajectory_id = "right:1:1";
  right.route_id = "right";
  for (auto& point : right.points) {
    point.y_m = -point.y_m;
  }
  auto_rover_control::PurePursuit right_tracker(config(), profile());
  right_tracker.update(
      input(right, ego(0.0, 0.0, 0.0), chassis(0.0), 1040000000LL,
            1040000000LL));
  const auto right_output = right_tracker.update(
      input(right, ego(0.0, 0.0, 0.0), chassis(0.0), 1050000000LL,
            1050000000LL));
  expect(right_output.reference.valid, "gentle right trajectory validates");
  expect(right_output.reference.target_curvature_inv_m < 0.0,
         "negative curvature means right turn");
}

void testCurvatureAndFreshnessFailures() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  tracker.update(input(trajectory, ego(0.0, -1.0, 0.0), chassis(0.0),
                       1040000000LL, 1040000000LL));
  auto far_offset = input(trajectory, ego(0.0, -1.0, 0.0), chassis(0.0),
                          1050000000LL, 1050000000LL);
  const auto curvature_failure = tracker.update(far_offset);
  expect(!curvature_failure.reference.valid,
         "required curvature beyond the approved radius fails closed");
  expectNear(curvature_failure.reference.target_speed_mps, 0.0, 1e-12,
             "curvature failure commands zero");

  struct StaleCase {
    enum class Field { kEgo, kTrajectory, kChassis } field;
    const char* name;
  };
  const StaleCase cases[] = {
      {StaleCase::Field::kEgo, "localization"},
      {StaleCase::Field::kTrajectory, "trajectory"},
      {StaleCase::Field::kChassis, "chassis"},
  };
  for (const auto& stale_case : cases) {
    auto_rover_control::PurePursuit local_tracker(config(), profile());
    auto stale = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                       1300000000LL, 1050000000LL);
    if (stale_case.field == StaleCase::Field::kEgo) {
      stale.ego.receipt_monotonic_ns = 1000000000LL;
      stale.trajectory.receipt_monotonic_ns = 1200000000LL;
      stale.chassis.receipt_monotonic_ns = 1200000000LL;
    } else if (stale_case.field == StaleCase::Field::kTrajectory) {
      stale.ego.receipt_monotonic_ns = 1200000000LL;
      stale.trajectory.receipt_monotonic_ns = 700000000LL;
      stale.chassis.receipt_monotonic_ns = 1200000000LL;
    } else {
      stale.ego.receipt_monotonic_ns = 1200000000LL;
      stale.trajectory.receipt_monotonic_ns = 1200000000LL;
      stale.chassis.receipt_monotonic_ns = 1000000000LL;
    }
    const auto result = local_tracker.update(stale);
    expect(!result.reference.valid,
           std::string("stale ") + stale_case.name + " fails closed");
    expectNear(result.reference.target_speed_mps, 0.0, 1e-12,
               std::string("stale ") + stale_case.name + " selects zero");
  }

  auto expired = input(trajectory, ego(0.0, 0.0, 0.0), chassis(0.0),
                       1050000000LL, 11000000001LL);
  const auto expired_result = tracker.update(expired);
  expect(!expired_result.reference.valid,
         "trajectory producer validity expiry fails closed");
}

void testForwardOnlyHeadingMismatchFailsClosed() {
  auto_rover_control::PurePursuit tracker(config(), profile());
  const auto trajectory = straightTrajectory();
  const auto reversed_ego = ego(0.0, 0.0, 3.14159265358979323846);
  tracker.update(input(trajectory, reversed_ego, chassis(0.0),
                       1050000000LL, 1050000000LL));
  const auto result = tracker.update(input(
      trajectory, reversed_ego, chassis(0.0), 1150000000LL,
      1150000000LL));
  expect(!result.reference.valid,
         "a forward route behind the vehicle heading fails closed");
  expectNear(result.reference.target_speed_mps, 0.0, 1e-12,
             "heading mismatch cannot command forward motion");
}

void testRequiredInputSourceOrderingFailsClosed() {
  const auto make_input = [](const auto_rover::EgoState& ego_value,
                             const auto_rover::Trajectory& trajectory_value,
                             const auto_rover::ChassisState& chassis_value,
                             std::int64_t now) {
    return input(trajectory_value, ego_value, chassis_value, now, now);
  };

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto ego_value = ego(0.0, 0.0, 0.0);
    ego_value.state_id = 10U;
    ego_value.stamp_ns = 1100000000LL;
    auto trajectory_value = straightTrajectory();
    auto chassis_value = chassis(0.0);
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1150000000LL));
    ego_value.state_id = 11U;
    ego_value.stamp_ns = 1200000000LL;
    expect(tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                                     1250000000LL)).reference.valid,
           "strictly advancing ego state identity and stamp are accepted");

    ego_value.state_id = 9U;
    ego_value.stamp_ns = 1300000000LL;
    const auto rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1350000000LL));
    expect(!rollback.reference.valid &&
               contains(rollback.reason, "state identity rolled back"),
           "a later-received ego identity rollback fails closed");

    ego_value.state_id = 12U;
    ego_value.stamp_ns = 1400000000LL;
    tracker.reset();
    const auto same_source = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1450000000LL));
    expect(!same_source.reference.valid &&
               contains(same_source.reason, "new identity is required"),
           "controller reset cannot authorize a restarted ego producer under the same identity");

    ego_value.source_id = "synthetic_localization_generation_2";
    ego_value.state_id = 1U;
    ego_value.stamp_ns = 1000000000LL;
    expect(tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                                     1500000000LL)).reference.valid,
           "a distinct ego source identity can establish a new sequence");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto ego_value = ego(0.0, 0.0, 0.0);
    ego_value.state_id = 10U;
    ego_value.stamp_ns = 1200000000LL;
    const auto trajectory_value = straightTrajectory();
    const auto chassis_value = chassis(0.0);
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1250000000LL));
    ego_value.state_id = 11U;
    ego_value.stamp_ns = 1100000000LL;
    const auto rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1300000000LL));
    expect(!rollback.reference.valid &&
               contains(rollback.reason, "source stamp rolled back"),
           "ego source stamp rollback is detected independently of identity");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    const auto ego_value = ego(0.0, 0.0, 0.0);
    const auto trajectory_value = straightTrajectory();
    auto chassis_value = chassis(0.0);
    chassis_value.state_id = 10U;
    chassis_value.stamp_ns = 1200000000LL;
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1250000000LL));
    chassis_value.state_id = 9U;
    chassis_value.stamp_ns = 1300000000LL;
    const auto identity_rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1350000000LL));
    expect(!identity_rollback.reference.valid &&
               contains(identity_rollback.reason,
                        "chassis state identity rolled back"),
           "a later-received chassis identity rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    const auto ego_value = ego(0.0, 0.0, 0.0);
    const auto trajectory_value = straightTrajectory();
    auto chassis_value = chassis(0.0);
    chassis_value.source_id =
        "deterministic_fake_vcu_v1#process=generation-1";
    chassis_value.state_id = 10U;
    chassis_value.stamp_ns = 1200000000LL;
    expect(tracker.update(make_input(ego_value, trajectory_value,
                                     chassis_value, 1250000000LL))
               .reference.valid,
           "the initial fake VCU process generation is accepted");

    chassis_value.source_id =
        "deterministic_fake_vcu_v1#process=generation-2";
    chassis_value.state_id = 1U;
    chassis_value.stamp_ns = 1300000000LL;
    expect(tracker.update(make_input(ego_value, trajectory_value,
                                     chassis_value, 1350000000LL))
               .reference.valid,
           "a new fake VCU process generation may restart state identity at one");

    chassis_value.source_id =
        "deterministic_fake_vcu_v1#process=generation-1";
    chassis_value.state_id = 11U;
    chassis_value.stamp_ns = 1400000000LL;
    const auto retired_replay = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1450000000LL));
    expect(!retired_replay.reference.valid &&
               contains(retired_replay.reason,
                        "chassis source identity was already retired"),
           "feedback replay from a retired fake VCU generation fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    const auto ego_value = ego(0.0, 0.0, 0.0);
    const auto trajectory_value = straightTrajectory();
    auto chassis_value = chassis(0.0);
    chassis_value.state_id = 10U;
    chassis_value.stamp_ns = 1200000000LL;
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1250000000LL));
    chassis_value.state_id = 11U;
    chassis_value.stamp_ns = 1100000000LL;
    const auto stamp_rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1300000000LL));
    expect(!stamp_rollback.reference.valid &&
               contains(stamp_rollback.reason,
                        "chassis source stamp rolled back"),
           "chassis source stamp rollback is detected independently");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    const auto ego_value = ego(0.0, 0.0, 0.0);
    const auto chassis_value = chassis(0.0);
    auto trajectory_value = straightTrajectory();
    trajectory_value.trajectory_id = "straight:2:1";
    trajectory_value.plan_version = 2U;
    trajectory_value.stamp_ns = 1200000000LL;
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1250000000LL));
    trajectory_value.stamp_ns = 1300000000LL;
    expect(tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                                     1350000000LL)).reference.valid,
           "one immutable trajectory identity may be refreshed normally");
    trajectory_value.stamp_ns = 1250000000LL;
    const auto stamp_rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1400000000LL));
    expect(!stamp_rollback.reference.valid &&
               contains(stamp_rollback.reason,
                        "trajectory source stamp rolled back"),
           "trajectory generation stamp rollback fails closed");
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    const auto ego_value = ego(0.0, 0.0, 0.0);
    const auto chassis_value = chassis(0.0);
    auto trajectory_value = straightTrajectory();
    trajectory_value.trajectory_id = "straight:2:1";
    trajectory_value.plan_version = 2U;
    tracker.update(make_input(ego_value, trajectory_value, chassis_value,
                              1150000000LL));
    trajectory_value.trajectory_id = "straight:1:restart";
    trajectory_value.plan_version = 1U;
    trajectory_value.stamp_ns = 1200000000LL;
    const auto plan_rollback = tracker.update(make_input(
        ego_value, trajectory_value, chassis_value, 1250000000LL));
    expect(!plan_rollback.reference.valid &&
               contains(plan_rollback.reason,
                        "plan version did not advance"),
           "trajectory plan-version rollback under the same route fails closed");
  }
}

void testRepeatedRequiredStateReplayFailsClosed() {
  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto cached = input(straightTrajectory(), ego(0.0, 0.0, 0.0),
                        chassis(0.0), 1050000000LL, 1050000000LL);
    expect(tracker.update(cached).reference.valid,
           "initial cached-state fixture is accepted");

    cached.now_monotonic_ns = 1100000000LL;
    cached.now_ros_ns = 1100000000LL;
    const auto reused = tracker.update(cached);
    expect(reused.reference.valid &&
               reused.reference.target_speed_mps > 0.0,
           "one Received snapshot may be reused by a later control cycle within freshness");

    for (std::int64_t index = 0; index < 6; ++index) {
      const std::int64_t now = 1150000000LL + index * 50000000LL;
      auto replayed = cached;
      replayed.now_monotonic_ns = now;
      replayed.now_ros_ns = now;
      replayed.ego.receipt_monotonic_ns = now - 1000000LL;
      replayed.trajectory.receipt_monotonic_ns = now - 1000000LL;
      replayed.chassis.receipt_monotonic_ns = now - 1000000LL;
      const auto result = tracker.update(replayed);
      expect(!result.reference.valid &&
                 result.reference.target_speed_mps == 0.0,
             "retransmitted old ego samples cannot refresh the watchdog or preserve nonzero motion");
      if (index == 0) {
        expect(contains(result.reason,
                        "ego semantic sample was replayed with a new receipt"),
               "the first retransmitted ego sample identifies a receipt replay");
      }
    }
  }

  {
    auto_rover_control::PurePursuit tracker(config(), profile());
    auto cached = input(straightTrajectory(), ego(0.0, 0.0, 0.0),
                        chassis(0.0), 1050000000LL, 1050000000LL);
    expect(tracker.update(cached).reference.valid,
           "initial chassis-replay fixture is accepted");

    cached.now_monotonic_ns = 1100000000LL;
    cached.now_ros_ns = 1100000000LL;
    const auto reused = tracker.update(cached);
    expect(reused.reference.valid &&
               reused.reference.target_speed_mps > 0.0,
           "cached chassis feedback may be reused by the timer within freshness");

    for (std::int64_t index = 0; index < 6; ++index) {
      const std::int64_t now = 1150000000LL + index * 50000000LL;
      auto replayed = cached;
      replayed.now_monotonic_ns = now;
      replayed.now_ros_ns = now;
      replayed.ego.value.state_id +=
          static_cast<std::uint64_t>(index + 1);
      replayed.ego.value.stamp_ns = now;
      replayed.ego.receipt_monotonic_ns = now - 1000000LL;
      replayed.trajectory.receipt_monotonic_ns = now - 1000000LL;
      replayed.chassis.receipt_monotonic_ns = now - 1000000LL;
      const auto result = tracker.update(replayed);
      expect(!result.reference.valid &&
                 result.reference.target_speed_mps == 0.0,
             "retransmitted old chassis samples cannot refresh the watchdog or preserve nonzero motion");
      if (index == 0) {
        expect(contains(
                   result.reason,
                   "chassis semantic sample was replayed with a new receipt"),
               "the first retransmitted chassis sample identifies a receipt replay");
      }
    }
  }
}

}  // namespace

int main() {
  testConfiguration();
  testStraightRampAndStop();
  testIndependentTrackerProcessesOwnIndependentCommandSequences();
  testDisabledControlHoldsAndRestartsRampFromZero();
  testSafetyAuthorizationModesHoldAndRestartRampFromZero();
  testUnavailableControlEnableCapabilityUsesOnlySoftwareSafetyState();
  testSafetyStateFreshnessValidityAndOrderingFailClosed();
  testGoalApproachRemainsValidWhileDecelerating();
  testTurnSigns();
  testCurvatureAndFreshnessFailures();
  testForwardOnlyHeadingMismatchFailsClosed();
  testRequiredInputSourceOrderingFailsClosed();
  testRepeatedRequiredStateReplayFailsClosed();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "pure pursuit tests passed\n";
  return EXIT_SUCCESS;
}
