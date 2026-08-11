#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include "auto_rover_control/pure_pursuit.hpp"
#include "auto_rover_core/geometry.hpp"
#include "auto_rover_localization/fast_livo_odometry_adapter_core.hpp"
#include "auto_rover_planning/trajectory_generator.hpp"
#include "auto_rover_planning/waypoint_loader.hpp"
#include "auto_rover_safety/command_guard.hpp"
#include "auto_rover_safety/safety_supervisor.hpp"
#include "auto_rover_vehicle/fake_vcu.hpp"
#include "auto_rover_vehicle/vehicle_motion_manager.hpp"

namespace {

constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr std::int64_t kTickNs = 100000000LL;
constexpr double kTickSeconds = 0.1;
constexpr double kTolerance = 1e-9;

int g_failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::isfinite(actual) && std::isfinite(expected) &&
             std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

auto_rover::VehicleProfile vehicleProfile() {
  auto_rover::VehicleProfile profile;
  profile.schema_version = 1U;
  profile.profile_id = "nuc_senior_akm_v1";
  profile.kinematic_model = auto_rover::KinematicModel::kAckermannBicycle;
  profile.direction_capability =
      auto_rover::DirectionCapability::kSignedSpeedDirection;
  profile.reference_frame = "rear_axle_center";
  profile.wheelbase_m = 0.3187;
  profile.max_forward_speed_mps = 0.50;
  profile.max_longitudinal_accel_mps2 = 0.20;
  profile.min_turning_radius_m = 0.95;
  profile.reverse_supported = false;
  return profile;
}

auto_rover::Pose3 nonIdentitySourceToRear() {
  auto_rover::Pose3 source_T_rear;
  source_T_rear.position.x = 0.37;
  source_T_rear.position.y = -0.11;
  source_T_rear.position.z = 0.04;
  source_T_rear.orientation = auto_rover::quaternionFromYaw(0.23);
  return source_T_rear;
}

auto_rover::Pose3 inversePose(const auto_rover::Pose3& parent_T_child) {
  auto_rover::Pose3 child_T_parent;
  child_T_parent.orientation.x = -parent_T_child.orientation.x;
  child_T_parent.orientation.y = -parent_T_child.orientation.y;
  child_T_parent.orientation.z = -parent_T_child.orientation.z;
  child_T_parent.orientation.w = parent_T_child.orientation.w;
  const auto_rover::Vector3 negative_translation{
      -parent_T_child.position.x, -parent_T_child.position.y,
      -parent_T_child.position.z};
  child_T_parent.position =
      auto_rover::rotateVector(child_T_parent.orientation,
                               negative_translation);
  return child_T_parent;
}

auto_rover::localization::FastLivoOdometryAdapterConfig localizationConfig(
    const auto_rover::Pose3& source_T_rear) {
  auto_rover::localization::FastLivoOdometryAdapterConfig config;
  config.expected_frame_id = "camera_init";
  config.expected_child_frame_id = "aft_mapped";
  config.source_revision =
      "3df020182aee52d81fd1a6b543bfb611c46d11bc";
  config.process_generation_id = "full-loop-localization-generation-1";
  config.timestamp_semantics = auto_rover::TimeSource::kPublishTime;
  config.extrinsic_known = true;
  config.source_T_rear = source_T_rear;
  config.freshness_limit_ns = 150000000LL;
  return config;
}

auto_rover_control::PurePursuitConfig pursuitConfig() {
  auto_rover_control::PurePursuitConfig config;
  config.world_frame = "camera_init";
  config.control_frame = "rear_axle_center";
  config.producer_generation_id = "full-loop-tracker-generation-1";
  config.lookahead_min_m = 0.30;
  config.lookahead_max_m = 0.60;
  config.lookahead_speed_gain_s = 0.50;
  config.goal_position_tolerance_m = 0.15;
  config.standstill_speed_threshold_mps = 0.01;
  config.localization_freshness_ns = 150000000LL;
  config.trajectory_freshness_ns = 500000000LL;
  config.chassis_freshness_ns = 150000000LL;
  config.safety_freshness_ns = 150000000LL;
  config.motion_valid_for_ns = 150000000LL;
  return config;
}

auto_rover_safety::SafetySupervisorConfig safetyConfig() {
  auto_rover_safety::SafetySupervisorConfig config;
  config.actuation_enabled = true;
  config.reset_service_enabled = true;
  config.fresh_recovery_count = 3U;
  config.state_valid_for_ns = 200000000LL;
  return config;
}

auto_rover_safety::GuardConfig guardConfig() {
  auto_rover_safety::GuardConfig config;
  config.world_frame = "camera_init";
  config.localization_freshness_ns = 150000000LL;
  config.trajectory_freshness_ns = 500000000LL;
  config.motion_freshness_ns = 150000000LL;
  config.chassis_freshness_ns = 150000000LL;
  config.safety_freshness_ns = 150000000LL;
  config.fresh_recovery_count = 3U;
  return config;
}

auto_rover_vehicle::VehicleMotionManagerConfig managerConfig() {
  auto_rover_vehicle::VehicleMotionManagerConfig config;
  config.command_valid_for_ns = 250000000LL;
  return config;
}

auto_rover_vehicle::FakeVcuConfig fakeConfig() {
  auto_rover_vehicle::FakeVcuConfig config;
  config.process_generation_id = "full-loop-fake-vcu-generation-1";
  config.command_watchdog_ns = 300000000LL;
  config.feedback_period_ns = kTickNs;
  config.fresh_recovery_count = 3U;
  config.max_accel_mps2 = 0.20;
  return config;
}

std::string straightRouteYaml() {
  return "schema_version: 1\n"
         "frame_id: camera_init\n"
         "route_id: deterministic_two_metre_route\n"
         "plan_version: 1\n"
         "loop: false\n"
         "waypoints:\n"
         "  - {x_m: 0.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n"
         "  - {x_m: 2.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n";
}

class InjectedClock {
 public:
  void advance() {
    monotonic_ns_ += kTickNs;
    ros_ns_ += kTickNs;
  }

  std::int64_t monotonicNs() const { return monotonic_ns_; }
  std::int64_t rosNs() const { return ros_ns_; }

 private:
  std::int64_t monotonic_ns_{kSecondNs};
  std::int64_t ros_ns_{kSecondNs};
};

class RearPoseIntegrator {
 public:
  explicit RearPoseIntegrator(std::int64_t initial_monotonic_ns)
      : last_monotonic_ns_(initial_monotonic_ns) {
    world_T_rear_.orientation.w = 1.0;
  }

  void integrate(const auto_rover::ChassisState& chassis,
                 std::int64_t now_monotonic_ns) {
    expect(now_monotonic_ns > last_monotonic_ns_,
           "rear-pose integration time advances");
    if (now_monotonic_ns <= last_monotonic_ns_) {
      return;
    }
    const double elapsed_seconds =
        static_cast<double>(now_monotonic_ns - last_monotonic_ns_) /
        static_cast<double>(kSecondNs);
    const double average_speed =
        0.5 * (previous_speed_mps_ + chassis.measured_speed_mps);
    const double average_yaw_rate =
        0.5 * (previous_yaw_rate_radps_ + chassis.yaw_rate_radps);
    const double yaw =
        auto_rover::yawFromQuaternion(world_T_rear_.orientation);
    const double midpoint_yaw =
        yaw + 0.5 * average_yaw_rate * elapsed_seconds;
    world_T_rear_.position.x +=
        average_speed * std::cos(midpoint_yaw) * elapsed_seconds;
    world_T_rear_.position.y +=
        average_speed * std::sin(midpoint_yaw) * elapsed_seconds;
    world_T_rear_.orientation = auto_rover::quaternionFromYaw(
        yaw + average_yaw_rate * elapsed_seconds);
    previous_speed_mps_ = chassis.measured_speed_mps;
    previous_yaw_rate_radps_ = chassis.yaw_rate_radps;
    last_monotonic_ns_ = now_monotonic_ns;
  }

  const auto_rover::Pose3& pose() const { return world_T_rear_; }

 private:
  auto_rover::Pose3 world_T_rear_{};
  std::int64_t last_monotonic_ns_{0};
  double previous_speed_mps_{0.0};
  double previous_yaw_rate_radps_{0.0};
};

struct LoopCycle {
  auto_rover_vehicle::FakeVcuStepResult feedback;
  auto_rover_control::TrackingResult tracking;
  auto_rover::SafetyState safety;
  auto_rover_safety::GuardResult guarded;
  auto_rover::VehicleExecutionCommand command;
  auto_rover_vehicle::FakeVcuReceiveResult delivery;
};

class FullLoopRig {
 public:
  FullLoopRig()
      : profile_(vehicleProfile()),
        source_T_rear_(nonIdentitySourceToRear()),
        rear_T_source_(inversePose(source_T_rear_)),
        localization_(localizationConfig(source_T_rear_)),
        tracker_(pursuitConfig(), profile_),
        supervisor_(safetyConfig()),
        guard_(guardConfig(), profile_),
        manager_(managerConfig(), profile_),
        fake_(fakeConfig(), profile_),
        rear_integrator_(clock_.monotonicNs()) {
    expect(std::abs(source_T_rear_.position.x) > 0.1 &&
               std::abs(auto_rover::yawFromQuaternion(
                            source_T_rear_.orientation)) > 0.1,
           "FAST-LIVO2 source-to-rear extrinsic is non-identity");

    const auto_rover::planning::WaypointYamlLoader loader(profile_,
                                                           "camera_init");
    const auto route = loader.loadString(straightRouteYaml(), clock_.rosNs());
    expect(route.validation.ok,
           "strict YAML loader creates the two-metre RoutePlan");

    auto_rover::planning::TrajectoryGeneratorConfig planner_config;
    planner_config.sampling_resolution_m = 0.05;
    planner_config.valid_for_ns = 120 * kSecondNs;
    const auto_rover::planning::HermiteTrajectoryGenerator generator(
        planner_config);
    const auto generated =
        generator.generate(route.route, profile_, clock_.rosNs());
    expect(generated.validation.ok && generated.trajectory.valid,
           "Hermite planner creates a valid straight Trajectory");
    trajectory_ = generated.trajectory;
    expectNear(trajectory_.points.back().x_m, 2.0, kTolerance,
               "planned endpoint is two metres forward");
    expectNear(trajectory_.points.back().curvature_inv_m, 0.0, kTolerance,
               "planned straight route has zero terminal curvature");

    fake_.setConnected(true, clock_.monotonicNs());
    manager_.setBackendConnected(true, clock_.monotonicNs());
    const auto boot = supervisor_.completeBoot(clock_.rosNs());
    expect(boot.mode == auto_rover::SafetyMode::kRecoveryInhibited,
           "boot begins recovery-inhibited");
  }

  LoopCycle cycle(bool publish_localization = true,
                  bool conditions_clear = true) {
    LoopCycle output;
    clock_.advance();
    output.feedback = fake_.step(clock_.monotonicNs(), clock_.rosNs());
    expect(output.feedback.available,
           "fake VCU publishes deterministic chassis feedback each tick");
    if (!output.feedback.available) {
      return output;
    }
    chassis_ = output.feedback.state;
    chassis_receipt_ns_ = clock_.monotonicNs();
    rear_integrator_.integrate(chassis_, clock_.monotonicNs());

    if (publish_localization) {
      auto_rover::localization::SourceOdometry source;
      source.provider_stamp_ns = clock_.rosNs();
      source.frame_id = "camera_init";
      source.child_frame_id = "aft_mapped";
      source.pose = auto_rover::composePoses(rear_integrator_.pose(),
                                             rear_T_source_);
      const auto adapted = localization_.adapt(
          source, clock_.monotonicNs(), clock_.monotonicNs());
      expect(adapted.ego_state.valid,
             "FAST-LIVO2 adapter accepts reverse-composed source pose");
      expectNear(adapted.ego_state.pose.position.x,
                 rear_integrator_.pose().position.x, 1e-10,
                 "localization recovers integrated rear-axle x");
      expectNear(adapted.ego_state.pose.position.y,
                 rear_integrator_.pose().position.y, 1e-10,
                 "localization recovers integrated rear-axle y");
      expectNear(auto_rover::yawFromQuaternion(
                     adapted.ego_state.pose.orientation),
                 auto_rover::yawFromQuaternion(
                     rear_integrator_.pose().orientation),
                 1e-10,
                 "localization recovers integrated rear-axle yaw");
      ego_ = adapted.ego_state;
      ego_receipt_ns_ = clock_.monotonicNs();
    }

    output.safety = supervisor_.observeConditions(
        conditions_clear,
        conditions_clear ? auto_rover::StopReason::kNone
                         : auto_rover::StopReason::kStaleLocalization,
        clock_.rosNs());

    auto_rover_control::TrackingInput tracking_input;
    tracking_input.trajectory = {trajectory_, clock_.monotonicNs()};
    tracking_input.ego = {ego_, ego_receipt_ns_};
    tracking_input.chassis = {chassis_, chassis_receipt_ns_};
    tracking_input.safety = {output.safety, clock_.monotonicNs()};
    tracking_input.now_monotonic_ns = clock_.monotonicNs();
    tracking_input.now_ros_ns = clock_.rosNs();
    output.tracking = tracker_.update(tracking_input);

    auto_rover_safety::GuardInput guard_input;
    guard_input.ego = {ego_, ego_receipt_ns_};
    guard_input.trajectory = {trajectory_, clock_.monotonicNs()};
    guard_input.motion = {output.tracking.reference,
                          clock_.monotonicNs()};
    guard_input.chassis = {chassis_, chassis_receipt_ns_};
    guard_input.safety = {output.safety, clock_.monotonicNs()};
    guard_input.now_monotonic_ns = clock_.monotonicNs();
    guard_input.now_ros_ns = clock_.rosNs();
    output.guarded = guard_.evaluate(guard_input);
    output.command =
        manager_.makeCommand(output.guarded, clock_.monotonicNs());
    output.delivery =
        fake_.receive(output.command, clock_.monotonicNs());

    expect(output.command.sequence_id > last_sequence_id_,
           "vehicle execution sequence strictly increases");
    last_sequence_id_ = output.command.sequence_id;
    expect(output.command.signed_speed_mps >= -kTolerance &&
               output.command.signed_speed_mps <= 0.50 + kTolerance,
           "every execution command remains within the commissioning limit");
    expectNear(output.command.curvature_inv_m, 0.0, 1e-8,
               "straight full loop commands zero curvature");
    expect(std::abs(rear_integrator_.pose().position.y) <= 1e-8,
           "integrated rear axle remains on the straight route");
    return output;
  }

  bool explicitArm(std::uint64_t authorization_id) {
    const auto state = supervisor_.currentState(clock_.rosNs());
    const auto safety_arm = supervisor_.requestArm(
        "integration_operator", state.state_id, clock_.rosNs());
    const bool manager_arm = manager_.acknowledgeOperatorArm(
        authorization_id, clock_.monotonicNs());
    if (safety_arm.accepted && manager_arm) {
      fake_.setControlEnabled(true, clock_.monotonicNs());
      tracker_.reset();
    }
    return safety_arm.accepted && manager_arm && fake_.controlEnabled();
  }

  auto_rover_safety::SafetyTransition assertEmergencyStop(
      const std::string& request_id) {
    auto_rover::EmergencyStop request;
    request.stamp_ns = clock_.rosNs();
    request.request_id = request_id;
    request.source_id = "integration_estop_panel";
    request.asserted = true;
    return supervisor_.assertEmergencyStop(request, clock_.rosNs());
  }

  auto_rover_safety::SafetyTransition resetEmergencyStop(
      std::uint64_t generation, bool authorized) {
    auto_rover_safety::ResetEmergencyStopRequest request;
    request.operator_id = "integration_operator";
    request.latch_generation = generation;
    request.conditions_cleared_acknowledged = true;
    request.authorization_granted = authorized;
    return supervisor_.resetEmergencyStop(request, clock_.rosNs());
  }

  double rearX() const { return rear_integrator_.pose().position.x; }
  std::int64_t monotonicNs() const { return clock_.monotonicNs(); }
  auto_rover::SafetyMode safetyMode() const {
    return supervisor_.currentState(clock_.rosNs()).mode;
  }

 private:
  auto_rover::VehicleProfile profile_;
  auto_rover::Pose3 source_T_rear_;
  auto_rover::Pose3 rear_T_source_;
  InjectedClock clock_;
  auto_rover::localization::FastLivoOdometryAdapterCore localization_;
  auto_rover::Trajectory trajectory_;
  auto_rover_control::PurePursuit tracker_;
  auto_rover_safety::SafetySupervisor supervisor_;
  auto_rover_safety::CommandGuard guard_;
  auto_rover_vehicle::VehicleMotionManager manager_;
  auto_rover_vehicle::FakeVcu fake_;
  RearPoseIntegrator rear_integrator_;
  auto_rover::EgoState ego_{};
  auto_rover::ChassisState chassis_{};
  std::int64_t ego_receipt_ns_{0};
  std::int64_t chassis_receipt_ns_{0};
  std::uint64_t last_sequence_id_{0U};
};

void expectExplicitStop(const LoopCycle& cycle,
                        const std::string& context) {
  expect(!cycle.command.motion_enabled, context + " inhibits vehicle motion");
  expect(cycle.command.hold, context + " requests hold");
  expectNear(cycle.command.signed_speed_mps, 0.0, kTolerance,
             context + " emits explicit zero speed");
  expectNear(cycle.command.curvature_inv_m, 0.0, kTolerance,
             context + " emits explicit zero curvature");
}

LoopCycle runUntilRearX(FullLoopRig* rig, double target_x,
                        int maximum_cycles) {
  LoopCycle output;
  int cycles = 0;
  while (rig->rearX() < target_x && cycles < maximum_cycles) {
    output = rig->cycle();
    ++cycles;
  }
  expect(rig->rearX() >= target_x,
         "closed loop reaches requested intermediate distance");
  return output;
}

void testDeterministicPureCoreFullLoop() {
  FullLoopRig rig;

  LoopCycle startup;
  for (int fresh = 0; fresh < 3; ++fresh) {
    startup = rig.cycle();
    expectExplicitStop(startup, "default startup");
    expect(!startup.delivery.motion_accepted,
           "default startup cannot be accepted as motion");
  }
  expect(startup.safety.mode == auto_rover::SafetyMode::kDisarmed,
         "three fresh sets recover only to disarmed");
  const LoopCycle still_disarmed = rig.cycle();
  expectExplicitStop(still_disarmed,
                     "fresh inputs without explicit operator arm");
  expectNear(rig.rearX(), 0.0, kTolerance,
             "vehicle remains stationary before explicit arm");

  expect(rig.explicitArm(1U),
         "explicit startup arm reaches supervisor, manager, and fake VCU");
  const LoopCycle armed_zero = rig.cycle();
  expect(armed_zero.command.motion_enabled && armed_zero.command.hold,
         "first armed cycle is an enabled zero and hold command");
  expectNear(armed_zero.command.signed_speed_mps, 0.0, kTolerance,
             "tracker reset establishes a zero ramp origin");

  const LoopCycle first_nonzero = rig.cycle();
  expect(first_nonzero.command.motion_enabled &&
             !first_nonzero.command.hold,
         "second armed cycle produces the first nonzero command");
  expect(first_nonzero.command.signed_speed_mps > 0.0 &&
             first_nonzero.command.signed_speed_mps < 0.50,
         "first nonzero ramp command is below the commissioning target");
  expect(first_nonzero.command.signed_speed_mps <=
             0.20 * kTickSeconds + 1e-12,
         "first nonzero ramp respects 0.20 m/s2");
  expect(!first_nonzero.delivery.motion_accepted &&
             first_nonzero.delivery.reason ==
                 auto_rover::StopReason::kRecoveryPending,
         "fake VCU independently requires consecutive fresh motion commands");

  rig.cycle();
  const LoopCycle fake_recovered = rig.cycle();
  expect(fake_recovered.delivery.motion_accepted,
         "third consecutive nonzero command completes fake VCU recovery");
  runUntilRearX(&rig, 0.30, 100);

  rig.cycle(false, false);
  const LoopCycle stale = rig.cycle(false, false);
  expect(!stale.tracking.reference.valid &&
             stale.tracking.reason.find("localization is stale") !=
                 std::string::npos,
         "frozen localization expires the tracker receiver watchdog");
  expect(!stale.guarded.execution_allowed &&
             stale.guarded.reason ==
                 auto_rover::StopReason::kStaleLocalization,
         "guard independently identifies stale localization");
  expectNear(stale.guarded.reference.target_speed_mps, 0.0, kTolerance,
             "stale localization produces guarded zero reference");
  expect(stale.safety.mode == auto_rover::SafetyMode::kFaultInhibited,
         "stale localization disarms the safety supervisor");
  expectExplicitStop(stale, "stale-localization transition");

  LoopCycle recovery;
  for (int fresh = 0; fresh < 3; ++fresh) {
    recovery = rig.cycle(true, true);
    expectExplicitStop(recovery,
                       "ordinary stale-localization recovery run");
  }
  expect(recovery.safety.mode == auto_rover::SafetyMode::kDisarmed,
         "fresh recovery run returns only to disarmed");
  expect(rig.safetyMode() == auto_rover::SafetyMode::kDisarmed,
         "fresh inputs do not implicitly re-arm after stale localization");
  expectExplicitStop(rig.cycle(),
                     "post-recovery state before second explicit arm");

  expect(rig.explicitArm(2U),
         "second explicit arm is required after localization recovery");
  const LoopCycle second_arm_zero = rig.cycle();
  expect(second_arm_zero.command.motion_enabled &&
             second_arm_zero.command.hold,
         "second arm restarts motion at enabled zero and hold");
  const LoopCycle second_arm_ramp = rig.cycle();
  expect(second_arm_ramp.command.signed_speed_mps > 0.0 &&
             second_arm_ramp.command.signed_speed_mps <=
                 0.20 * kTickSeconds + 1e-12,
         "second arm restarts the acceleration ramp from zero");
  rig.cycle();
  rig.cycle();
  runUntilRearX(&rig, 0.90, 150);

  const auto latched = rig.assertEmergencyStop("integration-estop-1");
  expect(latched.accepted &&
             latched.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "software emergency stop assertion latches immediately");
  const std::uint64_t latch_generation = latched.state.latch_generation;
  const LoopCycle estop_zero = rig.cycle();
  expect(!estop_zero.guarded.execution_allowed &&
             estop_zero.guarded.reason ==
                 auto_rover::StopReason::kEmergencyStop,
         "latched emergency stop blocks the guard");
  expectExplicitStop(estop_zero, "latched emergency stop");

  const auto recovery_incomplete =
      rig.resetEmergencyStop(latch_generation, true);
  expect(!recovery_incomplete.accepted &&
             recovery_incomplete.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "authorized reset cannot bypass the consecutive-fresh requirement");

  const auto unauthorized =
      rig.resetEmergencyStop(latch_generation, false);
  expect(!unauthorized.accepted &&
             unauthorized.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "unauthorized reset cannot clear the emergency-stop latch");
  rig.cycle();
  rig.cycle();
  const auto wrong_generation =
      rig.resetEmergencyStop(latch_generation + 1U, true);
  expect(!wrong_generation.accepted &&
             wrong_generation.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "wrong latch generation cannot clear emergency stop");
  const auto reset = rig.resetEmergencyStop(latch_generation, true);
  expect(reset.accepted &&
             reset.state.mode == auto_rover::SafetyMode::kDisarmed,
         "authorized generation-matched reset returns only to disarmed");
  expectExplicitStop(rig.cycle(),
                     "successful emergency-stop reset before re-arm");

  expect(rig.explicitArm(3U),
         "third explicit arm is required after emergency-stop reset");
  rig.cycle();
  rig.cycle();
  rig.cycle();
  rig.cycle();

  LoopCycle terminal;
  bool route_complete = false;
  for (int cycle = 0; cycle < 500 && !route_complete; ++cycle) {
    terminal = rig.cycle();
    route_complete = terminal.tracking.reference.route_complete;
  }
  expect(route_complete,
         "two-metre deterministic closed loop reaches route_complete");
  expect(terminal.tracking.reference.valid &&
             terminal.guarded.execution_allowed,
         "terminal hold remains a valid guarded reference");
  expect(terminal.guarded.reference.route_complete,
         "guard preserves route completion semantics");
  expect(terminal.command.motion_enabled && terminal.command.hold,
         "route completion emits enabled hold rather than topic silence");
  expectNear(terminal.command.signed_speed_mps, 0.0, kTolerance,
             "route completion emits explicit zero speed");
  expectNear(terminal.command.curvature_inv_m, 0.0, kTolerance,
             "route completion emits explicit zero curvature");
  expect(std::abs(rig.rearX() - 2.0) <= 0.15 + 1e-6,
         "integrated rear axle finishes within goal tolerance");
}

auto_rover::VehicleExecutionCommand freshExecution(
    std::uint64_t sequence, double speed_mps, std::int64_t now_ns) {
  auto_rover::VehicleExecutionCommand command;
  command.sequence_id = sequence;
  command.created_monotonic_ns = now_ns;
  command.deadline_monotonic_ns = now_ns + kSecondNs;
  command.signed_speed_mps = speed_mps;
  command.curvature_inv_m = 0.0;
  command.motion_enabled = true;
  command.hold = speed_mps == 0.0;
  command.stop_reason = auto_rover::StopReason::kNone;
  return command;
}

auto_rover_safety::GuardResult allowedReference(double speed_mps,
                                                 std::uint64_t command_id) {
  auto_rover_safety::GuardResult result;
  result.reference.stamp_ns = kSecondNs;
  result.reference.frame_id = "rear_axle_center";
  result.reference.command_id = command_id;
  result.reference.producer_generation_id =
      "full-loop-tracker-generation-1";
  result.reference.trajectory_id = "integration:trajectory:1";
  result.reference.direction = auto_rover::Direction::kForward;
  result.reference.target_speed_mps = speed_mps;
  result.reference.target_curvature_inv_m = 0.0;
  result.reference.valid_for_ns = kSecondNs;
  result.reference.valid = true;
  result.execution_allowed = true;
  result.reason = auto_rover::StopReason::kNone;
  return result;
}

void testFakeOutputWatchdog() {
  auto config = fakeConfig();
  config.command_watchdog_ns = 250000000LL;
  auto_rover_vehicle::FakeVcu fake(config, vehicleProfile());
  fake.setConnected(true, kSecondNs);
  fake.setControlEnabled(true, kSecondNs);
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    const std::int64_t now =
        kSecondNs + static_cast<std::int64_t>(sequence) * 10000000LL;
    const auto received = fake.receive(
        freshExecution(sequence, 0.10, now), now);
    if (sequence < 3U) {
      expect(!received.motion_accepted,
             "fake watchdog fixture observes recovery pending");
    } else {
      expect(received.motion_accepted,
             "fake watchdog fixture first accepts after three fresh commands");
    }
  }
  fake.step(kSecondNs + 200000000LL,
            kSecondNs + 200000000LL);
  const auto expired = fake.step(kSecondNs + 281000000LL,
                                 kSecondNs + 281000000LL);
  expect(expired.watchdog_expired,
         "fake output watchdog expires without command refresh");
  expect(!fake.controlEnabled(),
         "fake output watchdog clears control enable");
}

void testDisconnectDeliveryAndReconnectInhibit() {
  const auto profile = vehicleProfile();
  auto_rover_vehicle::VehicleMotionManager manager(managerConfig(), profile);
  auto_rover_vehicle::FakeVcu fake(fakeConfig(), profile);
  manager.setBackendConnected(true, kSecondNs);
  fake.setConnected(true, kSecondNs);
  expect(manager.acknowledgeOperatorArm(1U, kSecondNs + 10000000LL),
         "disconnect fixture explicitly arms vehicle manager");
  fake.setControlEnabled(true, kSecondNs + 10000000LL);

  const auto moving = manager.makeCommand(
      allowedReference(0.02, 1U), kSecondNs + 110000000LL);
  expect(moving.motion_enabled && moving.signed_speed_mps > 0.0,
         "disconnect fixture creates a valid pre-disconnect command");
  fake.receive(moving, kSecondNs + 110000000LL);

  manager.setBackendConnected(false, kSecondNs + 120000000LL);
  fake.setConnected(false, kSecondNs + 120000000LL);
  const auto disconnected_command = manager.makeCommand(
      allowedReference(0.02, 2U), kSecondNs + 130000000LL);
  const auto disconnected_delivery =
      fake.receive(disconnected_command, kSecondNs + 130000000LL);
  expect(disconnected_command.stop_reason ==
             auto_rover::StopReason::kBackendDisconnected,
         "manager converts disconnect to explicit backend stop");
  expect(!disconnected_delivery.delivered &&
             disconnected_delivery.delivery_unconfirmed,
         "disconnected fake delivery is explicitly unconfirmed");

  manager.setBackendConnected(true, kSecondNs + 140000000LL);
  fake.setConnected(true, kSecondNs + 140000000LL);
  const auto after_reconnect = manager.makeCommand(
      allowedReference(0.02, 3U), kSecondNs + 150000000LL);
  const auto reconnect_delivery =
      fake.receive(after_reconnect, kSecondNs + 150000000LL);
  expect(!manager.operatorArmed() && !fake.controlEnabled(),
         "reconnect clears manager and fake control authorization");
  expect(!after_reconnect.motion_enabled && after_reconnect.hold &&
             after_reconnect.signed_speed_mps == 0.0,
         "reconnect cannot automatically restore the old motion command");
  expect(reconnect_delivery.delivered &&
             !reconnect_delivery.motion_accepted,
         "reconnect delivers only the inhibited zero command");

  expect(manager.acknowledgeOperatorArm(2U,
                                        kSecondNs + 160000000LL),
         "reconnect requires a new manager authorization generation");
  fake.setControlEnabled(true, kSecondNs + 160000000LL);
  auto_rover_vehicle::FakeVcuReceiveResult reauthorized_delivery;
  for (std::uint64_t index = 0U; index < 3U; ++index) {
    const std::int64_t now =
        kSecondNs + 260000000LL +
        static_cast<std::int64_t>(index) * kTickNs;
    const auto reauthorized_command = manager.makeCommand(
        allowedReference(0.02, 4U + index), now);
    expect(reauthorized_command.motion_enabled &&
               reauthorized_command.signed_speed_mps == 0.02,
           "new authorization permits a fresh bounded command");
    reauthorized_delivery = fake.receive(reauthorized_command, now);
    if (index < 2U) {
      expect(!reauthorized_delivery.motion_accepted &&
                 reauthorized_delivery.reason ==
                     auto_rover::StopReason::kRecoveryPending,
             "reconnected fake VCU rebuilds its fresh-command run");
    }
  }
  expect(reauthorized_delivery.motion_accepted,
         "new authorization and three fresh commands restore motion");
}

}  // namespace

int main() {
  testDeterministicPureCoreFullLoop();
  testFakeOutputWatchdog();
  testDisconnectDeliveryAndReconnectInhibit();
  if (g_failures != 0) {
    std::cerr << g_failures << " integration assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "deterministic pure-core full-loop integration tests passed\n";
  return EXIT_SUCCESS;
}
