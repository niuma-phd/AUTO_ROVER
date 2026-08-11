#include <cstdlib>
#include <cstddef>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

#include "auto_rover_core/validation.hpp"
#include "auto_rover_vehicle/vehicle_execution_core.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
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

auto_rover_vehicle::VehicleExecutionCoreConfig config(bool actuation_enabled) {
  auto_rover_vehicle::VehicleExecutionCoreConfig value;
  value.vehicle_profile = profile();
  value.safety_supervisor.actuation_enabled = actuation_enabled;
  value.safety_supervisor.reset_service_enabled = true;
  value.safety_supervisor.fresh_recovery_count = 2U;
  value.safety_supervisor.state_valid_for_ns = 1000000000LL;
  value.authorized_reset_operator_id = "reset-operator";
  value.command_guard.world_frame = "camera_init";
  value.command_guard.localization_freshness_ns = 500000000LL;
  value.command_guard.trajectory_freshness_ns = 500000000LL;
  value.command_guard.motion_freshness_ns = 500000000LL;
  value.command_guard.chassis_freshness_ns = 500000000LL;
  value.command_guard.safety_freshness_ns = 500000000LL;
  value.command_guard.fresh_recovery_count = 2U;
  value.vehicle_manager.command_valid_for_ns = 100000000LL;
  value.fake_vcu.process_generation_id =
      "vehicle-execution-test-fake-vcu-generation-1";
  value.fake_vcu.command_watchdog_ns = 150000000LL;
  value.fake_vcu.feedback_period_ns = 50000000LL;
  value.fake_vcu.fresh_recovery_count = 2U;
  value.fake_vcu.max_accel_mps2 = 0.20;
  return value;
}

auto_rover_vehicle::VehicleExecutionCommonConfig commonConfig(
    bool actuation_enabled) {
  const auto_rover_vehicle::VehicleExecutionCoreConfig complete =
      config(actuation_enabled);
  auto_rover_vehicle::VehicleExecutionCommonConfig value;
  value.vehicle_profile = complete.vehicle_profile;
  value.safety_supervisor = complete.safety_supervisor;
  value.authorized_reset_operator_id =
      complete.authorized_reset_operator_id;
  value.command_guard = complete.command_guard;
  value.vehicle_manager = complete.vehicle_manager;
  return value;
}

auto_rover::EgoState ego() {
  auto_rover::EgoState value;
  value.stamp_ns = 2000000000LL;
  value.frame_id = "camera_init";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kPublishTime;
  value.reference_frame = "rear_axle_center";
  value.pose.orientation.w = 1.0;
  value.source_id = "synthetic_fast_livo";
  value.valid = true;
  return value;
}

auto_rover::Trajectory trajectory() {
  auto_rover::Trajectory value;
  value.stamp_ns = 2000000000LL;
  value.frame_id = "camera_init";
  value.trajectory_id = "route:1:1";
  value.route_id = "route";
  value.plan_version = 1U;
  value.vehicle_profile_id = "nuc_senior_akm_v1";
  value.valid_for_ns = 1000000000LL;
  value.completion_behavior = auto_rover::CompletionBehavior::kStopAndHold;
  value.valid = true;
  auto_rover::TrajectoryPoint first;
  first.direction = auto_rover::Direction::kForward;
  auto second = first;
  second.x_m = 1.0;
  second.arc_length_m = 1.0;
  value.points = {first, second};
  return value;
}

auto_rover::MotionReference motion() {
  auto_rover::MotionReference value;
  value.stamp_ns = 2000000000LL;
  value.frame_id = "rear_axle_center";
  value.command_id = 1U;
  value.producer_generation_id =
      "vehicle-execution-test-tracker-generation-1";
  value.trajectory_id = "route:1:1";
  value.direction = auto_rover::Direction::kForward;
  value.valid_for_ns = 1000000000LL;
  value.valid = true;
  return value;
}

auto_rover::ChassisState chassis(std::uint64_t state_id,
                                 std::int64_t stamp_ns,
                                 const std::string& source_id) {
  auto_rover::ChassisState value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "rear_axle_center";
  value.state_id = state_id;
  value.time_source = auto_rover::TimeSource::kReceiptTime;
  value.measured_speed_mps = 0.0;
  value.control_enabled = false;
  value.fault_state = auto_rover::FaultState::kOk;
  value.valid_mask = auto_rover::ChassisState::kMeasuredSpeedValid |
                     auto_rover::ChassisState::kControlEnabledValid |
                     auto_rover::ChassisState::kFaultValid;
  value.source_id = source_id;
  value.valid = true;
  return value;
}

class ScriptedBackend final : public auto_rover_vehicle::VehicleBackend {
 public:
  auto_rover::ValidationResult validate(
      const auto_rover::VehicleProfile& vehicle_profile) const override {
    if (!configuration_valid_) {
      return auto_rover::ValidationResult::failure(
          "scripted backend configuration is invalid");
    }
    return auto_rover::validateNucPhase1Profile(vehicle_profile);
  }

  bool initialize(std::int64_t now_monotonic_ns,
                  std::int64_t now_ros_ns) override {
    initialized_ = configuration_valid_ && now_monotonic_ns > 0 &&
                   now_ros_ns > 0;
    return initialized_;
  }

  auto_rover_vehicle::BackendFeedback poll(
      std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns) override {
    auto_rover_vehicle::BackendFeedback result;
    result.operation_completed_monotonic_ns =
        now_monotonic_ns + poll_completion_offset_ns_;
    result.operation_completed_ros_ns =
        now_ros_ns + poll_completion_offset_ns_;
    result.health = health();
    if (!initialized_ || !connected_ || !feedback_fresh_ ||
        now_monotonic_ns <= 0 || now_ros_ns <= 0) {
      return result;
    }
    ++state_id_;
    result.available = true;
    result.receipt_monotonic_ns = now_monotonic_ns + receipt_offset_ns_;
    result.state.stamp_ns = now_ros_ns + receipt_offset_ns_;
    result.state.frame_id = "rear_axle_center";
    result.state.state_id = state_id_;
    result.state.time_source = auto_rover::TimeSource::kReceiptTime;
    result.state.measured_speed_mps = 0.0;
    result.state.yaw_rate_radps = 0.0;
    result.state.supply_voltage_v = 24.0;
    result.state.valid_mask =
        auto_rover::ChassisState::kMeasuredSpeedValid |
        auto_rover::ChassisState::kYawRateValid |
        auto_rover::ChassisState::kVoltageValid;
    if (substantiate_control_enabled_) {
      result.state.valid_mask |=
          auto_rover::ChassisState::kControlEnabledValid;
      result.state.control_enabled = reported_control_enabled_;
    }
    if (substantiate_fault_) {
      result.state.valid_mask |= auto_rover::ChassisState::kFaultValid;
      result.state.fault_state = reported_fault_;
    }
    result.state.source_id = "scripted_backend_v1#process=test";
    result.state.valid = true;
    result.health = health();
    return result;
  }

  bool setAuthorization(bool enabled, std::uint64_t authorization_id,
                        std::int64_t now_monotonic_ns) override {
    if (!enabled) {
      authorized_ = false;
      return true;
    }
    if (!initialized_ || !connected_ || !actuation_permitted_ ||
        reject_authorization_ || authorization_id == 0U ||
        authorization_id <= highest_authorization_id_ ||
        now_monotonic_ns <= 0) {
      authorized_ = false;
      return false;
    }
    highest_authorization_id_ = authorization_id;
    authorized_ = true;
    if (throw_after_authorization_) {
      throw std::runtime_error(
          "scripted setAuthorization exception after mutation");
    }
    return true;
  }

  auto_rover_vehicle::BackendDelivery deliver(
      const auto_rover::VehicleExecutionCommand& command,
      std::int64_t receipt_monotonic_ns) override {
    ++delivery_calls_;
    last_delivery_receipt_monotonic_ns_ = receipt_monotonic_ns;
    auto_rover_vehicle::BackendDelivery result;
    result.reason = command.stop_reason;
    if (!connected_ || fail_next_delivery_ || receipt_monotonic_ns <= 0) {
      fail_next_delivery_ = false;
      result.delivery_unconfirmed = true;
      result.diagnostic = "scripted delivery is unconfirmed";
      delivery_unconfirmed_ = true;
      authorized_ = false;
      return result;
    }
    result.delivered = true;
    result.motion_accepted = command.motion_enabled && !command.hold &&
                             command.stop_reason ==
                                 auto_rover::StopReason::kNone &&
                             authorized_;
    if (result.motion_accepted) {
      ++motion_accepted_calls_;
    }
    result.reason = result.motion_accepted
                        ? auto_rover::StopReason::kNone
                        : command.stop_reason;
    result.diagnostic = "scripted local delivery complete";
    return result;
  }

  auto_rover_vehicle::BackendDelivery revokeAndStop(
      std::int64_t now_monotonic_ns) override {
    ++stop_calls_;
    if (throw_before_stop_revocation_) {
      throw std::runtime_error(
          "scripted stop exception before authorization revocation");
    }
    authorized_ = false;
    auto_rover_vehicle::BackendDelivery result;
    result.stop_attempted = true;
    result.reason = auto_rover::StopReason::kDisarmed;
    if (!connected_ || fail_stop_delivery_ || now_monotonic_ns <= 0) {
      result.delivery_unconfirmed = true;
      result.diagnostic = "scripted stop delivery is unconfirmed";
      return result;
    }
    result.delivered = true;
    result.diagnostic = "scripted immediate stop delivered";
    return result;
  }

  auto_rover_vehicle::BackendHealth health() const override {
    if (authorized_ && throw_health_when_authorized_) {
      if (throw_bad_alloc_from_health_) {
        throw std::bad_alloc();
      }
      throw std::runtime_error(
          "scripted health diagnostic materialization exception");
    }
    auto_rover_vehicle::BackendHealth result;
    result.configuration_valid = configuration_valid_;
    result.connected = connected_;
    result.actuation_permitted = actuation_permitted_;
    result.feedback_fresh = feedback_fresh_;
    result.authorization_active = authorized_;
    result.delivery_unconfirmed = delivery_unconfirmed_;
    result.connection_generation = connection_generation_;
    result.reason = connected_ && actuation_permitted_ && feedback_fresh_ &&
                            !delivery_unconfirmed_
                        ? auto_rover::StopReason::kNone
                        : auto_rover::StopReason::kBackendDisconnected;
    result.diagnostic = "scripted backend health";
    return result;
  }

  void changeConnectionGeneration() {
    ++connection_generation_;
    authorized_ = false;
  }
  void setConnectionState(bool connected, std::uint64_t generation) {
    connected_ = connected;
    connection_generation_ = generation;
    authorized_ = false;
    if (connected_) {
      delivery_unconfirmed_ = false;
    }
  }
  void setConnected(bool connected) {
    if (connected && !connected_) {
      ++connection_generation_;
    }
    connected_ = connected;
    if (!connected_) {
      authorized_ = false;
    } else {
      delivery_unconfirmed_ = false;
    }
  }
  void loseAuthorization() { authorized_ = false; }
  void failNextDelivery() { fail_next_delivery_ = true; }
  void reportFault(auto_rover::FaultState fault) {
    substantiate_fault_ = true;
    reported_fault_ = fault;
  }
  void reportControlEnabled(bool enabled) {
    substantiate_control_enabled_ = true;
    reported_control_enabled_ = enabled;
  }
  void rejectAuthorization(bool reject) { reject_authorization_ = reject; }
  void throwAfterAuthorization(bool enabled) {
    throw_after_authorization_ = enabled;
  }
  void throwFromHealthWhenAuthorized(bool bad_alloc) {
    throw_health_when_authorized_ = true;
    throw_bad_alloc_from_health_ = bad_alloc;
  }
  void clearThrows() {
    throw_after_authorization_ = false;
    throw_health_when_authorized_ = false;
    throw_bad_alloc_from_health_ = false;
    throw_before_stop_revocation_ = false;
  }
  void throwBeforeStopRevocation(bool enabled) {
    throw_before_stop_revocation_ = enabled;
  }
  void failStopDelivery(bool fail) { fail_stop_delivery_ = fail; }
  void setReceiptOffset(std::int64_t offset_ns) {
    receipt_offset_ns_ = offset_ns;
  }
  void setPollCompletionOffset(std::int64_t offset_ns) {
    poll_completion_offset_ns_ = offset_ns;
  }

  std::int64_t lastDeliveryReceiptMonotonicNs() const {
    return last_delivery_receipt_monotonic_ns_;
  }

  std::uint64_t stopCalls() const { return stop_calls_; }
  std::uint64_t deliveryCalls() const { return delivery_calls_; }
  std::uint64_t motionAcceptedCalls() const {
    return motion_accepted_calls_;
  }
  bool authorized() const { return authorized_; }

 private:
  bool configuration_valid_{true};
  bool initialized_{false};
  bool connected_{true};
  bool actuation_permitted_{true};
  bool feedback_fresh_{true};
  bool authorized_{false};
  bool delivery_unconfirmed_{false};
  bool fail_next_delivery_{false};
  bool fail_stop_delivery_{false};
  bool reject_authorization_{false};
  bool throw_after_authorization_{false};
  bool throw_health_when_authorized_{false};
  bool throw_bad_alloc_from_health_{false};
  bool throw_before_stop_revocation_{false};
  bool substantiate_control_enabled_{false};
  bool reported_control_enabled_{false};
  bool substantiate_fault_{false};
  auto_rover::FaultState reported_fault_{auto_rover::FaultState::kUnknown};
  std::uint64_t connection_generation_{1U};
  std::uint64_t highest_authorization_id_{0U};
  std::uint64_t state_id_{0U};
  std::uint64_t delivery_calls_{0U};
  std::uint64_t motion_accepted_calls_{0U};
  std::uint64_t stop_calls_{0U};
  std::int64_t receipt_offset_ns_{0};
  std::int64_t poll_completion_offset_ns_{0};
  std::int64_t last_delivery_receipt_monotonic_ns_{0};
};

void provideInputs(auto_rover_vehicle::VehicleExecutionCore* runtime,
                   std::int64_t receipt_ns) {
  runtime->updateEgoState(ego(), receipt_ns);
  runtime->updateTrajectory(trajectory(), receipt_ns);
  runtime->updateMotionReference(motion(), receipt_ns);
}

void recoverToDisarmed(auto_rover_vehicle::VehicleExecutionCore* runtime) {
  provideInputs(runtime, 1010000000LL);
  runtime->cycle(1050000000LL, 2050000000LL);
  runtime->cycle(1100000000LL, 2100000000LL);
}

auto_rover_vehicle::VehicleExecutionCycleResult establishNonzeroExecution(
    auto_rover_vehicle::VehicleExecutionCore* runtime) {
  recoverToDisarmed(runtime);
  const auto ready = runtime->currentSafetyState(2110000000LL);
  const auto armed = runtime->requestArm(
      "operator", ready.state_id, true, 1110000000LL, 2110000000LL);
  expect(armed.success && runtime->executionAuthorized(),
         "replay fixture explicitly arms a healthy runtime");

  auto reference = motion();
  reference.command_id = 2U;
  reference.stamp_ns = 2120000000LL;
  reference.target_speed_mps = 0.25;
  runtime->updateMotionReference(reference, 1120000000LL);
  runtime->cycle(1150000000LL, 2150000000LL);

  reference.command_id = 3U;
  reference.stamp_ns = 2160000000LL;
  runtime->updateMotionReference(reference, 1160000000LL);
  runtime->cycle(1200000000LL, 2200000000LL);
  const auto moving = runtime->cycle(1250000000LL, 2250000000LL);
  expect(moving.command.motion_enabled &&
             moving.command.signed_speed_mps > 0.0 &&
             moving.delivery.motion_accepted,
         "replay fixture reaches confirmed nonzero execution");
  return moving;
}

void testConfigurationDefaultsFailClosed() {
  auto invalid = config(false);
  invalid.command_guard.motion_freshness_ns = 0;
  expect(!auto_rover_vehicle::validateVehicleExecutionCoreConfig(invalid).ok,
         "missing safety freshness rejects execution configuration");

  invalid = config(false);
  invalid.fake_vcu.process_generation_id.clear();
  expect(!auto_rover_vehicle::validateVehicleExecutionCoreConfig(invalid).ok,
         "missing fake VCU process generation rejects execution configuration");

  auto disabled = config(false);
  auto_rover_vehicle::VehicleExecutionCore runtime(disabled);
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "actuation-disabled configuration can run safely");
  recoverToDisarmed(&runtime);
  const auto state = runtime.currentSafetyState(2110000000LL);
  const auto arm = runtime.requestArm("operator", state.state_id, true,
                                      1110000000LL, 2110000000LL);
  expect(!arm.success && !runtime.executionAuthorized(),
         "default-disabled actuation cannot be armed");

  std::unique_ptr<auto_rover_vehicle::VehicleBackend> missing_backend;
  auto_rover_vehicle::VehicleExecutionCore missing(
      commonConfig(false), std::move(missing_backend));
  expect(!missing.initialize(1000000000LL, 2000000000LL),
         "an injected core without a backend fails closed");

  auto* disconnected_scripted = new ScriptedBackend();
  disconnected_scripted->setConnected(false);
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> disconnected_backend(
      disconnected_scripted);
  auto_rover_vehicle::VehicleExecutionCore disconnected_runtime(
      commonConfig(false), std::move(disconnected_backend));
  expect(disconnected_runtime.initialize(1000000000LL, 2000000000LL),
         "a validated disconnected backend safely initializes into inhibit");
  const auto disconnected_cycle =
      disconnected_runtime.cycle(1050000000LL, 2050000000LL);
  const auto disconnected_state =
      disconnected_runtime.currentSafetyState(2060000000LL);
  const auto disconnected_arm = disconnected_runtime.requestArm(
      "operator", disconnected_state.state_id, true,
      1060000000LL, 2060000000LL);
  expect(!disconnected_cycle.health_clear &&
             disconnected_cycle.health_reason ==
                 auto_rover::StopReason::kBackendDisconnected &&
             !disconnected_arm.success &&
             !disconnected_runtime.executionAuthorized(),
         "a disconnected injected backend cannot recover or arm");
}

void testServicesEstopResetAndReconnect() {
  auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "valid fake execution configuration initializes");
  recoverToDisarmed(&runtime);

  auto state = runtime.currentSafetyState(2110000000LL);
  auto transition = runtime.requestArm("operator", state.state_id, true,
                                       1110000000LL, 2110000000LL);
  expect(transition.success && runtime.executionAuthorized() &&
             runtime.fakeControlEnabled(),
         "healthy explicit arm enables manager and fake VCU");

  auto_rover::EmergencyStop estop;
  estop.stamp_ns = 2120000000LL;
  estop.request_id = "test-estop";
  estop.source_id = "vehicle-ros-test";
  estop.asserted = true;
  transition = runtime.handleEmergencyStop(estop, 1120000000LL,
                                           2120000000LL);
  expect(transition.success &&
             transition.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             !runtime.executionAuthorized() &&
             !runtime.fakeControlEnabled(),
         "emergency stop latches and immediately revokes execution");

  runtime.cycle(1150000000LL, 2150000000LL);
  runtime.cycle(1200000000LL, 2200000000LL);
  auto reset = runtime.resetEmergencyStop(
      "wrong-operator", transition.state.latch_generation, true,
      1210000000LL, 2210000000LL);
  expect(!reset.success, "reset rejects an unauthorized operator");
  reset = runtime.resetEmergencyStop(
      "reset-operator", transition.state.latch_generation, true,
      1220000000LL, 2220000000LL);
  expect(reset.success &&
             reset.state.mode == auto_rover::SafetyMode::kDisarmed &&
             !runtime.executionAuthorized(),
         "authorized generation-matched reset remains disarmed");

  state = runtime.currentSafetyState(2230000000LL);
  transition = runtime.requestArm("operator", state.state_id, true,
                                  1230000000LL, 2230000000LL);
  expect(transition.success, "healthy disarmed state can explicitly re-arm");
  const auto disconnected = runtime.setFakeConnected(
      false, 1240000000LL, 2240000000LL);
  expect(disconnected.success && !runtime.fakeConnected() &&
             !runtime.executionAuthorized(),
         "disconnect immediately revokes authorization");
  const auto reconnected = runtime.setFakeConnected(
      true, 1250000000LL, 2250000000LL);
  expect(reconnected.success && runtime.fakeConnected() &&
             !runtime.executionAuthorized() &&
             !runtime.fakeControlEnabled(),
         "reconnect remains inhibited until a new healthy explicit arm");
}

void testRequiredInputSourceOrderingFailsClosed() {
  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "ego ordering runtime initializes");
    auto value = ego();
    value.state_id = 10U;
    value.stamp_ns = 2100000000LL;
    expect(runtime.updateEgoState(value, 1010000000LL).ok,
           "initial ego sequence is accepted");
    value.state_id = 11U;
    value.stamp_ns = 2200000000LL;
    expect(runtime.updateEgoState(value, 1020000000LL).ok,
           "advancing ego identity and stamp are accepted");
    value.state_id = 9U;
    value.stamp_ns = 2300000000LL;
    const auto rollback = runtime.updateEgoState(value, 1030000000LL);
    expect(!rollback.ok &&
               contains(rollback.reason, "state identity rolled back") &&
               !runtime.executionAuthorized(),
           "later-received ego identity rollback is rejected and revokes execution");
    value.state_id = 12U;
    value.stamp_ns = 2400000000LL;
    const auto same_source = runtime.updateEgoState(value, 1040000000LL);
    expect(!same_source.ok &&
               contains(same_source.reason, "new identity is required"),
           "same ego producer identity cannot silently recover after rollback");
    value.source_id = "synthetic_fast_livo_generation_2";
    value.state_id = 1U;
    value.stamp_ns = 2000000000LL;
    expect(runtime.updateEgoState(value, 1050000000LL).ok,
           "new ego source identity establishes a new sequence");
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "ego stamp-ordering runtime initializes");
    auto value = ego();
    value.state_id = 10U;
    value.stamp_ns = 2200000000LL;
    expect(runtime.updateEgoState(value, 1010000000LL).ok,
           "initial ego stamp is accepted");
    value.state_id = 11U;
    value.stamp_ns = 2100000000LL;
    const auto rollback = runtime.updateEgoState(value, 1020000000LL);
    expect(!rollback.ok &&
               contains(rollback.reason, "source stamp rolled back"),
           "ego source-stamp rollback is detected independently");
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "trajectory ordering runtime initializes");
    auto value = trajectory();
    value.trajectory_id = "route:2:1";
    value.plan_version = 2U;
    value.stamp_ns = 2100000000LL;
    expect(runtime.updateTrajectory(value, 1010000000LL).ok,
           "initial trajectory generation is accepted");
    value.stamp_ns = 2200000000LL;
    expect(runtime.updateTrajectory(value, 1020000000LL).ok,
           "same immutable trajectory generation may refresh its stamp");
    value.stamp_ns = 2150000000LL;
    const auto stamp_rollback =
        runtime.updateTrajectory(value, 1030000000LL);
    expect(!stamp_rollback.ok &&
               contains(stamp_rollback.reason,
                        "trajectory source stamp rolled back"),
           "trajectory source-stamp rollback is rejected");
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "plan-version ordering runtime initializes");
    auto value = trajectory();
    value.trajectory_id = "route:2:1";
    value.plan_version = 2U;
    expect(runtime.updateTrajectory(value, 1010000000LL).ok,
           "trajectory plan version two is accepted");
    value.trajectory_id = "route:1:restart";
    value.plan_version = 1U;
    value.stamp_ns = 2100000000LL;
    const auto rollback = runtime.updateTrajectory(value, 1020000000LL);
    expect(!rollback.ok &&
               contains(rollback.reason, "plan version did not advance"),
           "newly received trajectory generation cannot roll back a route plan version");
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "chassis ordering runtime initializes");
    expect(runtime.updateChassisState(
               chassis(10U, 2100000000LL, "external_fake_vcu"),
               1010000000LL).ok,
           "initial chassis source sequence is accepted");
    const auto identity_rollback = runtime.updateChassisState(
        chassis(9U, 2200000000LL, "external_fake_vcu"),
        1020000000LL);
    expect(!identity_rollback.ok &&
               contains(identity_rollback.reason,
                        "chassis state identity rolled back"),
           "later-received chassis identity rollback fails closed");
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "chassis stamp-ordering runtime initializes");
    expect(runtime.updateChassisState(
               chassis(10U, 2200000000LL, "external_fake_vcu"),
               1010000000LL).ok,
           "initial chassis stamp is accepted");
    const auto stamp_rollback = runtime.updateChassisState(
        chassis(11U, 2100000000LL, "external_fake_vcu"),
        1020000000LL);
    expect(!stamp_rollback.ok &&
               contains(stamp_rollback.reason,
                        "chassis source stamp rolled back"),
           "chassis source-stamp rollback is detected independently");
  }
}

void testRepeatedRequiredStateCallbacksCannotSustainExecution() {
  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "ego callback-replay runtime initializes");
    establishNonzeroExecution(&runtime);

    for (std::int64_t index = 0; index < 4; ++index) {
      const std::int64_t receipt = 1260000000LL + index * 20000000LL;
      const auto replay = runtime.updateEgoState(ego(), receipt);
      expect(!replay.ok,
             "a later ego callback cannot reuse an old semantic sample");
      if (index == 0) {
        expect(contains(replay.reason,
                        "ego semantic sample was replayed by a later callback"),
               "the first duplicate ego callback reports semantic replay");
      } else {
        expect(contains(replay.reason, "new identity is required"),
               "an ego replay compromises that producer identity");
      }
      const auto stopped = runtime.cycle(
          receipt + 10000000LL, 2260000000LL + index * 20000000LL);
      expect(!runtime.executionAuthorized() &&
                 !runtime.fakeControlEnabled() &&
                 !stopped.command.motion_enabled &&
                 stopped.command.signed_speed_mps == 0.0,
             "retransmitted old ego callbacks cannot refresh health or sustain nonzero execution");
    }
  }

  {
    auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "chassis callback-replay runtime initializes");
    const auto moving = establishNonzeroExecution(&runtime);
    expect(moving.chassis_available,
           "nonzero fixture exposes the cached chassis snapshot");

    for (std::int64_t index = 0; index < 4; ++index) {
      const std::int64_t receipt = 1260000000LL + index * 20000000LL;
      const auto replay = runtime.updateChassisState(
          moving.chassis, receipt);
      expect(!replay.ok,
             "a later chassis callback cannot reuse an old semantic sample");
      if (index == 0) {
        expect(contains(
                   replay.reason,
                   "chassis semantic sample was replayed by a later callback"),
               "the first duplicate chassis callback reports semantic replay");
      } else {
        expect(contains(replay.reason, "new identity is required"),
               "a chassis replay compromises that producer identity");
      }
      const auto stopped = runtime.cycle(
          receipt + 10000000LL, 2260000000LL + index * 20000000LL);
      expect(!runtime.executionAuthorized() &&
                 !runtime.fakeControlEnabled() &&
                 !stopped.command.motion_enabled &&
                 stopped.command.signed_speed_mps == 0.0,
             "retransmitted old chassis callbacks cannot refresh health or sustain nonzero execution");
    }
  }
}

void testEmergencyStopResetRechecksCurrentRequiredInputFreshness() {
  auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "reset freshness runtime initializes");
  recoverToDisarmed(&runtime);

  auto_rover::EmergencyStop assertion;
  assertion.stamp_ns = 2120000000LL;
  assertion.request_id = "reset-freshness-estop";
  assertion.source_id = "vehicle-ros-test";
  assertion.asserted = true;
  const auto latched = runtime.handleEmergencyStop(
      assertion, 1120000000LL, 2120000000LL);
  expect(latched.success &&
             latched.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "reset freshness fixture latches emergency stop");
  const std::uint64_t generation = latched.state.latch_generation;

  runtime.cycle(1150000000LL, 2150000000LL);
  runtime.cycle(1200000000LL, 2200000000LL);
  const auto stale_reset = runtime.resetEmergencyStop(
      "reset-operator", generation, true,
      1800000000LL, 2800000000LL);
  expect(!stale_reset.success &&
             stale_reset.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             stale_reset.state.latch_generation == generation &&
             !runtime.executionAuthorized() &&
             !runtime.fakeControlEnabled(),
         "required inputs expiring after clear observations reject reset and preserve the latch generation");
  expect(contains(stale_reset.reason, "watchdog expired"),
         "stale reset reports the current receiver-watchdog failure");

  auto fresh_ego = ego();
  fresh_ego.state_id = 2U;
  fresh_ego.stamp_ns = 2810000000LL;
  auto fresh_trajectory = trajectory();
  fresh_trajectory.stamp_ns = 2810000000LL;
  auto fresh_motion = motion();
  fresh_motion.command_id = 2U;
  fresh_motion.stamp_ns = 2810000000LL;
  expect(runtime.updateEgoState(fresh_ego, 1810000000LL).ok,
         "fresh reset recovery accepts an advancing ego sample");
  expect(runtime.updateTrajectory(fresh_trajectory, 1810000000LL).ok,
         "fresh reset recovery accepts a refreshed trajectory");
  runtime.updateMotionReference(fresh_motion, 1810000000LL);
  runtime.cycle(1850000000LL, 2850000000LL);
  runtime.cycle(1900000000LL, 2900000000LL);

  const auto fresh_reset = runtime.resetEmergencyStop(
      "reset-operator", generation, true,
      1910000000LL, 2910000000LL);
  expect(fresh_reset.success &&
             fresh_reset.state.mode == auto_rover::SafetyMode::kDisarmed &&
             fresh_reset.state.latch_generation == generation &&
             !runtime.executionAuthorized(),
         "authorized generation-matched reset succeeds only after a new full fresh run and remains disarmed");
}

void testEmergencyStopLatchesBeforeLocalClockValidation() {
  auto_rover_vehicle::VehicleExecutionCore runtime(config(true));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "emergency-stop clock-failure runtime initializes");
  recoverToDisarmed(&runtime);
  const auto ready = runtime.currentSafetyState(2110000000LL);
  const auto armed = runtime.requestArm(
      "operator", ready.state_id, true, 1110000000LL, 2110000000LL);
  expect(armed.success && runtime.executionAuthorized() &&
             runtime.fakeControlEnabled(),
         "emergency-stop clock-failure fixture begins authorized");

  auto_rover::EmergencyStop assertion;
  assertion.stamp_ns = 2000000000LL;
  assertion.request_id = "clock-failure-estop-1";
  assertion.source_id = "vehicle-ros-test";
  assertion.asserted = true;

  auto result = runtime.handleEmergencyStop(assertion, 0LL, 0LL);
  expect(result.success &&
             result.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             result.state.latch_generation == 1U && !result.state.valid &&
             !runtime.executionAuthorized() &&
             !runtime.fakeControlEnabled(),
         "valid emergency stop latches even when both local clocks are zero");

  assertion.request_id = "clock-failure-estop-2";
  result = runtime.handleEmergencyStop(assertion, 1000000000LL,
                                       2200000000LL);
  expect(result.success &&
             result.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             result.state.latch_generation == 2U && !result.state.valid,
         "valid emergency stop increments the latch on monotonic rollback");

  assertion.request_id = "clock-failure-estop-3";
  result = runtime.handleEmergencyStop(assertion, 1200000000LL,
                                       2000000000LL);
  expect(result.success &&
             result.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             result.state.latch_generation == 3U && !result.state.valid,
         "valid emergency stop increments the latch on ROS-time rollback");

  assertion.request_id = "clock-recovered-estop-4";
  result = runtime.handleEmergencyStop(assertion, 1200000000LL,
                                       2200000000LL);
  expect(result.success && result.state.latch_generation == 4U &&
             result.state.valid,
         "invalid-clock assertions do not record receipt and block a later valid event time");

  auto_rover::EmergencyStop invalid;
  invalid.stamp_ns = 2000000000LL;
  invalid.request_id = "not-an-assertion";
  invalid.source_id = "vehicle-ros-test";
  invalid.asserted = false;
  result = runtime.handleEmergencyStop(invalid, 0LL, 0LL);
  expect(!result.success && result.state.latch_generation == 4U &&
             result.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             !result.state.valid,
         "invalid emergency-stop request is rejected without changing the latch");
}

auto_rover_vehicle::VehicleExecutionServiceResult armInjectedRuntime(
    auto_rover_vehicle::VehicleExecutionCore* runtime) {
  recoverToDisarmed(runtime);
  const auto ready = runtime->currentSafetyState(2110000000LL);
  return runtime->requestArm("operator", ready.state_id, true,
                             1110000000LL, 2110000000LL);
}

void testInjectedBackendDoesNotFabricateOptionalChassisState() {
  auto* scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "an injected protocol-independent backend initializes");
  const auto armed = armInjectedRuntime(&runtime);
  expect(armed.success && runtime.executionAuthorized(),
         "unavailable optional fault and control fields are not fabricated or globally required");

  scripted->reportFault(auto_rover::FaultState::kFault);
  const auto faulted = runtime.cycle(1150000000LL, 2150000000LL);
  expect(!faulted.health_clear && !runtime.executionAuthorized() &&
             contains(faulted.health_diagnostic, "fault"),
         "a substantiated non-OK backend fault still fails closed");

  auto* control_scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> control_backend(
      control_scripted);
  auto_rover_vehicle::VehicleExecutionCore control_runtime(
      commonConfig(true), std::move(control_backend));
  expect(control_runtime.initialize(1000000000LL, 2000000000LL),
         "control-status fixture initializes");
  expect(armInjectedRuntime(&control_runtime).success,
         "control-status fixture arms before an evidenced disable");
  control_scripted->reportControlEnabled(false);
  const auto disabled =
      control_runtime.cycle(1150000000LL, 2150000000LL);
  expect(!disabled.health_clear &&
             !control_runtime.executionAuthorized() &&
             contains(disabled.health_diagnostic, "control"),
         "a substantiated disabled control state fails closed while armed");

  auto* unexpected_control = new ScriptedBackend();
  unexpected_control->reportControlEnabled(true);
  std::unique_ptr<auto_rover_vehicle::VehicleBackend>
      unexpected_control_backend(unexpected_control);
  auto_rover_vehicle::VehicleExecutionCore unexpected_control_runtime(
      commonConfig(true), std::move(unexpected_control_backend));
  expect(unexpected_control_runtime.initialize(1000000000LL, 2000000000LL),
         "unexpected-control fixture initializes");
  provideInputs(&unexpected_control_runtime, 1010000000LL);
  const auto unexpected =
      unexpected_control_runtime.cycle(1050000000LL, 2050000000LL);
  expect(!unexpected.health_clear &&
             contains(unexpected.health_diagnostic, "control enabled"),
         "a substantiated control-enabled state also fails closed before arm");
}

void testExecutionInputBudgetsFailClosedBeforeUnboundedWork() {
  auto* scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "execution-input budget fixture initializes");
  expect(armInjectedRuntime(&runtime).success &&
             runtime.executionAuthorized(),
         "execution-input budget fixture begins authorized");

  auto bounded = trajectory();
  bounded.points.clear();
  bounded.points.reserve(
      auto_rover_vehicle::kMaximumExecutionTrajectoryPoints);
  for (std::size_t index = 0U;
       index < auto_rover_vehicle::kMaximumExecutionTrajectoryPoints;
       ++index) {
    auto_rover::TrajectoryPoint point;
    point.x_m = static_cast<double>(index) * 0.05;
    point.arc_length_m = point.x_m;
    point.direction = auto_rover::Direction::kForward;
    bounded.points.push_back(point);
  }
  expect(runtime.updateTrajectory(bounded, 1120000000LL).ok &&
             runtime.executionAuthorized(),
         "the documented 4096-point execution boundary is accepted");

  auto oversized = bounded;
  oversized.points.push_back(bounded.points.back());
  const std::uint64_t stops_before = scripted->stopCalls();
  const auto rejected = runtime.updateTrajectory(
      oversized, 1130000000LL);
  expect(!rejected.ok && contains(rejected.reason, "point-count") &&
             scripted->stopCalls() > stops_before &&
             !runtime.executionAuthorized(),
         "a 4097-point trajectory is rejected and stops an authorized backend");

  auto oversized_config = commonConfig(true);
  oversized_config.authorized_reset_operator_id.assign(
      auto_rover_vehicle::kMaximumExecutionIdentifierBytes + 1U, 'x');
  expect(!auto_rover_vehicle::validateVehicleExecutionCommonConfig(
              oversized_config)
              .ok,
         "execution configuration identities have a fixed byte budget");
}

void testInjectedBackendAcceptsReceiptAfterCycleEntry() {
  auto* scripted = new ScriptedBackend();
  scripted->setReceiptOffset(1000000LL);
  scripted->setPollCompletionOffset(2000000LL);
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "post-entry receipt fixture initializes");
  provideInputs(&runtime, 1010000000LL);
  const auto first = runtime.cycle(1050000000LL, 2050000000LL);
  const auto second = runtime.cycle(1100000000LL, 2100000000LL);
  expect(first.chassis_available && second.chassis_available &&
             !contains(first.health_diagnostic, "receipt") &&
             !contains(second.health_diagnostic, "receipt"),
         "a read receipt before a later poll completion remains valid");
  expect(scripted->lastDeliveryReceiptMonotonicNs() == 1102000000LL,
         "delivery uses the bounded poll completion rather than reusing the earlier cycle entry or feedback receipt");
  const auto ready = runtime.currentSafetyState(2110000000LL);
  const auto armed = runtime.requestArm(
      "operator", ready.state_id, true, 1110000000LL, 2110000000LL);
  expect(armed.success && runtime.executionAuthorized(),
         "post-entry receipt timing does not prevent a later explicit arm");
}

void testInjectedBackendTransitionsRequireNewArmAndImmediateStop() {
  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "connection-generation fixture initializes");
    expect(armInjectedRuntime(&runtime).success &&
               runtime.executionAuthorized(),
           "connection-generation fixture arms");
    const std::uint64_t stop_calls = scripted->stopCalls();
    scripted->changeConnectionGeneration();
    runtime.cycle(1150000000LL, 2150000000LL);
    expect(!runtime.executionAuthorized() &&
               scripted->stopCalls() > stop_calls,
           "a connection-generation change revokes authorization and attempts an immediate stop");
  }


  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "disconnect fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "disconnect fixture arms");
    const std::uint64_t stop_calls = scripted->stopCalls();
    scripted->setConnected(false);
    const auto disconnected = runtime.cycle(1150000000LL, 2150000000LL);
    expect(!disconnected.health_clear &&
               disconnected.health_reason ==
                   auto_rover::StopReason::kBackendDisconnected &&
               !runtime.executionAuthorized() &&
               scripted->stopCalls() > stop_calls,
           "backend disconnect revokes authorization and immediately attempts a stop");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "authorization-loss fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "authorization-loss fixture arms");
    const std::uint64_t stop_calls = scripted->stopCalls();
    scripted->loseAuthorization();
    const auto lost = runtime.cycle(1150000000LL, 2150000000LL);
    expect(!lost.health_clear && !runtime.executionAuthorized() &&
               scripted->stopCalls() > stop_calls,
           "backend authorization loss revokes the manager and immediately stops");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "delivery-loss fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "delivery-loss fixture arms");
    const std::uint64_t stop_calls = scripted->stopCalls();
    scripted->failNextDelivery();
    const auto result = runtime.cycle(1150000000LL, 2150000000LL);
    expect(result.delivery.delivery_unconfirmed &&
               !runtime.executionAuthorized() &&
               scripted->stopCalls() > stop_calls,
           "delivery-unconfirmed revokes authorization and attempts an immediate stop");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "emergency-stop backend fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "emergency-stop backend fixture arms");
    const std::uint64_t stop_calls = scripted->stopCalls();
    auto_rover::EmergencyStop assertion;
    assertion.stamp_ns = 2120000000LL;
    assertion.request_id = "injected-backend-estop";
    assertion.source_id = "vehicle-ros-test";
    assertion.asserted = true;
    const auto stopped = runtime.handleEmergencyStop(
        assertion, 1120000000LL, 2120000000LL);
    expect(stopped.success && !runtime.executionAuthorized() &&
               scripted->stopCalls() > stop_calls,
           "software emergency stop invokes the backend immediate-stop path");
  }
}

void testInjectedBackendConnectionGenerationIsStrictHighWater() {
  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "same-generation reconnect fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "same-generation reconnect fixture arms");

    scripted->setConnectionState(false, 1U);
    runtime.cycle(1150000000LL, 2150000000LL);
    scripted->setConnectionState(true, 1U);
    const auto same_generation =
        runtime.cycle(1200000000LL, 2200000000LL);
    runtime.cycle(1250000000LL, 2250000000LL);
    const auto blocked_state = runtime.currentSafetyState(2260000000LL);
    const auto blocked_arm = runtime.requestArm(
        "operator", blocked_state.state_id, true,
        1260000000LL, 2260000000LL);
    expect(!same_generation.health_clear && !blocked_arm.success &&
               !runtime.executionAuthorized(),
           "disconnect followed by the same generation stays fail closed");
    expect(contains(same_generation.health_diagnostic,
                    "strictly newer"),
           "same-generation reconnect reports the strict high-water requirement");

    scripted->setConnectionState(true, 2U);
    runtime.cycle(1300000000LL, 2300000000LL);
    runtime.cycle(1350000000LL, 2350000000LL);
    runtime.cycle(1400000000LL, 2400000000LL);
    const auto recovered_state = runtime.currentSafetyState(2410000000LL);
    const auto recovered_arm = runtime.requestArm(
        "operator", recovered_state.state_id, true,
        1410000000LL, 2410000000LL);
    expect(recovered_arm.success && runtime.executionAuthorized(),
           "a strictly newer reconnect generation can recover but still requires a new explicit arm");
  }

  {
    auto* scripted = new ScriptedBackend();
    scripted->setConnectionState(true, 5U);
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "generation-rollback fixture initializes at generation five");
    expect(armInjectedRuntime(&runtime).success,
           "generation-rollback fixture arms");

    scripted->setConnectionState(true, 4U);
    const auto rollback = runtime.cycle(1150000000LL, 2150000000LL);
    scripted->setConnectionState(true, 5U);
    const auto retired_replay =
        runtime.cycle(1200000000LL, 2200000000LL);
    expect(!rollback.health_clear && !retired_replay.health_clear &&
               !runtime.executionAuthorized(),
           "generation rollback and replay of the retired high-water generation remain inhibited");
    expect(contains(rollback.health_diagnostic, "rolled back"),
           "generation rollback reports the high-water violation");
    expect(contains(retired_replay.health_diagnostic,
                    "strictly newer"),
           "retired generation replay reports the strict recovery requirement");

    scripted->setConnectionState(true, 6U);
    runtime.cycle(1250000000LL, 2250000000LL);
    runtime.cycle(1300000000LL, 2300000000LL);
    runtime.cycle(1350000000LL, 2350000000LL);
    const auto recovered_state = runtime.currentSafetyState(2360000000LL);
    const auto recovered_arm = runtime.requestArm(
        "operator", recovered_state.state_id, true,
        1360000000LL, 2360000000LL);
    expect(recovered_arm.success && runtime.executionAuthorized(),
           "a generation strictly above the rollback high-water mark permits recovery and explicit re-arm");
  }
}

void testStopDeliveryRemainsObservableWhenEmergencyStopLatches() {
  auto* scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "unconfirmed emergency-stop delivery fixture initializes");
  expect(armInjectedRuntime(&runtime).success,
         "unconfirmed emergency-stop delivery fixture arms");
  scripted->failStopDelivery(true);

  auto_rover::EmergencyStop assertion;
  assertion.stamp_ns = 2120000000LL;
  assertion.request_id = "unconfirmed-stop-estop";
  assertion.source_id = "vehicle-ros-test";
  assertion.asserted = true;
  const auto stopped = runtime.handleEmergencyStop(
      assertion, 1120000000LL, 2120000000LL);
  expect(stopped.success &&
             stopped.state.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched &&
             stopped.stop_delivery.stop_attempted &&
             stopped.stop_delivery.delivery_unconfirmed &&
             !stopped.stop_delivery.delivered &&
             contains(stopped.reason,
                      "physical stop delivery unconfirmed"),
         "E-stop latch success is distinct from an unconfirmed physical stop delivery");

  const auto cycle = runtime.cycle(1150000000LL, 2150000000LL);
  expect(cycle.stop_delivery.stop_attempted &&
             cycle.stop_delivery.delivery_unconfirmed &&
             contains(cycle.health_diagnostic,
                      "physical stop delivery unconfirmed"),
         "the execution cycle preserves and diagnoses its latest unconfirmed stop result");
}

void testInjectedBackendAuthorizationFailureRollsBackArm() {
  auto* scripted = new ScriptedBackend();
  scripted->rejectAuthorization(true);
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "authorization-rejection fixture initializes");
  const auto armed = armInjectedRuntime(&runtime);
  expect(!armed.success && !runtime.executionAuthorized() &&
             contains(armed.reason, "backend"),
         "backend authorization rejection rolls the safety state back to disarmed");
}

enum class ArmExceptionPoint {
  kSetAuthorizationAfterMutation,
  kHealthDiagnostic,
  kHealthBadAlloc,
};

void exerciseArmExceptionRollback(ArmExceptionPoint exception_point,
                                  const std::string& label) {
  auto* scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         label + " fixture initializes");
  recoverToDisarmed(&runtime);
  const auto ready = runtime.currentSafetyState(2110000000LL);
  if (exception_point ==
      ArmExceptionPoint::kSetAuthorizationAfterMutation) {
    scripted->throwAfterAuthorization(true);
  } else {
    scripted->throwFromHealthWhenAuthorized(
        exception_point == ArmExceptionPoint::kHealthBadAlloc);
  }

  const std::uint64_t stop_calls_before = scripted->stopCalls();
  auto_rover_vehicle::VehicleExecutionServiceResult arm_result;
  bool exception_escaped = false;
  try {
    arm_result = runtime.requestArm(
        "operator", ready.state_id, true,
        1110000000LL, 2110000000LL);
  } catch (...) {
    exception_escaped = true;
  }
  scripted->clearThrows();

  expect(!exception_escaped, label + " is contained by requestArm");
  expect(!arm_result.success &&
             arm_result.state.mode != auto_rover::SafetyMode::kArmed &&
             contains(arm_result.reason, "exception") &&
             contains(arm_result.reason, "restart"),
         label + " returns a fail-closed, restart-latched service result");
  expect(scripted->stopCalls() > stop_calls_before &&
             !scripted->authorized() &&
             !runtime.executionAuthorized(),
         label + " revokes manager/backend authorization and invokes stop");

  const std::uint64_t accepted_motion_before =
      scripted->motionAcceptedCalls();
  const auto stopped = runtime.cycle(
      1150000000LL, 2150000000LL);
  expect(!stopped.command.motion_enabled &&
             !stopped.delivery.motion_accepted &&
             scripted->motionAcceptedCalls() == accepted_motion_before &&
             !runtime.executionAuthorized(),
         label + " cannot produce motion on a later cycle");

  const auto inhibited_state =
      runtime.currentSafetyState(2160000000LL);
  const std::uint64_t deliveries_before_rearm =
      scripted->deliveryCalls();
  const auto rearm = runtime.requestArm(
      "operator", inhibited_state.state_id, true,
      1160000000LL, 2160000000LL);
  expect(!rearm.success && !scripted->authorized() &&
             !runtime.executionAuthorized() &&
             scripted->deliveryCalls() == deliveries_before_rearm,
         label + " remains latched against a later explicit arm");
}

void testInjectedBackendArmExceptionsAreAtomic() {
  exerciseArmExceptionRollback(
      ArmExceptionPoint::kSetAuthorizationAfterMutation,
      "setAuthorization post-mutation exception");
  exerciseArmExceptionRollback(
      ArmExceptionPoint::kHealthDiagnostic,
      "backend health diagnostic exception");
  exerciseArmExceptionRollback(
      ArmExceptionPoint::kHealthBadAlloc,
      "backend health diagnostic allocation exception");
}

void testAuthorizedRuntimeBackendExceptionsFailClosed() {
  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "authorized health-exception fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "authorized health-exception fixture arms");
    scripted->throwFromHealthWhenAuthorized(false);
    const std::uint64_t stop_calls_before = scripted->stopCalls();
    auto_rover_vehicle::VehicleExecutionCycleResult failed;
    bool exception_escaped = false;
    try {
      failed = runtime.cycle(1150000000LL, 2150000000LL);
    } catch (...) {
      exception_escaped = true;
    }
    scripted->clearThrows();
    expect(!exception_escaped && !failed.health_clear &&
               failed.safety.mode != auto_rover::SafetyMode::kArmed &&
               contains(failed.health_diagnostic, "exception") &&
               scripted->stopCalls() > stop_calls_before &&
               !scripted->authorized() &&
               !runtime.executionAuthorized(),
           "an authorized-cycle backend exception revokes every authorization layer and stops");
    const std::uint64_t deliveries_before = scripted->deliveryCalls();
    const auto later = runtime.cycle(1200000000LL, 2200000000LL);
    expect(!later.command.motion_enabled &&
               !later.delivery.motion_accepted &&
               scripted->deliveryCalls() == deliveries_before &&
               scripted->motionAcceptedCalls() == 0U,
           "a cycle exception terminally inhibits later backend delivery");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "stop-exception E-stop fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "stop-exception E-stop fixture arms");
    scripted->throwBeforeStopRevocation(true);
    auto_rover::EmergencyStop assertion;
    assertion.stamp_ns = 2120000000LL;
    assertion.request_id = "throwing-stop-estop";
    assertion.source_id = "vehicle-ros-test";
    assertion.asserted = true;
    const auto stopped = runtime.handleEmergencyStop(
        assertion, 1120000000LL, 2120000000LL);
    scripted->clearThrows();
    expect(stopped.success &&
               stopped.state.mode ==
                   auto_rover::SafetyMode::kEmergencyStopLatched &&
               stopped.stop_delivery.stop_attempted &&
               stopped.stop_delivery.delivery_unconfirmed &&
               !scripted->authorized() &&
               !runtime.executionAuthorized(),
           "a throwing immediate-stop path still latches E-stop and independently revokes backend authorization");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "stop-exception shutdown fixture initializes");
    expect(armInjectedRuntime(&runtime).success,
           "stop-exception shutdown fixture arms");
    scripted->throwBeforeStopRevocation(true);
    const auto stopped = runtime.shutdown(1120000000LL);
    scripted->clearThrows();
    expect(stopped.stop_attempted && stopped.delivery_unconfirmed &&
               !scripted->authorized() &&
               !runtime.executionAuthorized(),
           "a throwing shutdown stop is contained and authorization is independently revoked");
  }
}

void testFakeInjectionApiFailsClosedForAnInjectedRealBackend() {
  auto* scripted = new ScriptedBackend();
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
  auto_rover_vehicle::VehicleExecutionCore runtime(
      commonConfig(true), std::move(backend));
  expect(runtime.initialize(1000000000LL, 2000000000LL),
         "non-fake injection fixture initializes");
  const std::uint64_t stop_calls = scripted->stopCalls();
  const auto result = runtime.setFakeFaulted(
      true, 1010000000LL, 2010000000LL);
  expect(!result.success && contains(result.reason, "does not support") &&
             scripted->stopCalls() > stop_calls &&
             !runtime.executionAuthorized(),
         "fake-only injection APIs reject a non-fake backend and fail closed");
}

void testInjectedBackendOrderlyShutdownIsExplicitAndIdempotent() {
  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "orderly-shutdown fixture initializes");
    const std::uint64_t before = scripted->stopCalls();
    const auto first = runtime.shutdown(1010000000LL);
    const std::uint64_t after_first = scripted->stopCalls();
    const auto second = runtime.shutdown(1020000000LL);
    runtime.cycle(1030000000LL, 2030000000LL);
    expect(first.stop_attempted && first.delivered &&
               !first.delivery_unconfirmed &&
               after_first == before + 1U &&
               scripted->stopCalls() == after_first &&
               second.stop_attempted && second.delivered &&
               !runtime.executionAuthorized(),
           "orderly shutdown attempts one bounded stop and repeated calls return the cached result");
  }

  {
    auto* scripted = new ScriptedBackend();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(scripted);
    auto_rover_vehicle::VehicleExecutionCore runtime(
        commonConfig(true), std::move(backend));
    expect(runtime.initialize(1000000000LL, 2000000000LL),
           "disconnected-shutdown fixture initializes");
    scripted->setConnected(false);
    const auto result = runtime.shutdown(1010000000LL);
    expect(result.stop_attempted && result.delivery_unconfirmed &&
               !result.delivered,
           "orderly shutdown reports delivery-unconfirmed when the backend link is disconnected");
  }
}

}  // namespace

int main() {
  testConfigurationDefaultsFailClosed();
  testServicesEstopResetAndReconnect();
  testRequiredInputSourceOrderingFailsClosed();
  testRepeatedRequiredStateCallbacksCannotSustainExecution();
  testEmergencyStopResetRechecksCurrentRequiredInputFreshness();
  testEmergencyStopLatchesBeforeLocalClockValidation();
  testExecutionInputBudgetsFailClosedBeforeUnboundedWork();
  testInjectedBackendDoesNotFabricateOptionalChassisState();
  testInjectedBackendAcceptsReceiptAfterCycleEntry();
  testInjectedBackendTransitionsRequireNewArmAndImmediateStop();
  testInjectedBackendConnectionGenerationIsStrictHighWater();
  testStopDeliveryRemainsObservableWhenEmergencyStopLatches();
  testInjectedBackendAuthorizationFailureRollsBackArm();
  testInjectedBackendArmExceptionsAreAtomic();
  testAuthorizedRuntimeBackendExceptionsFailClosed();
  testFakeInjectionApiFailsClosedForAnInjectedRealBackend();
  testInjectedBackendOrderlyShutdownIsExplicitAndIdempotent();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "vehicle execution core tests passed\n";
  return EXIT_SUCCESS;
}
