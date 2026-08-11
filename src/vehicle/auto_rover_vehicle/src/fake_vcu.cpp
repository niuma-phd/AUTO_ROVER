#include "auto_rover_vehicle/fake_vcu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"

namespace auto_rover_vehicle {
namespace {

constexpr double kNanosecondsToSeconds = 1e-9;
constexpr double kProfileTolerance = 1e-12;
constexpr char kFakeVcuSourceIdPrefix[] = "deterministic_fake_vcu_v1";

std::string sourceId(const FakeVcuConfig& config) {
  return std::string(kFakeVcuSourceIdPrefix) +
         "#process=" + config.process_generation_id;
}

bool commandRequestsMotion(
    const auto_rover::VehicleExecutionCommand& command) {
  return command.motion_enabled && !command.hold &&
         command.stop_reason == auto_rover::StopReason::kNone &&
         command.signed_speed_mps > 0.0;
}

bool sameVehicleProfile(const auto_rover::VehicleProfile& first,
                        const auto_rover::VehicleProfile& second) {
  return first.schema_version == second.schema_version &&
         first.profile_id == second.profile_id &&
         first.kinematic_model == second.kinematic_model &&
         first.direction_capability == second.direction_capability &&
         first.reference_frame == second.reference_frame &&
         first.wheelbase_m == second.wheelbase_m &&
         first.max_forward_speed_mps == second.max_forward_speed_mps &&
         first.max_longitudinal_accel_mps2 ==
             second.max_longitudinal_accel_mps2 &&
         first.min_turning_radius_m == second.min_turning_radius_m &&
         first.reverse_supported == second.reverse_supported;
}

auto_rover::StopReason validationStopReason(
    const auto_rover::ValidationResult& validation) {
  if (validation.reason.find("deadline") != std::string::npos) {
    return auto_rover::StopReason::kBackendWatchdog;
  }
  if (validation.reason.find("profile") != std::string::npos) {
    return auto_rover::StopReason::kLimitViolation;
  }
  return auto_rover::StopReason::kInvalidInput;
}

}  // namespace

auto_rover::ValidationResult validateFakeVcuConfig(
    const FakeVcuConfig& config,
    const auto_rover::VehicleProfile& vehicle_profile) {
  const auto_rover::ValidationResult profile_validation =
      auto_rover::validateNucPhase1Profile(vehicle_profile);
  if (!profile_validation.ok) {
    return profile_validation;
  }
  if (config.process_generation_id.empty()) {
    return auto_rover::ValidationResult::failure(
        "fake VCU process generation identity is missing");
  }
  if (config.command_watchdog_ns <= 0 || config.feedback_period_ns <= 0 ||
      config.fresh_recovery_count == 0U ||
      !auto_rover::isFinite(config.max_accel_mps2) ||
      config.max_accel_mps2 <= 0.0) {
    return auto_rover::ValidationResult::failure(
        "fake VCU watchdog, feedback, recovery, and acceleration must be configured");
  }
  if (config.max_accel_mps2 >
      vehicle_profile.max_longitudinal_accel_mps2 + kProfileTolerance) {
    return auto_rover::ValidationResult::failure(
        "fake VCU acceleration exceeds the vehicle profile");
  }
  return auto_rover::ValidationResult::success();
}

FakeVcu::FakeVcu(FakeVcuConfig config,
                 auto_rover::VehicleProfile vehicle_profile)
    : config_(std::move(config)),
      vehicle_profile_(std::move(vehicle_profile)),
      configuration_validation_(
          validateFakeVcuConfig(config_, vehicle_profile_)),
      profile_validation_(
          auto_rover::validateNucPhase1Profile(vehicle_profile_)) {}

auto_rover::ValidationResult FakeVcu::validate(
    const auto_rover::VehicleProfile& vehicle_profile) const {
  const auto_rover::ValidationResult result =
      validateFakeVcuConfig(config_, vehicle_profile);
  if (!result.ok) {
    return result;
  }
  if (!sameVehicleProfile(vehicle_profile_, vehicle_profile)) {
    return auto_rover::ValidationResult::failure(
        "fake VCU vehicle profile does not match the execution core");
  }
  return auto_rover::ValidationResult::success();
}

bool FakeVcu::initialize(std::int64_t now_monotonic_ns,
                         std::int64_t now_ros_ns) {
  if (now_ros_ns <= 0 || !configuration_validation_.ok ||
      !profile_validation_.ok) {
    setConnected(false, now_monotonic_ns);
    return false;
  }
  setConnected(true, now_monotonic_ns);
  return connected_;
}

BackendFeedback FakeVcu::poll(std::int64_t now_monotonic_ns,
                              std::int64_t now_ros_ns) {
  return step(now_monotonic_ns, now_ros_ns);
}

bool FakeVcu::setAuthorization(bool enabled,
                               std::uint64_t authorization_id,
                               std::int64_t now_monotonic_ns) {
  if (!enabled) {
    setControlEnabled(false, now_monotonic_ns);
    return !control_enabled_;
  }
  if (authorization_id == 0U ||
      authorization_id <= highest_authorization_id_) {
    setControlEnabled(false, now_monotonic_ns);
    return false;
  }
  setControlEnabled(true, now_monotonic_ns);
  if (!control_enabled_) {
    return false;
  }
  highest_authorization_id_ = authorization_id;
  return true;
}

BackendDelivery FakeVcu::deliver(
    const auto_rover::VehicleExecutionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  return receive(command, receipt_monotonic_ns);
}

BackendDelivery FakeVcu::revokeAndStop(
    std::int64_t now_monotonic_ns) {
  setControlEnabled(false, now_monotonic_ns);
  BackendDelivery result;
  result.delivered = connected_;
  result.delivery_unconfirmed = !connected_;
  result.stop_attempted = true;
  result.reason = auto_rover::StopReason::kDisarmed;
  result.diagnostic = connected_
                          ? "fake VCU immediate stop applied"
                          : "fake VCU stop delivery is unconfirmed while disconnected";
  return result;
}

BackendHealth FakeVcu::health() const {
  BackendHealth result;
  result.configuration_valid = configuration_validation_.ok &&
                               profile_validation_.ok;
  result.connected = connected_;
  result.actuation_permitted = result.configuration_valid && connected_ &&
                               !faulted_ && !drop_feedback_;
  result.feedback_fresh = result.configuration_valid && connected_ &&
                          !drop_feedback_;
  result.authorization_active = control_enabled_;
  result.delivery_unconfirmed = !connected_;
  result.connection_generation = connection_generation_;
  if (!result.configuration_valid) {
    result.reason = auto_rover::StopReason::kInvalidInput;
    result.diagnostic = configuration_validation_.ok
                            ? profile_validation_.reason
                            : configuration_validation_.reason;
  } else if (!connected_) {
    result.reason = auto_rover::StopReason::kBackendDisconnected;
    result.diagnostic = "fake VCU backend is disconnected";
  } else if (faulted_) {
    result.reason = auto_rover::StopReason::kInvalidInput;
    result.diagnostic = "fake VCU fault is asserted";
  } else if (drop_feedback_) {
    result.reason = auto_rover::StopReason::kStaleChassisState;
    result.diagnostic = "fake VCU feedback is intentionally dropped";
  } else {
    result.reason = auto_rover::StopReason::kNone;
    result.diagnostic = "fake VCU backend health is clear";
  }
  return result;
}

bool FakeVcu::validEventTime(std::int64_t now_monotonic_ns) const {
  return now_monotonic_ns > 0 &&
         (last_event_monotonic_ns_ == 0 ||
          now_monotonic_ns >= last_event_monotonic_ns_);
}

void FakeVcu::integrateDynamicsTo(std::int64_t end_monotonic_ns) {
  if (last_dynamics_monotonic_ns_ <= 0) {
    last_dynamics_monotonic_ns_ = end_monotonic_ns;
    return;
  }
  if (end_monotonic_ns <= last_dynamics_monotonic_ns_) {
    return;
  }
  const double elapsed_seconds =
      static_cast<double>(end_monotonic_ns - last_dynamics_monotonic_ns_) *
      kNanosecondsToSeconds;
  const double maximum_change = config_.max_accel_mps2 * elapsed_seconds;
  const double difference = target_speed_mps_ - measured_speed_mps_;
  if (std::abs(difference) <= maximum_change) {
    measured_speed_mps_ = target_speed_mps_;
  } else {
    measured_speed_mps_ += std::copysign(maximum_change, difference);
  }
  last_dynamics_monotonic_ns_ = end_monotonic_ns;
}

std::int64_t FakeVcu::outputExpiryMonotonicNs() const {
  if (!control_watchdog_active_) {
    return 0;
  }
  std::int64_t receipt_expiry = std::numeric_limits<std::int64_t>::max();
  if (control_watchdog_refresh_ns_ <=
      std::numeric_limits<std::int64_t>::max() -
          config_.command_watchdog_ns) {
    receipt_expiry =
        control_watchdog_refresh_ns_ + config_.command_watchdog_ns;
  }
  if (!latest_command_valid_) {
    return receipt_expiry;
  }
  return std::min(latest_command_.deadline_monotonic_ns,
                  receipt_expiry);
}

void FakeVcu::clearMotionState(bool clear_control_enable) {
  latest_command_valid_ = false;
  latest_command_ = auto_rover::VehicleExecutionCommand{};
  consecutive_fresh_commands_ = 0U;
  target_speed_mps_ = 0.0;
  target_curvature_inv_m_ = 0.0;
  if (clear_control_enable) {
    control_enabled_ = false;
    control_watchdog_active_ = false;
    control_watchdog_refresh_ns_ = 0;
  }
}

void FakeVcu::expireOutputWatchdog() {
  clearMotionState(true);
  watchdog_expired_latched_ = true;
}

void FakeVcu::advanceDynamics(std::int64_t now_monotonic_ns) {
  if (last_dynamics_monotonic_ns_ <= 0) {
    last_dynamics_monotonic_ns_ = now_monotonic_ns;
  }
  const std::int64_t expiry_ns = outputExpiryMonotonicNs();
  if (expiry_ns > 0 && now_monotonic_ns > expiry_ns) {
    integrateDynamicsTo(std::max(expiry_ns, last_dynamics_monotonic_ns_));
    expireOutputWatchdog();
  }
  integrateDynamicsTo(now_monotonic_ns);
}

void FakeVcu::setConnected(bool connected,
                           std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      !validEventTime(now_monotonic_ns)) {
    connected_ = false;
    clearMotionState(true);
    return;
  }

  advanceDynamics(now_monotonic_ns);
  const bool connection_changed = connected != connected_;
  if (connection_changed && connected &&
      connection_generation_ !=
          std::numeric_limits<std::uint64_t>::max()) {
    ++connection_generation_;
  }
  connected_ = connected;
  last_event_monotonic_ns_ = now_monotonic_ns;
  if (connection_changed || !connected) {
    clearMotionState(true);
    watchdog_expired_latched_ = false;
  }
  if (connection_changed && connected) {
    last_feedback_monotonic_ns_ = now_monotonic_ns;
  }
}

void FakeVcu::setControlEnabled(bool enabled,
                                std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      !connected_ || faulted_ || !validEventTime(now_monotonic_ns)) {
    clearMotionState(true);
    return;
  }

  advanceDynamics(now_monotonic_ns);
  last_event_monotonic_ns_ = now_monotonic_ns;
  clearMotionState(false);
  control_enabled_ = enabled;
  if (enabled) {
    // Authorization changes a substantiated ChassisState field. Force the
    // next poll to publish that transition even when the regular feedback
    // period has not elapsed; otherwise a cached control_enabled=false sample
    // can revoke a just-accepted re-arm before the fake backend reports its
    // new state.
    last_feedback_monotonic_ns_ = 0;
    watchdog_expired_latched_ = false;
    control_watchdog_active_ = true;
    control_watchdog_refresh_ns_ = now_monotonic_ns;
  } else {
    clearMotionState(true);
  }
}

void FakeVcu::setFaulted(bool faulted,
                         std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      !validEventTime(now_monotonic_ns)) {
    faulted_ = true;
    clearMotionState(true);
    return;
  }
  advanceDynamics(now_monotonic_ns);
  last_event_monotonic_ns_ = now_monotonic_ns;
  faulted_ = faulted;
  if (faulted) {
    clearMotionState(true);
  }
}

void FakeVcu::setDropFeedback(bool drop_feedback) {
  drop_feedback_ = drop_feedback;
}

FakeVcuReceiveResult FakeVcu::rejectDelivered(
    auto_rover::StopReason reason, const std::string& diagnostic,
    bool clear_control_enable) {
  clearMotionState(clear_control_enable);
  FakeVcuReceiveResult result;
  result.delivered = true;
  result.motion_accepted = false;
  result.delivery_unconfirmed = false;
  result.reason = reason;
  result.diagnostic = diagnostic;
  return result;
}

FakeVcuReceiveResult FakeVcu::receive(
    const auto_rover::VehicleExecutionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok) {
    clearMotionState(true);
    FakeVcuReceiveResult result;
    result.delivery_unconfirmed = true;
    result.reason = auto_rover::StopReason::kInvalidInput;
    result.diagnostic = configuration_validation_.ok
                            ? profile_validation_.reason
                            : configuration_validation_.reason;
    return result;
  }
  if (!connected_) {
    FakeVcuReceiveResult result;
    result.delivery_unconfirmed = true;
    result.reason = auto_rover::StopReason::kBackendDisconnected;
    result.diagnostic = "fake VCU is disconnected";
    return result;
  }
  if (!validEventTime(receipt_monotonic_ns)) {
    return rejectDelivered(auto_rover::StopReason::kInvalidInput,
                           "fake VCU receipt clock is invalid", true);
  }

  advanceDynamics(receipt_monotonic_ns);
  last_event_monotonic_ns_ = receipt_monotonic_ns;
  if (command.sequence_id == 0U ||
      command.sequence_id <= highest_sequence_) {
    return rejectDelivered(auto_rover::StopReason::kInvalidInput,
                           "execution sequence is not strictly increasing",
                           false);
  }
  highest_sequence_ = command.sequence_id;

  const auto_rover::ValidationResult command_validation =
      auto_rover::validateExecutionCommand(command, vehicle_profile_,
                                            receipt_monotonic_ns);
  if (!command_validation.ok) {
    return rejectDelivered(validationStopReason(command_validation),
                           command_validation.reason, false);
  }
  if (receipt_monotonic_ns - command.created_monotonic_ns >
      config_.command_watchdog_ns) {
    return rejectDelivered(auto_rover::StopReason::kBackendWatchdog,
                           "execution command exceeded fake VCU age limit",
                           false);
  }

  if (!commandRequestsMotion(command)) {
    if (control_enabled_) {
      control_watchdog_active_ = true;
      control_watchdog_refresh_ns_ = receipt_monotonic_ns;
    }
    const auto_rover::StopReason reason =
        command.stop_reason == auto_rover::StopReason::kNone
            ? auto_rover::StopReason::kNone
            : command.stop_reason;
    return rejectDelivered(reason, "explicit zero and hold accepted", false);
  }
  if (faulted_) {
    return rejectDelivered(auto_rover::StopReason::kInvalidInput,
                           "fake VCU fault is asserted", true);
  }
  if (!control_enabled_) {
    return rejectDelivered(auto_rover::StopReason::kDisarmed,
                           "fake VCU control is not enabled", false);
  }

  latest_command_ = command;
  latest_command_valid_ = true;
  control_watchdog_active_ = true;
  control_watchdog_refresh_ns_ = receipt_monotonic_ns;
  watchdog_expired_latched_ = false;
  if (consecutive_fresh_commands_ < config_.fresh_recovery_count) {
    ++consecutive_fresh_commands_;
  }

  FakeVcuReceiveResult result;
  result.delivered = true;
  if (consecutive_fresh_commands_ < config_.fresh_recovery_count) {
    target_speed_mps_ = 0.0;
    target_curvature_inv_m_ = 0.0;
    result.reason = auto_rover::StopReason::kRecoveryPending;
    result.diagnostic = "fake VCU requires consecutive fresh commands";
    return result;
  }

  target_speed_mps_ = command.signed_speed_mps;
  target_curvature_inv_m_ = command.curvature_inv_m;
  result.motion_accepted = true;
  result.reason = auto_rover::StopReason::kNone;
  result.diagnostic = "fake VCU motion target accepted";
  return result;
}

FakeVcuStepResult FakeVcu::step(std::int64_t now_monotonic_ns,
                               std::int64_t now_ros_ns) {
  FakeVcuStepResult result;
  result.operation_completed_monotonic_ns = now_monotonic_ns;
  result.operation_completed_ros_ns = now_ros_ns;
  result.health = health();
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      now_ros_ns <= 0 || !validEventTime(now_monotonic_ns)) {
    clearMotionState(true);
    return result;
  }

  advanceDynamics(now_monotonic_ns);
  last_event_monotonic_ns_ = now_monotonic_ns;
  result.watchdog_expired = watchdog_expired_latched_;
  if (!connected_) {
    return result;
  }
  if (last_feedback_monotonic_ns_ > 0 &&
      now_monotonic_ns - last_feedback_monotonic_ns_ <
          config_.feedback_period_ns) {
    return result;
  }
  last_feedback_monotonic_ns_ = now_monotonic_ns;
  if (drop_feedback_ ||
      feedback_state_id_ == std::numeric_limits<std::uint64_t>::max()) {
    return result;
  }

  ++feedback_state_id_;
  result.available = true;
  result.receipt_monotonic_ns = now_monotonic_ns;
  result.state.stamp_ns = now_ros_ns;
  result.state.frame_id = vehicle_profile_.reference_frame;
  result.state.state_id = feedback_state_id_;
  result.state.time_source = auto_rover::TimeSource::kSampleTime;
  result.state.measured_speed_mps = measured_speed_mps_;
  result.state.yaw_rate_radps =
      measured_speed_mps_ * target_curvature_inv_m_;
  result.state.gear_state = auto_rover::GearState::kUnknown;
  result.state.control_enabled = control_enabled_;
  result.state.fault_state =
      faulted_ ? auto_rover::FaultState::kFault
               : auto_rover::FaultState::kOk;
  result.state.valid_mask =
      auto_rover::ChassisState::kMeasuredSpeedValid |
      auto_rover::ChassisState::kYawRateValid |
      auto_rover::ChassisState::kControlEnabledValid |
      auto_rover::ChassisState::kFaultValid;
  result.state.source_id = sourceId(config_);
  result.state.valid = true;
  result.health = health();
  return result;
}

bool FakeVcu::connected() const { return connected_; }

bool FakeVcu::controlEnabled() const { return control_enabled_; }

bool FakeVcu::faulted() const { return faulted_; }

bool FakeVcu::dropFeedback() const { return drop_feedback_; }

std::uint32_t FakeVcu::consecutiveFreshCommands() const {
  return consecutive_fresh_commands_;
}

std::uint64_t FakeVcu::highestObservedSequence() const {
  return highest_sequence_;
}

double FakeVcu::measuredSpeedMps() const { return measured_speed_mps_; }

}  // namespace auto_rover_vehicle
