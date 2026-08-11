#include "auto_rover_vehicle/vehicle_motion_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "auto_rover_core/validation.hpp"
#include "auto_rover_safety/command_guard.hpp"

namespace auto_rover_vehicle {
namespace {

constexpr double kAccelerationToleranceMps = 1e-12;
constexpr double kNanosecondsToSeconds = 1e-9;

}  // namespace

auto_rover::ValidationResult validateVehicleMotionManagerConfig(
    const VehicleMotionManagerConfig& config,
    const auto_rover::VehicleProfile& vehicle_profile) {
  const auto_rover::ValidationResult profile_validation =
      auto_rover::validateNucPhase1Profile(vehicle_profile);
  if (!profile_validation.ok) {
    return profile_validation;
  }
  if (config.command_valid_for_ns <= 0) {
    return auto_rover::ValidationResult::failure(
        "vehicle command validity horizon must be positive");
  }
  return auto_rover::ValidationResult::success();
}

VehicleMotionManager::VehicleMotionManager(
    VehicleMotionManagerConfig config,
    auto_rover::VehicleProfile vehicle_profile)
    : config_(std::move(config)),
      vehicle_profile_(std::move(vehicle_profile)),
      configuration_validation_(
          validateVehicleMotionManagerConfig(config_, vehicle_profile_)),
      profile_validation_(
          auto_rover::validateNucPhase1Profile(vehicle_profile_)) {}

void VehicleMotionManager::clearAuthorization() {
  operator_armed_ = false;
  commanded_speed_known_ = false;
  last_commanded_speed_mps_ = 0.0;
  last_commanded_speed_ns_ = 0;
}

void VehicleMotionManager::setBackendConnected(
    bool connected, std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      now_monotonic_ns <= 0 ||
      (last_event_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_event_monotonic_ns_)) {
    backend_connected_ = false;
    clearAuthorization();
    return;
  }

  const bool connection_changed = connected != backend_connected_;
  backend_connected_ = connected;
  last_event_monotonic_ns_ = now_monotonic_ns;
  if (connection_changed || !connected) {
    clearAuthorization();
  }
}

bool VehicleMotionManager::acknowledgeOperatorArm(
    std::uint64_t authorization_id, std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok ||
      !backend_connected_ || authorization_id == 0U ||
      authorization_id <= highest_authorization_id_ ||
      now_monotonic_ns <= 0 ||
      (last_event_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_event_monotonic_ns_)) {
    return false;
  }

  highest_authorization_id_ = authorization_id;
  operator_armed_ = true;
  last_event_monotonic_ns_ = now_monotonic_ns;
  recordCommandedSpeed(0.0, now_monotonic_ns);
  return true;
}

void VehicleMotionManager::revokeAuthorization(
    std::int64_t now_monotonic_ns) {
  clearAuthorization();
  if (now_monotonic_ns > 0 &&
      (last_event_monotonic_ns_ == 0 ||
       now_monotonic_ns >= last_event_monotonic_ns_)) {
    last_event_monotonic_ns_ = now_monotonic_ns;
  }
}

auto_rover::VehicleExecutionCommand
VehicleMotionManager::makeBaseCommand(
    std::int64_t now_monotonic_ns) {
  auto_rover::VehicleExecutionCommand command;
  if (last_sequence_id_ != std::numeric_limits<std::uint64_t>::max()) {
    ++last_sequence_id_;
    command.sequence_id = last_sequence_id_;
  }
  command.created_monotonic_ns = now_monotonic_ns;
  if (now_monotonic_ns > 0 && config_.command_valid_for_ns > 0 &&
      now_monotonic_ns <= std::numeric_limits<std::int64_t>::max() -
                              config_.command_valid_for_ns) {
    command.deadline_monotonic_ns =
        now_monotonic_ns + config_.command_valid_for_ns;
  }
  command.signed_speed_mps = 0.0;
  command.curvature_inv_m = 0.0;
  command.motion_enabled = false;
  command.hold = true;
  command.stop_reason = auto_rover::StopReason::kInvalidInput;
  return command;
}

void VehicleMotionManager::recordCommandedSpeed(
    double speed_mps, std::int64_t now_monotonic_ns) {
  commanded_speed_known_ = true;
  last_commanded_speed_mps_ = speed_mps;
  last_commanded_speed_ns_ = now_monotonic_ns;
}

auto_rover::VehicleExecutionCommand
VehicleMotionManager::makeStopCommand(
    std::int64_t now_monotonic_ns, auto_rover::StopReason reason,
    bool record_delivered_zero) {
  auto_rover::VehicleExecutionCommand command =
      makeBaseCommand(now_monotonic_ns);
  command.stop_reason =
      reason == auto_rover::StopReason::kNone
          ? auto_rover::StopReason::kInvalidInput
          : reason;
  if (record_delivered_zero && now_monotonic_ns > 0) {
    recordCommandedSpeed(0.0, now_monotonic_ns);
  }
  return command;
}

auto_rover::VehicleExecutionCommand VehicleMotionManager::makeCommand(
    const auto_rover_safety::GuardResult& guarded,
    std::int64_t now_monotonic_ns) {
  if (!configuration_validation_.ok || !profile_validation_.ok) {
    clearAuthorization();
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kInvalidInput, false);
  }
  if (now_monotonic_ns <= 0 ||
      (last_event_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_event_monotonic_ns_) ||
      now_monotonic_ns > std::numeric_limits<std::int64_t>::max() -
                             config_.command_valid_for_ns) {
    clearAuthorization();
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kInvalidInput, false);
  }
  last_event_monotonic_ns_ = now_monotonic_ns;

  if (!backend_connected_) {
    clearAuthorization();
    return makeStopCommand(
        now_monotonic_ns, auto_rover::StopReason::kBackendDisconnected,
        false);
  }
  if (!operator_armed_) {
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kDisarmed, true);
  }
  if (!guarded.execution_allowed) {
    return makeStopCommand(now_monotonic_ns, guarded.reason, true);
  }
  if (guarded.reason != auto_rover::StopReason::kNone) {
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kInvalidInput, true);
  }

  const auto_rover::ValidationResult reference_validation =
      auto_rover::validateMotionReference(guarded.reference,
                                          vehicle_profile_);
  if (!reference_validation.ok) {
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kInvalidInput, true);
  }

  if (!commanded_speed_known_ || last_commanded_speed_ns_ <= 0 ||
      now_monotonic_ns < last_commanded_speed_ns_) {
    clearAuthorization();
    return makeStopCommand(now_monotonic_ns,
                           auto_rover::StopReason::kInvalidInput, true);
  }
  const std::int64_t elapsed_ns =
      now_monotonic_ns - last_commanded_speed_ns_;
  const double allowed_speed_change =
      vehicle_profile_.max_longitudinal_accel_mps2 *
      static_cast<double>(elapsed_ns) * kNanosecondsToSeconds;
  const double minimum_speed =
      std::max(0.0, last_commanded_speed_mps_ - allowed_speed_change);
  const double maximum_speed =
      std::min(vehicle_profile_.max_forward_speed_mps,
               last_commanded_speed_mps_ + allowed_speed_change);
  const double limited_speed =
      std::max(minimum_speed,
               std::min(maximum_speed,
                        guarded.reference.target_speed_mps));

  auto_rover::VehicleExecutionCommand command =
      makeBaseCommand(now_monotonic_ns);
  if (command.sequence_id == 0U || command.deadline_monotonic_ns <=
                                       command.created_monotonic_ns) {
    clearAuthorization();
    command.stop_reason = auto_rover::StopReason::kInvalidInput;
    return command;
  }
  command.signed_speed_mps = limited_speed;
  command.curvature_inv_m = limited_speed <= kAccelerationToleranceMps
                                ? 0.0
                                : guarded.reference.target_curvature_inv_m;
  command.motion_enabled = true;
  command.hold = command.signed_speed_mps == 0.0;
  command.stop_reason = auto_rover::StopReason::kNone;
  recordCommandedSpeed(command.signed_speed_mps, now_monotonic_ns);
  return command;
}

bool VehicleMotionManager::backendConnected() const {
  return backend_connected_;
}

bool VehicleMotionManager::operatorArmed() const {
  return operator_armed_;
}

std::uint64_t VehicleMotionManager::lastSequenceId() const {
  return last_sequence_id_;
}

}  // namespace auto_rover_vehicle
