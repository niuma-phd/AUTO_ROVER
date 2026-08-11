#include "auto_rover_vcu_wheeltec_serial/vehicle_backend.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace auto_rover {
namespace wheeltec_serial {

WheeltecVehicleBackend::WheeltecVehicleBackend(
    const WheeltecVehicleBackendConfig& config, ByteTransport* transport,
    const WheeltecVehicleBackendOperations& operations)
    : config_(config), operations_(operations), transport_(transport),
      runtime_(config.runtime, operations.runtime) {}

auto_rover::ValidationResult WheeltecVehicleBackend::validate(
    const auto_rover::VehicleProfile& vehicle_profile) const {
  if (!runtimeConfigIsStructurallyValid(config_.runtime)) {
    return auto_rover::ValidationResult::failure(
        "wheeltec runtime configuration is invalid");
  }
  if (config_.chassis_frame_id.empty() || config_.source_id.empty()) {
    return auto_rover::ValidationResult::failure(
        "wheeltec feedback frame/source identifiers are required");
  }
  if (vehicle_profile.schema_version == 0U ||
      vehicle_profile.profile_id.empty() ||
      vehicle_profile.kinematic_model !=
          auto_rover::KinematicModel::kAckermannBicycle ||
      vehicle_profile.direction_capability !=
          auto_rover::DirectionCapability::kSignedSpeedDirection ||
      vehicle_profile.reference_frame != config_.chassis_frame_id ||
      !std::isfinite(vehicle_profile.wheelbase_m) ||
      vehicle_profile.wheelbase_m <= 0.0 ||
      !std::isfinite(vehicle_profile.max_forward_speed_mps) ||
      vehicle_profile.max_forward_speed_mps <= 0.0 ||
      vehicle_profile.max_forward_speed_mps >
          config_.runtime.adapter.codec_limits.max_forward_speed_mps ||
      !std::isfinite(vehicle_profile.max_longitudinal_accel_mps2) ||
      vehicle_profile.max_longitudinal_accel_mps2 <= 0.0 ||
      !std::isfinite(vehicle_profile.min_turning_radius_m) ||
      vehicle_profile.min_turning_radius_m < 0.95 ||
      vehicle_profile.reverse_supported) {
    return auto_rover::ValidationResult::failure(
        "vehicle profile is incompatible with the forward-only Wheeltec "
        "Ackermann contract");
  }
  return auto_rover::ValidationResult::success();
}

bool WheeltecVehicleBackend::initialize(std::int64_t now_monotonic_ns,
                                        std::int64_t now_ros_ns) {
  if (!runtime_.configurationValid() || now_monotonic_ns <= 0 ||
      now_ros_ns <= 0) {
    return false;
  }
  last_poll_monotonic_ns_ = now_monotonic_ns;
  last_poll_ros_ns_ = now_ros_ns;
  if (!config_.runtime.runtime_enabled) {
    initialized_ = runtime_.attachTransport(nullptr, now_monotonic_ns);
    return initialized_;
  }
  if (!runtimeActuationGatesAreSatisfied(config_.runtime) ||
      !operations_.ros_now_ns || transport_ == nullptr ||
      !runtime_.attachTransport(transport_, now_monotonic_ns)) {
    return false;
  }
  const RuntimeStepResult startup = runtime_.step(now_monotonic_ns);
  initialized_ = startup.phase == RuntimePhase::kDrainBacklog ||
                 startup.phase == RuntimePhase::kStartupZero;
  return initialized_;
}

auto_rover_vehicle::BackendFeedback WheeltecVehicleBackend::poll(
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) {
  auto_rover_vehicle::BackendFeedback result;
  result.operation_completed_monotonic_ns = now_monotonic_ns;
  result.operation_completed_ros_ns = now_ros_ns;
  if (!initialized_) {
    result.health = mapHealth(now_monotonic_ns);
    result.health.configuration_valid = false;
    result.health.reason = auto_rover::StopReason::kInvalidInput;
    result.health.diagnostic = "backend_not_initialized";
    return result;
  }
  const RuntimeStepResult step = runtime_.step(now_monotonic_ns);
  const std::int64_t effective_monotonic_ns = std::max(
      now_monotonic_ns, step.step_completed_monotonic_ns);
  const bool completion_clock_invalid =
      step.step_completed_monotonic_ns <= 0 && step.delivery_unconfirmed &&
      step.phase == RuntimePhase::kTerminalFault;
  result.operation_completed_monotonic_ns =
      completion_clock_invalid ? 0 : effective_monotonic_ns;
  const std::int64_t effective_ros_ns = rosNow(now_ros_ns);
  const bool ros_completion_invalid =
      effective_ros_ns <= 0 || effective_ros_ns < now_ros_ns ||
      (last_poll_ros_ns_ > 0 && effective_ros_ns < last_poll_ros_ns_);
  result.operation_completed_ros_ns =
      ros_completion_invalid ? 0 : effective_ros_ns;
  if (ros_completion_invalid) {
    // Preserve the poll-entry caller stamp.  The runtime samples a separate
    // fresh clock for the zero write itself; advancing the caller high-water
    // to that completion would make the core's same-cycle stop look like a
    // caller-clock rollback.
    const RuntimeDelivery stopped = runtime_.revokeAndStop(
        now_monotonic_ns);
    if (stopped.operation_completed_monotonic_ns >
        result.operation_completed_monotonic_ns) {
      result.operation_completed_monotonic_ns =
          stopped.operation_completed_monotonic_ns;
    }
    last_poll_monotonic_ns_ = result.operation_completed_monotonic_ns;
    result.health = mapHealth(last_poll_monotonic_ns_);
    result.health.reason = auto_rover::StopReason::kInvalidInput;
    result.health.diagnostic = "backend_ros_completion_clock_invalid";
    return result;
  }
  last_poll_monotonic_ns_ = effective_monotonic_ns;
  last_poll_ros_ns_ = effective_ros_ns;
  result.health = mapHealth(effective_monotonic_ns);
  result.watchdog_expired = step.feedback_watchdog_expired ||
                            step.step_overrun;
  if (!step.feedback_available || effective_ros_ns <= 0) {
    return result;
  }

  result.available = true;
  result.receipt_monotonic_ns = step.feedback.receipt_monotonic_ns;
  result.state.stamp_ns = effective_ros_ns;
  result.state.frame_id = config_.chassis_frame_id;
  result.state.state_id = ++feedback_state_id_;
  result.state.time_source = auto_rover::TimeSource::kReceiptTime;
  result.state.measured_speed_mps = step.feedback.signed_speed_mps;
  result.state.yaw_rate_radps =
      step.feedback.wheel_derived_yaw_rate_radps;
  result.state.supply_voltage_v = step.feedback.supply_voltage_v;
  result.state.gear_state = auto_rover::GearState::kUnknown;
  result.state.control_enabled = false;
  result.state.fault_state = auto_rover::FaultState::kUnknown;
  result.state.valid_mask =
      auto_rover::ChassisState::kMeasuredSpeedValid |
      auto_rover::ChassisState::kYawRateValid |
      auto_rover::ChassisState::kVoltageValid;
  result.state.source_id = config_.source_id;
  result.state.valid = true;
  return result;
}

bool WheeltecVehicleBackend::setAuthorization(
    bool enabled, std::uint64_t authorization_id,
    std::int64_t now_monotonic_ns) {
  return initialized_ && runtime_.setAuthorization(
                             enabled, authorization_id, now_monotonic_ns);
}

auto_rover_vehicle::BackendDelivery WheeltecVehicleBackend::deliver(
    const auto_rover::VehicleExecutionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  if (!initialized_) {
    auto_rover_vehicle::BackendDelivery result;
    result.reason = auto_rover::StopReason::kInvalidInput;
    result.diagnostic = "backend_not_initialized";
    return result;
  }
  return mapDelivery(runtime_.deliver(command, receipt_monotonic_ns));
}

auto_rover_vehicle::BackendDelivery WheeltecVehicleBackend::revokeAndStop(
    std::int64_t now_monotonic_ns) {
  if (!initialized_) {
    auto_rover_vehicle::BackendDelivery result;
    result.reason = auto_rover::StopReason::kInvalidInput;
    result.diagnostic = "backend_not_initialized";
    return result;
  }
  return mapDelivery(runtime_.revokeAndStop(now_monotonic_ns));
}

auto_rover_vehicle::BackendHealth WheeltecVehicleBackend::health() const {
  return mapHealth(last_poll_monotonic_ns_);
}

bool WheeltecVehicleBackend::setTransportForInitialization(
    ByteTransport* transport) noexcept {
  if (initialized_ || !config_.runtime.runtime_enabled ||
      transport_ != nullptr || transport == nullptr) {
    return false;
  }
  transport_ = transport;
  return true;
}

bool WheeltecVehicleBackend::attachTransport(
    ByteTransport* transport, std::int64_t now_monotonic_ns) {
  if (!initialized_ || !config_.runtime.runtime_enabled ||
      transport == nullptr) {
    return false;
  }
  if (!runtime_.attachTransport(transport, now_monotonic_ns)) {
    return false;
  }
  transport_ = transport;
  return true;
}

auto_rover_vehicle::BackendHealth WheeltecVehicleBackend::mapHealth(
    std::int64_t now_monotonic_ns) const {
  const RuntimeHealth runtime_health = runtime_.health(now_monotonic_ns);
  auto_rover_vehicle::BackendHealth result;
  result.configuration_valid = runtime_health.configuration_valid;
  result.connected = runtime_health.connected;
  result.actuation_permitted = runtime_health.actuation_permitted;
  result.feedback_fresh = runtime_health.feedback_fresh;
  result.authorization_active = runtime_health.authorization_active;
  result.delivery_unconfirmed = runtime_health.delivery_unconfirmed;
  result.connection_generation = runtime_health.connection_generation;
  result.reason = runtimeReason(runtime_health);
  result.diagnostic = runtime_health.diagnostic;
  return result;
}

auto_rover_vehicle::BackendDelivery WheeltecVehicleBackend::mapDelivery(
    const RuntimeDelivery& delivery) const {
  auto_rover_vehicle::BackendDelivery result;
  result.delivered = delivery.delivered;
  result.motion_accepted = delivery.motion_accepted;
  result.delivery_unconfirmed = delivery.delivery_unconfirmed;
  result.stop_attempted = delivery.stop_attempted;
  result.controller_ack_available = false;
  result.controller_acknowledged = false;
  result.diagnostic = runtime_.health(last_poll_monotonic_ns_).diagnostic;
  if (delivery.command_watchdog_expired) {
    result.reason = auto_rover::StopReason::kBackendWatchdog;
    return result;
  }
  if (delivery.delivered) {
    result.reason = auto_rover::StopReason::kNone;
    return result;
  }
  switch (delivery.submission.status) {
    case SubmissionStatus::kDisconnected:
      result.reason = auto_rover::StopReason::kBackendDisconnected;
      break;
    case SubmissionStatus::kNotAuthorized:
      result.reason = auto_rover::StopReason::kDisarmed;
      break;
    case SubmissionStatus::kRecoveryPending:
      result.reason = auto_rover::StopReason::kRecoveryPending;
      break;
    case SubmissionStatus::kStale:
      result.reason = auto_rover::StopReason::kBackendWatchdog;
      break;
    case SubmissionStatus::kAccepted:
      result.reason = auto_rover::StopReason::kRecoveryPending;
      break;
    default:
      result.reason = auto_rover::StopReason::kInvalidInput;
      break;
  }
  return result;
}

auto_rover::StopReason WheeltecVehicleBackend::runtimeReason(
    const RuntimeHealth& health) const {
  if (!health.configuration_valid) {
    return auto_rover::StopReason::kInvalidInput;
  }
  if (!config_.runtime.runtime_enabled) {
    return auto_rover::StopReason::kActuationDisabled;
  }
  if (!health.connected) {
    return auto_rover::StopReason::kBackendDisconnected;
  }
  if (health.step_overrun || health.delivery_unconfirmed) {
    return auto_rover::StopReason::kBackendWatchdog;
  }
  if (health.last_feedback_receipt_monotonic_ns > 0 &&
      !health.feedback_fresh) {
    return auto_rover::StopReason::kStaleChassisState;
  }
  if (!health.actuation_permitted) {
    return auto_rover::StopReason::kInvalidInput;
  }
  if (!health.authorization_active) {
    return auto_rover::StopReason::kDisarmed;
  }
  return auto_rover::StopReason::kNone;
}

std::int64_t WheeltecVehicleBackend::rosNow(
    std::int64_t fallback_ros_ns) const {
  if (!operations_.ros_now_ns) {
    return fallback_ros_ns;
  }
  try {
    const std::int64_t observed = operations_.ros_now_ns();
    return observed;
  } catch (...) {
    return 0;
  }
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
