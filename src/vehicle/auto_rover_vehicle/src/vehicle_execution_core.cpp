#include "auto_rover_vehicle/vehicle_execution_core.hpp"

#include <algorithm>
#include <utility>

#include "auto_rover_core/validation.hpp"

namespace auto_rover_vehicle {
namespace {

auto_rover::ValidationResult requireValidation(
    const auto_rover::ValidationResult& result) {
  return result;
}

bool hasValidBit(std::uint32_t mask, std::uint32_t bit) {
  return (mask & bit) != 0U;
}

bool executionStringIsBounded(const std::string& value) {
  return value.size() <= kMaximumExecutionIdentifierBytes;
}

auto_rover::ValidationResult validateExecutionEgoEnvelope(
    const auto_rover::EgoState& value) {
  if (!executionStringIsBounded(value.frame_id) ||
      !executionStringIsBounded(value.reference_frame) ||
      !executionStringIsBounded(value.source_id)) {
    return auto_rover::ValidationResult::failure(
        "ego identity exceeds the execution input byte limit");
  }
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult validateExecutionTrajectoryEnvelope(
    const auto_rover::Trajectory& value) {
  if (value.points.size() > kMaximumExecutionTrajectoryPoints) {
    return auto_rover::ValidationResult::failure(
        "trajectory exceeds the execution point-count limit");
  }
  if (!executionStringIsBounded(value.frame_id) ||
      !executionStringIsBounded(value.trajectory_id) ||
      !executionStringIsBounded(value.route_id) ||
      !executionStringIsBounded(value.vehicle_profile_id)) {
    return auto_rover::ValidationResult::failure(
        "trajectory identity exceeds the execution input byte limit");
  }
  return auto_rover::ValidationResult::success();
}

bool executionMotionEnvelopeIsBounded(
    const auto_rover::MotionReference& value) {
  return executionStringIsBounded(value.frame_id) &&
         executionStringIsBounded(value.producer_generation_id) &&
         executionStringIsBounded(value.trajectory_id);
}

bool executionChassisEnvelopeIsBounded(
    const auto_rover::ChassisState& value) {
  return executionStringIsBounded(value.frame_id) &&
         executionStringIsBounded(value.source_id);
}

BackendDelivery normalizeStopDelivery(BackendDelivery delivery) {
  if (!delivery.stop_attempted || !delivery.delivered) {
    delivery.delivery_unconfirmed = true;
  }
  if (delivery.delivery_unconfirmed && delivery.diagnostic.empty()) {
    delivery.diagnostic = "vehicle backend stop delivery is unconfirmed";
  }
  return delivery;
}

void appendUnconfirmedStopDiagnostic(
    const BackendDelivery& delivery, std::string* diagnostic) {
  if (!delivery.delivery_unconfirmed || diagnostic == nullptr) {
    return;
  }
  constexpr char kSummary[] = "physical stop delivery unconfirmed";
  if (diagnostic->find(kSummary) != std::string::npos) {
    return;
  }
  if (!diagnostic->empty()) {
    diagnostic->append("; ");
  }
  diagnostic->append(kSummary);
  if (!delivery.diagnostic.empty()) {
    diagnostic->append(": ");
    diagnostic->append(delivery.diagnostic);
  }
}

}  // namespace

auto_rover::ValidationResult validateVehicleExecutionCommonConfig(
    const VehicleExecutionCommonConfig& config) {
  auto result = requireValidation(
      auto_rover::validateNucPhase1Profile(config.vehicle_profile));
  if (!result.ok) {
    return result;
  }
  result = requireValidation(
      auto_rover_safety::validateSafetySupervisorConfig(
          config.safety_supervisor));
  if (!result.ok) {
    return result;
  }
  if (config.safety_supervisor.reset_service_enabled &&
      config.authorized_reset_operator_id.empty()) {
    return auto_rover::ValidationResult::failure(
        "enabled emergency-stop reset requires an authorized operator");
  }
  if (!executionStringIsBounded(config.authorized_reset_operator_id) ||
      !executionStringIsBounded(config.vehicle_profile.profile_id) ||
      !executionStringIsBounded(config.vehicle_profile.reference_frame) ||
      !executionStringIsBounded(config.command_guard.world_frame)) {
    return auto_rover::ValidationResult::failure(
        "vehicle execution configuration identity exceeds the byte limit");
  }
  result = requireValidation(
      auto_rover_safety::validateGuardConfig(config.command_guard));
  if (!result.ok) {
    return result;
  }
  result = requireValidation(validateVehicleMotionManagerConfig(
      config.vehicle_manager, config.vehicle_profile));
  if (!result.ok) {
    return result;
  }
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult validateVehicleExecutionCoreConfig(
    const VehicleExecutionCoreConfig& config) {
  const auto common_validation =
      validateVehicleExecutionCommonConfig(config);
  if (!common_validation.ok) {
    return common_validation;
  }
  return requireValidation(
      validateFakeVcuConfig(config.fake_vcu, config.vehicle_profile));
}

VehicleExecutionCore::VehicleExecutionCore(
    VehicleExecutionCoreConfig config)
    : VehicleExecutionCore(
          VehicleExecutionCommonConfig(config),
          std::unique_ptr<VehicleBackend>(
              new FakeVcu(config.fake_vcu, config.vehicle_profile))) {
  fake_vcu_ = dynamic_cast<FakeVcu*>(backend_.get());
}

VehicleExecutionCore::VehicleExecutionCore(
    VehicleExecutionCommonConfig config,
    std::unique_ptr<VehicleBackend> backend)
    : config_(std::move(config)),
      configuration_validation_(
          validateVehicleExecutionCommonConfig(config_)),
      safety_supervisor_(config_.safety_supervisor),
      command_guard_(config_.command_guard, config_.vehicle_profile),
      vehicle_manager_(config_.vehicle_manager, config_.vehicle_profile),
      backend_(std::move(backend)) {
  if (configuration_validation_.ok) {
    if (backend_ == nullptr) {
      configuration_validation_ = auto_rover::ValidationResult::failure(
          "vehicle execution backend is missing");
    } else {
      configuration_validation_ = backend_->validate(
          config_.vehicle_profile);
    }
  }
  fake_vcu_ = dynamic_cast<FakeVcu*>(backend_.get());
}

bool VehicleExecutionCore::validEventTime(
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) const {
  return now_monotonic_ns > 0 && now_ros_ns > 0 &&
         (last_event_monotonic_ns_ == 0 ||
          now_monotonic_ns >= last_event_monotonic_ns_) &&
         (last_event_ros_ns_ == 0 || now_ros_ns >= last_event_ros_ns_);
}

void VehicleExecutionCore::recordEventTime(
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) {
  if (now_monotonic_ns > 0 &&
      (last_event_monotonic_ns_ == 0 ||
       now_monotonic_ns >= last_event_monotonic_ns_)) {
    last_event_monotonic_ns_ = now_monotonic_ns;
  }
  if (now_ros_ns > 0 &&
      (last_event_ros_ns_ == 0 || now_ros_ns >= last_event_ros_ns_)) {
    last_event_ros_ns_ = now_ros_ns;
  }
}

bool VehicleExecutionCore::initialize(std::int64_t now_monotonic_ns,
                                      std::int64_t now_ros_ns) {
  if (!configuration_validation_.ok || backend_ == nullptr ||
      shutdown_called_ ||
      !validEventTime(now_monotonic_ns, now_ros_ns)) {
    revokeExecution(now_monotonic_ns);
    return false;
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  BackendHealth backend_health;
  try {
    if (!backend_->initialize(now_monotonic_ns, now_ros_ns)) {
      revokeExecution(now_monotonic_ns);
      return false;
    }
    backend_health = backend_->health();
  } catch (...) {
    backend_exception_inhibited_ = true;
    revokeExecution(now_monotonic_ns);
    return false;
  }
  if (!backend_health.configuration_valid ||
      (backend_health.connected &&
       backend_health.connection_generation == 0U)) {
    revokeExecution(now_monotonic_ns);
    return false;
  }
  backend_generation_initialized_ = backend_health.connected;
  backend_requires_new_generation_ = false;
  backend_connection_generation_ = backend_health.connected
                                       ? backend_health.connection_generation
                                       : 0U;
  backend_generation_diagnostic_.clear();
  vehicle_manager_.setBackendConnected(backend_health.connected,
                                       now_monotonic_ns);
  const auto state = safety_supervisor_.completeBoot(now_ros_ns);
  safety_ = {state, now_monotonic_ns};
  initialized_ = state.valid &&
                 vehicle_manager_.backendConnected() ==
                     backend_health.connected;
  if (!initialized_) {
    revokeExecution(now_monotonic_ns);
  }
  return initialized_;
}

auto_rover::ValidationResult VehicleExecutionCore::observeRequiredStateOrder(
    const std::string& label, const std::string& source_id,
    std::uint64_t state_id, std::int64_t stamp_ns,
    std::int64_t receipt_monotonic_ns,
    RequiredStateOrder* order) {
  if (!order->initialized) {
    order->initialized = true;
    order->source_id = source_id;
    order->state_id = state_id;
    order->stamp_ns = stamp_ns;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::success();
  }
  if (source_id != order->source_id) {
    if (order->retired_source_ids.count(source_id) != 0U) {
      return auto_rover::ValidationResult::failure(
          label + " source identity was already retired");
    }
    if (receipt_monotonic_ns <= order->receipt_monotonic_ns) {
      return auto_rover::ValidationResult::failure(
          label + " receipt did not advance for a new source identity");
    }
    if (order->retired_source_ids.size() >=
        kMaximumExecutionIdentityHistoryEntries) {
      return auto_rover::ValidationResult::failure(
          label + " source identity history reached its execution limit");
    }
    order->retired_source_ids.insert(order->source_id);
    order->source_id = source_id;
    order->state_id = state_id;
    order->stamp_ns = stamp_ns;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    order->compromised = false;
    return auto_rover::ValidationResult::success();
  }
  if (order->compromised) {
    return auto_rover::ValidationResult::failure(
        label + " source identity is compromised; a new identity is required");
  }
  if (receipt_monotonic_ns < order->receipt_monotonic_ns) {
    order->compromised = true;
    return auto_rover::ValidationResult::failure(
        label + " receipt time rolled back");
  }
  if (state_id < order->state_id) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " state identity rolled back");
  }
  if (stamp_ns < order->stamp_ns) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " source stamp rolled back");
  }
  if (state_id == order->state_id && stamp_ns == order->stamp_ns) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " semantic sample was replayed by a later callback");
  }
  if (state_id == order->state_id) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " state identity was reused with a different stamp");
  }
  if (stamp_ns == order->stamp_ns) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " source stamp did not advance with state identity");
  }
  if (receipt_monotonic_ns == order->receipt_monotonic_ns) {
    order->compromised = true;
    return auto_rover::ValidationResult::failure(
        label + " semantic state changed without a new callback receipt");
  }
  order->state_id = state_id;
  order->stamp_ns = stamp_ns;
  order->receipt_monotonic_ns = receipt_monotonic_ns;
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult VehicleExecutionCore::observeTrajectoryOrder(
    const auto_rover::Trajectory& trajectory) {
  if (!trajectory_order_.initialized) {
    trajectory_order_.initialized = true;
    trajectory_order_.trajectory_id = trajectory.trajectory_id;
    trajectory_order_.route_id = trajectory.route_id;
    trajectory_order_.plan_version = trajectory.plan_version;
    trajectory_order_.stamp_ns = trajectory.stamp_ns;
    trajectory_order_.highest_plan_versions[trajectory.route_id] =
        trajectory.plan_version;
    return auto_rover::ValidationResult::success();
  }
  if (trajectory.trajectory_id != trajectory_order_.trajectory_id) {
    if (trajectory_order_.retired_trajectory_ids.count(
            trajectory.trajectory_id) != 0U) {
      return auto_rover::ValidationResult::failure(
          "trajectory generation identity was already retired");
    }
    const auto previous = trajectory_order_.highest_plan_versions.find(
        trajectory.route_id);
    if (previous != trajectory_order_.highest_plan_versions.end() &&
        trajectory.plan_version <= previous->second) {
      return auto_rover::ValidationResult::failure(
          "trajectory plan version did not advance for a new generation");
    }
    if (trajectory_order_.retired_trajectory_ids.size() >=
        kMaximumExecutionIdentityHistoryEntries) {
      return auto_rover::ValidationResult::failure(
          "trajectory generation history reached its execution limit");
    }
    if (previous == trajectory_order_.highest_plan_versions.end() &&
        trajectory_order_.highest_plan_versions.size() >=
            kMaximumExecutionIdentityHistoryEntries) {
      return auto_rover::ValidationResult::failure(
          "trajectory route history reached its execution limit");
    }
    trajectory_order_.retired_trajectory_ids.insert(
        trajectory_order_.trajectory_id);
    trajectory_order_.trajectory_id = trajectory.trajectory_id;
    trajectory_order_.route_id = trajectory.route_id;
    trajectory_order_.plan_version = trajectory.plan_version;
    trajectory_order_.stamp_ns = trajectory.stamp_ns;
    trajectory_order_.highest_plan_versions[trajectory.route_id] =
        trajectory.plan_version;
    trajectory_order_.compromised = false;
    return auto_rover::ValidationResult::success();
  }
  if (trajectory_order_.compromised) {
    return auto_rover::ValidationResult::failure(
        "trajectory generation identity is compromised; a new identity is required");
  }
  if (trajectory.route_id != trajectory_order_.route_id ||
      trajectory.plan_version != trajectory_order_.plan_version) {
    trajectory_order_.compromised = true;
    return auto_rover::ValidationResult::failure(
        "trajectory route or plan identity changed within one generation");
  }
  if (trajectory.stamp_ns < trajectory_order_.stamp_ns) {
    trajectory_order_.compromised = true;
    return auto_rover::ValidationResult::failure(
        "trajectory source stamp rolled back");
  }
  trajectory_order_.stamp_ns = trajectory.stamp_ns;
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult VehicleExecutionCore::updateEgoState(
    const auto_rover::EgoState& value,
    std::int64_t receipt_monotonic_ns) {
  if (!initialized_ || receipt_monotonic_ns <= 0 ||
      receipt_monotonic_ns < last_event_monotonic_ns_) {
    ego_ = {};
    revokeExecution(receipt_monotonic_ns);
    return auto_rover::ValidationResult::failure(
        "ego receipt time is invalid or regressed");
  }
  auto result = validateExecutionEgoEnvelope(value);
  if (result.ok) {
    result = auto_rover::validateEgoState(
        value, config_.command_guard.world_frame,
        config_.vehicle_profile.reference_frame);
  }
  if (result.ok) {
    result = observeRequiredStateOrder(
        "ego", value.source_id, value.state_id, value.stamp_ns,
        receipt_monotonic_ns,
        &ego_order_);
  }
  if (!result.ok) {
    ego_ = {};
    revokeExecution(receipt_monotonic_ns);
    recordEventTime(receipt_monotonic_ns);
    return result;
  }
  ego_ = {value, receipt_monotonic_ns};
  recordEventTime(receipt_monotonic_ns);
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult VehicleExecutionCore::updateTrajectory(
    const auto_rover::Trajectory& value,
    std::int64_t receipt_monotonic_ns) {
  if (!initialized_ || receipt_monotonic_ns <= 0 ||
      receipt_monotonic_ns < last_event_monotonic_ns_) {
    trajectory_ = {};
    revokeExecution(receipt_monotonic_ns);
    return auto_rover::ValidationResult::failure(
        "trajectory receipt time is invalid or regressed");
  }
  auto result = validateExecutionTrajectoryEnvelope(value);
  if (result.ok) {
    result = auto_rover::validateTrajectory(
        value, config_.vehicle_profile, config_.command_guard.world_frame);
  }
  if (result.ok) {
    result = observeTrajectoryOrder(value);
  }
  if (!result.ok) {
    trajectory_ = {};
    revokeExecution(receipt_monotonic_ns);
    recordEventTime(receipt_monotonic_ns);
    return result;
  }
  trajectory_ = {value, receipt_monotonic_ns};
  recordEventTime(receipt_monotonic_ns);
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult VehicleExecutionCore::updateChassisState(
    const auto_rover::ChassisState& value,
    std::int64_t receipt_monotonic_ns) {
  if (!initialized_ || receipt_monotonic_ns <= 0 ||
      receipt_monotonic_ns < last_event_monotonic_ns_) {
    chassis_ = {};
    revokeExecution(receipt_monotonic_ns);
    return auto_rover::ValidationResult::failure(
        "chassis receipt time is invalid or regressed");
  }
  auto result = executionChassisEnvelopeIsBounded(value)
                    ? auto_rover::validateChassisState(
                          value, config_.vehicle_profile, true)
                    : auto_rover::ValidationResult::failure(
                          "chassis identity exceeds the execution input byte limit");
  if (result.ok) {
    result = observeRequiredStateOrder(
        "chassis", value.source_id, value.state_id, value.stamp_ns,
        receipt_monotonic_ns,
        &chassis_order_);
  }
  if (!result.ok) {
    chassis_ = {};
    revokeExecution(receipt_monotonic_ns);
    recordEventTime(receipt_monotonic_ns);
    return result;
  }
  chassis_ = {value, receipt_monotonic_ns};
  recordEventTime(receipt_monotonic_ns);
  return auto_rover::ValidationResult::success();
}

void VehicleExecutionCore::updateMotionReference(
    const auto_rover::MotionReference& value,
    std::int64_t receipt_monotonic_ns) {
  if (!initialized_ || receipt_monotonic_ns <= 0 ||
      receipt_monotonic_ns < last_event_monotonic_ns_) {
    motion_ = {};
    revokeExecution(receipt_monotonic_ns);
    return;
  }
  if (!executionMotionEnvelopeIsBounded(value)) {
    motion_ = {};
    revokeExecution(receipt_monotonic_ns);
    recordEventTime(receipt_monotonic_ns);
    return;
  }
  motion_ = {value, receipt_monotonic_ns};
  recordEventTime(receipt_monotonic_ns);
}

BackendDelivery VehicleExecutionCore::revokeExecution(
    std::int64_t now_monotonic_ns) {
  vehicle_manager_.revokeAuthorization(now_monotonic_ns);
  command_guard_.resetRecovery();
  if (shutdown_called_) {
    return shutdown_delivery_;
  }
  BackendDelivery delivery;
  if (backend_ == nullptr) {
    delivery.stop_attempted = false;
    delivery.delivery_unconfirmed = true;
    delivery.reason = auto_rover::StopReason::kInvalidInput;
    delivery.diagnostic = "vehicle backend is missing during stop request";
  } else {
    try {
      delivery = backend_->revokeAndStop(now_monotonic_ns);
    } catch (...) {
      // revokeAndStop is the primary physical-zero boundary.  If it throws,
      // authorization state is unknowable, so latch the backend and make one
      // independent best-effort authorization revocation.  No later core
      // cycle may deliver motion through this backend instance.
      backend_exception_inhibited_ = true;
      try {
        (void)backend_->setAuthorization(false, 0U,
                                         now_monotonic_ns);
      } catch (...) {
      }
      delivery.stop_attempted = true;
      delivery.delivery_unconfirmed = true;
      delivery.reason = auto_rover::StopReason::kBackendDisconnected;
      delivery.diagnostic = "backend stop raised an exception";
    }
  }
  try {
    last_stop_delivery_ = normalizeStopDelivery(std::move(delivery));
  } catch (...) {
    backend_exception_inhibited_ = true;
    last_stop_delivery_ = BackendDelivery{};
    last_stop_delivery_.stop_attempted = true;
    last_stop_delivery_.delivery_unconfirmed = true;
    last_stop_delivery_.reason =
        auto_rover::StopReason::kBackendDisconnected;
  }
  ++stop_delivery_sequence_;
  return last_stop_delivery_;
}

bool VehicleExecutionCore::synchronizeBackendConnection(
    const BackendHealth& health,
    std::int64_t now_monotonic_ns) {
  if (!health.configuration_valid || !health.connected ||
      health.connection_generation == 0U) {
    if (backend_generation_initialized_) {
      backend_requires_new_generation_ = true;
      backend_generation_diagnostic_ =
          "vehicle backend disconnected; a strictly newer connection generation is required";
    } else {
      backend_generation_diagnostic_ =
          "vehicle backend has not established a valid connection generation";
    }
    vehicle_manager_.setBackendConnected(false, now_monotonic_ns);
    revokeExecution(now_monotonic_ns);
    return false;
  }
  if (!backend_generation_initialized_) {
    backend_generation_initialized_ = true;
    backend_connection_generation_ = health.connection_generation;
    backend_requires_new_generation_ = false;
    backend_generation_diagnostic_.clear();
    vehicle_manager_.setBackendConnected(true, now_monotonic_ns);
    return true;
  }

  if (backend_requires_new_generation_ &&
      health.connection_generation <= backend_connection_generation_) {
    if (backend_generation_diagnostic_.empty()) {
      backend_generation_diagnostic_ =
          "vehicle backend connection generation is not strictly newer than the retired high-water mark";
    }
    vehicle_manager_.setBackendConnected(false, now_monotonic_ns);
    revokeExecution(now_monotonic_ns);
    return false;
  }
  if (!backend_requires_new_generation_ &&
      health.connection_generation < backend_connection_generation_) {
    backend_requires_new_generation_ = true;
    backend_generation_diagnostic_ =
        "vehicle backend connection generation rolled back; a strictly newer generation is required";
    vehicle_manager_.setBackendConnected(false, now_monotonic_ns);
    revokeExecution(now_monotonic_ns);
    return false;
  }
  if (health.connection_generation > backend_connection_generation_) {
    revokeExecution(now_monotonic_ns);
    vehicle_manager_.setBackendConnected(false, now_monotonic_ns);
    backend_connection_generation_ = health.connection_generation;
    backend_requires_new_generation_ = false;
    backend_generation_diagnostic_.clear();
    vehicle_manager_.setBackendConnected(true, now_monotonic_ns);
    return false;
  }
  if (backend_requires_new_generation_) {
    if (backend_generation_diagnostic_.empty()) {
      backend_generation_diagnostic_ =
          "vehicle backend connection generation is not strictly newer than the retired high-water mark";
    }
    vehicle_manager_.setBackendConnected(false, now_monotonic_ns);
    revokeExecution(now_monotonic_ns);
    return false;
  }
  backend_generation_diagnostic_.clear();
  vehicle_manager_.setBackendConnected(true, now_monotonic_ns);
  return true;
}

VehicleExecutionCore::HealthResult VehicleExecutionCore::evaluateHealth(
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) const {
  if (!initialized_ || !configuration_validation_.ok ||
      backend_ == nullptr || shutdown_called_ ||
      backend_exception_inhibited_ || now_monotonic_ns <= 0 ||
      now_ros_ns <= 0) {
    return {false, auto_rover::StopReason::kInvalidInput,
            backend_exception_inhibited_
                ? "vehicle backend exception is latched; restart required"
                : "vehicle execution core is not initialized"};
  }
  const BackendHealth backend_health = backend_->health();
  if (!backend_health.configuration_valid) {
    return {false, auto_rover::StopReason::kInvalidInput,
            backend_health.diagnostic.empty()
                ? "vehicle backend configuration is invalid"
                : backend_health.diagnostic};
  }
  if (!backend_health.connected ||
      !vehicle_manager_.backendConnected() ||
      backend_health.connection_generation == 0U ||
      !backend_generation_initialized_ ||
      backend_requires_new_generation_ ||
      backend_health.connection_generation !=
          backend_connection_generation_) {
    return {false, auto_rover::StopReason::kBackendDisconnected,
            !backend_generation_diagnostic_.empty()
                ? backend_generation_diagnostic_
                : (backend_health.diagnostic.empty()
                       ? "vehicle backend is disconnected or changed generation"
                       : backend_health.diagnostic)};
  }
  if (backend_health.delivery_unconfirmed) {
    return {false, auto_rover::StopReason::kBackendDisconnected,
            backend_health.diagnostic.empty()
                ? "vehicle backend delivery is unconfirmed"
                : backend_health.diagnostic};
  }
  if (!backend_health.feedback_fresh) {
    return {false, auto_rover::StopReason::kStaleChassisState,
            backend_health.diagnostic.empty()
                ? "vehicle backend feedback is not fresh"
                : backend_health.diagnostic};
  }
  if (!backend_health.actuation_permitted) {
    const auto reason = backend_health.reason ==
                                auto_rover::StopReason::kNone
                            ? auto_rover::StopReason::kInvalidInput
                            : backend_health.reason;
    return {false, reason,
            backend_health.diagnostic.empty()
                ? "vehicle backend currently inhibits actuation"
                : backend_health.diagnostic};
  }
  if (!auto_rover::isFresh(ego_.receipt_monotonic_ns,
                           now_monotonic_ns,
                           config_.command_guard.localization_freshness_ns)) {
    return {false, auto_rover::StopReason::kStaleLocalization,
            "localization receiver watchdog expired"};
  }
  if (!auto_rover::isFresh(trajectory_.receipt_monotonic_ns,
                           now_monotonic_ns,
                           config_.command_guard.trajectory_freshness_ns)) {
    return {false, auto_rover::StopReason::kStaleTrajectory,
            "trajectory receiver watchdog expired"};
  }
  if (!auto_rover::isFresh(motion_.receipt_monotonic_ns,
                           now_monotonic_ns,
                           config_.command_guard.motion_freshness_ns)) {
    return {false, auto_rover::StopReason::kStaleMotionReference,
            "motion receiver watchdog expired"};
  }
  if (!auto_rover::isFresh(chassis_.receipt_monotonic_ns,
                           now_monotonic_ns,
                           config_.command_guard.chassis_freshness_ns)) {
    return {false, auto_rover::StopReason::kStaleChassisState,
            "chassis receiver watchdog expired"};
  }
  if (!auto_rover::withinDeclaredValidity(
          trajectory_.value.stamp_ns, now_ros_ns,
          trajectory_.value.valid_for_ns)) {
    return {false, auto_rover::StopReason::kStaleTrajectory,
            "trajectory producer validity expired"};
  }
  if (!auto_rover::withinDeclaredValidity(
          motion_.value.stamp_ns, now_ros_ns,
          motion_.value.valid_for_ns)) {
    return {false, auto_rover::StopReason::kStaleMotionReference,
            "motion producer validity expired"};
  }

  auto validation = auto_rover::validateEgoState(
      ego_.value, config_.command_guard.world_frame,
      config_.vehicle_profile.reference_frame);
  if (!validation.ok) {
    return {false, auto_rover::StopReason::kInvalidInput,
            validation.reason};
  }
  validation = auto_rover::validateTrajectory(
      trajectory_.value, config_.vehicle_profile,
      config_.command_guard.world_frame);
  if (!validation.ok) {
    return {false, auto_rover::StopReason::kInvalidInput,
            validation.reason};
  }
  validation = auto_rover::validateMotionReference(
      motion_.value, config_.vehicle_profile);
  if (!validation.ok) {
    return {false, auto_rover::StopReason::kInvalidInput,
            validation.reason};
  }
  validation = auto_rover::validateChassisState(
      chassis_.value, config_.vehicle_profile, true);
  if (!validation.ok) {
    return {false, auto_rover::StopReason::kInvalidInput,
            validation.reason};
  }
  if (motion_.value.trajectory_id != trajectory_.value.trajectory_id) {
    return {false, auto_rover::StopReason::kInvalidInput,
            "motion and trajectory identities do not match"};
  }
  if (hasValidBit(chassis_.value.valid_mask,
                  auto_rover::ChassisState::kFaultValid) &&
      chassis_.value.fault_state != auto_rover::FaultState::kOk) {
    return {false, auto_rover::StopReason::kInvalidInput,
            "chassis reports a non-OK fault state"};
  }

  const auto safety_state = safety_supervisor_.currentState(now_ros_ns);
  if (!safety_state.valid || safety_state.valid_for_ns <= 0) {
    return {false, auto_rover::StopReason::kInvalidInput,
            "internally generated safety state is invalid"};
  }
  if (safety_state.mode == auto_rover::SafetyMode::kArmed &&
      hasValidBit(chassis_.value.valid_mask,
                  auto_rover::ChassisState::kControlEnabledValid) &&
      !chassis_.value.control_enabled) {
    return {false, auto_rover::StopReason::kDisarmed,
            "chassis reports control disabled while armed"};
  }
  if (safety_state.mode != auto_rover::SafetyMode::kArmed &&
      hasValidBit(chassis_.value.valid_mask,
                  auto_rover::ChassisState::kControlEnabledValid) &&
      chassis_.value.control_enabled) {
    return {false, auto_rover::StopReason::kInvalidInput,
            "chassis reports control enabled without an armed safety state"};
  }
  if (safety_state.mode == auto_rover::SafetyMode::kArmed &&
      (!vehicle_manager_.operatorArmed() ||
       !backend_health.authorization_active)) {
    return {false, auto_rover::StopReason::kDisarmed,
            "armed safety state has no execution authorization"};
  }
  return {true, auto_rover::StopReason::kNone,
          "all receiver and backend health checks passed"};
}

VehicleExecutionCycleResult VehicleExecutionCore::cycle(
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) {
  VehicleExecutionCycleResult result;
  const std::uint64_t stop_sequence_at_entry = stop_delivery_sequence_;
  try {
  if (shutdown_called_ || backend_ == nullptr ||
      backend_exception_inhibited_ ||
      !validEventTime(now_monotonic_ns, now_ros_ns)) {
    result.stop_delivery = revokeExecution(now_monotonic_ns);
    result.safety = backend_exception_inhibited_
                        ? safety_supervisor_.observeConditions(
                              false,
                              auto_rover::StopReason::kBackendDisconnected,
                              now_ros_ns)
                        : safety_supervisor_.currentState(now_ros_ns);
    result.health_reason = backend_exception_inhibited_
                               ? auto_rover::StopReason::kBackendDisconnected
                               : auto_rover::StopReason::kInvalidInput;
    result.health_diagnostic =
        backend_exception_inhibited_
            ? "vehicle backend exception is latched; restart required"
            : "execution cycle time is invalid";
    appendUnconfirmedStopDiagnostic(result.stop_delivery,
                                    &result.health_diagnostic);
    return result;
  }

  const BackendFeedback feedback =
      backend_->poll(now_monotonic_ns, now_ros_ns);
  const bool poll_contract_valid =
      feedback.operation_completed_monotonic_ns > 0 &&
      feedback.operation_completed_monotonic_ns >= now_monotonic_ns &&
      feedback.operation_completed_ros_ns > 0 &&
      feedback.operation_completed_ros_ns >= now_ros_ns &&
      (last_event_monotonic_ns_ == 0 ||
       feedback.operation_completed_monotonic_ns >=
           last_event_monotonic_ns_) &&
      (last_event_ros_ns_ == 0 ||
       feedback.operation_completed_ros_ns >= last_event_ros_ns_);
  std::int64_t effective_monotonic_ns =
      poll_contract_valid ? feedback.operation_completed_monotonic_ns
                          : now_monotonic_ns;
  std::int64_t effective_ros_ns =
      poll_contract_valid ? feedback.operation_completed_ros_ns : now_ros_ns;
  bool feedback_contract_valid = poll_contract_valid;
  if (feedback.available) {
    feedback_contract_valid = feedback_contract_valid &&
        feedback.receipt_monotonic_ns > 0 && feedback.state.stamp_ns > 0 &&
        feedback.receipt_monotonic_ns <=
            feedback.operation_completed_monotonic_ns &&
        feedback.state.stamp_ns <= feedback.operation_completed_ros_ns &&
        (last_event_monotonic_ns_ == 0 ||
         feedback.receipt_monotonic_ns >= last_event_monotonic_ns_) &&
        (last_event_ros_ns_ == 0 ||
         feedback.state.stamp_ns >= last_event_ros_ns_);
    if (feedback_contract_valid) {
    }
  }
  if (!validEventTime(effective_monotonic_ns, effective_ros_ns)) {
    feedback_contract_valid = false;
    effective_monotonic_ns = now_monotonic_ns;
    effective_ros_ns = now_ros_ns;
  }
  synchronizeBackendConnection(feedback.health,
                               effective_monotonic_ns);
  result.chassis_available = feedback.available;
  result.chassis = feedback.state;
  if (feedback.available) {
    auto_rover::ValidationResult chassis_update =
        auto_rover::ValidationResult::failure(
            "backend feedback receipt is invalid");
    if (feedback_contract_valid) {
      chassis_update = updateChassisState(
          feedback.state, feedback.receipt_monotonic_ns);
    } else {
      revokeExecution(effective_monotonic_ns);
    }
    if (!chassis_update.ok) {
      result.chassis_available = false;
      result.chassis = {};
    }
  }
  // A feedback receipt may legitimately precede the completion of the
  // bounded backend poll that returned it.  Observe and validate the sample
  // first, then advance the execution event clock to the poll boundary so the
  // next backend operation cannot reuse a pre-I/O time.
  recordEventTime(effective_monotonic_ns, effective_ros_ns);

  const HealthResult health = feedback_contract_valid
                                  ? evaluateHealth(effective_monotonic_ns,
                                                   effective_ros_ns)
                                  : HealthResult{
                                        false,
                                        auto_rover::StopReason::kInvalidInput,
                                        "vehicle backend poll completion, feedback receipt, or stamp is invalid"};
  result.health_clear = health.clear;
  result.health_reason = health.reason;
  result.health_diagnostic = health.diagnostic;
  result.safety = safety_supervisor_.observeConditions(
      health.clear, health.reason, effective_ros_ns);
  safety_ = {result.safety, effective_monotonic_ns};

  if (!health.clear ||
      result.safety.mode != auto_rover::SafetyMode::kArmed) {
    revokeExecution(effective_monotonic_ns);
  }

  auto_rover_safety::GuardInput guard_input;
  guard_input.ego = ego_;
  guard_input.trajectory = trajectory_;
  guard_input.motion = motion_;
  guard_input.chassis = chassis_;
  guard_input.safety = safety_;
  guard_input.now_monotonic_ns = effective_monotonic_ns;
  guard_input.now_ros_ns = effective_ros_ns;
  result.guard = command_guard_.evaluate(guard_input);
  result.command = vehicle_manager_.makeCommand(
      result.guard, effective_monotonic_ns);
  result.delivery = backend_->deliver(result.command,
                                      effective_monotonic_ns);
  if (result.delivery.delivery_unconfirmed) {
    result.health_clear = false;
    result.health_reason = auto_rover::StopReason::kBackendDisconnected;
    result.health_diagnostic = result.delivery.diagnostic.empty()
                                   ? "vehicle backend delivery is unconfirmed"
                                   : result.delivery.diagnostic;
    result.safety = safety_supervisor_.observeConditions(
        false, result.health_reason, effective_ros_ns);
    safety_ = {result.safety, effective_monotonic_ns};
    revokeExecution(effective_monotonic_ns);
  } else {
    const BackendHealth post_delivery_health = backend_->health();
    const bool connection_stable = synchronizeBackendConnection(
        post_delivery_health, effective_monotonic_ns);
    const bool authorization_lost =
        result.safety.mode == auto_rover::SafetyMode::kArmed &&
        !post_delivery_health.authorization_active;
    if (!connection_stable || authorization_lost) {
      result.health_clear = false;
      result.health_reason = connection_stable
                                 ? auto_rover::StopReason::kDisarmed
                                 : auto_rover::StopReason::kBackendDisconnected;
      result.health_diagnostic =
          !connection_stable && !backend_generation_diagnostic_.empty()
              ? backend_generation_diagnostic_
              : (post_delivery_health.diagnostic.empty()
                     ? "vehicle backend connection or authorization changed during delivery"
                     : post_delivery_health.diagnostic);
      result.safety = safety_supervisor_.observeConditions(
          false, result.health_reason, effective_ros_ns);
      safety_ = {result.safety, effective_monotonic_ns};
      revokeExecution(effective_monotonic_ns);
    }
  }
  if (stop_delivery_sequence_ != stop_sequence_at_entry) {
    result.stop_delivery = last_stop_delivery_;
    appendUnconfirmedStopDiagnostic(result.stop_delivery,
                                    &result.health_diagnostic);
    if (result.stop_delivery.delivery_unconfirmed) {
      result.health_clear = false;
      result.health_reason =
          auto_rover::StopReason::kBackendDisconnected;
      result.safety = safety_supervisor_.observeConditions(
          false, result.health_reason, effective_ros_ns);
      safety_ = {result.safety, effective_monotonic_ns};
    }
  }
  return result;
  } catch (...) {
    // A backend can throw after changing its internal state or after a local
    // write.  Never let that leave an armed core able to issue another motion
    // command.  The backend instance remains terminally inhibited.
    backend_exception_inhibited_ = true;
    vehicle_manager_.revokeAuthorization(now_monotonic_ns);
    command_guard_.resetRecovery();
    try {
      (void)safety_supervisor_.observeConditions(
          false, auto_rover::StopReason::kBackendDisconnected,
          now_ros_ns);
    } catch (...) {
    }
    VehicleExecutionCycleResult failed;
    failed.stop_delivery = revokeExecution(now_monotonic_ns);
    failed.safety = safety_supervisor_.currentState(now_ros_ns);
    failed.health_clear = false;
    failed.health_reason =
        auto_rover::StopReason::kBackendDisconnected;
    failed.health_diagnostic =
        "vehicle backend exception aborted execution; restart required";
    appendUnconfirmedStopDiagnostic(failed.stop_delivery,
                                    &failed.health_diagnostic);
    return failed;
  }
}

auto_rover::SafetyState VehicleExecutionCore::currentSafetyState(
    std::int64_t now_ros_ns) const {
  return safety_supervisor_.currentState(now_ros_ns);
}

BackendDelivery VehicleExecutionCore::shutdown(
    std::int64_t now_monotonic_ns) {
  if (shutdown_called_) {
    return shutdown_delivery_;
  }
  shutdown_called_ = true;
  vehicle_manager_.revokeAuthorization(now_monotonic_ns);
  command_guard_.resetRecovery();
  if (backend_ == nullptr) {
    shutdown_delivery_.stop_attempted = false;
    shutdown_delivery_.delivery_unconfirmed = true;
    shutdown_delivery_.reason = auto_rover::StopReason::kInvalidInput;
    shutdown_delivery_.diagnostic = "vehicle backend is missing at shutdown";
  } else {
    try {
      shutdown_delivery_ = backend_->revokeAndStop(now_monotonic_ns);
    } catch (...) {
      backend_exception_inhibited_ = true;
      try {
        (void)backend_->setAuthorization(false, 0U,
                                         now_monotonic_ns);
      } catch (...) {
      }
      shutdown_delivery_ = BackendDelivery{};
      shutdown_delivery_.stop_attempted = true;
      shutdown_delivery_.delivery_unconfirmed = true;
      shutdown_delivery_.reason =
          auto_rover::StopReason::kBackendDisconnected;
      shutdown_delivery_.diagnostic =
          "backend shutdown stop raised an exception";
    }
  }
  try {
    shutdown_delivery_ = normalizeStopDelivery(
        std::move(shutdown_delivery_));
  } catch (...) {
    backend_exception_inhibited_ = true;
    shutdown_delivery_ = BackendDelivery{};
    shutdown_delivery_.stop_attempted = true;
    shutdown_delivery_.delivery_unconfirmed = true;
    shutdown_delivery_.reason =
        auto_rover::StopReason::kBackendDisconnected;
  }
  last_stop_delivery_ = shutdown_delivery_;
  ++stop_delivery_sequence_;
  initialized_ = false;
  return shutdown_delivery_;
}

VehicleExecutionServiceResult VehicleExecutionCore::serviceResult(
    const auto_rover_safety::SafetyTransition& transition,
    BackendDelivery stop_delivery) const {
  VehicleExecutionServiceResult result{
      transition.accepted, transition.reason, transition.state,
      std::move(stop_delivery)};
  appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
  return result;
}

VehicleExecutionServiceResult VehicleExecutionCore::requestArm(
    const std::string& operator_id, std::uint64_t expected_state_id,
    bool arm, std::int64_t now_monotonic_ns,
    std::int64_t now_ros_ns) {
  if (shutdown_called_ || backend_ == nullptr ||
      !validEventTime(now_monotonic_ns, now_ros_ns) ||
      !executionStringIsBounded(operator_id)) {
    const BackendDelivery stop_delivery =
        revokeExecution(now_monotonic_ns);
    VehicleExecutionServiceResult result{
        false, "arm service time or operator identity is invalid",
        currentSafetyState(now_ros_ns), stop_delivery};
    appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
    return result;
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  if (!arm) {
    const BackendDelivery stop_delivery =
        revokeExecution(now_monotonic_ns);
    return serviceResult(
        safety_supervisor_.requestDisarm(operator_id, now_ros_ns),
        stop_delivery);
  }
  try {
    const HealthResult health = evaluateHealth(now_monotonic_ns, now_ros_ns);
    if (!health.clear) {
      const BackendDelivery stop_delivery =
          revokeExecution(now_monotonic_ns);
      safety_supervisor_.observeConditions(false, health.reason, now_ros_ns);
      VehicleExecutionServiceResult result{
          false, health.diagnostic, currentSafetyState(now_ros_ns),
          stop_delivery};
      appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
      return result;
    }
    const auto transition = safety_supervisor_.requestArm(
        operator_id, expected_state_id, now_ros_ns);
    if (!transition.accepted) {
      const BackendDelivery stop_delivery =
          revokeExecution(now_monotonic_ns);
      return serviceResult(transition, stop_delivery);
    }
    if (!vehicle_manager_.acknowledgeOperatorArm(
            transition.state.state_id, now_monotonic_ns)) {
      const BackendDelivery stop_delivery =
          revokeExecution(now_monotonic_ns);
      const auto disarmed = safety_supervisor_.requestDisarm(
          operator_id, now_ros_ns);
      VehicleExecutionServiceResult result{
          false, "vehicle manager rejected arm authorization",
          disarmed.state, stop_delivery};
      appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
      return result;
    }
    if (!backend_->setAuthorization(
            true, transition.state.state_id, now_monotonic_ns) ||
        !backend_->health().authorization_active) {
      const BackendDelivery stop_delivery =
          revokeExecution(now_monotonic_ns);
      const auto disarmed = safety_supervisor_.requestDisarm(
          operator_id, now_ros_ns);
      VehicleExecutionServiceResult result{
          false, "vehicle backend rejected execution authorization",
          disarmed.state, stop_delivery};
      appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
      return result;
    }
    safety_ = {transition.state, now_monotonic_ns};
    return serviceResult(transition);
  } catch (...) {
    // Treat the entire arm operation as one transaction.  This catch also
    // covers exceptions while copying backend diagnostics or building the
    // successful service result after the backend has accepted authorization.
    // Local authorization is revoked before any further backend call.
    backend_exception_inhibited_ = true;
    vehicle_manager_.revokeAuthorization(now_monotonic_ns);
    command_guard_.resetRecovery();
    try {
      (void)safety_supervisor_.requestDisarm(operator_id, now_ros_ns);
    } catch (...) {
      // requestDisarm changes the mode before materializing its result.  If
      // that materialization itself failed, force a non-clear observation as
      // a second no-motion transition attempt.
      try {
        (void)safety_supervisor_.observeConditions(
            false, auto_rover::StopReason::kInvalidInput, now_ros_ns);
      } catch (...) {
      }
    }
    const BackendDelivery stop_delivery =
        revokeExecution(now_monotonic_ns);
    VehicleExecutionServiceResult result{
        false,
        "vehicle backend exception aborted arm; restart required",
        currentSafetyState(now_ros_ns), stop_delivery};
    appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
    return result;
  }
}

VehicleExecutionServiceResult VehicleExecutionCore::handleEmergencyStop(
    const auto_rover::EmergencyStop& request,
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) {
  const BackendDelivery stop_delivery =
      revokeExecution(now_monotonic_ns);
  if (!executionStringIsBounded(request.request_id) ||
      !executionStringIsBounded(request.source_id)) {
    VehicleExecutionServiceResult result{
        false, "emergency-stop identity exceeds the execution input byte limit",
        currentSafetyState(now_ros_ns), stop_delivery};
    appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
    return result;
  }
  const bool event_time_valid =
      validEventTime(now_monotonic_ns, now_ros_ns);
  const auto transition = safety_supervisor_.assertEmergencyStop(
      request, now_ros_ns);
  auto result = serviceResult(transition, stop_delivery);
  if (!event_time_valid) {
    result.state.valid = false;
    return result;
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  safety_ = {result.state, now_monotonic_ns};
  return result;
}

VehicleExecutionServiceResult VehicleExecutionCore::resetEmergencyStop(
    const std::string& operator_id, std::uint64_t latch_generation,
    bool conditions_cleared_acknowledged,
    std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) {
  const BackendDelivery stop_delivery =
      revokeExecution(now_monotonic_ns);
  if (!validEventTime(now_monotonic_ns, now_ros_ns) ||
      !executionStringIsBounded(operator_id)) {
    VehicleExecutionServiceResult result{
        false, "emergency-stop reset time or operator identity is invalid",
        currentSafetyState(now_ros_ns), stop_delivery};
    appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
    return result;
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  const HealthResult health = evaluateHealth(now_monotonic_ns, now_ros_ns);
  if (!health.clear) {
    const auto state = safety_supervisor_.observeConditions(
        false, health.reason, now_ros_ns);
    safety_ = {state, now_monotonic_ns};
    VehicleExecutionServiceResult result{
        false, health.diagnostic, state, stop_delivery};
    appendUnconfirmedStopDiagnostic(result.stop_delivery, &result.reason);
    return result;
  }
  auto_rover_safety::ResetEmergencyStopRequest request;
  request.operator_id = operator_id;
  request.latch_generation = latch_generation;
  request.conditions_cleared_acknowledged =
      conditions_cleared_acknowledged;
  request.authorization_granted =
      config_.safety_supervisor.reset_service_enabled &&
      !config_.authorized_reset_operator_id.empty() &&
      operator_id == config_.authorized_reset_operator_id;
  const auto transition = safety_supervisor_.resetEmergencyStop(
      request, now_ros_ns);
  safety_ = {transition.state, now_monotonic_ns};
  return serviceResult(transition, stop_delivery);
}

FakeVcuInjectionResult VehicleExecutionCore::rejectInjection(
    const std::string& reason, std::int64_t now_monotonic_ns) {
  revokeExecution(now_monotonic_ns);
  return {false, reason};
}

FakeVcuInjectionResult VehicleExecutionCore::setFakeConnected(
    bool connected, std::int64_t now_monotonic_ns,
    std::int64_t now_ros_ns) {
  if (fake_vcu_ == nullptr) {
    return rejectInjection(
        "selected vehicle backend does not support fake connection injection",
        now_monotonic_ns);
  }
  if (!validEventTime(now_monotonic_ns, now_ros_ns)) {
    return rejectInjection("fake connection service time is invalid",
                           now_monotonic_ns);
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  revokeExecution(now_monotonic_ns);
  fake_vcu_->setConnected(connected, now_monotonic_ns);
  synchronizeBackendConnection(fake_vcu_->health(), now_monotonic_ns);
  safety_supervisor_.observeConditions(
      false, connected ? auto_rover::StopReason::kRecoveryPending
                       : auto_rover::StopReason::kBackendDisconnected,
      now_ros_ns);
  const bool applied = fake_vcu_->connected() == connected &&
                       vehicle_manager_.backendConnected() == connected;
  return {applied, applied ? (connected ? "fake VCU reconnected; re-arm required"
                                       : "fake VCU disconnected")
                           : "fake VCU connection injection was rejected"};
}

FakeVcuInjectionResult VehicleExecutionCore::setFakeFaulted(
    bool faulted, std::int64_t now_monotonic_ns,
    std::int64_t now_ros_ns) {
  if (fake_vcu_ == nullptr) {
    return rejectInjection(
        "selected vehicle backend does not support fake fault injection",
        now_monotonic_ns);
  }
  if (!validEventTime(now_monotonic_ns, now_ros_ns)) {
    return rejectInjection("fake fault service time is invalid",
                           now_monotonic_ns);
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  revokeExecution(now_monotonic_ns);
  fake_vcu_->setFaulted(faulted, now_monotonic_ns);
  safety_supervisor_.observeConditions(
      false, faulted ? auto_rover::StopReason::kInvalidInput
                     : auto_rover::StopReason::kRecoveryPending,
      now_ros_ns);
  const bool applied = fake_vcu_->faulted() == faulted;
  return {applied, applied ? (faulted ? "fake VCU fault asserted"
                                     : "fake VCU fault cleared; re-arm required")
                           : "fake VCU fault injection was rejected"};
}

FakeVcuInjectionResult VehicleExecutionCore::setFakeDropFeedback(
    bool drop_feedback, std::int64_t now_monotonic_ns,
    std::int64_t now_ros_ns) {
  if (fake_vcu_ == nullptr) {
    return rejectInjection(
        "selected vehicle backend does not support fake feedback injection",
        now_monotonic_ns);
  }
  if (!validEventTime(now_monotonic_ns, now_ros_ns)) {
    return rejectInjection("fake feedback service time is invalid",
                           now_monotonic_ns);
  }
  recordEventTime(now_monotonic_ns, now_ros_ns);
  revokeExecution(now_monotonic_ns);
  fake_vcu_->setDropFeedback(drop_feedback);
  safety_supervisor_.observeConditions(
      false, drop_feedback ? auto_rover::StopReason::kStaleChassisState
                           : auto_rover::StopReason::kRecoveryPending,
      now_ros_ns);
  const bool applied = fake_vcu_->dropFeedback() == drop_feedback;
  return {applied,
          applied ? (drop_feedback
                         ? "fake VCU feedback drop asserted"
                         : "fake VCU feedback restored; re-arm required")
                  : "fake VCU feedback injection was rejected"};
}

bool VehicleExecutionCore::executionAuthorized() const {
  if (!initialized_ || shutdown_called_ || backend_ == nullptr ||
      backend_exception_inhibited_ ||
      !vehicle_manager_.operatorArmed()) {
    return false;
  }
  try {
    return backend_->health().authorization_active;
  } catch (...) {
    return false;
  }
}

bool VehicleExecutionCore::fakeConnected() const {
  return fake_vcu_ != nullptr && fake_vcu_->connected();
}

bool VehicleExecutionCore::fakeControlEnabled() const {
  return fake_vcu_ != nullptr && fake_vcu_->controlEnabled();
}

bool VehicleExecutionCore::fakeFaulted() const {
  return fake_vcu_ != nullptr && fake_vcu_->faulted();
}

bool VehicleExecutionCore::fakeDropFeedback() const {
  return fake_vcu_ != nullptr && fake_vcu_->dropFeedback();
}

}  // namespace auto_rover_vehicle
