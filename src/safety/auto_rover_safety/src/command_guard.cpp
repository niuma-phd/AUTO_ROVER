#include "auto_rover_safety/command_guard.hpp"

#include <utility>

#include "auto_rover_core/validation.hpp"

namespace auto_rover_safety {

auto_rover::ValidationResult validateGuardConfig(const GuardConfig& config) {
  if (config.world_frame.empty() || config.localization_freshness_ns <= 0 ||
      config.trajectory_freshness_ns <= 0 ||
      config.motion_freshness_ns <= 0 || config.chassis_freshness_ns <= 0 ||
      config.safety_freshness_ns <= 0 || config.fresh_recovery_count == 0U) {
    return auto_rover::ValidationResult::failure(
        "guard frames, freshness, and recovery count must be configured");
  }
  return auto_rover::ValidationResult::success();
}

CommandGuard::CommandGuard(GuardConfig config,
                           auto_rover::VehicleProfile vehicle_profile)
    : config_(std::move(config)), vehicle_profile_(std::move(vehicle_profile)) {}

void CommandGuard::resetRecovery() {
  consecutive_fresh_count_ = 0U;
  last_recovery_motion_producer_generation_id_ =
      active_motion_producer_generation_id_;
  last_recovery_motion_command_id_ = highest_motion_command_id_;
  last_recovery_motion_receipt_monotonic_ns_ =
      last_motion_receipt_monotonic_ns_;
}

std::size_t CommandGuard::consecutiveFreshCount() const {
  return consecutive_fresh_count_;
}

GuardResult CommandGuard::stop(const GuardInput& input,
                               auto_rover::StopReason reason,
                               const std::string& diagnostic,
                               bool reset_recovery) {
  if (reset_recovery) {
    resetRecovery();
  }
  GuardResult output;
  output.reference = input.motion.value;
  output.reference.stamp_ns = input.now_ros_ns;
  output.reference.frame_id = vehicle_profile_.reference_frame;
  output.reference.direction = auto_rover::Direction::kForward;
  output.reference.target_speed_mps = 0.0;
  output.reference.target_curvature_inv_m = 0.0;
  output.reference.route_complete = false;
  output.reference.valid = true;
  output.execution_allowed = false;
  output.reason = reason;
  output.diagnostic = diagnostic;
  return output;
}

GuardResult CommandGuard::evaluate(const GuardInput& input) {
  const auto_rover::ValidationResult config_result = validateGuardConfig(config_);
  if (!config_result.ok) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                config_result.reason, true);
  }
  const auto_rover::ValidationResult profile_result =
      auto_rover::validateNucPhase1Profile(vehicle_profile_);
  if (!profile_result.ok || input.now_monotonic_ns <= 0 ||
      input.now_ros_ns <= 0) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                profile_result.ok ? "guard current time is invalid"
                                  : profile_result.reason,
                true);
  }
  if (!auto_rover::isFresh(input.ego.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.localization_freshness_ns)) {
    return stop(input, auto_rover::StopReason::kStaleLocalization,
                "localization receiver watchdog expired", true);
  }
  if (!auto_rover::isFresh(input.trajectory.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.trajectory_freshness_ns)) {
    return stop(input, auto_rover::StopReason::kStaleTrajectory,
                "trajectory receiver watchdog expired", true);
  }
  if (!auto_rover::isFresh(input.motion.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.motion_freshness_ns)) {
    return stop(input, auto_rover::StopReason::kStaleMotionReference,
                "motion receiver watchdog expired", true);
  }
  if (!auto_rover::isFresh(input.chassis.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.chassis_freshness_ns)) {
    return stop(input, auto_rover::StopReason::kStaleChassisState,
                "chassis receiver watchdog expired", true);
  }
  if (!auto_rover::isFresh(input.safety.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.safety_freshness_ns)) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                "safety receiver watchdog expired", true);
  }
  if (!auto_rover::withinDeclaredValidity(input.trajectory.value.stamp_ns,
                                          input.now_ros_ns,
                                          input.trajectory.value.valid_for_ns)) {
    return stop(input, auto_rover::StopReason::kStaleTrajectory,
                "trajectory producer validity expired", true);
  }
  if (!auto_rover::withinDeclaredValidity(input.motion.value.stamp_ns,
                                          input.now_ros_ns,
                                          input.motion.value.valid_for_ns)) {
    return stop(input, auto_rover::StopReason::kStaleMotionReference,
                "motion producer validity expired", true);
  }
  const auto_rover::ValidationResult safety_result =
      auto_rover::validateSafetyState(input.safety.value);
  if (!safety_result.ok ||
      !auto_rover::withinDeclaredValidity(input.safety.value.stamp_ns,
                                          input.now_ros_ns,
                                          input.safety.value.valid_for_ns)) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                safety_result.ok ? "safety state producer validity expired"
                                 : safety_result.reason,
                true);
  }

  const auto_rover::ValidationResult ego_result = auto_rover::validateEgoState(
      input.ego.value, config_.world_frame, vehicle_profile_.reference_frame);
  if (!ego_result.ok) {
    return stop(input, auto_rover::StopReason::kInvalidInput, ego_result.reason,
                true);
  }
  const auto_rover::ValidationResult trajectory_result =
      auto_rover::validateTrajectory(input.trajectory.value, vehicle_profile_,
                                     config_.world_frame);
  if (!trajectory_result.ok) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                trajectory_result.reason, true);
  }
  const auto_rover::ValidationResult motion_result =
      auto_rover::validateMotionReference(input.motion.value, vehicle_profile_);
  if (!motion_result.ok) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                motion_result.reason, true);
  }
  const auto_rover::ValidationResult chassis_result =
      auto_rover::validateChassisState(input.chassis.value, vehicle_profile_, true);
  if (!chassis_result.ok) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                chassis_result.reason, true);
  }
  if (input.motion.value.trajectory_id !=
      input.trajectory.value.trajectory_id) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                "motion and trajectory identities do not match", true);
  }

  const std::string& motion_generation =
      input.motion.value.producer_generation_id;
  const std::int64_t previous_motion_receipt_monotonic_ns =
      last_motion_receipt_monotonic_ns_;
  if (motion_order_initialized_ &&
      input.motion.receipt_monotonic_ns <
          last_motion_receipt_monotonic_ns_) {
    return stop(input, auto_rover::StopReason::kInvalidInput,
                "motion receipt time regressed", true);
  }
  bool motion_generation_changed = false;
  if (motion_order_initialized_ &&
      input.motion.receipt_monotonic_ns ==
          last_motion_receipt_monotonic_ns_) {
    if (motion_generation != active_motion_producer_generation_id_ ||
        input.motion.value.command_id != highest_motion_command_id_) {
      return stop(input, auto_rover::StopReason::kInvalidInput,
                  "motion identity changed without a new receipt", true);
    }
  } else {
    if (!motion_order_initialized_) {
      motion_order_initialized_ = true;
      active_motion_producer_generation_id_ = motion_generation;
      motion_generation_changed = true;
    } else if (motion_generation !=
               active_motion_producer_generation_id_) {
      if (retired_motion_producer_generation_ids_.count(
              motion_generation) != 0U) {
        return stop(input, auto_rover::StopReason::kInvalidInput,
                    "motion producer generation is retired", true);
      }
      if (retired_motion_producer_generation_ids_.size() >=
          kMaximumMotionProducerGenerationHistoryEntries) {
        return stop(input, auto_rover::StopReason::kInvalidInput,
                    "motion producer generation history reached its limit",
                    true);
      }
      retired_motion_producer_generation_ids_.insert(
          active_motion_producer_generation_id_);
      active_motion_producer_generation_id_ = motion_generation;
      motion_generation_changed = true;
    } else if (input.motion.value.command_id <=
               highest_motion_command_id_) {
      return stop(input, auto_rover::StopReason::kInvalidInput,
                  "motion command identity is replayed or out of order",
                  true);
    }
    last_motion_receipt_monotonic_ns_ =
        input.motion.receipt_monotonic_ns;
    highest_motion_command_id_ = input.motion.value.command_id;
  }

  if (motion_generation_changed) {
    consecutive_fresh_count_ = 0U;
    last_recovery_motion_producer_generation_id_ = motion_generation;
    last_recovery_motion_command_id_ = 0U;
    last_recovery_motion_receipt_monotonic_ns_ =
        previous_motion_receipt_monotonic_ns;
  }

  if (consecutive_fresh_count_ < config_.fresh_recovery_count) {
    const bool new_recovery_command =
        motion_generation ==
            last_recovery_motion_producer_generation_id_ &&
        input.motion.receipt_monotonic_ns >
            last_recovery_motion_receipt_monotonic_ns_ &&
        input.motion.value.command_id >
            last_recovery_motion_command_id_;
    if (new_recovery_command) {
      last_recovery_motion_receipt_monotonic_ns_ =
          input.motion.receipt_monotonic_ns;
      last_recovery_motion_command_id_ = input.motion.value.command_id;
      ++consecutive_fresh_count_;
    }
  }
  if (consecutive_fresh_count_ < config_.fresh_recovery_count) {
    return stop(input, auto_rover::StopReason::kRecoveryPending,
                "guard requires consecutive fresh inputs", false);
  }
  if (input.safety.value.mode != auto_rover::SafetyMode::kArmed) {
    const bool estop = input.safety.value.mode ==
                       auto_rover::SafetyMode::kEmergencyStopLatched;
    return stop(input,
                estop ? auto_rover::StopReason::kEmergencyStop
                      : auto_rover::StopReason::kDisarmed,
                estop ? "software emergency stop is latched"
                      : "vehicle is not armed",
                false);
  }

  GuardResult output;
  output.reference = input.motion.value;
  output.execution_allowed = true;
  output.reason = auto_rover::StopReason::kNone;
  output.diagnostic = "guarded motion accepted";
  return output;
}

}  // namespace auto_rover_safety
