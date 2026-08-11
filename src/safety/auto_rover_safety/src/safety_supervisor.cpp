#include "auto_rover_safety/safety_supervisor.hpp"

#include <utility>

namespace auto_rover_safety {

auto_rover::ValidationResult validateSafetySupervisorConfig(
    const SafetySupervisorConfig& config) {
  if (config.fresh_recovery_count == 0U || config.state_valid_for_ns <= 0) {
    return auto_rover::ValidationResult::failure(
        "safety recovery and validity configuration must be positive");
  }
  return auto_rover::ValidationResult::success();
}

SafetySupervisor::SafetySupervisor(SafetySupervisorConfig config)
    : config_(std::move(config)) {}

void SafetySupervisor::setMode(auto_rover::SafetyMode mode,
                               auto_rover::StopReason reason) {
  if (mode_ != mode || reason_ != reason) {
    ++state_id_;
  }
  mode_ = mode;
  reason_ = reason;
}

auto_rover::SafetyState SafetySupervisor::currentState(
    std::int64_t now_ros_ns) const {
  auto_rover::SafetyState state;
  state.stamp_ns = now_ros_ns;
  state.state_id = state_id_;
  state.mode = mode_;
  state.latch_generation = latch_generation_;
  state.valid_for_ns = config_.state_valid_for_ns;
  if (reason_ != auto_rover::StopReason::kNone) {
    state.reasons.push_back(reason_);
  }
  state.valid = validateSafetySupervisorConfig(config_).ok && now_ros_ns > 0;
  return state;
}

SafetyTransition SafetySupervisor::transitionResult(
    bool accepted, const std::string& reason, std::int64_t now_ros_ns) const {
  return {accepted, reason, currentState(now_ros_ns)};
}

auto_rover::SafetyState SafetySupervisor::completeBoot(
    std::int64_t now_ros_ns) {
  if (mode_ == auto_rover::SafetyMode::kEmergencyStopLatched ||
      boot_complete_) {
    return currentState(now_ros_ns);
  }
  if (!validateSafetySupervisorConfig(config_).ok || now_ros_ns <= 0) {
    setMode(auto_rover::SafetyMode::kBootInhibited,
            auto_rover::StopReason::kInvalidInput);
    return currentState(now_ros_ns);
  }
  boot_complete_ = true;
  fresh_count_ = 0U;
  conditions_clear_ = false;
  setMode(auto_rover::SafetyMode::kRecoveryInhibited,
          auto_rover::StopReason::kRecoveryPending);
  return currentState(now_ros_ns);
}

auto_rover::SafetyState SafetySupervisor::observeConditions(
    bool conditions_clear, auto_rover::StopReason reason,
    std::int64_t now_ros_ns) {
  if (!boot_complete_ || now_ros_ns <= 0) {
    fresh_count_ = 0U;
    conditions_clear_ = false;
    if (mode_ != auto_rover::SafetyMode::kEmergencyStopLatched) {
      setMode(auto_rover::SafetyMode::kBootInhibited,
              auto_rover::StopReason::kInvalidInput);
    }
    return currentState(now_ros_ns);
  }
  conditions_clear_ = conditions_clear;
  if (!conditions_clear) {
    fresh_count_ = 0U;
    if (mode_ != auto_rover::SafetyMode::kEmergencyStopLatched) {
      setMode(auto_rover::SafetyMode::kFaultInhibited,
              reason == auto_rover::StopReason::kNone
                  ? auto_rover::StopReason::kInvalidInput
                  : reason);
    }
    return currentState(now_ros_ns);
  }

  if (fresh_count_ < config_.fresh_recovery_count) {
    ++fresh_count_;
  }
  if (mode_ == auto_rover::SafetyMode::kEmergencyStopLatched) {
    return currentState(now_ros_ns);
  }
  if (fresh_count_ < config_.fresh_recovery_count) {
    setMode(auto_rover::SafetyMode::kRecoveryInhibited,
            auto_rover::StopReason::kRecoveryPending);
  } else if (mode_ != auto_rover::SafetyMode::kArmed) {
    setMode(auto_rover::SafetyMode::kDisarmed,
            auto_rover::StopReason::kDisarmed);
  }
  return currentState(now_ros_ns);
}

SafetyTransition SafetySupervisor::requestArm(
    const std::string& operator_id, std::uint64_t expected_state_id,
    std::int64_t now_ros_ns) {
  if (now_ros_ns <= 0) {
    return transitionResult(false, "arm request time is invalid", now_ros_ns);
  }
  if (!config_.actuation_enabled) {
    return transitionResult(false, "actuation is disabled by configuration",
                            now_ros_ns);
  }
  if (operator_id.empty() || expected_state_id != state_id_) {
    return transitionResult(false, "operator identity or state generation mismatch",
                            now_ros_ns);
  }
  if (!boot_complete_ || !conditions_clear_ ||
      fresh_count_ < config_.fresh_recovery_count ||
      mode_ != auto_rover::SafetyMode::kDisarmed) {
    return transitionResult(false, "safety conditions are not ready for arm",
                            now_ros_ns);
  }
  setMode(auto_rover::SafetyMode::kArmed, auto_rover::StopReason::kNone);
  return transitionResult(true, "vehicle armed", now_ros_ns);
}

SafetyTransition SafetySupervisor::requestDisarm(
    const std::string& operator_id, std::int64_t now_ros_ns) {
  if (operator_id.empty()) {
    return transitionResult(false, "operator identity is required", now_ros_ns);
  }
  if (mode_ == auto_rover::SafetyMode::kEmergencyStopLatched) {
    return transitionResult(false, "emergency stop remains latched", now_ros_ns);
  }
  setMode(auto_rover::SafetyMode::kDisarmed,
          auto_rover::StopReason::kDisarmed);
  return transitionResult(true, "vehicle disarmed", now_ros_ns);
}

SafetyTransition SafetySupervisor::assertEmergencyStop(
    const auto_rover::EmergencyStop& request, std::int64_t now_ros_ns) {
  if (!request.asserted || request.stamp_ns <= 0 || request.request_id.empty() ||
      request.source_id.empty()) {
    return transitionResult(false, "invalid emergency stop assertion", now_ros_ns);
  }
  ++latch_generation_;
  ++state_id_;
  fresh_count_ = 0U;
  conditions_clear_ = false;
  mode_ = auto_rover::SafetyMode::kEmergencyStopLatched;
  reason_ = auto_rover::StopReason::kEmergencyStop;
  return transitionResult(true, "emergency stop latched", now_ros_ns);
}

SafetyTransition SafetySupervisor::resetEmergencyStop(
    const ResetEmergencyStopRequest& request, std::int64_t now_ros_ns) {
  if (now_ros_ns <= 0) {
    return transitionResult(false, "emergency-stop reset time is invalid",
                            now_ros_ns);
  }
  if (!config_.reset_service_enabled || !request.authorization_granted) {
    return transitionResult(false, "reset authorization boundary rejected request",
                            now_ros_ns);
  }
  if (request.operator_id.empty() ||
      !request.conditions_cleared_acknowledged ||
      request.latch_generation != latch_generation_) {
    return transitionResult(false, "reset identity or generation is invalid",
                            now_ros_ns);
  }
  if (mode_ != auto_rover::SafetyMode::kEmergencyStopLatched ||
      !conditions_clear_ || fresh_count_ < config_.fresh_recovery_count) {
    return transitionResult(false, "emergency stop conditions have not cleared",
                            now_ros_ns);
  }
  setMode(auto_rover::SafetyMode::kDisarmed,
          auto_rover::StopReason::kDisarmed);
  return transitionResult(true, "emergency stop reset to disarmed", now_ros_ns);
}

}  // namespace auto_rover_safety
