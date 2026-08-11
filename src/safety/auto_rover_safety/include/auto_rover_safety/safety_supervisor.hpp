#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"

namespace auto_rover_safety {

struct SafetySupervisorConfig {
  bool actuation_enabled{false};
  bool reset_service_enabled{false};
  std::size_t fresh_recovery_count{0U};
  std::int64_t state_valid_for_ns{0};
};

struct ResetEmergencyStopRequest {
  std::string operator_id;
  std::uint64_t latch_generation{0U};
  bool conditions_cleared_acknowledged{false};
  bool authorization_granted{false};
};

struct SafetyTransition {
  bool accepted{false};
  std::string reason;
  auto_rover::SafetyState state;
};

auto_rover::ValidationResult validateSafetySupervisorConfig(
    const SafetySupervisorConfig& config);

class SafetySupervisor {
 public:
  explicit SafetySupervisor(SafetySupervisorConfig config);

  auto_rover::SafetyState completeBoot(std::int64_t now_ros_ns);
  auto_rover::SafetyState observeConditions(bool conditions_clear,
                                            auto_rover::StopReason reason,
                                            std::int64_t now_ros_ns);
  SafetyTransition requestArm(const std::string& operator_id,
                              std::uint64_t expected_state_id,
                              std::int64_t now_ros_ns);
  SafetyTransition requestDisarm(const std::string& operator_id,
                                 std::int64_t now_ros_ns);
  SafetyTransition assertEmergencyStop(
      const auto_rover::EmergencyStop& request, std::int64_t now_ros_ns);
  SafetyTransition resetEmergencyStop(
      const ResetEmergencyStopRequest& request, std::int64_t now_ros_ns);
  auto_rover::SafetyState currentState(std::int64_t now_ros_ns) const;

 private:
  void setMode(auto_rover::SafetyMode mode, auto_rover::StopReason reason);
  SafetyTransition transitionResult(bool accepted, const std::string& reason,
                                    std::int64_t now_ros_ns) const;

  SafetySupervisorConfig config_;
  auto_rover::SafetyMode mode_{auto_rover::SafetyMode::kBootInhibited};
  auto_rover::StopReason reason_{auto_rover::StopReason::kRecoveryPending};
  std::uint64_t state_id_{1U};
  std::uint64_t latch_generation_{0U};
  std::size_t fresh_count_{0U};
  bool boot_complete_{false};
  bool conditions_clear_{false};
};

}  // namespace auto_rover_safety
