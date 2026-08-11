#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

#include "auto_rover_core/types.hpp"

namespace auto_rover_safety {

constexpr std::size_t kMaximumMotionProducerGenerationHistoryEntries = 1024U;

struct GuardConfig {
  std::string world_frame;
  std::int64_t localization_freshness_ns{0};
  std::int64_t trajectory_freshness_ns{0};
  std::int64_t motion_freshness_ns{0};
  std::int64_t chassis_freshness_ns{0};
  std::int64_t safety_freshness_ns{0};
  std::size_t fresh_recovery_count{0U};
};

struct GuardInput {
  auto_rover::Received<auto_rover::EgoState> ego;
  auto_rover::Received<auto_rover::Trajectory> trajectory;
  auto_rover::Received<auto_rover::MotionReference> motion;
  auto_rover::Received<auto_rover::ChassisState> chassis;
  auto_rover::Received<auto_rover::SafetyState> safety;
  std::int64_t now_monotonic_ns{0};
  std::int64_t now_ros_ns{0};
};

struct GuardResult {
  auto_rover::MotionReference reference;
  bool execution_allowed{false};
  auto_rover::StopReason reason{auto_rover::StopReason::kInvalidInput};
  std::string diagnostic;
};

auto_rover::ValidationResult validateGuardConfig(const GuardConfig& config);

class CommandGuard {
 public:
  CommandGuard(GuardConfig config, auto_rover::VehicleProfile vehicle_profile);

  GuardResult evaluate(const GuardInput& input);
  void resetRecovery();
  std::size_t consecutiveFreshCount() const;

 private:
  GuardResult stop(const GuardInput& input, auto_rover::StopReason reason,
                   const std::string& diagnostic, bool reset_recovery);

  GuardConfig config_;
  auto_rover::VehicleProfile vehicle_profile_;
  std::size_t consecutive_fresh_count_{0U};
  bool motion_order_initialized_{false};
  std::string active_motion_producer_generation_id_;
  std::unordered_set<std::string>
      retired_motion_producer_generation_ids_;
  std::uint64_t highest_motion_command_id_{0U};
  std::int64_t last_motion_receipt_monotonic_ns_{0};
  std::string last_recovery_motion_producer_generation_id_;
  std::uint64_t last_recovery_motion_command_id_{0U};
  std::int64_t last_recovery_motion_receipt_monotonic_ns_{0};
};

}  // namespace auto_rover_safety
