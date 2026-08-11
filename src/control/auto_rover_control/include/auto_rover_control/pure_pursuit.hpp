#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "auto_rover_core/types.hpp"

namespace auto_rover_control {

struct PurePursuitConfig {
  std::string world_frame;
  std::string control_frame;
  std::string producer_generation_id;
  double lookahead_min_m{0.0};
  double lookahead_max_m{0.0};
  double lookahead_speed_gain_s{0.0};
  double goal_position_tolerance_m{0.0};
  double standstill_speed_threshold_mps{0.0};
  std::int64_t localization_freshness_ns{0};
  std::int64_t trajectory_freshness_ns{0};
  std::int64_t chassis_freshness_ns{0};
  std::int64_t safety_freshness_ns{0};
  std::int64_t motion_valid_for_ns{0};
};

struct TrackingInput {
  auto_rover::Received<auto_rover::Trajectory> trajectory;
  auto_rover::Received<auto_rover::EgoState> ego;
  auto_rover::Received<auto_rover::ChassisState> chassis;
  auto_rover::Received<auto_rover::SafetyState> safety;
  std::int64_t now_monotonic_ns{0};
  std::int64_t now_ros_ns{0};
};

struct TrackingResult {
  auto_rover::MotionReference reference;
  std::string reason;
  std::size_t nearest_index{0U};
  std::size_t target_index{0U};
  double remaining_distance_m{0.0};
};

auto_rover::ValidationResult validatePurePursuitConfig(
    const PurePursuitConfig& config);

class PurePursuit {
 public:
  PurePursuit(PurePursuitConfig config,
              auto_rover::VehicleProfile vehicle_profile);

  TrackingResult update(const TrackingInput& input);
  void reset();

 private:
  struct RequiredStateOrder {
    bool initialized{false};
    bool compromised{false};
    std::string source_id;
    std::uint64_t state_id{0U};
    std::int64_t stamp_ns{0};
    std::int64_t receipt_monotonic_ns{0};
    std::unordered_set<std::string> retired_source_ids;
  };

  struct TrajectoryOrder {
    bool initialized{false};
    bool compromised{false};
    std::string trajectory_id;
    std::string route_id;
    std::uint64_t plan_version{0U};
    std::int64_t stamp_ns{0};
    std::unordered_set<std::string> retired_trajectory_ids;
    std::unordered_map<std::string, std::uint64_t> highest_plan_versions;
  };

  struct SafetyStateOrder {
    bool initialized{false};
    bool compromised{false};
    std::uint64_t state_id{0U};
    std::uint64_t latch_generation{0U};
    std::int64_t stamp_ns{0};
    std::int64_t receipt_monotonic_ns{0};
    auto_rover::SafetyMode mode{auto_rover::SafetyMode::kBootInhibited};
    std::vector<auto_rover::StopReason> reasons;
  };

  TrackingResult invalidResult(const TrackingInput& input,
                               const std::string& reason);
  auto_rover::MotionReference makeReference(const TrackingInput& input,
                                            double speed_mps,
                                            double curvature_inv_m,
                                            bool route_complete,
                                            bool valid);
  double rateLimitSpeed(double desired_speed_mps,
                        std::int64_t now_monotonic_ns);
  auto_rover::ValidationResult observeRequiredStateOrder(
      const std::string& label, const std::string& source_id,
      std::uint64_t state_id, std::int64_t stamp_ns,
      std::int64_t receipt_monotonic_ns,
      RequiredStateOrder* order);
  auto_rover::ValidationResult observeTrajectoryOrder(
      const auto_rover::Trajectory& trajectory);
  auto_rover::ValidationResult observeSafetyStateOrder(
      const auto_rover::SafetyState& safety,
      std::int64_t receipt_monotonic_ns);

  PurePursuitConfig config_;
  auto_rover::VehicleProfile vehicle_profile_;
  std::string active_trajectory_id_;
  std::size_t progress_index_{0U};
  std::uint64_t next_command_id_{1U};
  std::int64_t last_update_monotonic_ns_{0};
  double last_commanded_speed_mps_{0.0};
  std::string active_chassis_source_id_;
  bool control_enable_capability_observed_{false};
  bool execution_enable_observed_{false};
  RequiredStateOrder ego_order_;
  RequiredStateOrder chassis_order_;
  TrajectoryOrder trajectory_order_;
  SafetyStateOrder safety_order_;
};

}  // namespace auto_rover_control
