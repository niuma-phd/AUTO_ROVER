#pragma once

#include <cstdint>

#include "auto_rover_core/types.hpp"

namespace auto_rover_safety {
struct GuardResult;
}

namespace auto_rover_vehicle {

struct VehicleMotionManagerConfig {
  std::int64_t command_valid_for_ns{0};
};

auto_rover::ValidationResult validateVehicleMotionManagerConfig(
    const VehicleMotionManagerConfig& config,
    const auto_rover::VehicleProfile& vehicle_profile);

class VehicleMotionManager {
 public:
  VehicleMotionManager(VehicleMotionManagerConfig config,
                       auto_rover::VehicleProfile vehicle_profile);

  void setBackendConnected(bool connected,
                           std::int64_t now_monotonic_ns);
  bool acknowledgeOperatorArm(std::uint64_t authorization_id,
                              std::int64_t now_monotonic_ns);
  void revokeAuthorization(std::int64_t now_monotonic_ns);
  auto_rover::VehicleExecutionCommand makeCommand(
      const auto_rover_safety::GuardResult& guarded,
      std::int64_t now_monotonic_ns);

  bool backendConnected() const;
  bool operatorArmed() const;
  std::uint64_t lastSequenceId() const;

 private:
  auto_rover::VehicleExecutionCommand makeBaseCommand(
      std::int64_t now_monotonic_ns);
  auto_rover::VehicleExecutionCommand makeStopCommand(
      std::int64_t now_monotonic_ns, auto_rover::StopReason reason,
      bool record_delivered_zero);
  void clearAuthorization();
  void recordCommandedSpeed(double speed_mps,
                            std::int64_t now_monotonic_ns);

  VehicleMotionManagerConfig config_;
  auto_rover::VehicleProfile vehicle_profile_;
  auto_rover::ValidationResult configuration_validation_;
  auto_rover::ValidationResult profile_validation_;
  bool backend_connected_{false};
  bool operator_armed_{false};
  std::uint64_t highest_authorization_id_{0U};
  std::uint64_t last_sequence_id_{0U};
  std::int64_t last_event_monotonic_ns_{0};
  bool commanded_speed_known_{false};
  double last_commanded_speed_mps_{0.0};
  std::int64_t last_commanded_speed_ns_{0};
};

}  // namespace auto_rover_vehicle
