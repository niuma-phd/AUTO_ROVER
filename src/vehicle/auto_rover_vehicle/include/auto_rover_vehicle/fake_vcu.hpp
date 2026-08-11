#pragma once

#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"
#include "auto_rover_vehicle/vehicle_backend.hpp"

namespace auto_rover_vehicle {

struct FakeVcuConfig {
  std::string process_generation_id;
  std::int64_t command_watchdog_ns{0};
  std::int64_t feedback_period_ns{0};
  std::uint32_t fresh_recovery_count{0U};
  double max_accel_mps2{0.0};
};

auto_rover::ValidationResult validateFakeVcuConfig(
    const FakeVcuConfig& config,
    const auto_rover::VehicleProfile& vehicle_profile);

using FakeVcuReceiveResult = BackendDelivery;
using FakeVcuStepResult = BackendFeedback;

class FakeVcu final : public VehicleBackend {
 public:
  FakeVcu(FakeVcuConfig config,
          auto_rover::VehicleProfile vehicle_profile);

  auto_rover::ValidationResult validate(
      const auto_rover::VehicleProfile& vehicle_profile) const override;
  bool initialize(std::int64_t now_monotonic_ns,
                  std::int64_t now_ros_ns) override;
  BackendFeedback poll(std::int64_t now_monotonic_ns,
                       std::int64_t now_ros_ns) override;
  bool setAuthorization(bool enabled, std::uint64_t authorization_id,
                        std::int64_t now_monotonic_ns) override;
  BackendDelivery deliver(
      const auto_rover::VehicleExecutionCommand& command,
      std::int64_t receipt_monotonic_ns) override;
  BackendDelivery revokeAndStop(
      std::int64_t now_monotonic_ns) override;
  BackendHealth health() const override;

  void setConnected(bool connected, std::int64_t now_monotonic_ns);
  void setControlEnabled(bool enabled, std::int64_t now_monotonic_ns);
  void setFaulted(bool faulted, std::int64_t now_monotonic_ns);
  void setDropFeedback(bool drop_feedback);

  FakeVcuReceiveResult receive(
      const auto_rover::VehicleExecutionCommand& command,
      std::int64_t receipt_monotonic_ns);
  FakeVcuStepResult step(std::int64_t now_monotonic_ns,
                         std::int64_t now_ros_ns);

  bool connected() const;
  bool controlEnabled() const;
  bool faulted() const;
  bool dropFeedback() const;
  std::uint32_t consecutiveFreshCommands() const;
  std::uint64_t highestObservedSequence() const;
  double measuredSpeedMps() const;

 private:
  bool validEventTime(std::int64_t now_monotonic_ns) const;
  void advanceDynamics(std::int64_t now_monotonic_ns);
  void integrateDynamicsTo(std::int64_t end_monotonic_ns);
  void expireOutputWatchdog();
  std::int64_t outputExpiryMonotonicNs() const;
  void clearMotionState(bool clear_control_enable);
  FakeVcuReceiveResult rejectDelivered(auto_rover::StopReason reason,
                                       const std::string& diagnostic,
                                       bool clear_control_enable);

  FakeVcuConfig config_;
  auto_rover::VehicleProfile vehicle_profile_;
  auto_rover::ValidationResult configuration_validation_;
  auto_rover::ValidationResult profile_validation_;
  bool connected_{false};
  std::uint64_t connection_generation_{0U};
  std::uint64_t highest_authorization_id_{0U};
  bool control_enabled_{false};
  bool faulted_{false};
  bool drop_feedback_{false};
  bool watchdog_expired_latched_{false};
  bool control_watchdog_active_{false};
  std::uint32_t consecutive_fresh_commands_{0U};
  std::uint64_t highest_sequence_{0U};
  bool latest_command_valid_{false};
  auto_rover::VehicleExecutionCommand latest_command_;
  std::int64_t control_watchdog_refresh_ns_{0};
  std::int64_t last_event_monotonic_ns_{0};
  std::int64_t last_dynamics_monotonic_ns_{0};
  std::int64_t last_feedback_monotonic_ns_{0};
  double target_speed_mps_{0.0};
  double target_curvature_inv_m_{0.0};
  double measured_speed_mps_{0.0};
  std::uint64_t feedback_state_id_{0U};
};

}  // namespace auto_rover_vehicle
