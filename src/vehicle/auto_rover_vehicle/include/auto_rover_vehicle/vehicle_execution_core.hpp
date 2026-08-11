#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "auto_rover_core/types.hpp"
#include "auto_rover_safety/command_guard.hpp"
#include "auto_rover_safety/safety_supervisor.hpp"
#include "auto_rover_vehicle/fake_vcu.hpp"
#include "auto_rover_vehicle/vehicle_backend.hpp"
#include "auto_rover_vehicle/vehicle_motion_manager.hpp"

namespace auto_rover_vehicle {

// Execution-bound inputs are deliberately smaller than the planning package's
// offline generation ceiling.  These limits bound copy, validation, and
// identity-history work on the safety-critical vehicle execution path.
constexpr std::size_t kMaximumExecutionTrajectoryPoints = 4096U;
constexpr std::size_t kMaximumExecutionIdentifierBytes = 256U;
constexpr std::size_t kMaximumExecutionIdentityHistoryEntries = 1024U;

struct VehicleExecutionCommonConfig {
  auto_rover::VehicleProfile vehicle_profile;
  auto_rover_safety::SafetySupervisorConfig safety_supervisor;
  std::string authorized_reset_operator_id;
  auto_rover_safety::GuardConfig command_guard;
  VehicleMotionManagerConfig vehicle_manager;
};

struct VehicleExecutionCoreConfig : VehicleExecutionCommonConfig {
  FakeVcuConfig fake_vcu;
};

struct VehicleExecutionServiceResult {
  bool success{false};
  std::string reason;
  auto_rover::SafetyState state;
  BackendDelivery stop_delivery;
};

struct FakeVcuInjectionResult {
  bool success{false};
  std::string reason;
};

struct VehicleExecutionCycleResult {
  bool chassis_available{false};
  auto_rover::ChassisState chassis;
  auto_rover::SafetyState safety;
  bool health_clear{false};
  auto_rover::StopReason health_reason{
      auto_rover::StopReason::kInvalidInput};
  std::string health_diagnostic;
  auto_rover_safety::GuardResult guard;
  auto_rover::VehicleExecutionCommand command;
  BackendDelivery delivery;
  BackendDelivery stop_delivery;
};

auto_rover::ValidationResult validateVehicleExecutionCommonConfig(
    const VehicleExecutionCommonConfig& config);
auto_rover::ValidationResult validateVehicleExecutionCoreConfig(
    const VehicleExecutionCoreConfig& config);

class VehicleExecutionCore {
 public:
  explicit VehicleExecutionCore(VehicleExecutionCoreConfig config);
  VehicleExecutionCore(VehicleExecutionCommonConfig config,
                       std::unique_ptr<VehicleBackend> backend);

  bool initialize(std::int64_t now_monotonic_ns,
                  std::int64_t now_ros_ns);

  auto_rover::ValidationResult updateEgoState(
      const auto_rover::EgoState& value,
      std::int64_t receipt_monotonic_ns);
  auto_rover::ValidationResult updateTrajectory(
      const auto_rover::Trajectory& value,
      std::int64_t receipt_monotonic_ns);
  auto_rover::ValidationResult updateChassisState(
      const auto_rover::ChassisState& value,
      std::int64_t receipt_monotonic_ns);
  void updateMotionReference(const auto_rover::MotionReference& value,
                             std::int64_t receipt_monotonic_ns);

  VehicleExecutionCycleResult cycle(std::int64_t now_monotonic_ns,
                                    std::int64_t now_ros_ns);
  auto_rover::SafetyState currentSafetyState(
      std::int64_t now_ros_ns) const;
  BackendDelivery shutdown(std::int64_t now_monotonic_ns);

  VehicleExecutionServiceResult requestArm(
      const std::string& operator_id, std::uint64_t expected_state_id,
      bool arm, std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns);
  VehicleExecutionServiceResult handleEmergencyStop(
      const auto_rover::EmergencyStop& request,
      std::int64_t now_monotonic_ns, std::int64_t now_ros_ns);
  VehicleExecutionServiceResult resetEmergencyStop(
      const std::string& operator_id, std::uint64_t latch_generation,
      bool conditions_cleared_acknowledged,
      std::int64_t now_monotonic_ns, std::int64_t now_ros_ns);

  FakeVcuInjectionResult setFakeConnected(
      bool connected, std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns);
  FakeVcuInjectionResult setFakeFaulted(
      bool faulted, std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns);
  FakeVcuInjectionResult setFakeDropFeedback(
      bool drop_feedback, std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns);

  bool executionAuthorized() const;
  bool fakeConnected() const;
  bool fakeControlEnabled() const;
  bool fakeFaulted() const;
  bool fakeDropFeedback() const;

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

  struct HealthResult {
    bool clear{false};
    auto_rover::StopReason reason{auto_rover::StopReason::kInvalidInput};
    std::string diagnostic;
  };

  bool validEventTime(std::int64_t now_monotonic_ns,
                      std::int64_t now_ros_ns) const;
  void recordEventTime(std::int64_t now_monotonic_ns,
                       std::int64_t now_ros_ns = 0);
  BackendDelivery revokeExecution(std::int64_t now_monotonic_ns);
  bool synchronizeBackendConnection(const BackendHealth& health,
                                    std::int64_t now_monotonic_ns);
  HealthResult evaluateHealth(std::int64_t now_monotonic_ns,
                              std::int64_t now_ros_ns) const;
  VehicleExecutionServiceResult serviceResult(
      const auto_rover_safety::SafetyTransition& transition,
      BackendDelivery stop_delivery = BackendDelivery{}) const;
  FakeVcuInjectionResult rejectInjection(const std::string& reason,
                                         std::int64_t now_monotonic_ns);
  auto_rover::ValidationResult observeRequiredStateOrder(
      const std::string& label, const std::string& source_id,
      std::uint64_t state_id, std::int64_t stamp_ns,
      std::int64_t receipt_monotonic_ns,
      RequiredStateOrder* order);
  auto_rover::ValidationResult observeTrajectoryOrder(
      const auto_rover::Trajectory& trajectory);

  VehicleExecutionCommonConfig config_;
  auto_rover::ValidationResult configuration_validation_;
  auto_rover_safety::SafetySupervisor safety_supervisor_;
  auto_rover_safety::CommandGuard command_guard_;
  VehicleMotionManager vehicle_manager_;
  std::unique_ptr<VehicleBackend> backend_;
  FakeVcu* fake_vcu_{nullptr};
  auto_rover::Received<auto_rover::EgoState> ego_;
  auto_rover::Received<auto_rover::Trajectory> trajectory_;
  auto_rover::Received<auto_rover::MotionReference> motion_;
  auto_rover::Received<auto_rover::ChassisState> chassis_;
  auto_rover::Received<auto_rover::SafetyState> safety_;
  RequiredStateOrder ego_order_;
  RequiredStateOrder chassis_order_;
  TrajectoryOrder trajectory_order_;
  bool initialized_{false};
  bool backend_generation_initialized_{false};
  bool backend_requires_new_generation_{false};
  // A backend exception makes the in-process authorization contract
  // unknowable.  Keep that condition latched for the lifetime of this core;
  // only a process restart may create a fresh backend instance.
  bool backend_exception_inhibited_{false};
  std::uint64_t backend_connection_generation_{0U};
  std::string backend_generation_diagnostic_;
  bool shutdown_called_{false};
  BackendDelivery shutdown_delivery_;
  BackendDelivery last_stop_delivery_;
  std::uint64_t stop_delivery_sequence_{0U};
  std::int64_t last_event_monotonic_ns_{0};
  std::int64_t last_event_ros_ns_{0};
};

}  // namespace auto_rover_vehicle
