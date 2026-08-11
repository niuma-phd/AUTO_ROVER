#pragma once

#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"

namespace auto_rover_vehicle {

// BackendHealth contains only protocol-independent facts needed by vehicle
// execution.  In particular, actuation_permitted is a current backend inhibit
// gate; it is not a fabricated ChassisState control-enable or fault value.
struct BackendHealth {
  bool configuration_valid{false};
  bool connected{false};
  bool actuation_permitted{false};
  bool feedback_fresh{false};
  bool authorization_active{false};
  bool delivery_unconfirmed{false};
  std::uint64_t connection_generation{0U};
  auto_rover::StopReason reason{auto_rover::StopReason::kInvalidInput};
  std::string diagnostic;
};

// A delivered result means that the selected backend completed its local
// delivery boundary.  It does not imply a controller ACK unless both ACK
// fields explicitly say so.
struct BackendDelivery {
  bool delivered{false};
  bool motion_accepted{false};
  bool delivery_unconfirmed{false};
  bool stop_attempted{false};
  bool controller_ack_available{false};
  bool controller_acknowledged{false};
  auto_rover::StopReason reason{auto_rover::StopReason::kInvalidInput};
  std::string diagnostic;
};

// At most one normalized latest-value sample is returned per poll.  Its local
// monotonic receipt is carried separately because a backend must never invent
// a VCU source timestamp.  Both operation-completion clocks are required even
// when no sample is available: a bounded backend poll may perform I/O and move
// past receiver-validity or command deadlines after the execution-cycle entry.
struct BackendFeedback {
  bool available{false};
  bool watchdog_expired{false};
  auto_rover::ChassisState state;
  std::int64_t receipt_monotonic_ns{0};
  std::int64_t operation_completed_monotonic_ns{0};
  std::int64_t operation_completed_ros_ns{0};
  BackendHealth health;
};

class VehicleBackend {
 public:
  virtual ~VehicleBackend() = default;

  virtual auto_rover::ValidationResult validate(
      const auto_rover::VehicleProfile& vehicle_profile) const = 0;
  virtual bool initialize(std::int64_t now_monotonic_ns,
                          std::int64_t now_ros_ns) = 0;
  virtual BackendFeedback poll(std::int64_t now_monotonic_ns,
                               std::int64_t now_ros_ns) = 0;
  virtual bool setAuthorization(bool enabled,
                                std::uint64_t authorization_id,
                                std::int64_t now_monotonic_ns) = 0;
  virtual BackendDelivery deliver(
      const auto_rover::VehicleExecutionCommand& command,
      std::int64_t receipt_monotonic_ns) = 0;
  virtual BackendDelivery revokeAndStop(
      std::int64_t now_monotonic_ns) = 0;
  virtual BackendHealth health() const = 0;
};

}  // namespace auto_rover_vehicle
