#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "auto_rover_vcu_wheeltec_serial/runtime.hpp"
#include "auto_rover_vehicle/vehicle_backend.hpp"

namespace auto_rover {
namespace wheeltec_serial {

struct WheeltecVehicleBackendConfig {
  RuntimeConfig runtime{};
  std::string chassis_frame_id{"rear_axle_center"};
  std::string source_id{"wheeltec_serial_unverified"};
};

struct WheeltecVehicleBackendOperations {
  RuntimeOperations runtime{};
  // Required by an enabled backend.  It is sampled after a successful read;
  // no VCU source timestamp exists.
  std::function<std::int64_t()> ros_now_ns;
};

class WheeltecVehicleBackend final
    : public auto_rover_vehicle::VehicleBackend {
 public:
  WheeltecVehicleBackend(
      const WheeltecVehicleBackendConfig& config,
      ByteTransport* transport,
      const WheeltecVehicleBackendOperations& operations =
          WheeltecVehicleBackendOperations{});

  auto_rover::ValidationResult validate(
      const auto_rover::VehicleProfile& vehicle_profile) const override;
  bool initialize(std::int64_t now_monotonic_ns,
                  std::int64_t now_ros_ns) override;
  auto_rover_vehicle::BackendFeedback poll(
      std::int64_t now_monotonic_ns,
      std::int64_t now_ros_ns) override;
  bool setAuthorization(bool enabled, std::uint64_t authorization_id,
                        std::int64_t now_monotonic_ns) override;
  auto_rover_vehicle::BackendDelivery deliver(
      const auto_rover::VehicleExecutionCommand& command,
      std::int64_t receipt_monotonic_ns) override;
  auto_rover_vehicle::BackendDelivery revokeAndStop(
      std::int64_t now_monotonic_ns) override;
  auto_rover_vehicle::BackendHealth health() const override;

  // Two-phase physical activation support.  This only stores a stable
  // already-open transport pointer; it performs no transport or protocol I/O.
  // initialize() remains the owner of runtime attach and startup exact-zero.
  bool setTransportForInitialization(ByteTransport* transport) noexcept;
  bool attachTransport(ByteTransport* transport,
                       std::int64_t now_monotonic_ns);

 private:
  auto_rover_vehicle::BackendHealth mapHealth(
      std::int64_t now_monotonic_ns) const;
  auto_rover_vehicle::BackendDelivery mapDelivery(
      const RuntimeDelivery& delivery) const;
  auto_rover::StopReason runtimeReason(
      const RuntimeHealth& health) const;
  std::int64_t rosNow(std::int64_t fallback_ros_ns) const;

  WheeltecVehicleBackendConfig config_;
  WheeltecVehicleBackendOperations operations_;
  ByteTransport* transport_{nullptr};
  WheeltecSerialRuntime runtime_;
  bool initialized_{false};
  std::uint64_t feedback_state_id_{0U};
  std::int64_t last_poll_monotonic_ns_{0};
  std::int64_t last_poll_ros_ns_{0};
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
