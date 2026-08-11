#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "auto_rover_vcu_wheeltec_serial/adapter.hpp"
#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"

namespace auto_rover {
namespace wheeltec_serial {

// The runtime is deliberately single-threaded.  Every public operation is one
// bounded worker transaction; a ROS wrapper may schedule it from a steady
// timer or a dedicated worker without putting ROS APIs in this library.
struct RuntimeConfig {
  AdapterConfig adapter{};
  std::int64_t read_timeout_ns{1000000};
  std::int64_t maximum_feedback_age_ns{150000000};
  std::int64_t drain_read_timeout_ns{1000000};
  std::uint32_t maximum_drain_reads{64U};
  std::uint32_t control_allowed_receipts_required{5U};
  std::int64_t control_allowed_minimum_span_ns{200000000};
  std::int64_t maximum_normal_write_gap_ns{100000000};
  std::int64_t maximum_step_duration_ns{50000000};

  // A disabled runtime is a valid, safely inhibited configuration and touches
  // no transport.  The remaining gates are required only when runtime_enabled
  // requests real actuation.
  bool runtime_enabled{false};
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  bool actuation_opt_in{false};
  bool readiness_gate_passed{false};
  bool external_or_durable_estop_strategy_approved{false};
};

struct RuntimeOperations {
  // Called after every transport operation so a read receipt and write
  // completion are never stamped with the worker-step entry time.
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<bool(std::int64_t)> wait_until_monotonic_ns;
};

enum class RuntimePhase : std::uint8_t {
  kDisabled = 0,
  kDetached,
  kStartupZero,
  kDrainBacklog,
  kAwaitingFeedback,
  kReady,
  kTerminalDisconnected,
  kTerminalFault,
  kShutdown,
};

struct NormalizedWheeltecFeedback {
  double signed_speed_mps{0.0};
  double wheel_derived_yaw_rate_radps{0.0};
  double supply_voltage_v{0.0};
  std::int64_t receipt_monotonic_ns{0};

  bool gear_available{false};
  bool control_state_available{false};
  bool fault_state_available{false};
  bool controller_ack_available{false};
  bool source_time_available{false};
};

struct RuntimeHealth {
  bool configuration_valid{false};
  bool enabled{false};
  bool connected{false};
  bool startup_zero_completed{false};
  bool backlog_drained{false};
  bool feedback_fresh{false};
  bool feedback_recovery_complete{false};
  bool actuation_permitted{false};
  bool authorization_active{false};
  bool delivery_unconfirmed{false};
  bool step_overrun{false};
  std::uint64_t connection_generation{0U};
  std::uint64_t highest_observed_sequence{0U};
  std::uint64_t highest_authorization_id{0U};
  std::int64_t last_feedback_receipt_monotonic_ns{0};
  std::int64_t last_successful_write_completion_ns{0};
  std::int64_t last_step_elapsed_ns{0};
  RuntimePhase phase{RuntimePhase::kDetached};
  std::string diagnostic;
};

struct RuntimeStepResult {
  RuntimePhase phase{RuntimePhase::kDetached};
  AdapterCycleResult cycle{};
  IoResult read{};
  bool feedback_available{false};
  NormalizedWheeltecFeedback feedback{};
  std::size_t valid_frames_in_read{0U};
  std::size_t feedback_receipts_in_read{0U};
  bool inhibit_observed_in_read{false};
  bool invalid_composite_stop_observed_in_read{false};
  bool feedback_watchdog_expired{false};
  bool delivery_unconfirmed{false};
  bool step_overrun{false};
  std::int64_t step_started_monotonic_ns{0};
  std::int64_t step_completed_monotonic_ns{0};
  std::int64_t step_elapsed_ns{0};
};

struct RuntimeDelivery {
  SubmitResult submission{};
  AdapterCycleResult cycle{};
  bool delivered{false};
  bool motion_accepted{false};
  bool delivery_unconfirmed{false};
  bool stop_attempted{false};
  bool controller_ack_available{false};
  bool controller_acknowledged{false};
  bool command_watchdog_expired{false};
  std::int64_t operation_started_monotonic_ns{0};
  std::int64_t operation_completed_monotonic_ns{0};
  std::int64_t operation_elapsed_ns{0};
};

bool runtimeConfigIsStructurallyValid(const RuntimeConfig& config);
bool runtimeActuationGatesAreSatisfied(const RuntimeConfig& config);

class WheeltecSerialRuntime {
 public:
  explicit WheeltecSerialRuntime(
      const RuntimeConfig& config,
      const RuntimeOperations& operations = RuntimeOperations{});

  // Remount is explicit.  A terminal session never reopens a path itself; the
  // owner supplies a newly opened transport with a strictly newer generation.
  // Sequence and authorization high-water marks survive the remount.
  bool attachTransport(ByteTransport* transport,
                       std::int64_t now_monotonic_ns);

  RuntimeStepResult step(std::int64_t now_monotonic_ns);
  bool setAuthorization(bool enabled, std::uint64_t authorization_id,
                        std::int64_t now_monotonic_ns);
  RuntimeDelivery deliver(const VehicleExecutionCommand& command,
                          std::int64_t receipt_monotonic_ns);
  RuntimeDelivery revokeAndStop(std::int64_t now_monotonic_ns);
  RuntimeDelivery shutdown(std::int64_t now_monotonic_ns);

  RuntimeHealth health(std::int64_t now_monotonic_ns) const;
  RuntimePhase phase() const { return phase_; }
  bool configurationValid() const { return configuration_valid_; }
  const ParserStatistics& parserStatistics() const {
    return parser_.statistics();
  }

 private:
  bool observeCallerTime(std::int64_t now_monotonic_ns);
  std::int64_t clockNow() const;
  bool synchronizeTransport();
  void resetSessionEvidence();
  void enterDisconnected(const std::string& diagnostic);
  void enterFault(const std::string& diagnostic,
                  bool delivery_unconfirmed);
  void revokeAuthorization(std::int64_t now_monotonic_ns);
  bool feedbackIsFresh(std::int64_t now_monotonic_ns) const;
  bool feedbackRecoveryComplete() const;
  bool normalWriteGapExpired(std::int64_t now_monotonic_ns) const;
  void noteCycleCompletion(const AdapterCycleResult& cycle,
                           std::int64_t completed_monotonic_ns);
  RuntimeStepResult completeStep(RuntimeStepResult result,
                                 std::int64_t completed_monotonic_ns);
  RuntimeStepResult stepStartupZero(std::int64_t started_monotonic_ns);
  RuntimeStepResult stepDrain(std::int64_t started_monotonic_ns);
  RuntimeStepResult stepRunning(std::int64_t started_monotonic_ns);
  RuntimeDelivery cycleForDelivery(bool motion_requested,
                                   std::int64_t started_monotonic_ns);
  RuntimeDelivery stopNow(std::int64_t now_monotonic_ns,
                          bool enter_shutdown);

  RuntimeConfig config_;
  RuntimeOperations operations_;
  bool configuration_valid_{false};
  RuntimePhase phase_{RuntimePhase::kDetached};
  ByteTransport* transport_{nullptr};
  std::unique_ptr<WheeltecSerialAdapter> adapter_;
  FeedbackStreamParser parser_;
  std::uint64_t connection_generation_{0U};
  std::uint64_t highest_connection_generation_{0U};
  std::uint64_t highest_observed_sequence_{0U};
  std::uint64_t highest_authorization_id_{0U};
  // Public callers may legitimately reuse one cycle timestamp across
  // poll -> revoke -> deliver.  Keep that ordering separate from the steady
  // clock sampled after transport I/O, which can advance within that cycle.
  std::int64_t last_caller_monotonic_ns_{0};
  std::int64_t last_observed_monotonic_ns_{0};
  std::int64_t last_feedback_receipt_monotonic_ns_{0};
  std::int64_t first_allowed_receipt_monotonic_ns_{0};
  std::int64_t last_allowed_receipt_monotonic_ns_{0};
  std::uint32_t consecutive_allowed_receipts_{0U};
  std::int64_t last_successful_write_completion_ns_{0};
  std::uint32_t drain_reads_{0U};
  bool startup_zero_completed_{false};
  bool backlog_drained_{false};
  bool local_inhibit_{true};
  bool delivery_unconfirmed_{false};
  bool write_stream_poisoned_{false};
  bool step_overrun_{false};
  std::int64_t last_step_elapsed_ns_{0};
  NormalizedWheeltecFeedback latest_feedback_{};
  std::string diagnostic_;
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
