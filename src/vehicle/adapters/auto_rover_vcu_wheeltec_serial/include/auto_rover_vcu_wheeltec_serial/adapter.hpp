#pragma once

#include <cstddef>
#include <cstdint>

#include "auto_rover_core/types.hpp"
#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

namespace auto_rover {
namespace wheeltec_serial {

struct AdapterConfig {
  CodecLimits codec_limits{};
  std::int64_t max_command_age_ns{100000000};
  std::uint32_t fresh_commands_required{3U};
  std::int64_t zero_retry_interval_ns{20000000};
  std::uint32_t max_zero_write_attempts{3U};
  std::int64_t write_timeout_ns{10000000};
};

enum class SubmissionStatus : std::uint8_t {
  kAccepted = 0,
  kRecoveryPending,
  kDisconnected,
  kNotAuthorized,
  kSequenceInvalid,
  kTimestampInvalid,
  kStale,
  kCodecRejected,
  kConfigurationInvalid,
  kPredatesAuthorization,
};

struct SubmitResult {
  SubmissionStatus status{SubmissionStatus::kConfigurationInvalid};
  CodecError codec_error{CodecError::kNone};
};

enum class AdapterCycleAction : std::uint8_t {
  kNoAction = 0,
  kMotionWritten,
  kMotionWriteFailed,
  kZeroWritten,
  kZeroWriteFailed,
  kZeroRetryPending,
  kZeroRetriesExhausted,
  kDisconnected,
  kConfigurationInvalid,
  kClockInvalid,
};

struct AdapterCycleResult {
  AdapterCycleAction action{AdapterCycleAction::kNoAction};
  TransportStatus transport_status{TransportStatus::kOk};
  bool delivery_unconfirmed{false};
  std::uint32_t zero_attempts{0U};
  std::size_t transferred_bytes{0U};
  // True means the transport may have emitted an unknown prefix.  No further
  // frame may be appended on this connection generation.
  bool write_stream_poisoned{false};
  std::int64_t attempt_deadline_monotonic_ns{0};
};

class WheeltecSerialAdapter {
 public:
  WheeltecSerialAdapter(const AdapterConfig& config,
                        ByteTransport* transport);

  bool setAuthorization(bool protocol_enabled, bool operator_armed,
                        std::uint64_t authorization_id,
                        std::int64_t authorization_monotonic_ns);
  SubmitResult submit(const VehicleExecutionCommand& command,
                      std::int64_t receipt_monotonic_ns);
  // Used by the formal runtime to make a recovery-pending submission refresh
  // an exact-zero host write without clearing its authorization epoch or
  // accumulated fresh-command evidence.
  void requireRecoveryZeroWrite();
  AdapterCycleResult cycle(std::int64_t now_monotonic_ns);

  bool isAuthorized() const;
  bool recoveryComplete() const;
  std::uint32_t consecutiveFreshCommands() const;
  std::uint64_t highestObservedSequence() const;

 private:
  bool synchronizeConnection();
  void invalidateAuthorizationAndMotion();
  void clearMotionState();
  void requireNewZeroEpisode();
  bool motionIsFresh(std::int64_t now_monotonic_ns) const;
  AdapterCycleResult cycleZero(std::int64_t now_monotonic_ns);
  std::int64_t writeDeadline(std::int64_t now_monotonic_ns,
                             std::int64_t command_deadline_ns) const;

  AdapterConfig config_;
  ByteTransport* transport_{nullptr};
  bool configuration_valid_{false};
  bool connection_known_{false};
  std::uint64_t connection_generation_{0U};
  bool protocol_enabled_{false};
  bool operator_armed_{false};
  std::uint64_t highest_authorization_id_{0U};
  std::uint64_t authorized_connection_generation_{0U};
  std::int64_t authorization_monotonic_ns_{0};
  std::int64_t last_observed_monotonic_ns_{0};
  std::uint64_t highest_sequence_{0U};
  std::uint32_t consecutive_fresh_commands_{0U};
  bool latest_command_valid_{false};
  VehicleExecutionCommand latest_command_{};
  CommandFrame latest_frame_{};
  bool zero_episode_latched_{false};
  bool zero_retry_active_{false};
  bool zero_retries_exhausted_{false};
  std::uint32_t zero_attempts_{0U};
  std::int64_t next_zero_attempt_ns_{0};
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
