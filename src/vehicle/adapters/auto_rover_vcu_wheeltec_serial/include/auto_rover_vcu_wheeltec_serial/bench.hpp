#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

namespace auto_rover {
namespace wheeltec_serial {

constexpr const char* kBenchOperatorConfirmationToken =
    "I_CONFIRM_RAISED_WHEELTEC_BENCH_ACTUATION";
constexpr const char* kPassiveEvidenceTokenPrefix =
    "passive-capture-sha256:";
constexpr const char* kExactZeroEvidenceTokenPrefix =
    "exact-zero-sha256:";
constexpr double kBenchMaximumForwardSpeedMps = 0.50;
constexpr double kBenchLongitudinalAccelerationMps2 = 0.20;
constexpr std::int16_t kBenchMaximumForwardWireSpeed = 500;
constexpr std::int64_t kBenchWireUnitAccelerationPeriodNs = 5000000;

// The candidate protocol represents forward speed in 0.001 m/s wire units.
// At 0.20 m/s^2, changing by one wire unit requires at least 5 ms.  This
// integer state machine is shared by the command generator and the independent
// transport-boundary check so neither path relies on a floating-point margin.
class BenchWireAccelerationEnvelope {
 public:
  bool transitionAllowed(std::int16_t next_wire_speed,
                         std::int64_t attempt_monotonic_ns) const;
  bool coupleToward(std::int16_t desired_wire_speed,
                    std::int64_t request_monotonic_ns,
                    std::int16_t* coupled_wire_speed) const;
  bool noteSuccessfulNormalWrite(std::int16_t wire_speed,
                                 std::int64_t completed_monotonic_ns);

  bool hasBaseline() const { return has_baseline_; }
  std::int16_t lastSuccessfulWireSpeed() const {
    return last_successful_wire_speed_;
  }
  std::int64_t lastSuccessfulCompletionNs() const {
    return last_successful_completion_ns_;
  }

 private:
  bool has_baseline_{false};
  std::int16_t last_successful_wire_speed_{0};
  std::int64_t last_successful_completion_ns_{0};
};

enum class BenchMode : std::uint8_t {
  kDisabled = 0,
  kExactZero,
  kStraightRamp,
};

enum class BenchStatus : std::uint8_t {
  kCompleted = 0,
  kInvalidConfiguration,
  kAuthorizationDenied,
  kEvidenceMissing,
  kClockInvalid,
  kDisconnected,
  kFeedbackMissing,
  kFeedbackInvalid,
  kFeedbackLimitViolation,
  kStandstillViolation,
  kAdapterRejected,
  kWriteFailed,
  kRecordError,
  kZeroHostWriteFailed,
  kInterrupted,
  kSessionDeadlineExceeded,
};

struct BenchConfig {
  BenchMode mode{BenchMode::kDisabled};
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  bool actuation_opt_in{false};
  bool allow_raised_wheel_start_asymmetry{false};
  bool record_uncalibrated_motion_feedback{false};
  std::string operator_confirmation_token;
  std::string passive_evidence_token;
  std::string exact_zero_evidence_token;
  double target_speed_mps{0.0};
  std::int64_t exact_zero_duration_ns{0};
  std::int64_t hold_duration_ns{0};
  std::int64_t read_timeout_ns{20000000};
  std::int64_t maximum_feedback_age_ns{150000000};
  std::int64_t initial_feedback_deadline_ns{1000000000};
  std::int64_t command_valid_for_ns{100000000};
};

struct BenchOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<bool(std::int64_t)> wait_until_monotonic_ns;
  std::function<bool(const std::string&)> record_json_line;
  std::function<bool()> stop_requested;
};

struct BenchStatistics {
  std::uint64_t read_calls{0U};
  std::uint64_t raw_rx_bytes{0U};
  std::uint64_t valid_feedback_frames{0U};
  std::uint64_t tx_attempts{0U};
  std::uint64_t tx_host_writes_completed{0U};
  std::uint64_t zero_frame_host_writes_completed{0U};
  std::uint64_t nonzero_frame_host_writes_completed{0U};
  std::uint64_t motion_commands_submitted{0U};
  std::uint32_t maximum_feedback_recovery_run{0U};
  double first_nonzero_speed_mps{0.0};
  double maximum_commanded_speed_mps{0.0};
  std::int64_t started_monotonic_ns{0};
  std::int64_t ended_monotonic_ns{0};
};

struct BenchResult {
  BenchStatus status{BenchStatus::kInvalidConfiguration};
  BenchStatistics statistics{};
  bool authorization_revoked{true};
  bool zero_host_write_completed{false};
  bool delivery_unconfirmed{false};

  bool completed() const { return status == BenchStatus::kCompleted; }
};

const char* benchModeName(BenchMode mode);
const char* benchStatusName(BenchStatus status);
bool benchConfigIsValid(const BenchConfig& config);
BenchStatus benchPreflightStatus(const BenchConfig& config);
std::string benchSummaryRecordJson(const BenchResult& result);

class WheeltecBenchSession {
 public:
  BenchResult run(ByteTransport* transport, const BenchConfig& config,
                  const BenchOperations& operations);
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
