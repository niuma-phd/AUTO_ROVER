#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

namespace auto_rover {
namespace wheeltec_serial {

constexpr const char* kRawProfileOperatorConfirmationToken =
    "I_CONFIRM_RAISED_WHEELTEC_RAW_PROFILE_ACTUATION";
constexpr double kRawProfileMaximumCommandSpeedMps = 6.0;
constexpr std::int16_t kRawProfileMaximumCommandForwardWire = 6000;
constexpr double kRawProfileAccelerationMps2 = 0.20;
constexpr double kRawProfileMinimumTurnRadiusM = 0.95;
constexpr double kRawProfileRearTrackM = 0.322;
constexpr std::int64_t kRawProfileTargetHoldDurationNs = 60000000000LL;
constexpr std::int64_t kRawProfileMaximumSessionDurationNs =
    130000000000LL;
constexpr std::int64_t kRawProfileWireUnitAccelerationPeriodNs = 5000000;
constexpr std::size_t kRawProfileEvidenceBufferCapacityBytes =
    32U * 1024U * 1024U;

class RawProfileEvidenceBuffer {
 public:
  explicit RawProfileEvidenceBuffer(
      std::size_t capacity_bytes =
          kRawProfileEvidenceBufferCapacityBytes)
      : capacity_bytes_(capacity_bytes) {}

  bool prepare();
  bool appendLine(const std::string& line);

  const std::string& contents() const { return data_; }
  std::size_t capacityBytes() const { return capacity_bytes_; }
  bool failed() const { return failed_; }

 private:
  std::size_t capacity_bytes_{0U};
  std::string data_;
  bool prepared_{false};
  bool failed_{false};
};

enum class RawProfile : std::uint8_t {
  kDisabled = 0,
  kStraight0p5,
  kStraight1p0,
  kStraight1p5,
  kStraight2p0,
  kLeft0p5Radius2p0,
  kRight0p5Radius2p0,
  kLeft0p5Radius0p95,
  kRight0p5Radius0p95,
  kStraight2p5,
  kStraight3p0,
  kStraight3p5,
  kStraight4p0,
  kStraight4p5,
  kStraight5p0,
  kStraight5p5,
  kStraight6p0,
  kLeftOuter0p5Radius2p0,
  kLeftOuter1p0Radius2p0,
  kLeftOuter1p5Radius2p0,
  kLeftOuter2p0Radius2p0,
  kLeftOuter2p5Radius2p0,
  kLeftOuter3p0Radius2p0,
  kLeftOuter3p5Radius2p0,
  kLeftOuter4p0Radius2p0,
  kLeftOuter4p5Radius2p0,
  kLeftOuter5p0Radius2p0,
  kLeftOuter5p5Radius2p0,
  kLeftOuter6p0Radius2p0,
  kRightOuter0p5Radius2p0,
  kRightOuter1p0Radius2p0,
  kRightOuter1p5Radius2p0,
  kRightOuter2p0Radius2p0,
  kRightOuter2p5Radius2p0,
  kRightOuter3p0Radius2p0,
  kRightOuter3p5Radius2p0,
  kRightOuter4p0Radius2p0,
  kRightOuter4p5Radius2p0,
  kRightOuter5p0Radius2p0,
  kRightOuter5p5Radius2p0,
  kRightOuter6p0Radius2p0,
  kLeftOuter0p5Radius0p95,
  kLeftOuter1p0Radius0p95,
  kLeftOuter1p5Radius0p95,
  kLeftOuter2p0Radius0p95,
  kLeftOuter2p5Radius0p95,
  kLeftOuter3p0Radius0p95,
  kLeftOuter3p5Radius0p95,
  kLeftOuter4p0Radius0p95,
  kLeftOuter4p5Radius0p95,
  kLeftOuter5p0Radius0p95,
  kLeftOuter5p5Radius0p95,
  kLeftOuter6p0Radius0p95,
  kRightOuter0p5Radius0p95,
  kRightOuter1p0Radius0p95,
  kRightOuter1p5Radius0p95,
  kRightOuter2p0Radius0p95,
  kRightOuter2p5Radius0p95,
  kRightOuter3p0Radius0p95,
  kRightOuter3p5Radius0p95,
  kRightOuter4p0Radius0p95,
  kRightOuter4p5Radius0p95,
  kRightOuter5p0Radius0p95,
  kRightOuter5p5Radius0p95,
  kRightOuter6p0Radius0p95,
};

constexpr std::size_t kRawProfileFixedProfileCount = 64U;

enum class RawProfileSpeedTierSemantics : std::uint8_t {
  kStraightCenterEqualsRearWheels = 0,
  kLegacyCenterForwardCommand,
  kOuterRearWheelCommandUpperLimit,
};

struct RawProfileDefinition {
  RawProfile profile{RawProfile::kDisabled};
  const char* name{"disabled"};
  RawProfileSpeedTierSemantics speed_tier_semantics{
      RawProfileSpeedTierSemantics::kStraightCenterEqualsRearWheels};
  std::int16_t speed_tier_wire{0};
  std::int16_t target_forward_wire{0};
  std::int16_t target_yaw_wire{0};
  std::int64_t target_outer_wire_milli{0};
  std::int64_t target_inner_wire_milli{0};
  double speed_tier_mps{0.0};
  double target_speed_mps{0.0};
  double target_outer_command_mps{0.0};
  double target_inner_command_mps{0.0};
  double curvature_inv_m{0.0};
  double turn_radius_m{0.0};
  std::int64_t hold_duration_ns{0};
};

const char* rawProfileName(RawProfile profile);
const char* rawProfileSpeedTierSemanticsName(
    RawProfileSpeedTierSemantics semantics);
bool rawProfileAt(std::size_t index, RawProfile* profile);
bool rawProfileFromName(const std::string& name, RawProfile* profile);
bool rawProfileDefinition(RawProfile profile,
                          RawProfileDefinition* definition);

enum class RawProfileEncodeStatus : std::uint8_t {
  kOk = 0,
  kInvalidProfile,
  kForwardWireOutOfRange,
  kYawWireOutOfRange,
};

struct RawProfileEncodeResult {
  RawProfileEncodeStatus status{RawProfileEncodeStatus::kInvalidProfile};
  CommandFrame frame{};
  std::int16_t forward_wire{0};
  std::int16_t yaw_wire{0};

  bool ok() const { return status == RawProfileEncodeStatus::kOk; }
};

RawProfileEncodeResult encodeRawProfileCommand(
    RawProfile profile, std::int16_t forward_wire);

class RawProfileWireAccelerationEnvelope {
 public:
  bool transitionAllowed(std::int16_t next_forward_wire,
                         std::int64_t attempt_monotonic_ns) const;
  bool coupleToward(std::int16_t desired_forward_wire,
                    std::int64_t request_monotonic_ns,
                    std::int16_t* coupled_forward_wire) const;
  bool noteSuccessfulNormalWrite(std::int16_t forward_wire,
                                 std::int64_t completed_monotonic_ns);

  bool hasBaseline() const { return has_baseline_; }
  std::int16_t lastSuccessfulForwardWire() const {
    return last_successful_forward_wire_;
  }
  std::int64_t lastSuccessfulCompletionNs() const {
    return last_successful_completion_ns_;
  }

 private:
  bool has_baseline_{false};
  std::int16_t last_successful_forward_wire_{0};
  std::int64_t last_successful_completion_ns_{0};
};

enum class RawProfileStatus : std::uint8_t {
  kCompleted = 0,
  kInvalidConfiguration,
  kAuthorizationDenied,
  kEvidenceMissing,
  kClockInvalid,
  kDisconnected,
  kFeedbackMissing,
  kFeedbackInvalid,
  kControlInhibited,
  kCommandWatchdogExpired,
  kWriteFailed,
  kRecordError,
  kInterrupted,
  kSessionDeadlineExceeded,
  kZeroHostWriteFailed,
};

const char* rawProfileStatusName(RawProfileStatus status);

struct RawProfileConfig {
  RawProfile profile{RawProfile::kDisabled};
  bool unverified_protocol_acknowledged{false};
  bool physical_device_opt_in{false};
  bool actuation_opt_in{false};
  bool raw_raised_bench_opt_in{false};
  std::string operator_confirmation_token;
  std::string passive_evidence_token;
  std::string exact_zero_evidence_token;
  std::int64_t read_timeout_ns{20000000};
  std::int64_t maximum_feedback_age_ns{150000000};
  std::int64_t initial_feedback_deadline_ns{1000000000};
  std::int64_t maximum_session_duration_ns{
      kRawProfileMaximumSessionDurationNs};
};

struct RawProfileOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<bool(std::int64_t)> wait_until_monotonic_ns;
  std::function<bool(const std::string&)> record_json_line;
  std::function<bool()> stop_requested;
};

struct RawProfileImmediateZeroResult {
  std::uint32_t attempts{0U};
  bool zero_host_write_completed{false};
  bool delivery_unconfirmed{false};
  TransportStatus terminal_transport_status{TransportStatus::kDisabled};
};

RawProfileImmediateZeroResult writeRawProfileImmediateZeroNoRecord(
    ByteTransport* transport, const CommandFrame& preencoded_exact_zero,
    const RawProfileOperations& operations);

struct RawProfileStatistics {
  std::uint64_t read_calls{0U};
  std::uint64_t raw_rx_bytes{0U};
  std::uint64_t valid_feedback_frames{0U};
  std::uint64_t feedback_observation_events{0U};
  std::uint64_t parser_checksum_failures{0U};
  std::uint64_t parser_framing_failures{0U};
  std::uint64_t parser_discarded_bytes{0U};
  std::uint64_t parser_trailing_buffered_bytes{0U};
  std::uint64_t read_timeouts{0U};
  std::uint64_t tx_attempts{0U};
  std::uint64_t tx_host_writes_completed{0U};
  std::uint64_t normal_tx_host_writes_completed{0U};
  std::uint64_t zero_frame_host_writes_completed{0U};
  std::uint64_t nonzero_frame_host_writes_completed{0U};
  std::int16_t maximum_commanded_forward_wire{0};
  std::int64_t started_monotonic_ns{0};
  std::int64_t ended_monotonic_ns{0};
};

struct RawProfileResult {
  RawProfileStatus status{RawProfileStatus::kInvalidConfiguration};
  RawProfileStatistics statistics{};
  bool zero_host_write_completed{false};
  bool delivery_unconfirmed{false};

  bool completed() const { return status == RawProfileStatus::kCompleted; }
};

bool rawProfileConfigIsValid(const RawProfileConfig& config);
RawProfileStatus rawProfilePreflightStatus(const RawProfileConfig& config);
std::string rawProfileSummaryRecordJson(const RawProfileResult& result);

class WheeltecRawProfileSession {
 public:
  RawProfileResult run(ByteTransport* transport,
                       const RawProfileConfig& config,
                       const RawProfileOperations& operations);
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
