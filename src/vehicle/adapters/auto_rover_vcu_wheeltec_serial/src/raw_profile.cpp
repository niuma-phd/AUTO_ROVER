#include "auto_rover_vcu_wheeltec_serial/raw_profile.hpp"

#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <new>
#include <sstream>
#include <string>
#include <vector>

namespace auto_rover {
namespace wheeltec_serial {

bool RawProfileEvidenceBuffer::prepare() {
  if (prepared_ || failed_ || capacity_bytes_ == 0U ||
      capacity_bytes_ > kRawProfileEvidenceBufferCapacityBytes) {
    failed_ = true;
    return false;
  }
  try {
    data_.reserve(capacity_bytes_);
  } catch (const std::bad_alloc&) {
    failed_ = true;
    return false;
  }
  prepared_ = true;
  return true;
}

bool RawProfileEvidenceBuffer::appendLine(const std::string& line) {
  if (!prepared_ || failed_ || line.size() >= capacity_bytes_ ||
      data_.size() > capacity_bytes_ - line.size() - 1U) {
    failed_ = true;
    return false;
  }
  try {
    data_.append(line);
    data_.push_back('\n');
  } catch (const std::bad_alloc&) {
    failed_ = true;
    return false;
  }
  return true;
}

bool RawProfileWireAccelerationEnvelope::transitionAllowed(
    std::int16_t next_forward_wire,
    std::int64_t attempt_monotonic_ns) const {
  if (next_forward_wire < 0 ||
      next_forward_wire > kRawProfileMaximumCommandForwardWire ||
      attempt_monotonic_ns <= 0) {
    return false;
  }
  if (!has_baseline_) {
    return next_forward_wire == 0;
  }
  if (attempt_monotonic_ns < last_successful_completion_ns_) {
    return false;
  }
  const std::int64_t delta = std::abs(
      static_cast<std::int64_t>(next_forward_wire) -
      static_cast<std::int64_t>(last_successful_forward_wire_));
  return delta * kRawProfileWireUnitAccelerationPeriodNs <=
         attempt_monotonic_ns - last_successful_completion_ns_;
}

bool RawProfileWireAccelerationEnvelope::coupleToward(
    std::int16_t desired_forward_wire,
    std::int64_t request_monotonic_ns,
    std::int16_t* coupled_forward_wire) const {
  if (coupled_forward_wire == nullptr || desired_forward_wire < 0 ||
      desired_forward_wire > kRawProfileMaximumCommandForwardWire ||
      request_monotonic_ns <= 0 ||
      !has_baseline_ ||
      request_monotonic_ns < last_successful_completion_ns_) {
    return false;
  }
  const std::int64_t elapsed_ns =
      request_monotonic_ns - last_successful_completion_ns_;
  const std::int64_t available_delta = std::min<std::int64_t>(
      kRawProfileMaximumCommandForwardWire,
      elapsed_ns / kRawProfileWireUnitAccelerationPeriodNs);
  const std::int64_t previous = last_successful_forward_wire_;
  const std::int64_t desired = desired_forward_wire;
  const std::int64_t lower = std::max<std::int64_t>(
      0, previous - available_delta);
  const std::int64_t upper = std::min<std::int64_t>(
      kRawProfileMaximumCommandForwardWire, previous + available_delta);
  *coupled_forward_wire = static_cast<std::int16_t>(
      std::max(lower, std::min(upper, desired)));
  return true;
}

bool RawProfileWireAccelerationEnvelope::noteSuccessfulNormalWrite(
    std::int16_t forward_wire,
    std::int64_t completed_monotonic_ns) {
  if (forward_wire < 0 ||
      forward_wire > kRawProfileMaximumCommandForwardWire ||
      completed_monotonic_ns <= 0 ||
      (has_baseline_ &&
       completed_monotonic_ns < last_successful_completion_ns_)) {
    return false;
  }
  has_baseline_ = true;
  last_successful_forward_wire_ = forward_wire;
  last_successful_completion_ns_ = completed_monotonic_ns;
  return true;
}

namespace {

constexpr const char* kSchema =
    "auto_rover.wheeltec.raw_profile_capture.v1";
constexpr const char* kPassiveEvidencePrefix = "passive-capture-sha256:";
constexpr const char* kExactZeroEvidencePrefix = "exact-zero-sha256:";
constexpr std::size_t kMaximumEvidenceTokenBytes = 256U;
constexpr std::size_t kReadBufferBytes = 512U;
constexpr std::uint32_t kRequiredFreshFeedbackReceipts = 5U;
constexpr std::int64_t kMinimumPrearmObservationNs = 200000000;
constexpr std::int64_t kCommandPeriodNs = 20000000;
constexpr std::int64_t kMaximumCommandGapNs = 100000000;
constexpr std::int64_t kWriteTimeoutNs = 10000000;
constexpr std::int64_t kDrainReadDeadlineNs = 1000000;
constexpr std::uint32_t kMaximumDrainReads = 64U;
constexpr std::int64_t kPostZeroObservationNs = 2000000000LL;
constexpr std::int64_t kZeroRetryIntervalNs = 20000000;
constexpr std::uint32_t kMaximumZeroWriteAttempts = 3U;
constexpr std::int64_t kHalfRearTrackMilli = 161LL;
constexpr std::int64_t kWireScale = 1000LL;

struct RawProfileCatalogEntry {
  RawProfile profile;
  const char* name;
  std::int16_t speed_tier_wire;
  std::int16_t signed_radius_milli;
  bool outer_tier_limited;
};

const std::array<RawProfileCatalogEntry, kRawProfileFixedProfileCount>
    kRawProfileCatalog = {{
        {RawProfile::kStraight0p5, "straight-0.5", 500, 0, false},
        {RawProfile::kStraight1p0, "straight-1.0", 1000, 0, false},
        {RawProfile::kStraight1p5, "straight-1.5", 1500, 0, false},
        {RawProfile::kStraight2p0, "straight-2.0", 2000, 0, false},
        {RawProfile::kStraight2p5, "straight-2.5", 2500, 0, false},
        {RawProfile::kStraight3p0, "straight-3.0", 3000, 0, false},
        {RawProfile::kStraight3p5, "straight-3.5", 3500, 0, false},
        {RawProfile::kStraight4p0, "straight-4.0", 4000, 0, false},
        {RawProfile::kStraight4p5, "straight-4.5", 4500, 0, false},
        {RawProfile::kStraight5p0, "straight-5.0", 5000, 0, false},
        {RawProfile::kStraight5p5, "straight-5.5", 5500, 0, false},
        {RawProfile::kStraight6p0, "straight-6.0", 6000, 0, false},
        {RawProfile::kLeft0p5Radius2p0, "left-0.5-r2.0", 500, 2000,
         false},
        {RawProfile::kRight0p5Radius2p0, "right-0.5-r2.0", 500, -2000,
         false},
        {RawProfile::kLeft0p5Radius0p95, "left-0.5-r0.95", 500, 950,
         false},
        {RawProfile::kRight0p5Radius0p95, "right-0.5-r0.95", 500, -950,
         false},
        {RawProfile::kLeftOuter0p5Radius2p0, "left-outer0.5-r2.0", 500,
         2000, true},
        {RawProfile::kLeftOuter1p0Radius2p0, "left-outer1.0-r2.0", 1000,
         2000, true},
        {RawProfile::kLeftOuter1p5Radius2p0, "left-outer1.5-r2.0", 1500,
         2000, true},
        {RawProfile::kLeftOuter2p0Radius2p0, "left-outer2.0-r2.0", 2000,
         2000, true},
        {RawProfile::kLeftOuter2p5Radius2p0, "left-outer2.5-r2.0", 2500,
         2000, true},
        {RawProfile::kLeftOuter3p0Radius2p0, "left-outer3.0-r2.0", 3000,
         2000, true},
        {RawProfile::kLeftOuter3p5Radius2p0, "left-outer3.5-r2.0", 3500,
         2000, true},
        {RawProfile::kLeftOuter4p0Radius2p0, "left-outer4.0-r2.0", 4000,
         2000, true},
        {RawProfile::kLeftOuter4p5Radius2p0, "left-outer4.5-r2.0", 4500,
         2000, true},
        {RawProfile::kLeftOuter5p0Radius2p0, "left-outer5.0-r2.0", 5000,
         2000, true},
        {RawProfile::kLeftOuter5p5Radius2p0, "left-outer5.5-r2.0", 5500,
         2000, true},
        {RawProfile::kLeftOuter6p0Radius2p0, "left-outer6.0-r2.0", 6000,
         2000, true},
        {RawProfile::kRightOuter0p5Radius2p0, "right-outer0.5-r2.0", 500,
         -2000, true},
        {RawProfile::kRightOuter1p0Radius2p0, "right-outer1.0-r2.0", 1000,
         -2000, true},
        {RawProfile::kRightOuter1p5Radius2p0, "right-outer1.5-r2.0", 1500,
         -2000, true},
        {RawProfile::kRightOuter2p0Radius2p0, "right-outer2.0-r2.0", 2000,
         -2000, true},
        {RawProfile::kRightOuter2p5Radius2p0, "right-outer2.5-r2.0", 2500,
         -2000, true},
        {RawProfile::kRightOuter3p0Radius2p0, "right-outer3.0-r2.0", 3000,
         -2000, true},
        {RawProfile::kRightOuter3p5Radius2p0, "right-outer3.5-r2.0", 3500,
         -2000, true},
        {RawProfile::kRightOuter4p0Radius2p0, "right-outer4.0-r2.0", 4000,
         -2000, true},
        {RawProfile::kRightOuter4p5Radius2p0, "right-outer4.5-r2.0", 4500,
         -2000, true},
        {RawProfile::kRightOuter5p0Radius2p0, "right-outer5.0-r2.0", 5000,
         -2000, true},
        {RawProfile::kRightOuter5p5Radius2p0, "right-outer5.5-r2.0", 5500,
         -2000, true},
        {RawProfile::kRightOuter6p0Radius2p0, "right-outer6.0-r2.0", 6000,
         -2000, true},
        {RawProfile::kLeftOuter0p5Radius0p95, "left-outer0.5-r0.95", 500,
         950, true},
        {RawProfile::kLeftOuter1p0Radius0p95, "left-outer1.0-r0.95", 1000,
         950, true},
        {RawProfile::kLeftOuter1p5Radius0p95, "left-outer1.5-r0.95", 1500,
         950, true},
        {RawProfile::kLeftOuter2p0Radius0p95, "left-outer2.0-r0.95", 2000,
         950, true},
        {RawProfile::kLeftOuter2p5Radius0p95, "left-outer2.5-r0.95", 2500,
         950, true},
        {RawProfile::kLeftOuter3p0Radius0p95, "left-outer3.0-r0.95", 3000,
         950, true},
        {RawProfile::kLeftOuter3p5Radius0p95, "left-outer3.5-r0.95", 3500,
         950, true},
        {RawProfile::kLeftOuter4p0Radius0p95, "left-outer4.0-r0.95", 4000,
         950, true},
        {RawProfile::kLeftOuter4p5Radius0p95, "left-outer4.5-r0.95", 4500,
         950, true},
        {RawProfile::kLeftOuter5p0Radius0p95, "left-outer5.0-r0.95", 5000,
         950, true},
        {RawProfile::kLeftOuter5p5Radius0p95, "left-outer5.5-r0.95", 5500,
         950, true},
        {RawProfile::kLeftOuter6p0Radius0p95, "left-outer6.0-r0.95", 6000,
         950, true},
        {RawProfile::kRightOuter0p5Radius0p95, "right-outer0.5-r0.95", 500,
         -950, true},
        {RawProfile::kRightOuter1p0Radius0p95, "right-outer1.0-r0.95", 1000,
         -950, true},
        {RawProfile::kRightOuter1p5Radius0p95, "right-outer1.5-r0.95", 1500,
         -950, true},
        {RawProfile::kRightOuter2p0Radius0p95, "right-outer2.0-r0.95", 2000,
         -950, true},
        {RawProfile::kRightOuter2p5Radius0p95, "right-outer2.5-r0.95", 2500,
         -950, true},
        {RawProfile::kRightOuter3p0Radius0p95, "right-outer3.0-r0.95", 3000,
         -950, true},
        {RawProfile::kRightOuter3p5Radius0p95, "right-outer3.5-r0.95", 3500,
         -950, true},
        {RawProfile::kRightOuter4p0Radius0p95, "right-outer4.0-r0.95", 4000,
         -950, true},
        {RawProfile::kRightOuter4p5Radius0p95, "right-outer4.5-r0.95", 4500,
         -950, true},
        {RawProfile::kRightOuter5p0Radius0p95, "right-outer5.0-r0.95", 5000,
         -950, true},
        {RawProfile::kRightOuter5p5Radius0p95, "right-outer5.5-r0.95", 5500,
         -950, true},
        {RawProfile::kRightOuter6p0Radius0p95, "right-outer6.0-r0.95", 6000,
         -950, true},
    }};

const RawProfileCatalogEntry* catalogEntry(RawProfile profile) {
  const auto found = std::find_if(
      kRawProfileCatalog.begin(), kRawProfileCatalog.end(),
      [profile](const RawProfileCatalogEntry& entry) {
        return entry.profile == profile;
      });
  return found == kRawProfileCatalog.end() ? nullptr : &(*found);
}

std::int16_t yawMagnitudeForRadius(std::int16_t forward_wire,
                                   std::int16_t radius_milli) {
  if (forward_wire < 0 || radius_milli <= 0) {
    return -1;
  }
  return static_cast<std::int16_t>(
      static_cast<std::int64_t>(forward_wire) * kWireScale /
      static_cast<std::int64_t>(radius_milli));
}

std::int16_t maximumCenterWireForOuterTier(std::int16_t tier_wire,
                                           std::int16_t radius_milli) {
  std::int16_t lower = 0;
  std::int16_t upper = tier_wire;
  while (lower < upper) {
    const std::int16_t candidate = static_cast<std::int16_t>(
        lower + (static_cast<std::int32_t>(upper) - lower + 1) / 2);
    const std::int16_t yaw = yawMagnitudeForRadius(candidate, radius_milli);
    const std::int64_t outer_milli =
        static_cast<std::int64_t>(candidate) * kWireScale +
        static_cast<std::int64_t>(yaw) * kHalfRearTrackMilli;
    if (yaw >= 0 &&
        outer_milli <= static_cast<std::int64_t>(tier_wire) * kWireScale) {
      lower = candidate;
    } else {
      upper = static_cast<std::int16_t>(candidate - 1);
    }
  }
  return lower;
}

enum class Phase : std::uint8_t {
  kBaseline = 0,
  kRampUp,
  kHold,
  kRampDown,
  kPostStop,
};

const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::kBaseline:
      return "baseline";
    case Phase::kRampUp:
      return "ramp_up";
    case Phase::kHold:
      return "hold";
    case Phase::kRampDown:
      return "ramp_down";
    case Phase::kPostStop:
      return "post_stop";
  }
  return "unknown";
}

const char* transportStatusText(TransportStatus status) {
  switch (status) {
    case TransportStatus::kOk:
      return "ok";
    case TransportStatus::kWouldBlock:
      return "would_block";
    case TransportStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case TransportStatus::kDisconnected:
      return "disconnected";
    case TransportStatus::kDisabled:
      return "disabled";
    case TransportStatus::kInvalidArgument:
      return "invalid_argument";
    case TransportStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

std::int64_t saturatedAdd(std::int64_t first, std::int64_t second) {
  if (second > 0 &&
      first > std::numeric_limits<std::int64_t>::max() - second) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return first + second;
}

void putSignedBigEndian(std::int16_t value, std::uint8_t* high,
                        std::uint8_t* low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  *high = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  *low = static_cast<std::uint8_t>(bits & 0xFFU);
}

std::uint8_t xorBytes(const std::uint8_t* data, std::size_t size) {
  std::uint8_t checksum = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    checksum = static_cast<std::uint8_t>(checksum ^ data[index]);
  }
  return checksum;
}

bool frameIsExactZero(const CommandFrame& frame) {
  return frame[0U] == kFrameHeader && frame[1U] == 0U &&
         frame[2U] == 0U && frame[3U] == 0U && frame[4U] == 0U &&
         frame[5U] == 0U && frame[6U] == 0U && frame[7U] == 0U &&
         frame[8U] == 0U && frame[9U] == xorBytes(frame.data(), 9U) &&
         frame[10U] == kFrameTail;
}

std::string rawHex(const std::uint8_t* data, std::size_t size) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  if (data != nullptr) {
    for (std::size_t index = 0U; index < size; ++index) {
      stream << std::setw(2) << static_cast<unsigned int>(data[index]);
    }
  }
  return stream.str();
}

void appendDouble(std::ostringstream* stream, double value) {
  *stream << std::setprecision(17) << value;
}

bool hasEvidencePrefix(const std::string& token, const char* prefix) {
  const std::string required(prefix);
  constexpr std::size_t kSha256HexCharacters = 64U;
  if (token.size() != required.size() + kSha256HexCharacters ||
      token.size() > kMaximumEvidenceTokenBytes ||
      token.compare(0U, required.size(), required) != 0) {
    return false;
  }
  for (std::size_t index = required.size(); index < token.size(); ++index) {
    const unsigned char byte = static_cast<unsigned char>(token[index]);
    if (!((byte >= '0' && byte <= '9') ||
          (byte >= 'a' && byte <= 'f'))) {
      return false;
    }
  }
  return true;
}

bool feedbackCandidateIsFinite(const FeedbackCandidate& candidate) {
  if (candidate.composite_stop_flag_raw > 1U ||
      candidate.control_allowed == candidate.control_inhibited ||
      candidate.control_allowed !=
          (candidate.composite_stop_flag_raw == 0U) ||
      candidate.control_inhibited !=
          (candidate.composite_stop_flag_raw == 1U) ||
      !std::isfinite(candidate.forward_speed_mps) ||
      !std::isfinite(candidate.lateral_speed_mps) ||
      !std::isfinite(candidate.yaw_rate_radps) ||
      !std::isfinite(candidate.supply_voltage_v)) {
    return false;
  }
  for (const double value : candidate.linear_acceleration_mps2) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : candidate.angular_velocity_radps) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

std::string metadataRecord(const RawProfileConfig& config,
                           const RawProfileDefinition& definition) {
  const double target_yaw_radps =
      static_cast<double>(definition.target_yaw_wire) / 1000.0;
  const double target_left_mps =
      definition.target_speed_mps -
      target_yaw_radps * kRawProfileRearTrackM / 2.0;
  const double target_right_mps =
      definition.target_speed_mps +
      target_yaw_radps * kRawProfileRearTrackM / 2.0;
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"metadata\",\"profile_id\":\""
         << definition.name << "\",\"protocol_status\":\"UNVERIFIED\""
         << ",\"raised_bench_only\":true"
         << ",\"phase1_main_loop_connected\":false"
         << ",\"phase1_codec_used\":false"
         << ",\"installed\":false"
         << ",\"acceptance_evidence\":false"
         << ",\"vcu_ack_available\":false"
         << ",\"successful_write_semantics\":\"host_os_full_write_only\""
         << ",\"one_profile_per_invocation\":true"
         << ",\"startup_exact_zero_required_before_any_read\":true"
         << ",\"prearm_commands_exact_zero_only\":true"
         << ",\"speed_tier_semantics\":\""
         << rawProfileSpeedTierSemanticsName(
                definition.speed_tier_semantics)
         << "\",\"speed_tier_wire\":" << definition.speed_tier_wire
         << ",\"speed_tier_mps\":";
  appendDouble(&stream, definition.speed_tier_mps);
  stream << ",\"target_center_forward_wire\":"
         << definition.target_forward_wire
         << ",\"target_forward_mps\":";
  appendDouble(&stream, definition.target_speed_mps);
  stream << ",\"target_yaw_wire\":" << definition.target_yaw_wire
         << ",\"target_yaw_radps\":";
  appendDouble(&stream, target_yaw_radps);
  stream << ",\"target_outer_command_mps\":";
  appendDouble(&stream, definition.target_outer_command_mps);
  stream << ",\"target_inner_command_mps\":";
  appendDouble(&stream, definition.target_inner_command_mps);
  stream << ",\"derived_target_left_mps\":";
  appendDouble(&stream, target_left_mps);
  stream << ",\"derived_target_right_mps\":";
  appendDouble(&stream, target_right_mps);
  stream << ",\"outer_tier_slack_mps\":";
  if (definition.speed_tier_semantics ==
      RawProfileSpeedTierSemantics::kOuterRearWheelCommandUpperLimit) {
    appendDouble(&stream, definition.speed_tier_mps -
                              definition.target_outer_command_mps);
  } else {
    stream << "null";
  }
  stream << ",\"turn_direction\":\"";
  if (definition.target_yaw_wire > 0) {
    stream << "left";
  } else if (definition.target_yaw_wire < 0) {
    stream << "right";
  } else {
    stream << "straight";
  }
  stream << "\",\"curvature_inv_m\":";
  appendDouble(&stream, definition.curvature_inv_m);
  stream << ",\"turn_radius_m\":";
  if (definition.turn_radius_m == 0.0) {
    stream << "null";
  } else {
    appendDouble(&stream, definition.turn_radius_m);
  }
  stream << ",\"turn_radius_mm\":";
  if (definition.turn_radius_m == 0.0) {
    stream << "null";
  } else {
    stream << static_cast<std::int32_t>(
        std::lround(definition.turn_radius_m * 1000.0));
  }
  stream << ",\"hold_duration_ns\":" << definition.hold_duration_ns
         << ",\"command_acceleration_mps2\":";
  appendDouble(&stream, kRawProfileAccelerationMps2);
  stream << ",\"command_acceleration_reference\":\"center_forward_command\"";
  stream << ",\"command_cap_mps\":";
  appendDouble(&stream, kRawProfileMaximumCommandSpeedMps);
  stream << ",\"command_cap_reference\":\"center_forward_command\""
         << ",\"maximum_outer_speed_tier_mps\":";
  appendDouble(&stream, kRawProfileMaximumCommandSpeedMps);
  stream << ",\"turn_center_selection_rule\":\"maximum_integer_c_with_1000*c+161*abs(trunc(1000*c/radius_mm))<=1000*tier_wire\""
         << ",\"yaw_quantization_rule\":\"magnitude=floor(1000*center_wire/radius_mm),then_apply_direction_sign\""
         << ",\"command_period_ns\":" << kCommandPeriodNs
         << ",\"normal_tx_maximum_hz\":50"
         << ",\"maximum_command_gap_ns\":" << kMaximumCommandGapNs
         << ",\"maximum_feedback_age_ns\":"
         << config.maximum_feedback_age_ns
         << ",\"maximum_session_duration_ns\":"
         << config.maximum_session_duration_ns
         << ",\"prearm_distinct_receipts_required\":"
         << kRequiredFreshFeedbackReceipts
         << ",\"prearm_minimum_receipt_span_ns\":"
         << kMinimumPrearmObservationNs
         << ",\"post_zero_observation_ns\":"
         << kPostZeroObservationNs
         << ",\"normal_wire_acceleration_integer_invariant\":\"abs(delta_raw)*5000000<=current_tx_before-previous_successful_tx_after\""
         << ",\"direct_emergency_zero_step_exception\":true"
         << ",\"feedback_tracking_acceptance_gate\":false"
         << ",\"feedback_asymmetry_acceptance_gate\":false"
         << ",\"feedback_standstill_acceptance_gate\":false"
         << ",\"parser_errors_recorded_not_gated\":true"
         << ",\"feedback_values_are_observation_only\":true"
         << ",\"feedback_flagstop_finite_freshness_are_hard_gates\":true"
         << ",\"left_right_difference_sign\":\"left_minus_right\""
         << ",\"derived_rear_track_mm\":322"
         << ",\"derived_rear_track_m\":";
  appendDouble(&stream, kRawProfileRearTrackM);
  stream << ",\"passive_evidence_token\":\""
         << config.passive_evidence_token
         << "\",\"exact_zero_evidence_token\":\""
         << config.exact_zero_evidence_token << "\"}";
  return stream.str();
}

std::string rawChunkRecord(const std::uint8_t* data, std::size_t size,
                           std::int64_t receipt_ns, Phase phase,
                           const RawProfileDefinition& definition) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"raw_rx_chunk\""
         << ",\"receipt_clock\":\"CLOCK_MONOTONIC\""
         << ",\"receipt_monotonic_ns\":" << receipt_ns
         << ",\"phase\":\"" << phaseName(phase)
         << "\",\"profile_id\":\"" << definition.name
         << "\",\"transferred\":" << size
         << ",\"raw_hex\":\"" << rawHex(data, size) << "\"}";
  return stream.str();
}

std::string parserObservationRecord(
    const ParserStatistics& before, const ParserStatistics& after,
    std::size_t trailing_bytes, std::int64_t receipt_ns, Phase phase,
    const RawProfileDefinition& definition) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"parser_observation_event\""
         << ",\"receipt_monotonic_ns\":" << receipt_ns
         << ",\"phase\":\"" << phaseName(phase)
         << "\",\"profile_id\":\"" << definition.name
         << "\",\"checksum_failures_delta\":"
         << after.checksum_failures - before.checksum_failures
         << ",\"framing_failures_delta\":"
         << after.framing_failures - before.framing_failures
         << ",\"discarded_bytes_delta\":"
         << after.discarded_bytes - before.discarded_bytes
         << ",\"checksum_failures_total\":" << after.checksum_failures
         << ",\"framing_failures_total\":" << after.framing_failures
         << ",\"discarded_bytes_total\":" << after.discarded_bytes
         << ",\"trailing_buffered_bytes\":" << trailing_bytes
         << ",\"online_acceptance_gate_applied\":false}";
  return stream.str();
}

std::string ioAttemptRecord(const char* record_type, Phase phase,
                            const RawProfileDefinition& definition,
                            const RawProfileEncodeResult* encoded,
                            std::size_t requested_bytes,
                            std::int64_t before_ns,
                            std::int64_t deadline_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"" << record_type
         << "\",\"phase\":\"" << phaseName(phase)
         << "\",\"profile_id\":\"" << definition.name
         << "\",\"clock\":\"CLOCK_MONOTONIC\""
         << ",\"before_monotonic_ns\":" << before_ns
         << ",\"absolute_deadline_monotonic_ns\":" << deadline_ns
         << ",\"requested_bytes\":" << requested_bytes;
  if (encoded != nullptr) {
    stream << ",\"forward_wire\":" << encoded->forward_wire
           << ",\"yaw_wire\":" << encoded->yaw_wire
           << ",\"forward_command_mps\":";
    appendDouble(&stream,
                 static_cast<double>(encoded->forward_wire) / 1000.0);
    stream << ",\"yaw_command_radps\":";
    appendDouble(&stream,
                 static_cast<double>(encoded->yaw_wire) / 1000.0);
    stream << ",\"raw_hex\":\""
           << rawHex(encoded->frame.data(), encoded->frame.size()) << '"';
  }
  stream << '}';
  return stream.str();
}

std::string ioResultRecord(const char* record_type, Phase phase,
                           const RawProfileDefinition& definition,
                           const RawProfileEncodeResult* encoded,
                           const IoResult& result,
                           std::int64_t before_ns,
                           std::int64_t after_ns,
                           std::int64_t deadline_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"" << record_type
         << "\",\"phase\":\"" << phaseName(phase)
         << "\",\"profile_id\":\"" << definition.name
         << "\",\"clock\":\"CLOCK_MONOTONIC\""
         << ",\"before_monotonic_ns\":" << before_ns
         << ",\"after_monotonic_ns\":" << after_ns
         << ",\"absolute_deadline_monotonic_ns\":" << deadline_ns
         << ",\"status\":\"" << transportStatusText(result.status)
         << "\",\"transferred\":" << result.transferred
         << ",\"os_error\":" << result.os_error
         << ",\"delivery_unconfirmed\":"
         << (result.delivery_unconfirmed ? "true" : "false");
  if (encoded != nullptr) {
    stream << ",\"forward_wire\":" << encoded->forward_wire
           << ",\"yaw_wire\":" << encoded->yaw_wire
           << ",\"forward_command_mps\":";
    appendDouble(&stream,
                 static_cast<double>(encoded->forward_wire) / 1000.0);
    stream << ",\"yaw_command_radps\":";
    appendDouble(&stream,
                 static_cast<double>(encoded->yaw_wire) / 1000.0);
    stream << ",\"raw_hex\":\""
           << rawHex(encoded->frame.data(), encoded->frame.size()) << '"';
  }
  stream << '}';
  return stream.str();
}

std::string feedbackObservationRecord(
    const ParsedFeedbackFrame& frame, std::int64_t receipt_ns, Phase phase,
    const RawProfileDefinition& definition,
    std::int16_t last_forward_wire, std::int16_t last_yaw_wire) {
  const double half_track = kRawProfileRearTrackM / 2.0;
  const double left = frame.candidate.forward_speed_mps -
                      frame.candidate.yaw_rate_radps * half_track;
  const double right = frame.candidate.forward_speed_mps +
                       frame.candidate.yaw_rate_radps * half_track;
  const double requested = static_cast<double>(last_forward_wire) / 1000.0;
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"feedback_observation_event\""
         << ",\"receipt_clock\":\"CLOCK_MONOTONIC\""
         << ",\"receipt_monotonic_ns\":" << receipt_ns
         << ",\"phase\":\"" << phaseName(phase)
         << "\",\"profile_id\":\"" << definition.name
         << "\",\"last_successful_forward_wire\":" << last_forward_wire
         << ",\"last_successful_yaw_wire\":" << last_yaw_wire
         << ",\"raw_hex\":\""
         << rawHex(frame.raw_frame.data(), frame.raw_frame.size())
         << "\",\"forward_speed_mps\":";
  appendDouble(&stream, frame.candidate.forward_speed_mps);
  stream << ",\"lateral_speed_mps\":";
  appendDouble(&stream, frame.candidate.lateral_speed_mps);
  stream << ",\"yaw_rate_radps\":";
  appendDouble(&stream, frame.candidate.yaw_rate_radps);
  stream << ",\"derived_left_mps\":";
  appendDouble(&stream, left);
  stream << ",\"derived_right_mps\":";
  appendDouble(&stream, right);
  stream << ",\"tracking_error_mps\":";
  appendDouble(&stream, frame.candidate.forward_speed_mps - requested);
  stream << ",\"left_right_difference_mps\":";
  appendDouble(&stream, left - right);
  stream << ",\"post_zero_tail\":"
         << (phase == Phase::kPostStop ? "true" : "false")
         << ",\"online_acceptance_gate_applied\":false"
         << ",\"composite_stop_flag_raw\":"
         << static_cast<unsigned int>(
                frame.candidate.composite_stop_flag_raw)
         << ",\"control_allowed\":"
         << (frame.candidate.control_allowed ? "true" : "false")
         << ",\"control_inhibited\":"
         << (frame.candidate.control_inhibited ? "true" : "false")
         << ",\"source_time_available\":false"
         << ",\"vcu_ack_available\":false}";
  return stream.str();
}

bool recordLine(const RawProfileOperations& operations,
                const std::string& line, bool* record_failed) {
  if (record_failed == nullptr || *record_failed ||
      !operations.record_json_line ||
      !operations.record_json_line(line)) {
    if (record_failed != nullptr) {
      *record_failed = true;
    }
    return false;
  }
  return true;
}

void recordBestEffort(const RawProfileOperations& operations,
                      const std::string& line, bool* record_failed) {
  if (!operations.record_json_line ||
      !operations.record_json_line(line)) {
    if (record_failed != nullptr) {
      *record_failed = true;
    }
  }
}

bool boundedExactZeroWrite(ByteTransport* transport,
                           const RawProfileOperations& operations,
                           const RawProfileDefinition& definition,
                           Phase phase, std::int64_t last_good_clock_ns,
                           RawProfileStatistics* statistics,
                           bool* delivery_unconfirmed,
                           bool* record_failed,
                           bool* write_stream_poisoned) {
  if (transport == nullptr || statistics == nullptr ||
      delivery_unconfirmed == nullptr ||
      write_stream_poisoned == nullptr ||
      !operations.monotonic_now_ns ||
      !operations.wait_until_monotonic_ns) {
    return false;
  }
  if (*write_stream_poisoned) {
    *delivery_unconfirmed = true;
    return false;
  }
  const RawProfileEncodeResult zero =
      encodeRawProfileCommand(definition.profile, 0);
  if (!zero.ok()) {
    *delivery_unconfirmed = true;
    return false;
  }
  const std::int64_t observed_now = operations.monotonic_now_ns();
  const std::int64_t base = std::max(
      last_good_clock_ns, observed_now > 0 ? observed_now : 0);
  struct ZeroAttemptEvidence {
    std::int64_t attempt_ns{0};
    std::int64_t deadline_ns{0};
    std::int64_t after_ns{0};
    IoResult written{};
  };
  std::array<ZeroAttemptEvidence, kMaximumZeroWriteAttempts> evidence{};
  std::size_t evidence_count = 0U;
  bool zero_completed = false;
  for (std::uint32_t attempt = 0U;
       attempt < kMaximumZeroWriteAttempts; ++attempt) {
    const std::int64_t attempt_ns = saturatedAdd(
        base, static_cast<std::int64_t>(attempt) * kZeroRetryIntervalNs);
    if (attempt > 0U &&
        !operations.wait_until_monotonic_ns(attempt_ns)) {
      *delivery_unconfirmed = true;
      break;
    }
    const std::int64_t deadline_ns =
        saturatedAdd(attempt_ns, kWriteTimeoutNs);
    ++statistics->tx_attempts;
    // Until writeAll returns a definitive full-frame or zero-byte result, an
    // exception can mean that an unknown frame prefix reached the stream.
    *write_stream_poisoned = true;
    const IoResult written = transport->writeAll(
        zero.frame.data(), zero.frame.size(), deadline_ns);
    const bool full_host_write =
        written.status == TransportStatus::kOk &&
        written.transferred == zero.frame.size() &&
        !written.delivery_unconfirmed;
    const bool confirmed_no_delivery =
        written.transferred == 0U && !written.delivery_unconfirmed;
    if (full_host_write || confirmed_no_delivery) {
      *write_stream_poisoned = false;
    }
    const std::int64_t after_ns = operations.monotonic_now_ns();
    evidence[evidence_count].attempt_ns = attempt_ns;
    evidence[evidence_count].deadline_ns = deadline_ns;
    evidence[evidence_count].after_ns = after_ns;
    evidence[evidence_count].written = written;
    ++evidence_count;
    if (full_host_write) {
      ++statistics->tx_host_writes_completed;
      ++statistics->zero_frame_host_writes_completed;
      zero_completed = true;
      break;
    }
    *delivery_unconfirmed = true;
    const bool retry_is_safe =
        confirmed_no_delivery && after_ns >= attempt_ns &&
        after_ns <= deadline_ns;
    if (!retry_is_safe ||
        written.status == TransportStatus::kDisconnected) {
      break;
    }
  }
  for (std::size_t index = 0U; index < evidence_count; ++index) {
    try {
      recordBestEffort(
          operations,
          ioAttemptRecord("emergency_zero_tx_attempt_postwrite_record", phase,
                          definition, &zero, zero.frame.size(),
                          evidence[index].attempt_ns,
                          evidence[index].deadline_ns),
          record_failed);
      recordBestEffort(
          operations,
          ioResultRecord("emergency_zero_tx_result_postwrite_record", phase,
                         definition, &zero, evidence[index].written,
                         evidence[index].attempt_ns, evidence[index].after_ns,
                         evidence[index].deadline_ns),
          record_failed);
    } catch (...) {
      if (record_failed != nullptr) {
        *record_failed = true;
      }
    }
  }
  return zero_completed;
}

}  // namespace

const char* rawProfileName(RawProfile profile) {
  if (profile == RawProfile::kDisabled) {
    return "disabled";
  }
  const RawProfileCatalogEntry* const entry = catalogEntry(profile);
  return entry == nullptr ? "unknown" : entry->name;
}

const char* rawProfileSpeedTierSemanticsName(
    RawProfileSpeedTierSemantics semantics) {
  switch (semantics) {
    case RawProfileSpeedTierSemantics::kStraightCenterEqualsRearWheels:
      return "straight_center_equals_rear_wheels";
    case RawProfileSpeedTierSemantics::kLegacyCenterForwardCommand:
      return "legacy_center_forward_command";
    case RawProfileSpeedTierSemantics::kOuterRearWheelCommandUpperLimit:
      return "outer_rear_wheel_command_upper_limit";
  }
  return "unknown";
}

bool rawProfileAt(std::size_t index, RawProfile* profile) {
  if (profile == nullptr || index >= kRawProfileCatalog.size()) {
    return false;
  }
  *profile = kRawProfileCatalog[index].profile;
  return true;
}

bool rawProfileFromName(const std::string& name, RawProfile* profile) {
  if (profile == nullptr) {
    return false;
  }
  const auto found = std::find_if(
      kRawProfileCatalog.begin(), kRawProfileCatalog.end(),
      [&name](const RawProfileCatalogEntry& entry) {
        return name == entry.name;
      });
  if (found == kRawProfileCatalog.end()) {
    return false;
  }
  *profile = found->profile;
  return true;
}

bool rawProfileDefinition(RawProfile profile,
                          RawProfileDefinition* definition) {
  if (definition == nullptr) {
    return false;
  }
  const RawProfileCatalogEntry* const entry = catalogEntry(profile);
  if (entry == nullptr || entry->speed_tier_wire <= 0 ||
      entry->speed_tier_wire > kRawProfileMaximumCommandForwardWire) {
    return false;
  }
  RawProfileDefinition result;
  result.profile = profile;
  result.name = entry->name;
  result.hold_duration_ns = kRawProfileTargetHoldDurationNs;
  result.speed_tier_wire = entry->speed_tier_wire;
  result.speed_tier_mps =
      static_cast<double>(entry->speed_tier_wire) / 1000.0;
  const std::int16_t radius_milli = static_cast<std::int16_t>(
      std::abs(static_cast<std::int32_t>(entry->signed_radius_milli)));
  if (radius_milli == 0) {
    result.speed_tier_semantics =
        RawProfileSpeedTierSemantics::kStraightCenterEqualsRearWheels;
    result.target_forward_wire = entry->speed_tier_wire;
  } else {
    result.speed_tier_semantics = entry->outer_tier_limited
        ? RawProfileSpeedTierSemantics::kOuterRearWheelCommandUpperLimit
        : RawProfileSpeedTierSemantics::kLegacyCenterForwardCommand;
    result.target_forward_wire = entry->outer_tier_limited
        ? maximumCenterWireForOuterTier(entry->speed_tier_wire, radius_milli)
        : entry->speed_tier_wire;
    const std::int16_t yaw_magnitude = yawMagnitudeForRadius(
        result.target_forward_wire, radius_milli);
    if (yaw_magnitude < 0) {
      return false;
    }
    result.target_yaw_wire = entry->signed_radius_milli > 0
        ? yaw_magnitude
        : static_cast<std::int16_t>(-yaw_magnitude);
    result.turn_radius_m = static_cast<double>(radius_milli) / 1000.0;
    result.curvature_inv_m =
        (entry->signed_radius_milli > 0 ? 1.0 : -1.0) /
        result.turn_radius_m;
  }
  const std::int64_t yaw_magnitude = std::abs(
      static_cast<std::int64_t>(result.target_yaw_wire));
  result.target_outer_wire_milli =
      static_cast<std::int64_t>(result.target_forward_wire) * kWireScale +
      yaw_magnitude * kHalfRearTrackMilli;
  result.target_inner_wire_milli =
      static_cast<std::int64_t>(result.target_forward_wire) * kWireScale -
      yaw_magnitude * kHalfRearTrackMilli;
  if (result.target_forward_wire <= 0 ||
      result.target_forward_wire > kRawProfileMaximumCommandForwardWire ||
      result.target_inner_wire_milli < 0 ||
      (entry->outer_tier_limited &&
       result.target_outer_wire_milli >
           static_cast<std::int64_t>(entry->speed_tier_wire) * kWireScale)) {
    return false;
  }
  result.target_speed_mps =
      static_cast<double>(result.target_forward_wire) / 1000.0;
  result.target_outer_command_mps =
      static_cast<double>(result.target_outer_wire_milli) / 1000000.0;
  result.target_inner_command_mps =
      static_cast<double>(result.target_inner_wire_milli) / 1000000.0;
  *definition = result;
  return true;
}

RawProfileEncodeResult encodeRawProfileCommand(
    RawProfile profile, std::int16_t forward_wire) {
  RawProfileEncodeResult result;
  RawProfileDefinition definition;
  if (!rawProfileDefinition(profile, &definition)) {
    result.status = RawProfileEncodeStatus::kInvalidProfile;
    return result;
  }
  if (forward_wire < 0 || forward_wire > definition.target_forward_wire ||
      forward_wire > kRawProfileMaximumCommandForwardWire) {
    result.status = RawProfileEncodeStatus::kForwardWireOutOfRange;
    return result;
  }
  const RawProfileCatalogEntry* const entry = catalogEntry(profile);
  if (entry == nullptr) {
    result.status = RawProfileEncodeStatus::kInvalidProfile;
    return result;
  }
  const std::int16_t radius_milli = static_cast<std::int16_t>(
      std::abs(static_cast<std::int32_t>(entry->signed_radius_milli)));
  const std::int16_t yaw_magnitude = radius_milli == 0
      ? 0
      : yawMagnitudeForRadius(forward_wire, radius_milli);
  if (yaw_magnitude < 0) {
    result.status = RawProfileEncodeStatus::kYawWireOutOfRange;
    return result;
  }
  result.forward_wire = forward_wire;
  result.yaw_wire = entry->signed_radius_milli < 0
      ? static_cast<std::int16_t>(-yaw_magnitude)
      : yaw_magnitude;
  result.frame[0U] = kFrameHeader;
  result.frame[1U] = 0U;
  result.frame[2U] = 0U;
  putSignedBigEndian(result.forward_wire,
                     &result.frame[3U], &result.frame[4U]);
  putSignedBigEndian(0, &result.frame[5U], &result.frame[6U]);
  putSignedBigEndian(result.yaw_wire,
                     &result.frame[7U], &result.frame[8U]);
  result.frame[9U] = xorBytes(result.frame.data(), 9U);
  result.frame[10U] = kFrameTail;
  result.status = RawProfileEncodeStatus::kOk;
  return result;
}

const char* rawProfileStatusName(RawProfileStatus status) {
  switch (status) {
    case RawProfileStatus::kCompleted:
      return "completed";
    case RawProfileStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case RawProfileStatus::kAuthorizationDenied:
      return "authorization_denied";
    case RawProfileStatus::kEvidenceMissing:
      return "evidence_missing";
    case RawProfileStatus::kClockInvalid:
      return "clock_invalid";
    case RawProfileStatus::kDisconnected:
      return "disconnected";
    case RawProfileStatus::kFeedbackMissing:
      return "feedback_missing";
    case RawProfileStatus::kFeedbackInvalid:
      return "feedback_invalid";
    case RawProfileStatus::kControlInhibited:
      return "control_inhibited";
    case RawProfileStatus::kCommandWatchdogExpired:
      return "command_watchdog_expired";
    case RawProfileStatus::kWriteFailed:
      return "write_failed";
    case RawProfileStatus::kRecordError:
      return "record_error";
    case RawProfileStatus::kInterrupted:
      return "interrupted";
    case RawProfileStatus::kSessionDeadlineExceeded:
      return "session_deadline_exceeded";
    case RawProfileStatus::kZeroHostWriteFailed:
      return "zero_host_write_failed";
  }
  return "unknown";
}

RawProfileImmediateZeroResult writeRawProfileImmediateZeroNoRecord(
    ByteTransport* transport, const CommandFrame& preencoded_exact_zero,
    const RawProfileOperations& operations) {
  RawProfileImmediateZeroResult result;
  if (transport == nullptr || !frameIsExactZero(preencoded_exact_zero) ||
      !operations.monotonic_now_ns ||
      !operations.wait_until_monotonic_ns) {
    result.delivery_unconfirmed = true;
    result.terminal_transport_status = TransportStatus::kInvalidArgument;
    return result;
  }
  try {
    const std::int64_t base_ns = operations.monotonic_now_ns();
    if (base_ns <= 0) {
      result.delivery_unconfirmed = true;
      result.terminal_transport_status = TransportStatus::kInvalidArgument;
      return result;
    }
    for (std::uint32_t attempt = 0U;
         attempt < kMaximumZeroWriteAttempts; ++attempt) {
      const std::int64_t scheduled_ns = saturatedAdd(
          base_ns,
          static_cast<std::int64_t>(attempt) * kZeroRetryIntervalNs);
      if (attempt > 0U &&
          !operations.wait_until_monotonic_ns(scheduled_ns)) {
        result.delivery_unconfirmed = true;
        result.terminal_transport_status =
            TransportStatus::kDeadlineExceeded;
        break;
      }
      const std::int64_t before_ns = operations.monotonic_now_ns();
      if (before_ns <= 0 || before_ns < base_ns) {
        result.delivery_unconfirmed = true;
        result.terminal_transport_status =
            TransportStatus::kInvalidArgument;
        break;
      }
      const std::int64_t deadline_ns =
          saturatedAdd(before_ns, kWriteTimeoutNs);
      ++result.attempts;
      const IoResult written = transport->writeAll(
          preencoded_exact_zero.data(), preencoded_exact_zero.size(),
          deadline_ns);
      const std::int64_t after_ns = operations.monotonic_now_ns();
      result.terminal_transport_status = written.status;
      const bool completed = written.status == TransportStatus::kOk &&
                             written.transferred ==
                                 preencoded_exact_zero.size() &&
                             !written.delivery_unconfirmed &&
                             after_ns >= before_ns &&
                             after_ns <= deadline_ns;
      if (completed) {
        result.zero_host_write_completed = true;
        result.delivery_unconfirmed = false;
        return result;
      }
      result.delivery_unconfirmed = true;
      // Retrying is safe only when the transport positively reports that no
      // byte was transferred. A partial/full transfer, an unconfirmed
      // delivery result, or an invalid completion clock may have changed the
      // controller's frame assembly state; appending another frame could turn
      // the stream into an unintended command.
      const bool confirmed_no_delivery =
          written.transferred == 0U && !written.delivery_unconfirmed &&
          after_ns >= before_ns && after_ns <= deadline_ns;
      if (!confirmed_no_delivery ||
          written.status == TransportStatus::kDisconnected) {
        break;
      }
    }
  } catch (...) {
    result.delivery_unconfirmed = true;
    result.terminal_transport_status = TransportStatus::kIoError;
  }
  return result;
}

bool rawProfileConfigIsValid(const RawProfileConfig& config) {
  RawProfileDefinition definition;
  return rawProfileDefinition(config.profile, &definition) &&
         config.read_timeout_ns > 0 &&
         config.read_timeout_ns <= kCommandPeriodNs &&
         config.maximum_feedback_age_ns >= config.read_timeout_ns &&
         config.maximum_feedback_age_ns <= 1000000000LL &&
         config.initial_feedback_deadline_ns > 0 &&
         config.initial_feedback_deadline_ns <= 5000000000LL &&
         config.maximum_session_duration_ns >= kCommandPeriodNs &&
         config.maximum_session_duration_ns <=
             kRawProfileMaximumSessionDurationNs;
}

RawProfileStatus rawProfilePreflightStatus(const RawProfileConfig& config) {
  if (!rawProfileConfigIsValid(config)) {
    return RawProfileStatus::kInvalidConfiguration;
  }
  if (!config.unverified_protocol_acknowledged ||
      !config.physical_device_opt_in || !config.actuation_opt_in ||
      !config.raw_raised_bench_opt_in ||
      config.operator_confirmation_token !=
          kRawProfileOperatorConfirmationToken) {
    return RawProfileStatus::kAuthorizationDenied;
  }
  if (!hasEvidencePrefix(config.passive_evidence_token,
                         kPassiveEvidencePrefix) ||
      !hasEvidencePrefix(config.exact_zero_evidence_token,
                         kExactZeroEvidencePrefix)) {
    return RawProfileStatus::kEvidenceMissing;
  }
  return RawProfileStatus::kCompleted;
}

std::string rawProfileSummaryRecordJson(const RawProfileResult& result) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kSchema
         << "\",\"record_type\":\"summary\",\"status\":\""
         << rawProfileStatusName(result.status)
         << "\",\"zero_host_write_completed\":"
         << (result.zero_host_write_completed ? "true" : "false")
         << ",\"delivery_unconfirmed\":"
         << (result.delivery_unconfirmed ? "true" : "false")
         << ",\"acceptance_evidence\":false"
         << ",\"statistics\":{\"read_calls\":"
         << result.statistics.read_calls
         << ",\"raw_rx_bytes\":" << result.statistics.raw_rx_bytes
         << ",\"valid_feedback_frames\":"
         << result.statistics.valid_feedback_frames
         << ",\"feedback_observation_events\":"
         << result.statistics.feedback_observation_events
         << ",\"parser_checksum_failures\":"
         << result.statistics.parser_checksum_failures
         << ",\"parser_framing_failures\":"
         << result.statistics.parser_framing_failures
         << ",\"parser_discarded_bytes\":"
         << result.statistics.parser_discarded_bytes
         << ",\"parser_trailing_buffered_bytes\":"
         << result.statistics.parser_trailing_buffered_bytes
         << ",\"read_timeouts\":"
         << result.statistics.read_timeouts
         << ",\"tx_attempts\":" << result.statistics.tx_attempts
         << ",\"tx_host_writes_completed\":"
         << result.statistics.tx_host_writes_completed
         << ",\"normal_tx_host_writes_completed\":"
         << result.statistics.normal_tx_host_writes_completed
         << ",\"zero_frame_host_writes_completed\":"
         << result.statistics.zero_frame_host_writes_completed
         << ",\"nonzero_frame_host_writes_completed\":"
         << result.statistics.nonzero_frame_host_writes_completed
         << ",\"maximum_commanded_forward_wire\":"
         << result.statistics.maximum_commanded_forward_wire
         << ",\"started_monotonic_ns\":"
         << result.statistics.started_monotonic_ns
         << ",\"ended_monotonic_ns\":"
         << result.statistics.ended_monotonic_ns << "}}";
  return stream.str();
}

namespace {

RawProfileResult runRawProfileSessionImpl(
    ByteTransport* transport, const RawProfileConfig& config,
    const RawProfileOperations& operations, bool* session_started,
    bool* write_stream_poisoned) {
  RawProfileResult result;
  if (session_started != nullptr) {
    *session_started = false;
  }
  if (write_stream_poisoned != nullptr) {
    *write_stream_poisoned = false;
  }
  if (transport == nullptr || !operations.monotonic_now_ns ||
      !operations.wait_until_monotonic_ns ||
      !operations.record_json_line || !operations.stop_requested ||
      write_stream_poisoned == nullptr) {
    return result;
  }
  result.status = rawProfilePreflightStatus(config);
  if (result.status != RawProfileStatus::kCompleted) {
    return result;
  }
  if (!transport->isConnected()) {
    result.status = RawProfileStatus::kDisconnected;
    result.delivery_unconfirmed = true;
    return result;
  }
  RawProfileDefinition definition;
  if (!rawProfileDefinition(config.profile, &definition)) {
    result.status = RawProfileStatus::kInvalidConfiguration;
    return result;
  }
  if (session_started != nullptr) {
    *session_started = true;
  }

  const std::int64_t started_ns = operations.monotonic_now_ns();
  result.statistics.started_monotonic_ns = started_ns;
  result.statistics.ended_monotonic_ns = started_ns;
  bool record_failed = false;
  if (started_ns <= 0) {
    result.status = RawProfileStatus::kClockInvalid;
    result.zero_host_write_completed = boundedExactZeroWrite(
        transport, operations, definition, Phase::kBaseline, 0,
        &result.statistics, &result.delivery_unconfirmed, &record_failed,
        write_stream_poisoned);
    return result;
  }
  if (!recordLine(operations, metadataRecord(config, definition),
                  &record_failed)) {
    result.status = RawProfileStatus::kRecordError;
    result.zero_host_write_completed = boundedExactZeroWrite(
        transport, operations, definition, Phase::kBaseline, started_ns,
        &result.statistics, &result.delivery_unconfirmed, &record_failed,
        write_stream_poisoned);
    return result;
  }

  std::int64_t last_good_clock_ns = started_ns;
  Phase phase = Phase::kBaseline;
  RawProfileWireAccelerationEnvelope envelope;
  std::int16_t last_successful_forward_wire = 0;
  std::int16_t last_successful_yaw_wire = 0;
  std::array<std::uint8_t, kReadBufferBytes> read_buffer{};
  const RawProfileEncodeResult initial_zero =
      encodeRawProfileCommand(definition.profile, 0);
  const std::int64_t initial_zero_before_ns = operations.monotonic_now_ns();
  const std::int64_t initial_zero_deadline_ns = saturatedAdd(
      initial_zero_before_ns, kWriteTimeoutNs);
  if (!initial_zero.ok() || initial_zero_before_ns <= 0 ||
      initial_zero_before_ns < last_good_clock_ns) {
    result.status = RawProfileStatus::kClockInvalid;
  } else if (!recordLine(
                 operations,
                 ioAttemptRecord("normal_tx_attempt", phase, definition,
                                 &initial_zero,
                                 initial_zero.frame.size(),
                                 initial_zero_before_ns,
                                 initial_zero_deadline_ns),
                 &record_failed)) {
    result.status = RawProfileStatus::kRecordError;
  } else {
    ++result.statistics.tx_attempts;
    *write_stream_poisoned = true;
    const IoResult written = transport->writeAll(
        initial_zero.frame.data(), initial_zero.frame.size(),
        initial_zero_deadline_ns);
    const bool full_host_write =
        written.status == TransportStatus::kOk &&
        written.transferred == initial_zero.frame.size() &&
        !written.delivery_unconfirmed;
    const bool confirmed_no_delivery =
        written.transferred == 0U && !written.delivery_unconfirmed;
    if (full_host_write || confirmed_no_delivery) {
      *write_stream_poisoned = false;
    }
    const std::int64_t after_ns = operations.monotonic_now_ns();
    if (!recordLine(
            operations,
            ioResultRecord("normal_tx_result", phase, definition,
                           &initial_zero, written, initial_zero_before_ns,
                           after_ns, initial_zero_deadline_ns),
            &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
    } else {
      if (full_host_write) {
        ++result.statistics.tx_host_writes_completed;
        ++result.statistics.normal_tx_host_writes_completed;
        ++result.statistics.zero_frame_host_writes_completed;
      }
      if (after_ns <= 0 || after_ns < initial_zero_before_ns) {
        result.status = RawProfileStatus::kClockInvalid;
      } else if (!full_host_write ||
                 after_ns > initial_zero_deadline_ns) {
        result.status = written.status == TransportStatus::kDisconnected
                            ? RawProfileStatus::kDisconnected
                            : RawProfileStatus::kWriteFailed;
        result.delivery_unconfirmed = !full_host_write;
      } else if (!envelope.noteSuccessfulNormalWrite(0, after_ns)) {
        result.status = RawProfileStatus::kClockInvalid;
      } else {
        last_good_clock_ns = after_ns;
      }
    }
  }
  bool drain_complete = false;
  for (std::uint32_t drain = 0U;
       result.status == RawProfileStatus::kCompleted &&
       drain < kMaximumDrainReads; ++drain) {
    const std::int64_t before_ns = operations.monotonic_now_ns();
    if (before_ns <= 0 || before_ns < last_good_clock_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    const std::int64_t deadline_ns =
        saturatedAdd(before_ns, kDrainReadDeadlineNs);
    if (!recordLine(operations,
                    ioAttemptRecord("drain_rx_attempt", Phase::kBaseline,
                                    definition, nullptr, read_buffer.size(),
                                    before_ns, deadline_ns),
                    &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    ++result.statistics.read_calls;
    const IoResult drained = transport->readSome(
        read_buffer.data(), read_buffer.size(), deadline_ns);
    const std::int64_t after_ns = operations.monotonic_now_ns();
    if (!recordLine(operations,
                    ioResultRecord("drain_rx_result", Phase::kBaseline,
                                   definition, nullptr, drained, before_ns,
                                   after_ns, deadline_ns),
                    &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    if (after_ns <= 0 || after_ns < before_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    last_good_clock_ns = after_ns;
    if (drained.status == TransportStatus::kWouldBlock ||
        drained.status == TransportStatus::kDeadlineExceeded) {
      drain_complete = true;
      break;
    }
    if (drained.status == TransportStatus::kDisconnected) {
      result.status = RawProfileStatus::kDisconnected;
      result.delivery_unconfirmed = true;
      break;
    }
    if (drained.status != TransportStatus::kOk ||
        drained.transferred == 0U ||
        drained.transferred > read_buffer.size()) {
      result.status = RawProfileStatus::kFeedbackInvalid;
      break;
    }
    result.statistics.raw_rx_bytes += drained.transferred;
    if (!recordLine(operations,
                    rawChunkRecord(read_buffer.data(), drained.transferred,
                                   after_ns, Phase::kBaseline, definition),
                    &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
  }
  if (result.status == RawProfileStatus::kCompleted && !drain_complete) {
    result.status = RawProfileStatus::kFeedbackInvalid;
  }

  const std::int64_t observation_started_ns = operations.monotonic_now_ns();
  if (result.status == RawProfileStatus::kCompleted &&
      (observation_started_ns <= 0 ||
       observation_started_ns < last_good_clock_ns)) {
    result.status = RawProfileStatus::kClockInvalid;
  }
  if (observation_started_ns > last_good_clock_ns) {
    last_good_clock_ns = observation_started_ns;
  }
  const std::int64_t session_deadline_ns = saturatedAdd(
      observation_started_ns, config.maximum_session_duration_ns);
  const std::int64_t initial_feedback_deadline_ns = saturatedAdd(
      observation_started_ns, config.initial_feedback_deadline_ns);
  std::int64_t next_cycle_ns =
      saturatedAdd(observation_started_ns, kCommandPeriodNs);
  std::int64_t last_feedback_ns = 0;
  std::int64_t first_prearm_feedback_ns = 0;
  std::int64_t last_prearm_feedback_ns = 0;
  std::uint32_t prearm_receipts = 0U;
  std::int64_t hold_deadline_ns = 0;
  std::int64_t post_stop_deadline_ns = 0;
  FeedbackStreamParser parser;

  while (result.status == RawProfileStatus::kCompleted) {
    if (operations.stop_requested()) {
      result.status = RawProfileStatus::kInterrupted;
      break;
    }
    std::int64_t before_read_ns = operations.monotonic_now_ns();
    if (before_read_ns <= 0 || before_read_ns < last_good_clock_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    if (before_read_ns < next_cycle_ns) {
      if (!operations.wait_until_monotonic_ns(next_cycle_ns)) {
        result.status = RawProfileStatus::kCommandWatchdogExpired;
        break;
      }
      before_read_ns = operations.monotonic_now_ns();
    }
    if (before_read_ns <= 0 || before_read_ns < last_good_clock_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    if (before_read_ns > saturatedAdd(next_cycle_ns,
                                      kMaximumCommandGapNs) ||
        (envelope.hasBaseline() &&
         before_read_ns - envelope.lastSuccessfulCompletionNs() >
             kMaximumCommandGapNs)) {
      result.status = RawProfileStatus::kCommandWatchdogExpired;
      break;
    }
    if (before_read_ns >= session_deadline_ns) {
      result.status = RawProfileStatus::kSessionDeadlineExceeded;
      break;
    }
    last_good_clock_ns = before_read_ns;
    const std::int64_t read_deadline_ns = std::min(
        saturatedAdd(before_read_ns, config.read_timeout_ns),
        session_deadline_ns);
    if (!recordLine(operations,
                    ioAttemptRecord("rx_attempt", phase, definition, nullptr,
                                    read_buffer.size(), before_read_ns,
                                    read_deadline_ns),
                    &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    ++result.statistics.read_calls;
    const IoResult read_result = transport->readSome(
        read_buffer.data(), read_buffer.size(), read_deadline_ns);
    const std::int64_t receipt_ns = operations.monotonic_now_ns();
    result.statistics.ended_monotonic_ns = receipt_ns;
    if (!recordLine(operations,
                    ioResultRecord("rx_result", phase, definition, nullptr,
                                   read_result, before_read_ns, receipt_ns,
                                   read_deadline_ns),
                    &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    if (receipt_ns <= 0 || receipt_ns < before_read_ns ||
        receipt_ns < last_good_clock_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    if (receipt_ns - before_read_ns > kMaximumCommandGapNs) {
      result.status = RawProfileStatus::kCommandWatchdogExpired;
      break;
    }
    last_good_clock_ns = receipt_ns;
    if (read_result.status == TransportStatus::kDisconnected) {
      result.status = RawProfileStatus::kDisconnected;
      result.delivery_unconfirmed = true;
      break;
    }
    if (read_result.status == TransportStatus::kOk) {
      if (read_result.transferred == 0U ||
          read_result.transferred > read_buffer.size()) {
        result.status = RawProfileStatus::kFeedbackInvalid;
        break;
      }
      result.statistics.raw_rx_bytes += read_result.transferred;
      if (!recordLine(operations,
                      rawChunkRecord(read_buffer.data(),
                                     read_result.transferred, receipt_ns,
                                     phase, definition),
                      &record_failed)) {
        result.status = RawProfileStatus::kRecordError;
        break;
      }
      const ParserStatistics before_statistics = parser.statistics();
      const std::vector<ParsedFeedbackFrame> frames = parser.consumeFrames(
          read_buffer.data(), read_result.transferred);
      const ParserStatistics after_statistics = parser.statistics();
      result.statistics.parser_checksum_failures =
          after_statistics.checksum_failures;
      result.statistics.parser_framing_failures =
          after_statistics.framing_failures;
      result.statistics.parser_discarded_bytes =
          after_statistics.discarded_bytes;
      result.statistics.parser_trailing_buffered_bytes =
          static_cast<std::uint64_t>(parser.bufferedBytes());
      if (!recordLine(
              operations,
              parserObservationRecord(
                  before_statistics, after_statistics,
                  parser.bufferedBytes(), receipt_ns, phase, definition),
              &record_failed)) {
        result.status = RawProfileStatus::kRecordError;
        break;
      }
      bool received_valid_allowed_frame = false;
      for (const ParsedFeedbackFrame& frame : frames) {
        if (!feedbackCandidateIsFinite(frame.candidate)) {
          result.status = RawProfileStatus::kFeedbackInvalid;
          break;
        }
        if (!frame.candidate.control_allowed ||
            frame.candidate.control_inhibited) {
          result.status = RawProfileStatus::kControlInhibited;
          break;
        }
        ++result.statistics.valid_feedback_frames;
        ++result.statistics.feedback_observation_events;
        last_feedback_ns = receipt_ns;
        received_valid_allowed_frame = true;
        if (!recordLine(
                operations,
                feedbackObservationRecord(
                    frame, receipt_ns, phase, definition,
                    last_successful_forward_wire,
                    last_successful_yaw_wire),
                &record_failed)) {
          result.status = RawProfileStatus::kRecordError;
          break;
        }
      }
      if (result.status != RawProfileStatus::kCompleted) {
        break;
      }
      if (phase == Phase::kBaseline && received_valid_allowed_frame &&
          receipt_ns > last_prearm_feedback_ns) {
        if (first_prearm_feedback_ns == 0) {
          first_prearm_feedback_ns = receipt_ns;
        }
        last_prearm_feedback_ns = receipt_ns;
        ++prearm_receipts;
      }
    } else if (read_result.status == TransportStatus::kWouldBlock ||
               read_result.status == TransportStatus::kDeadlineExceeded) {
      ++result.statistics.read_timeouts;
    } else {
      result.status = RawProfileStatus::kFeedbackInvalid;
      break;
    }

    if ((last_feedback_ns == 0 &&
         receipt_ns >= initial_feedback_deadline_ns) ||
        (last_feedback_ns > 0 &&
         receipt_ns - last_feedback_ns >
             config.maximum_feedback_age_ns)) {
      result.status = RawProfileStatus::kFeedbackMissing;
      break;
    }
    if (receipt_ns >= session_deadline_ns) {
      result.status = RawProfileStatus::kSessionDeadlineExceeded;
      break;
    }
    if (envelope.hasBaseline() &&
        receipt_ns - envelope.lastSuccessfulCompletionNs() >
            kMaximumCommandGapNs) {
      result.status = RawProfileStatus::kCommandWatchdogExpired;
      break;
    }

    if (phase == Phase::kBaseline &&
        prearm_receipts >= kRequiredFreshFeedbackReceipts &&
        first_prearm_feedback_ns > 0 &&
        receipt_ns - first_prearm_feedback_ns >=
            kMinimumPrearmObservationNs) {
      phase = Phase::kRampUp;
    }
    if (phase == Phase::kHold && receipt_ns >= hold_deadline_ns) {
      phase = Phase::kRampDown;
    }
    if (phase == Phase::kPostStop &&
        receipt_ns >= post_stop_deadline_ns) {
      break;
    }

    std::int16_t desired_forward_wire = 0;
    if (phase == Phase::kRampUp || phase == Phase::kHold) {
      desired_forward_wire = definition.target_forward_wire;
    }
    std::int16_t coupled_forward_wire = 0;
    if (!envelope.hasBaseline()) {
      coupled_forward_wire = 0;
    } else if (!envelope.coupleToward(
                   desired_forward_wire, receipt_ns,
                   &coupled_forward_wire)) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    if (!envelope.transitionAllowed(coupled_forward_wire, receipt_ns)) {
      result.status = RawProfileStatus::kCommandWatchdogExpired;
      break;
    }
    const RawProfileEncodeResult encoded = encodeRawProfileCommand(
        definition.profile, coupled_forward_wire);
    if (!encoded.ok()) {
      result.status = RawProfileStatus::kInvalidConfiguration;
      break;
    }
    const bool had_write_baseline = envelope.hasBaseline();
    const std::int64_t previous_write_completion_ns =
        envelope.lastSuccessfulCompletionNs();
    const std::int64_t write_deadline_ns = std::min(
        saturatedAdd(receipt_ns, kWriteTimeoutNs), session_deadline_ns);
    if (!recordLine(
            operations,
            ioAttemptRecord("normal_tx_attempt", phase, definition,
                            &encoded, encoded.frame.size(), receipt_ns,
                            write_deadline_ns),
            &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    ++result.statistics.tx_attempts;
    *write_stream_poisoned = true;
    const IoResult written = transport->writeAll(
        encoded.frame.data(), encoded.frame.size(), write_deadline_ns);
    const bool full_host_write =
        written.status == TransportStatus::kOk &&
        written.transferred == encoded.frame.size() &&
        !written.delivery_unconfirmed;
    const bool confirmed_no_delivery =
        written.transferred == 0U && !written.delivery_unconfirmed;
    if (full_host_write || confirmed_no_delivery) {
      *write_stream_poisoned = false;
    }
    const std::int64_t after_write_ns = operations.monotonic_now_ns();
    if (!recordLine(
            operations,
            ioResultRecord("normal_tx_result", phase, definition,
                           &encoded, written, receipt_ns, after_write_ns,
                           write_deadline_ns),
            &record_failed)) {
      result.status = RawProfileStatus::kRecordError;
      break;
    }
    if (full_host_write) {
      ++result.statistics.tx_host_writes_completed;
      ++result.statistics.normal_tx_host_writes_completed;
      if (coupled_forward_wire == 0) {
        ++result.statistics.zero_frame_host_writes_completed;
      } else {
        ++result.statistics.nonzero_frame_host_writes_completed;
        result.statistics.maximum_commanded_forward_wire = std::max(
            result.statistics.maximum_commanded_forward_wire,
            coupled_forward_wire);
      }
    }
    if (after_write_ns <= 0 || after_write_ns < receipt_ns) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    last_good_clock_ns = after_write_ns;
    if (written.status == TransportStatus::kDisconnected) {
      result.status = RawProfileStatus::kDisconnected;
      result.delivery_unconfirmed = true;
      break;
    }
    if (!full_host_write) {
      result.status = RawProfileStatus::kWriteFailed;
      result.delivery_unconfirmed = true;
      break;
    }
    if (after_write_ns > session_deadline_ns) {
      result.status = RawProfileStatus::kSessionDeadlineExceeded;
      break;
    }
    if (had_write_baseline &&
        after_write_ns - previous_write_completion_ns >
            kMaximumCommandGapNs) {
      result.status = RawProfileStatus::kCommandWatchdogExpired;
      break;
    }
    if (after_write_ns > write_deadline_ns) {
      result.status = RawProfileStatus::kWriteFailed;
      break;
    }
    if (!envelope.noteSuccessfulNormalWrite(
            coupled_forward_wire, after_write_ns)) {
      result.status = RawProfileStatus::kClockInvalid;
      break;
    }
    last_successful_forward_wire = encoded.forward_wire;
    last_successful_yaw_wire = encoded.yaw_wire;

    if (phase == Phase::kRampUp &&
        coupled_forward_wire == definition.target_forward_wire) {
      phase = Phase::kHold;
      hold_deadline_ns = saturatedAdd(
          after_write_ns, definition.hold_duration_ns);
    } else if (phase == Phase::kRampDown &&
               coupled_forward_wire == 0) {
      phase = Phase::kPostStop;
      post_stop_deadline_ns = saturatedAdd(
          after_write_ns, kPostZeroObservationNs);
    }
    next_cycle_ns = saturatedAdd(after_write_ns, kCommandPeriodNs);
  }

  const bool zero_written = boundedExactZeroWrite(
      transport, operations, definition, phase, last_good_clock_ns,
      &result.statistics, &result.delivery_unconfirmed, &record_failed,
      write_stream_poisoned);
  result.zero_host_write_completed = zero_written;
  if (result.status == RawProfileStatus::kCompleted && !zero_written) {
    result.status = RawProfileStatus::kZeroHostWriteFailed;
  }
  if (result.status == RawProfileStatus::kCompleted && record_failed) {
    result.status = RawProfileStatus::kRecordError;
  }
  const std::int64_t ended_ns = operations.monotonic_now_ns();
  if (ended_ns > 0) {
    result.statistics.ended_monotonic_ns = ended_ns;
  }
  return result;
}

}  // namespace

RawProfileResult WheeltecRawProfileSession::run(
    ByteTransport* transport, const RawProfileConfig& config,
    const RawProfileOperations& operations) {
  bool session_started = false;
  bool write_stream_poisoned = false;
  try {
    return runRawProfileSessionImpl(
        transport, config, operations, &session_started,
        &write_stream_poisoned);
  } catch (...) {
    RawProfileResult result;
    result.status = RawProfileStatus::kRecordError;
    if (!session_started || transport == nullptr) {
      return result;
    }
    RawProfileDefinition definition;
    if (!rawProfileDefinition(config.profile, &definition)) {
      return result;
    }
    bool record_failed = true;
    if (write_stream_poisoned) {
      result.delivery_unconfirmed = true;
    } else {
      try {
        result.zero_host_write_completed = boundedExactZeroWrite(
            transport, operations, definition, Phase::kBaseline, 0,
            &result.statistics, &result.delivery_unconfirmed,
            &record_failed, &write_stream_poisoned);
      } catch (...) {
        result.delivery_unconfirmed = true;
      }
    }
    try {
      const std::int64_t ended_ns =
          operations.monotonic_now_ns ? operations.monotonic_now_ns() : 0;
      if (ended_ns > 0) {
        result.statistics.ended_monotonic_ns = ended_ns;
      }
    } catch (...) {
      result.delivery_unconfirmed = true;
    }
    return result;
  }
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
