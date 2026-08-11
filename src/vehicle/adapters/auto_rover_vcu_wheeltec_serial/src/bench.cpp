#include "auto_rover_vcu_wheeltec_serial/bench.hpp"

#include "auto_rover_core/types.hpp"
#include "auto_rover_vcu_wheeltec_serial/adapter.hpp"
#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/feedback_capture.hpp"
#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace auto_rover {
namespace wheeltec_serial {

bool BenchWireAccelerationEnvelope::transitionAllowed(
    std::int16_t next_wire_speed,
    std::int64_t attempt_monotonic_ns) const {
  if (next_wire_speed < 0 ||
      next_wire_speed > kBenchMaximumForwardWireSpeed ||
      attempt_monotonic_ns <= 0) {
    return false;
  }
  if (!has_baseline_) {
    return next_wire_speed == 0;
  }
  if (attempt_monotonic_ns < last_successful_completion_ns_) {
    return false;
  }
  const std::int64_t delta_wire = std::abs(
      static_cast<std::int64_t>(next_wire_speed) -
      static_cast<std::int64_t>(last_successful_wire_speed_));
  const std::int64_t elapsed_ns =
      attempt_monotonic_ns - last_successful_completion_ns_;
  return delta_wire * kBenchWireUnitAccelerationPeriodNs <= elapsed_ns;
}

bool BenchWireAccelerationEnvelope::coupleToward(
    std::int16_t desired_wire_speed,
    std::int64_t request_monotonic_ns,
    std::int16_t* coupled_wire_speed) const {
  if (coupled_wire_speed == nullptr || desired_wire_speed < 0 ||
      desired_wire_speed > kBenchMaximumForwardWireSpeed ||
      request_monotonic_ns <= 0 || !has_baseline_ ||
      request_monotonic_ns < last_successful_completion_ns_) {
    return false;
  }
  const std::int64_t elapsed_ns =
      request_monotonic_ns - last_successful_completion_ns_;
  const std::int64_t available_delta = std::min<std::int64_t>(
      kBenchMaximumForwardWireSpeed,
      elapsed_ns / kBenchWireUnitAccelerationPeriodNs);
  const std::int64_t previous = last_successful_wire_speed_;
  const std::int64_t desired = desired_wire_speed;
  const std::int64_t lower = std::max<std::int64_t>(
      0, previous - available_delta);
  const std::int64_t upper = std::min<std::int64_t>(
      kBenchMaximumForwardWireSpeed, previous + available_delta);
  const std::int64_t coupled = std::max(lower, std::min(upper, desired));
  *coupled_wire_speed = static_cast<std::int16_t>(coupled);
  return true;
}

bool BenchWireAccelerationEnvelope::noteSuccessfulNormalWrite(
    std::int16_t wire_speed,
    std::int64_t completed_monotonic_ns) {
  if (wire_speed < 0 || wire_speed > kBenchMaximumForwardWireSpeed ||
      completed_monotonic_ns <= 0 ||
      (has_baseline_ &&
       completed_monotonic_ns < last_successful_completion_ns_)) {
    return false;
  }
  has_baseline_ = true;
  last_successful_wire_speed_ = wire_speed;
  last_successful_completion_ns_ = completed_monotonic_ns;
  return true;
}

namespace {

constexpr const char* kBenchSchema =
    "auto_rover.wheeltec.bench_characterization.v1";
constexpr std::size_t kReadBufferBytes = 512U;
constexpr std::uint32_t kRequiredFreshFeedbackReceipts = 5U;
constexpr std::uint32_t kRequiredFreshCommands = 3U;
constexpr std::int64_t kCommandPeriodNs = 20000000;
constexpr std::int64_t kMaximumCommandGapNs = 100000000;
constexpr std::int64_t kMinimumStandstillObservationNs = 200000000;
constexpr std::int64_t kPostZeroStandstillDeadlineNs = 1000000000;
constexpr std::int64_t kPreflightDrainDeadlineNs = 1000000;
constexpr std::uint32_t kMaximumPreflightDrainReads = 64U;
constexpr std::int64_t kZeroRetryIntervalNs = 20000000;
constexpr std::uint32_t kMaximumZeroWriteAttempts = 3U;
constexpr std::int64_t kWriteTimeoutNs = 10000000;
constexpr std::int64_t kMaximumExactZeroDurationNs = 30000000000LL;
constexpr std::int64_t kMinimumExactZeroDurationNs = 500000000LL;
constexpr std::int64_t kMaximumHoldDurationNs = 5000000000LL;
constexpr std::int64_t kMaximumRampSessionNs = 15000000000LL;
constexpr std::int64_t kMaximumRaisedWheelStartHoldDurationNs =
    60000000000LL;
constexpr std::int64_t kMaximumRaisedWheelStartSessionNs = 70000000000LL;
constexpr std::size_t kMaximumEvidenceTokenBytes = 256U;
constexpr double kBenchStandstillAbsForwardMps = 0.005;
constexpr double kBenchMaximumAuthorizedReverseForwardMps = 0.001;
constexpr double kBenchMaximumAbsLateralMps = 0.001;
constexpr double kBenchMaximumAbsYawRateRadps = 0.023;
constexpr double kBenchFeedbackOverspeedMarginMps = 0.001;
constexpr double kRaisedWheelStartRearTrackM = 0.322;
constexpr double kDerivedRearWheelSpeedToleranceMps = 0.005;
constexpr double kRaisedWheelStartTrackingOverspeedMarginMps = 0.020;
constexpr double kRawCalibrationFeedbackEmergencyCeilingMps = 1.000;

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
  constexpr std::size_t kSha256HexBytes = 64U;
  if (token.size() != required.size() + kSha256HexBytes ||
      token.size() > kMaximumEvidenceTokenBytes ||
      token.compare(0U, required.size(), required) != 0) {
    return false;
  }
  for (std::size_t index = required.size(); index < token.size(); ++index) {
    const unsigned char byte =
        static_cast<unsigned char>(token[index]);
    const bool hex = (byte >= '0' && byte <= '9') ||
                     (byte >= 'a' && byte <= 'f');
    if (!hex) {
      return false;
    }
  }
  return true;
}

bool gatesAreOpen(const BenchConfig& config) {
  return config.unverified_protocol_acknowledged &&
         config.physical_device_opt_in && config.actuation_opt_in &&
         config.operator_confirmation_token ==
             kBenchOperatorConfirmationToken;
}

bool evidenceIsPresent(const BenchConfig& config) {
  const bool passive = hasEvidencePrefix(config.passive_evidence_token,
                                         kPassiveEvidenceTokenPrefix);
  if (config.mode == BenchMode::kExactZero) {
    return passive && config.exact_zero_evidence_token.empty();
  }
  return passive &&
         hasEvidencePrefix(config.exact_zero_evidence_token,
                           kExactZeroEvidenceTokenPrefix);
}

std::int16_t readSignedBigEndian(std::uint8_t high, std::uint8_t low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low));
  const std::int32_t expanded =
      bits <= 0x7FFFU ? static_cast<std::int32_t>(bits)
                      : static_cast<std::int32_t>(bits) - 65536;
  return static_cast<std::int16_t>(expanded);
}

std::uint8_t xorBytes(const std::uint8_t* data, std::size_t size) {
  std::uint8_t checksum = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    checksum = static_cast<std::uint8_t>(checksum ^ data[index]);
  }
  return checksum;
}

bool commandFrameIsSafe(const std::uint8_t* data, std::size_t size,
                        bool motion_permitted, double* speed_mps,
                        bool* is_zero, std::int16_t* forward_wire) {
  if (data == nullptr || speed_mps == nullptr || is_zero == nullptr ||
      forward_wire == nullptr ||
      size != kCommandFrameSize || data[0U] != kFrameHeader ||
      data[10U] != kFrameTail || xorBytes(data, 9U) != data[9U]) {
    return false;
  }
  const std::int16_t forward = readSignedBigEndian(data[3U], data[4U]);
  const std::int16_t lateral = readSignedBigEndian(data[5U], data[6U]);
  const std::int16_t yaw = readSignedBigEndian(data[7U], data[8U]);
  *forward_wire = forward;
  *speed_mps = static_cast<double>(forward) / 1000.0;
  *is_zero = forward == 0 && lateral == 0 && yaw == 0;
  if (data[1U] != 0U || data[2U] != 0U || forward < 0 || forward > 500 ||
      lateral != 0 || yaw != 0) {
    return false;
  }
  if (!motion_permitted && !*is_zero) {
    return false;
  }
  return true;
}

std::string metadataRecord(const BenchConfig& config) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"metadata\",\"mode\":\""
         << benchModeName(config.mode)
         << "\",\"protocol_status\":\"UNVERIFIED\","
         << "\"characterization_only\":true,"
         << "\"acceptance_evidence\":false,"
         << "\"phase1_main_loop_connected\":false,"
         << "\"control_enabled_inferred\":false,"
         << "\"fault_state_inferred\":false,"
         << "\"vcu_ack_available\":false,"
         << "\"feedback_byte1_semantics\":\"binary_composite_current_cycle_control_allow_or_inhibit\","
         << "\"feedback_byte1_is_specific_fault\":false,"
         << "\"feedback_byte1_is_command_echo\":false,"
         << "\"successful_write_semantics\":\"host_os_full_write_only\","
         << "\"hard_max_forward_speed_mps\":";
  appendDouble(&stream, kBenchMaximumForwardSpeedMps);
  stream << ",\"fixed_acceleration_mps2\":";
  appendDouble(&stream, kBenchLongitudinalAccelerationMps2);
  stream << ",\"target_speed_mps\":";
  appendDouble(&stream, config.target_speed_mps);
  stream << ",\"hold_duration_ns\":" << config.hold_duration_ns
         << ",\"maximum_hold_duration_ns\":"
         << (config.allow_raised_wheel_start_asymmetry
                 ? kMaximumRaisedWheelStartHoldDurationNs
                 : kMaximumHoldDurationNs)
         << ",\"maximum_session_duration_ns\":"
         << (config.mode == BenchMode::kExactZero
                 ? config.exact_zero_duration_ns
                 : (config.allow_raised_wheel_start_asymmetry
                        ? kMaximumRaisedWheelStartSessionNs
                        : kMaximumRampSessionNs))
         << ",\"exact_zero_duration_ns\":"
         << config.exact_zero_duration_ns
         << ",\"fresh_feedback_receipts_required\":"
         << kRequiredFreshFeedbackReceipts
         << ",\"fresh_commands_required\":" << kRequiredFreshCommands
         << ",\"command_period_ns\":" << kCommandPeriodNs
         << ",\"normal_wire_acceleration_ns_per_raw_unit\":"
         << kBenchWireUnitAccelerationPeriodNs
         << ",\"normal_wire_acceleration_time_basis\":\"current_tx_before_minus_previous_successful_tx_after\","
         << "\"normal_wire_acceleration_integer_invariant\":\"abs(delta_raw)*5000000<=elapsed_ns\","
         << "\"normal_wire_acceleration_margin_raw\":0,"
         << "\"direct_emergency_zero_step_exception\":true"
         << ",\"allow_raised_wheel_start_asymmetry\":"
         << (config.allow_raised_wheel_start_asymmetry ? "true" : "false")
         << ",\"record_uncalibrated_motion_feedback\":"
         << (config.record_uncalibrated_motion_feedback ? "true" : "false")
         << ",\"tracking_deadband_asymmetry\":\""
         << (config.record_uncalibrated_motion_feedback
                 ? "recorded_not_gated"
                 : "gated")
         << "\""
         << ",\"uncalibrated_motion_feedback_scope\":\"authorized_motion_only\""
         << ",\"uncalibrated_motion_feedback_hold_basis\":\"user_requested_60s_raised_only\""
         << ",\"uncalibrated_motion_feedback_vcu_ack_available\":false"
         << ",\"raw_calibration_feedback_emergency_ceiling_mps\":";
  appendDouble(&stream, kRawCalibrationFeedbackEmergencyCeilingMps);
  stream << ",\"raw_calibration_feedback_emergency_ceiling_basis\":\"1.0_user_authorized_not_acceptance_limit\""
         << ",\"uncalibrated_motion_feedback_standstill_gate\":\"overall_and_each_derived_rear_abs_le_0.005_five_fresh_receipts_spanning_0.2s\""
         << ",\"raised_wheel_start_rear_track_m\":";
  appendDouble(&stream, kRaisedWheelStartRearTrackM);
  stream << ",\"derived_rear_wheel_speed_tolerance_mps\":";
  appendDouble(&stream, kDerivedRearWheelSpeedToleranceMps);
  stream << ",\"derived_rear_wheel_minimum_mps\":";
  appendDouble(&stream, -kDerivedRearWheelSpeedToleranceMps);
  stream << ",\"raised_wheel_start_overall_forward_minimum_mps\":";
  appendDouble(&stream, -kDerivedRearWheelSpeedToleranceMps);
  stream << ",\"ordinary_authorized_forward_minimum_mps\":";
  appendDouble(&stream, -kBenchMaximumAuthorizedReverseForwardMps);
  stream << ",\"derived_rear_wheel_maximum_mps\":";
  appendDouble(&stream, kBenchMaximumForwardSpeedMps);
  stream << ",\"derived_rear_wheel_reverse_quantization_tolerance_mps\":";
  appendDouble(&stream, kDerivedRearWheelSpeedToleranceMps);
  stream << ",\"raised_wheel_start_tracking_overspeed_margin_mps\":";
  appendDouble(&stream, kRaisedWheelStartTrackingOverspeedMarginMps);
  stream << ",\"raised_wheel_start_track_basis\":\"firmware_encoder_derived_UNVERIFIED\""
         << ",\"raised_wheel_start_tracking_overspeed_margin_basis\":\"observed_0.0122_plus_type9_encoder_count_0.0037905_plus_reconstruction_truncation_0.001161_equals_0.0171515_rounded_up_to_0.020_UNVERIFIED\""
         << ",\"raised_wheel_start_tracking_baseline\":\"last_successful_normal_host_write_wire_request\""
         << ",\"raised_wheel_start_gate_active_phases\":\"authorized_ramp_up_hold_ramp_down\""
         << ",\"strict_yaw_gate_active_phases\":\"all_phases_without_opt_in_and_prearm_post_zero_with_opt_in\""
         << ",\"feedback_threshold_scope\":\"bench_evidence_only_UNVERIFIED\","
         << "\"feedback_threshold_basis\":\"operator_attested_passive_capture_sha256_not_a_deployment_acceptance_limit\","
         << "\"standstill_abs_forward_mps\":"
         << kBenchStandstillAbsForwardMps << ','
         << "\"standstill_forward_threshold_basis\":\"one_type9_encoder_count_0.0037905_rounded_up_to_0.005_UNVERIFIED\","
         << "\"maximum_abs_lateral_mps\":"
         << kBenchMaximumAbsLateralMps << ','
         << "\"maximum_abs_yaw_rate_radps\":"
         << kBenchMaximumAbsYawRateRadps << ','
         << "\"feedback_overspeed_margin_mps\":"
         << kBenchFeedbackOverspeedMarginMps << ','
         << "\"relative_overspeed_gate_phases\":\"ramp_up_and_hold_only\","
         << "\"passive_evidence_token\":\""
         << config.passive_evidence_token
         << "\",\"exact_zero_evidence_token\":\""
         << config.exact_zero_evidence_token << '"'
         << '}';
  return stream.str();
}

std::string ioAttemptRecord(const char* type, const std::uint8_t* data,
                            std::size_t size, std::int64_t before_ns,
                            std::int64_t deadline_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"" << type
         << "\",\"clock\":\"CLOCK_MONOTONIC\","
         << "\"before_monotonic_ns\":" << before_ns
         << ",\"absolute_deadline_monotonic_ns\":" << deadline_ns
         << ",\"requested_bytes\":" << size;
  if (data != nullptr) {
    stream << ",\"raw_hex\":\"" << rawHex(data, size) << '"';
  }
  stream << '}';
  return stream.str();
}

std::string ioResultRecord(const char* type, const IoResult& result,
                           const std::uint8_t* data, std::size_t data_size,
                           std::int64_t before_ns, std::int64_t after_ns,
                           std::int64_t deadline_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"" << type
         << "\",\"clock\":\"CLOCK_MONOTONIC\","
         << "\"before_monotonic_ns\":" << before_ns
         << ",\"after_monotonic_ns\":" << after_ns
         << ",\"absolute_deadline_monotonic_ns\":" << deadline_ns
         << ",\"status\":\"" << transportStatusName(result.status)
         << "\",\"transferred\":" << result.transferred
         << ",\"os_error\":" << result.os_error
         << ",\"delivery_unconfirmed\":"
         << (result.delivery_unconfirmed ? "true" : "false");
  if (data != nullptr && data_size > 0U) {
    stream << ",\"raw_hex\":\"" << rawHex(data, data_size) << '"';
  }
  stream << '}';
  return stream.str();
}

std::string feedbackRecord(const ParsedFeedbackFrame& frame,
                           std::int64_t receipt_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"feedback_frame\","
         << "\"receipt_clock\":\"CLOCK_MONOTONIC\","
         << "\"receipt_monotonic_ns\":" << receipt_ns
         << ",\"raw_hex\":\""
         << rawHex(frame.raw_frame.data(), frame.raw_frame.size())
         << "\",\"forward_speed_mps\":";
  appendDouble(&stream, frame.candidate.forward_speed_mps);
  stream << ",\"lateral_speed_mps\":";
  appendDouble(&stream, frame.candidate.lateral_speed_mps);
  stream << ",\"yaw_rate_radps\":";
  appendDouble(&stream, frame.candidate.yaw_rate_radps);
  stream << ",\"supply_voltage_v\":";
  appendDouble(&stream, frame.candidate.supply_voltage_v);
  stream << ",\"composite_stop_flag_raw\":"
         << static_cast<unsigned int>(
                frame.candidate.composite_stop_flag_raw)
         << ",\"control_allowed\":"
         << (frame.candidate.control_allowed ? "true" : "false")
         << ",\"control_inhibited\":"
         << (frame.candidate.control_inhibited ? "true" : "false")
         << ",\"source_time_available\":false,"
         << "\"control_enabled_available\":false,"
         << "\"vcu_ack_available\":false,"
         << "\"specific_fault_available\":false,"
         << "\"command_echo_available\":false}";
  return stream.str();
}

class RecordingBenchTransport final : public ByteTransport {
 public:
  RecordingBenchTransport(ByteTransport* delegate,
                          const BenchOperations* operations,
                          BenchStatistics* statistics,
                          std::int64_t initial_clock_ns)
      : delegate_(delegate),
        operations_(operations),
        statistics_(statistics),
        last_clock_ns_(initial_clock_ns) {}

  bool isConnected() const override {
    return delegate_ != nullptr && delegate_->isConnected();
  }

  std::uint64_t connectionGeneration() const override {
    return delegate_ == nullptr ? 0U : delegate_->connectionGeneration();
  }

  IoResult writeAll(const std::uint8_t* data, std::size_t size,
                    std::int64_t absolute_deadline_ns) override {
    if (write_stream_poisoned_) {
      delivery_unconfirmed_ = true;
      return {TransportStatus::kIoError, 0U, 0, true};
    }
    ++statistics_->tx_attempts;
    const std::int64_t before_ns = observeClock();
    double speed_mps = 0.0;
    bool is_zero = false;
    std::int16_t forward_wire = 0;
    const bool frame_safe = commandFrameIsSafe(
        data, size, motion_permitted_, &speed_mps, &is_zero, &forward_wire);
    const bool acceleration_safe =
        frame_safe &&
        normal_wire_envelope_.transitionAllowed(forward_wire, before_ns);
    try {
      if (!record(ioAttemptRecord("tx_attempt", data, size, before_ns,
                                  absolute_deadline_ns))) {
        return {TransportStatus::kDisabled, 0U, 0, false};
      }
    } catch (...) {
      record_failed_ = true;
      return {TransportStatus::kDisabled, 0U, 0, false};
    }
    if (clock_invalid_ || !frame_safe || !acceleration_safe ||
        delegate_ == nullptr) {
      safety_rejected_ = !frame_safe || !acceleration_safe;
      const IoResult rejected{TransportStatus::kInvalidArgument, 0U, 0, true};
      record(ioResultRecord("tx_result", rejected, data, size, before_ns,
                            before_ns, absolute_deadline_ns));
      delivery_unconfirmed_ = true;
      return rejected;
    }
    if (!is_zero &&
        statistics_->nonzero_frame_host_writes_completed == 0U &&
        speed_mps >= kBenchMaximumForwardSpeedMps) {
      safety_rejected_ = true;
      const IoResult rejected{TransportStatus::kInvalidArgument, 0U, 0, true};
      record(ioResultRecord("tx_result", rejected, data, size, before_ns,
                            before_ns, absolute_deadline_ns));
      delivery_unconfirmed_ = true;
      return rejected;
    }

    IoResult result;
    try {
      result = delegate_->writeAll(data, size, absolute_deadline_ns);
    } catch (...) {
      // Unknown progress must be latched before clock/evidence code can throw.
      write_stream_poisoned_ = true;
      delivery_unconfirmed_ = true;
      return {TransportStatus::kIoError, 0U, 0, true};
    }
    const bool complete_host_write =
        result.status == TransportStatus::kOk &&
        result.transferred == size && !result.delivery_unconfirmed;
    if (result.delivery_unconfirmed ||
        (result.transferred > 0U && !complete_host_write)) {
      write_stream_poisoned_ = true;
      delivery_unconfirmed_ = true;
    }
    const std::int64_t after_ns = observeClock();
    if (result.delivery_unconfirmed) {
      delivery_unconfirmed_ = true;
    }
    if (complete_host_write) {
      if (!normal_wire_envelope_.noteSuccessfulNormalWrite(forward_wire,
                                                           after_ns)) {
        clock_invalid_ = true;
        delivery_unconfirmed_ = true;
      }
      ++statistics_->tx_host_writes_completed;
      if (is_zero) {
        ++statistics_->zero_frame_host_writes_completed;
      } else {
        ++statistics_->nonzero_frame_host_writes_completed;
        if (statistics_->first_nonzero_speed_mps == 0.0) {
          statistics_->first_nonzero_speed_mps = speed_mps;
        }
        statistics_->maximum_commanded_speed_mps =
            std::max(statistics_->maximum_commanded_speed_mps, speed_mps);
      }
    }
    try {
      (void)record(ioResultRecord("tx_result", result, data, size,
                                  before_ns, after_ns,
                                  absolute_deadline_ns));
    } catch (...) {
      // The transport outcome is already known.  Evidence allocation failure
      // cannot be allowed to turn a complete frame into unknown stream
      // progress or suppress the subsequent bounded stop.
      record_failed_ = true;
    }
    return result;
  }

  IoResult readSome(std::uint8_t* data, std::size_t capacity,
                    std::int64_t absolute_deadline_ns) override {
    ++statistics_->read_calls;
    const std::int64_t before_ns = observeClock();
    if (!record(ioAttemptRecord("rx_attempt", nullptr, capacity, before_ns,
                                absolute_deadline_ns))) {
      return {TransportStatus::kDisabled, 0U, 0, false};
    }
    if (clock_invalid_ || delegate_ == nullptr) {
      const IoResult rejected{TransportStatus::kInvalidArgument, 0U, 0, false};
      record(ioResultRecord("rx_result", rejected, nullptr, 0U, before_ns,
                            before_ns, absolute_deadline_ns));
      return rejected;
    }
    const IoResult result =
        delegate_->readSome(data, capacity, absolute_deadline_ns);
    const std::int64_t after_ns = observeClock();
    const std::size_t recorded_size =
        result.status == TransportStatus::kOk &&
                result.transferred <= capacity
            ? result.transferred
            : 0U;
    record(ioResultRecord("rx_result", result, data, recorded_size,
                          before_ns, after_ns, absolute_deadline_ns));
    return result;
  }

  IoResult emergencyExactZeroWrite(std::int64_t attempt_monotonic_ns,
                                   std::int64_t absolute_deadline_ns) {
    if (write_stream_poisoned_) {
      delivery_unconfirmed_ = true;
      return {TransportStatus::kIoError, 0U, 0, true};
    }
    CodecLimits emergency_limits;
    emergency_limits.max_forward_speed_mps =
        kBenchMaximumForwardSpeedMps;
    const EncodeResult encoded =
        encodeWireMotion(WireMotionCandidate{}, emergency_limits);
    ++statistics_->tx_attempts;
    const std::int64_t before_ns = attempt_monotonic_ns;
    if (!encoded.ok() || delegate_ == nullptr) {
      const IoResult rejected{TransportStatus::kInvalidArgument, 0U, 0, true};
      recordBestEffort(ioAttemptRecord(
          "emergency_zero_tx_attempt_postwrite_record", encoded.frame.data(),
          encoded.frame.size(), before_ns, absolute_deadline_ns));
      recordBestEffort(ioResultRecord(
          "emergency_zero_tx_result_postwrite_record", rejected,
          encoded.frame.data(),
          encoded.frame.size(), before_ns, before_ns,
          absolute_deadline_ns));
      delivery_unconfirmed_ = true;
      return rejected;
    }
    IoResult result;
    try {
      result = delegate_->writeAll(
          encoded.frame.data(), encoded.frame.size(), absolute_deadline_ns);
    } catch (...) {
      write_stream_poisoned_ = true;
      delivery_unconfirmed_ = true;
      return {TransportStatus::kIoError, 0U, 0, true};
    }
    const bool complete_host_write =
        result.status == TransportStatus::kOk &&
        result.transferred == encoded.frame.size() &&
        !result.delivery_unconfirmed;
    if (result.delivery_unconfirmed ||
        (result.transferred > 0U && !complete_host_write)) {
      write_stream_poisoned_ = true;
      delivery_unconfirmed_ = true;
    }
    std::int64_t after_ns = before_ns;
    try {
      after_ns = operations_ != nullptr && operations_->monotonic_now_ns
                     ? operations_->monotonic_now_ns()
                     : before_ns;
    } catch (...) {
      delivery_unconfirmed_ = true;
      result.delivery_unconfirmed = true;
    }
    if (after_ns <= 0 || after_ns < before_ns ||
        after_ns > absolute_deadline_ns) {
      delivery_unconfirmed_ = true;
      result.delivery_unconfirmed = true;
    }
    if (complete_host_write) {
      ++statistics_->tx_host_writes_completed;
      ++statistics_->zero_frame_host_writes_completed;
    } else {
      delivery_unconfirmed_ = true;
    }
    if (result.delivery_unconfirmed) {
      delivery_unconfirmed_ = true;
    }
    try {
      recordBestEffort(ioAttemptRecord(
          "emergency_zero_tx_attempt_postwrite_record", encoded.frame.data(),
          encoded.frame.size(), before_ns, absolute_deadline_ns));
      recordBestEffort(ioResultRecord(
          "emergency_zero_tx_result_postwrite_record", result,
          encoded.frame.data(), encoded.frame.size(), before_ns, after_ns,
          absolute_deadline_ns));
    } catch (...) {
      record_failed_ = true;
    }
    return result;
  }

  void setMotionPermitted(bool permitted) { motion_permitted_ = permitted; }
  bool coupleNormalWireSpeed(std::int16_t desired_wire_speed,
                            std::int64_t request_monotonic_ns,
                            std::int16_t* coupled_wire_speed) const {
    return normal_wire_envelope_.coupleToward(
        desired_wire_speed, request_monotonic_ns, coupled_wire_speed);
  }
  bool lastSuccessfulNormalWireSpeed(std::int16_t* wire_speed) const {
    if (wire_speed == nullptr || !normal_wire_envelope_.hasBaseline()) {
      return false;
    }
    *wire_speed = normal_wire_envelope_.lastSuccessfulWireSpeed();
    return true;
  }
  bool recordFailed() const { return record_failed_; }
  bool clockInvalid() const { return clock_invalid_; }
  bool safetyRejected() const { return safety_rejected_; }
  bool deliveryUnconfirmed() const { return delivery_unconfirmed_; }
  bool writeStreamPoisoned() const { return write_stream_poisoned_; }

 private:
  std::int64_t observeClock() noexcept {
    if (operations_ == nullptr || !operations_->monotonic_now_ns) {
      clock_invalid_ = true;
      return 0;
    }
    std::int64_t observed = 0;
    try {
      observed = operations_->monotonic_now_ns();
    } catch (...) {
      clock_invalid_ = true;
      return 0;
    }
    if (observed <= 0 || observed < last_clock_ns_) {
      clock_invalid_ = true;
    } else {
      last_clock_ns_ = observed;
    }
    return observed;
  }

  bool record(const std::string& line) noexcept {
    if (record_failed_ || operations_ == nullptr ||
        !operations_->record_json_line) {
      record_failed_ = true;
      return false;
    }
    try {
      if (!operations_->record_json_line(line)) {
        record_failed_ = true;
        return false;
      }
      return true;
    } catch (...) {
      record_failed_ = true;
      return false;
    }
  }

  void recordBestEffort(const std::string& line) noexcept {
    if (operations_ == nullptr || !operations_->record_json_line) {
      record_failed_ = true;
      return;
    }
    try {
      if (!operations_->record_json_line(line)) {
        record_failed_ = true;
      }
    } catch (...) {
      record_failed_ = true;
    }
  }

  ByteTransport* delegate_{nullptr};
  const BenchOperations* operations_{nullptr};
  BenchStatistics* statistics_{nullptr};
  std::int64_t last_clock_ns_{0};
  BenchWireAccelerationEnvelope normal_wire_envelope_{};
  bool motion_permitted_{false};
  bool record_failed_{false};
  bool clock_invalid_{false};
  bool safety_rejected_{false};
  bool delivery_unconfirmed_{false};
  bool write_stream_poisoned_{false};
};

BenchStatus statusForCycle(const AdapterCycleResult& cycle,
                           const RecordingBenchTransport& transport) {
  if (transport.writeStreamPoisoned()) {
    return BenchStatus::kWriteFailed;
  }
  if (transport.recordFailed()) {
    return BenchStatus::kRecordError;
  }
  if (transport.clockInvalid() ||
      cycle.action == AdapterCycleAction::kClockInvalid) {
    return BenchStatus::kClockInvalid;
  }
  if (transport.safetyRejected()) {
    return BenchStatus::kAdapterRejected;
  }
  if (cycle.action == AdapterCycleAction::kDisconnected ||
      cycle.transport_status == TransportStatus::kDisconnected) {
    return BenchStatus::kDisconnected;
  }
  if (cycle.action == AdapterCycleAction::kMotionWriteFailed ||
      cycle.action == AdapterCycleAction::kZeroWriteFailed ||
      cycle.action == AdapterCycleAction::kZeroRetriesExhausted ||
      cycle.action == AdapterCycleAction::kConfigurationInvalid ||
      cycle.delivery_unconfirmed) {
    return BenchStatus::kWriteFailed;
  }
  return BenchStatus::kCompleted;
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

std::int64_t saturatedAdd(std::int64_t first, std::int64_t second) {
  if (second > 0 &&
      first > std::numeric_limits<std::int64_t>::max() - second) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return first + second;
}

bool boundedZeroHostWrite(WheeltecSerialAdapter* adapter,
                          RecordingBenchTransport* transport,
                          const BenchOperations& operations,
                          std::int64_t last_good_clock_ns) noexcept {
  if (adapter == nullptr || transport == nullptr ||
      !operations.monotonic_now_ns || !operations.wait_until_monotonic_ns) {
    return false;
  }
  transport->setMotionPermitted(false);
  std::int64_t now_ns = last_good_clock_ns;
  try {
    const std::int64_t observed = operations.monotonic_now_ns();
    if (observed > 0 && observed >= last_good_clock_ns) {
      now_ns = observed;
    }
  } catch (...) {
    // The last validated monotonic sample still gives the immediate zero a
    // bounded deadline; no evidence callback precedes that write.
  }
  try {
    if (now_ns > 0) {
      (void)adapter->setAuthorization(false, false, 0U, now_ns);
    }
  } catch (...) {
    // The emergency path below is transport-local and does not depend on the
    // adapter accepting this local authorization revocation.
  }
  if (transport->writeStreamPoisoned()) {
    return false;
  }
  const std::int64_t deadline_base_ns =
      std::max(last_good_clock_ns, now_ns > 0 ? now_ns : 0);
  for (std::uint32_t attempt = 0U;
       attempt < kMaximumZeroWriteAttempts; ++attempt) {
    if (transport->writeStreamPoisoned()) {
      return false;
    }
    const std::int64_t attempt_offset =
        static_cast<std::int64_t>(attempt) * kZeroRetryIntervalNs;
    const std::int64_t attempt_time_ns =
        saturatedAdd(deadline_base_ns, attempt_offset);
    if (attempt > 0U) {
      try {
        if (!operations.wait_until_monotonic_ns(attempt_time_ns)) {
          return false;
        }
      } catch (...) {
        return false;
      }
    }
    const std::int64_t write_deadline_ns =
        saturatedAdd(attempt_time_ns, kWriteTimeoutNs);
    const IoResult write =
        transport->emergencyExactZeroWrite(attempt_time_ns,
                                           write_deadline_ns);
    if (write.status == TransportStatus::kOk &&
        write.transferred == kCommandFrameSize &&
        !write.delivery_unconfirmed) {
      return true;
    }
    if (transport->writeStreamPoisoned() ||
        write.delivery_unconfirmed || write.transferred > 0U ||
        write.status == TransportStatus::kDisconnected) {
      return false;
    }
  }
  return false;
}

}  // namespace

const char* benchModeName(BenchMode mode) {
  switch (mode) {
    case BenchMode::kExactZero:
      return "exact-zero";
    case BenchMode::kStraightRamp:
      return "straight-ramp";
    case BenchMode::kDisabled:
      return "disabled";
  }
  return "unknown";
}

const char* benchStatusName(BenchStatus status) {
  switch (status) {
    case BenchStatus::kCompleted:
      return "completed";
    case BenchStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case BenchStatus::kAuthorizationDenied:
      return "authorization_denied";
    case BenchStatus::kEvidenceMissing:
      return "evidence_missing";
    case BenchStatus::kClockInvalid:
      return "clock_invalid";
    case BenchStatus::kDisconnected:
      return "disconnected";
    case BenchStatus::kFeedbackMissing:
      return "feedback_missing";
    case BenchStatus::kFeedbackInvalid:
      return "feedback_invalid";
    case BenchStatus::kFeedbackLimitViolation:
      return "feedback_limit_violation";
    case BenchStatus::kStandstillViolation:
      return "standstill_violation";
    case BenchStatus::kAdapterRejected:
      return "adapter_rejected";
    case BenchStatus::kWriteFailed:
      return "write_failed";
    case BenchStatus::kRecordError:
      return "record_error";
    case BenchStatus::kZeroHostWriteFailed:
      return "zero_host_write_failed";
    case BenchStatus::kInterrupted:
      return "interrupted";
    case BenchStatus::kSessionDeadlineExceeded:
      return "session_deadline_exceeded";
  }
  return "unknown";
}

bool benchConfigIsValid(const BenchConfig& config) {
  if (config.read_timeout_ns <= 0 ||
      config.read_timeout_ns > kCommandPeriodNs ||
      config.maximum_feedback_age_ns < config.read_timeout_ns ||
      config.initial_feedback_deadline_ns <= 0 ||
      config.command_valid_for_ns <= config.read_timeout_ns ||
      config.command_valid_for_ns > 1000000000LL) {
    return false;
  }
  if (config.mode == BenchMode::kExactZero) {
    return config.exact_zero_duration_ns >= kMinimumExactZeroDurationNs &&
           config.exact_zero_duration_ns <= kMaximumExactZeroDurationNs &&
           config.target_speed_mps == 0.0 &&
           config.hold_duration_ns == 0 &&
           !config.allow_raised_wheel_start_asymmetry &&
           !config.record_uncalibrated_motion_feedback;
  }
  if (config.mode == BenchMode::kStraightRamp) {
    const std::int64_t maximum_hold_duration_ns =
        config.allow_raised_wheel_start_asymmetry
            ? kMaximumRaisedWheelStartHoldDurationNs
            : kMaximumHoldDurationNs;
    return config.exact_zero_duration_ns == 0 &&
           std::isfinite(config.target_speed_mps) &&
           config.target_speed_mps >= 0.001 &&
           config.target_speed_mps <= kBenchMaximumForwardSpeedMps &&
           config.hold_duration_ns > 0 &&
           config.hold_duration_ns <= maximum_hold_duration_ns &&
           (!config.record_uncalibrated_motion_feedback ||
            config.allow_raised_wheel_start_asymmetry);
  }
  return false;
}

BenchStatus benchPreflightStatus(const BenchConfig& config) {
  if (!benchConfigIsValid(config)) {
    return BenchStatus::kInvalidConfiguration;
  }
  if (!gatesAreOpen(config)) {
    return BenchStatus::kAuthorizationDenied;
  }
  if (!evidenceIsPresent(config)) {
    return BenchStatus::kEvidenceMissing;
  }
  return BenchStatus::kCompleted;
}

std::string benchSummaryRecordJson(const BenchResult& result) {
  const BenchStatistics& statistics = result.statistics;
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kBenchSchema
         << "\",\"record_type\":\"summary\",\"status\":\""
         << benchStatusName(result.status)
         << "\",\"authorization_revoked\":"
         << (result.authorization_revoked ? "true" : "false")
         << ",\"zero_host_write_completed\":"
         << (result.zero_host_write_completed ? "true" : "false")
         << ",\"delivery_unconfirmed\":"
         << (result.delivery_unconfirmed ? "true" : "false")
         << ",\"statistics\":{\"read_calls\":"
         << statistics.read_calls << ",\"raw_rx_bytes\":"
         << statistics.raw_rx_bytes << ",\"valid_feedback_frames\":"
         << statistics.valid_feedback_frames << ",\"tx_attempts\":"
         << statistics.tx_attempts << ",\"tx_host_writes_completed\":"
         << statistics.tx_host_writes_completed
         << ",\"zero_frame_host_writes_completed\":"
         << statistics.zero_frame_host_writes_completed
         << ",\"nonzero_frame_host_writes_completed\":"
         << statistics.nonzero_frame_host_writes_completed
         << ",\"motion_commands_submitted\":"
         << statistics.motion_commands_submitted
         << ",\"maximum_feedback_recovery_run\":"
         << statistics.maximum_feedback_recovery_run
         << ",\"first_nonzero_speed_mps\":";
  appendDouble(&stream, statistics.first_nonzero_speed_mps);
  stream << ",\"maximum_commanded_speed_mps\":";
  appendDouble(&stream, statistics.maximum_commanded_speed_mps);
  stream << ",\"started_monotonic_ns\":"
         << statistics.started_monotonic_ns
         << ",\"ended_monotonic_ns\":"
         << statistics.ended_monotonic_ns << "}}";
  return stream.str();
}

BenchResult WheeltecBenchSession::run(
    ByteTransport* transport, const BenchConfig& config,
    const BenchOperations& operations) {
  BenchResult result;
  if (transport == nullptr ||
      !operations.monotonic_now_ns ||
      !operations.wait_until_monotonic_ns ||
      !operations.record_json_line || !operations.stop_requested) {
    return result;
  }
  const BenchStatus preflight_status = benchPreflightStatus(config);
  if (preflight_status != BenchStatus::kCompleted) {
    result.status = preflight_status;
    return result;
  }
  if (!transport->isConnected()) {
    result.status = BenchStatus::kDisconnected;
    result.delivery_unconfirmed = true;
    return result;
  }

  RecordingBenchTransport recording_transport(
      transport, &operations, &result.statistics, 0);
  AdapterConfig adapter_config;
  adapter_config.codec_limits.max_forward_speed_mps =
      kBenchMaximumForwardSpeedMps;
  adapter_config.codec_limits.max_abs_curvature_inv_m =
      kPhase1MaximumAbsCurvatureInvM;
  adapter_config.max_command_age_ns = config.command_valid_for_ns;
  adapter_config.fresh_commands_required = kRequiredFreshCommands;
  adapter_config.zero_retry_interval_ns = kZeroRetryIntervalNs;
  adapter_config.max_zero_write_attempts = kMaximumZeroWriteAttempts;
  adapter_config.write_timeout_ns = kWriteTimeoutNs;
  WheeltecSerialAdapter adapter(adapter_config, &recording_transport);

  std::int64_t last_good_clock_ns = 0;
  try {
  const std::int64_t started_ns = operations.monotonic_now_ns();
  result.statistics.started_monotonic_ns = started_ns;
  result.statistics.ended_monotonic_ns = started_ns;
  if (started_ns <= 0 ||
      !operations.record_json_line(metadataRecord(config))) {
    result.status = started_ns <= 0 ? BenchStatus::kClockInvalid
                                    : BenchStatus::kRecordError;
    return result;
  }
  last_good_clock_ns = started_ns;
  std::array<std::uint8_t, kReadBufferBytes> read_buffer{};
  result.status = BenchStatus::kCompleted;

  bool drain_complete = false;
  for (std::uint32_t drain_read = 0U;
       drain_read < kMaximumPreflightDrainReads; ++drain_read) {
    const std::int64_t before_drain_ns = operations.monotonic_now_ns();
    if (before_drain_ns <= 0 || before_drain_ns < last_good_clock_ns) {
      adapter.cycle(before_drain_ns);
      result.status = BenchStatus::kClockInvalid;
      break;
    }
    const IoResult drained = recording_transport.readSome(
        read_buffer.data(), read_buffer.size(),
        saturatedAdd(before_drain_ns, kPreflightDrainDeadlineNs));
    const std::int64_t after_drain_ns = operations.monotonic_now_ns();
    if (after_drain_ns <= 0 || after_drain_ns < before_drain_ns ||
        recording_transport.clockInvalid()) {
      adapter.cycle(after_drain_ns);
      result.status = BenchStatus::kClockInvalid;
      break;
    }
    last_good_clock_ns = after_drain_ns;
    if (recording_transport.recordFailed()) {
      result.status = BenchStatus::kRecordError;
      break;
    }
    if (drained.status == TransportStatus::kOk) {
      if (drained.transferred == 0U ||
          drained.transferred > read_buffer.size()) {
        result.status = BenchStatus::kFeedbackInvalid;
        break;
      }
      result.statistics.raw_rx_bytes += drained.transferred;
      continue;
    }
    if (drained.status == TransportStatus::kWouldBlock ||
        drained.status == TransportStatus::kDeadlineExceeded) {
      drain_complete = true;
      break;
    }
    if (drained.status == TransportStatus::kDisconnected) {
      result.status = BenchStatus::kDisconnected;
      result.delivery_unconfirmed = true;
    } else {
      result.status = BenchStatus::kFeedbackInvalid;
    }
    break;
  }
  if (result.status == BenchStatus::kCompleted && !drain_complete) {
    result.status = BenchStatus::kFeedbackInvalid;
  }

  const std::int64_t observation_started_ns = operations.monotonic_now_ns();
  if (result.status == BenchStatus::kCompleted &&
      (observation_started_ns <= 0 ||
       observation_started_ns < last_good_clock_ns)) {
    adapter.cycle(observation_started_ns);
    result.status = BenchStatus::kClockInvalid;
  }
  if (observation_started_ns > last_good_clock_ns) {
    last_good_clock_ns = observation_started_ns;
  }
  const std::int64_t session_duration_ns =
      config.mode == BenchMode::kExactZero
          ? config.exact_zero_duration_ns
          : (config.allow_raised_wheel_start_asymmetry
                 ? kMaximumRaisedWheelStartSessionNs
                 : kMaximumRampSessionNs);
  const std::int64_t session_deadline_ns =
      saturatedAdd(observation_started_ns, session_duration_ns);
  const std::int64_t initial_feedback_deadline_ns = saturatedAdd(
      observation_started_ns, config.initial_feedback_deadline_ns);
  std::int64_t next_control_ns =
      saturatedAdd(observation_started_ns, kCommandPeriodNs);
  std::int64_t last_feedback_ns = 0;
  std::int64_t first_standstill_receipt_ns = 0;
  std::int64_t last_command_receipt_ns = 0;
  std::int64_t hold_deadline_ns = 0;
  std::int64_t post_zero_deadline_ns = 0;
  std::uint32_t standstill_receipts = 0U;
  std::uint64_t next_sequence = 1U;
  const std::int16_t target_wire_speed = static_cast<std::int16_t>(
      std::trunc(config.target_speed_mps * 1000.0));
  double last_requested_speed_mps = 0.0;
  bool authorized = false;
  bool holding_target = false;
  bool ramping_down = false;
  bool post_zero_observation = false;
  FeedbackStreamParser parser;

  while (result.status == BenchStatus::kCompleted) {
    if (operations.stop_requested()) {
      result.status = BenchStatus::kInterrupted;
      break;
    }
    std::int64_t before_read_ns = operations.monotonic_now_ns();
    if (before_read_ns <= 0 || before_read_ns < last_good_clock_ns) {
      adapter.cycle(before_read_ns);
      result.status = BenchStatus::kClockInvalid;
      break;
    }
    if (before_read_ns < next_control_ns) {
      if (!operations.wait_until_monotonic_ns(next_control_ns)) {
        result.status = BenchStatus::kSessionDeadlineExceeded;
        break;
      }
      before_read_ns = operations.monotonic_now_ns();
    }
    if (before_read_ns <= 0 || before_read_ns < last_good_clock_ns) {
      adapter.cycle(before_read_ns);
      result.status = BenchStatus::kClockInvalid;
      break;
    }
    if (before_read_ns >
        saturatedAdd(next_control_ns, kMaximumCommandGapNs)) {
      result.status = BenchStatus::kSessionDeadlineExceeded;
      break;
    }
    last_good_clock_ns = before_read_ns;
    if (before_read_ns >= session_deadline_ns) {
      if (config.mode == BenchMode::kExactZero &&
          standstill_receipts >= kRequiredFreshFeedbackReceipts &&
          first_standstill_receipt_ns > 0 &&
          before_read_ns - first_standstill_receipt_ns >=
              kMinimumStandstillObservationNs &&
          last_feedback_ns > 0 &&
          before_read_ns - last_feedback_ns <=
              config.maximum_feedback_age_ns) {
        result.status = BenchStatus::kCompleted;
      } else if (config.mode == BenchMode::kExactZero) {
        result.status = BenchStatus::kFeedbackMissing;
      } else {
        result.status = BenchStatus::kSessionDeadlineExceeded;
      }
      break;
    }
    const std::int64_t read_deadline_ns = std::min(
        saturatedAdd(before_read_ns, config.read_timeout_ns),
        session_deadline_ns);
    const IoResult read_result = recording_transport.readSome(
        read_buffer.data(), read_buffer.size(), read_deadline_ns);
    const std::int64_t receipt_ns = operations.monotonic_now_ns();
    result.statistics.ended_monotonic_ns = receipt_ns;
    if (receipt_ns <= 0 || receipt_ns < before_read_ns ||
        receipt_ns < last_good_clock_ns || recording_transport.clockInvalid()) {
      adapter.cycle(receipt_ns);
      result.status = BenchStatus::kClockInvalid;
      break;
    }
    if (receipt_ns - before_read_ns > kMaximumCommandGapNs) {
      result.status = BenchStatus::kSessionDeadlineExceeded;
      break;
    }
    last_good_clock_ns = receipt_ns;
    next_control_ns = std::max(
        next_control_ns, saturatedAdd(receipt_ns, kCommandPeriodNs));
    if (recording_transport.recordFailed()) {
      result.status = BenchStatus::kRecordError;
      break;
    }
    if (read_result.status == TransportStatus::kDisconnected) {
      result.status = BenchStatus::kDisconnected;
      result.delivery_unconfirmed = true;
      break;
    }
    if (config.mode == BenchMode::kStraightRamp &&
        receipt_ns >= session_deadline_ns) {
      result.status = BenchStatus::kSessionDeadlineExceeded;
      break;
    }
    bool received_valid_frame = false;
    bool received_standstill_only = true;
    if (read_result.status == TransportStatus::kOk) {
      if (read_result.transferred == 0U ||
          read_result.transferred > read_buffer.size()) {
        result.status = BenchStatus::kFeedbackInvalid;
        break;
      }
      result.statistics.raw_rx_bytes += read_result.transferred;
      const ParserStatistics before_statistics = parser.statistics();
      const std::vector<ParsedFeedbackFrame> frames = parser.consumeFrames(
          read_buffer.data(), read_result.transferred);
      const ParserStatistics after_statistics = parser.statistics();
      if (after_statistics.checksum_failures !=
              before_statistics.checksum_failures ||
          after_statistics.framing_failures !=
              before_statistics.framing_failures ||
          after_statistics.discarded_bytes !=
              before_statistics.discarded_bytes) {
        result.status = BenchStatus::kFeedbackInvalid;
        break;
      }
      for (const ParsedFeedbackFrame& frame : frames) {
        if (!feedbackCandidateIsFinite(frame.candidate)) {
          result.status = BenchStatus::kFeedbackInvalid;
          break;
        }
        if (!frame.candidate.control_allowed ||
            frame.candidate.control_inhibited) {
          result.status = BenchStatus::kFeedbackInvalid;
          break;
        }
        const bool raised_wheel_start_gate_active =
            config.allow_raised_wheel_start_asymmetry && authorized &&
            !post_zero_observation;
        const bool standstill_feedback_phase =
            config.mode == BenchMode::kExactZero || !authorized ||
            post_zero_observation;
        const double maximum_reverse_forward_mps =
            standstill_feedback_phase || raised_wheel_start_gate_active
                ? kBenchStandstillAbsForwardMps
                : kBenchMaximumAuthorizedReverseForwardMps;
        const double maximum_feedback_forward_mps =
            config.record_uncalibrated_motion_feedback
                ? kRawCalibrationFeedbackEmergencyCeilingMps
                : kBenchMaximumForwardSpeedMps;
        if (frame.candidate.forward_speed_mps <
                -maximum_reverse_forward_mps ||
            frame.candidate.forward_speed_mps >
                maximum_feedback_forward_mps) {
          result.status = BenchStatus::kFeedbackLimitViolation;
          break;
        }
        if (!raised_wheel_start_gate_active && authorized &&
            !post_zero_observation && !ramping_down &&
            frame.candidate.forward_speed_mps >
                last_requested_speed_mps +
                    kBenchFeedbackOverspeedMarginMps) {
          result.status = BenchStatus::kFeedbackLimitViolation;
          break;
        }
        if (std::abs(frame.candidate.lateral_speed_mps) >
                kBenchMaximumAbsLateralMps ||
            (!raised_wheel_start_gate_active &&
             std::abs(frame.candidate.yaw_rate_radps) >
                 kBenchMaximumAbsYawRateRadps)) {
          result.status = BenchStatus::kFeedbackLimitViolation;
          break;
        }
        if (raised_wheel_start_gate_active) {
          const double half_track_m = kRaisedWheelStartRearTrackM * 0.5;
          const double left_rear_speed_mps =
              frame.candidate.forward_speed_mps -
              frame.candidate.yaw_rate_radps * half_track_m;
          const double right_rear_speed_mps =
              frame.candidate.forward_speed_mps +
              frame.candidate.yaw_rate_radps * half_track_m;
          std::int16_t successful_wire_request = 0;
          if (!recording_transport.lastSuccessfulNormalWireSpeed(
                  &successful_wire_request)) {
            result.status = BenchStatus::kFeedbackLimitViolation;
            break;
          }
          const double successful_request_mps =
              static_cast<double>(successful_wire_request) / 1000.0;
          const double relative_maximum_mps = std::min(
              kBenchMaximumForwardSpeedMps,
              successful_request_mps +
                  kRaisedWheelStartTrackingOverspeedMarginMps);
          if (!std::isfinite(left_rear_speed_mps) ||
              !std::isfinite(right_rear_speed_mps) ||
              left_rear_speed_mps <
                  -kDerivedRearWheelSpeedToleranceMps ||
              right_rear_speed_mps <
                  -kDerivedRearWheelSpeedToleranceMps ||
              left_rear_speed_mps > maximum_feedback_forward_mps ||
              right_rear_speed_mps > maximum_feedback_forward_mps ||
              (!ramping_down &&
               !config.record_uncalibrated_motion_feedback &&
               (left_rear_speed_mps > relative_maximum_mps ||
                right_rear_speed_mps > relative_maximum_mps))) {
            result.status = BenchStatus::kFeedbackLimitViolation;
            break;
          }
        }
        bool frame_standstill =
            std::abs(frame.candidate.forward_speed_mps) <=
                kBenchStandstillAbsForwardMps;
        if (config.record_uncalibrated_motion_feedback &&
            standstill_feedback_phase) {
          const double half_track_m = kRaisedWheelStartRearTrackM * 0.5;
          const double left_rear_speed_mps =
              frame.candidate.forward_speed_mps -
              frame.candidate.yaw_rate_radps * half_track_m;
          const double right_rear_speed_mps =
              frame.candidate.forward_speed_mps +
              frame.candidate.yaw_rate_radps * half_track_m;
          frame_standstill =
              frame_standstill &&
              std::abs(left_rear_speed_mps) <=
                  kDerivedRearWheelSpeedToleranceMps &&
              std::abs(right_rear_speed_mps) <=
                  kDerivedRearWheelSpeedToleranceMps;
        }
        received_standstill_only =
            received_standstill_only && frame_standstill;
        if ((config.mode == BenchMode::kExactZero ||
             (!authorized && !post_zero_observation)) &&
            !frame_standstill) {
          result.status = BenchStatus::kStandstillViolation;
          break;
        }
        if (!operations.record_json_line(feedbackRecord(frame, receipt_ns))) {
          result.status = BenchStatus::kRecordError;
          break;
        }
        ++result.statistics.valid_feedback_frames;
        received_valid_frame = true;
      }
      if (result.status != BenchStatus::kCompleted) {
        break;
      }
    } else if (read_result.status != TransportStatus::kWouldBlock &&
               read_result.status != TransportStatus::kDeadlineExceeded) {
      result.status = BenchStatus::kFeedbackInvalid;
      break;
    }

    if (received_valid_frame) {
      last_feedback_ns = receipt_ns;
      if (received_standstill_only) {
        if (standstill_receipts == 0U) {
          first_standstill_receipt_ns = receipt_ns;
        }
        if (standstill_receipts < kRequiredFreshFeedbackReceipts) {
          ++standstill_receipts;
        }
      } else {
        standstill_receipts = 0U;
        first_standstill_receipt_ns = 0;
      }
      result.statistics.maximum_feedback_recovery_run =
          std::max(result.statistics.maximum_feedback_recovery_run,
                   standstill_receipts);
    } else {
      if ((last_feedback_ns == 0 &&
           receipt_ns >= initial_feedback_deadline_ns) ||
          (last_feedback_ns > 0 &&
           receipt_ns - last_feedback_ns >
               config.maximum_feedback_age_ns)) {
        result.status = BenchStatus::kFeedbackMissing;
        break;
      }
    }

    const bool standstill_ready =
        standstill_receipts >= kRequiredFreshFeedbackReceipts &&
        first_standstill_receipt_ns > 0 &&
        receipt_ns - first_standstill_receipt_ns >=
            kMinimumStandstillObservationNs;

    if (config.mode == BenchMode::kExactZero) {
      recording_transport.setMotionPermitted(false);
      if (!adapter.setAuthorization(false, false, 0U, receipt_ns)) {
        result.status = BenchStatus::kAdapterRejected;
        break;
      }
      const AdapterCycleResult cycle = adapter.cycle(receipt_ns);
      const BenchStatus cycle_status =
          statusForCycle(cycle, recording_transport);
      if (cycle_status != BenchStatus::kCompleted ||
          cycle.action != AdapterCycleAction::kZeroWritten) {
        result.status = cycle_status == BenchStatus::kCompleted
                            ? BenchStatus::kWriteFailed
                            : cycle_status;
        break;
      }
      continue;
    }

    if (post_zero_observation) {
      recording_transport.setMotionPermitted(false);
      if (!adapter.setAuthorization(false, false, 0U, receipt_ns)) {
        result.status = BenchStatus::kAdapterRejected;
        break;
      }
      const AdapterCycleResult cycle = adapter.cycle(receipt_ns);
      const BenchStatus cycle_status =
          statusForCycle(cycle, recording_transport);
      if (cycle_status != BenchStatus::kCompleted ||
          cycle.action != AdapterCycleAction::kZeroWritten) {
        result.status = cycle_status == BenchStatus::kCompleted
                            ? BenchStatus::kWriteFailed
                            : cycle_status;
        break;
      }
      if (standstill_ready) {
        result.status = BenchStatus::kCompleted;
        break;
      }
      if (receipt_ns >= post_zero_deadline_ns) {
        result.status = BenchStatus::kStandstillViolation;
        break;
      }
      continue;
    }

    if (!authorized) {
      recording_transport.setMotionPermitted(false);
      if (!standstill_ready) {
        if (!adapter.setAuthorization(false, false, 0U, receipt_ns)) {
          result.status = BenchStatus::kAdapterRejected;
          break;
        }
        const AdapterCycleResult cycle = adapter.cycle(receipt_ns);
        const BenchStatus cycle_status =
            statusForCycle(cycle, recording_transport);
        if (cycle_status != BenchStatus::kCompleted ||
            cycle.action != AdapterCycleAction::kZeroWritten) {
          result.status = cycle_status == BenchStatus::kCompleted
                              ? BenchStatus::kWriteFailed
                              : cycle_status;
          break;
        }
        continue;
      }
      if (!adapter.setAuthorization(false, false, 0U, receipt_ns)) {
        result.status = BenchStatus::kAdapterRejected;
        break;
      }
      const AdapterCycleResult final_prearm_zero = adapter.cycle(receipt_ns);
      const BenchStatus final_prearm_zero_status =
          statusForCycle(final_prearm_zero, recording_transport);
      if (final_prearm_zero_status != BenchStatus::kCompleted ||
          final_prearm_zero.action != AdapterCycleAction::kZeroWritten) {
        result.status =
            final_prearm_zero_status == BenchStatus::kCompleted
                ? BenchStatus::kWriteFailed
                : final_prearm_zero_status;
        break;
      }
      if (!adapter.setAuthorization(true, true, 1U, receipt_ns)) {
        result.status = BenchStatus::kAdapterRejected;
        break;
      }
      authorized = true;
      last_command_receipt_ns = receipt_ns;
      last_requested_speed_mps = 0.0;
      recording_transport.setMotionPermitted(true);
      continue;
    }

    if (last_feedback_ns == 0 || receipt_ns < last_feedback_ns ||
        receipt_ns - last_feedback_ns > config.maximum_feedback_age_ns) {
      result.status = BenchStatus::kFeedbackMissing;
      break;
    }
    const std::int64_t command_interval_ns =
        receipt_ns - last_command_receipt_ns;
    if (command_interval_ns <= 0 ||
        command_interval_ns > kMaximumCommandGapNs) {
      result.status = BenchStatus::kSessionDeadlineExceeded;
      break;
    }
    if (holding_target && receipt_ns >= hold_deadline_ns) {
      holding_target = false;
      ramping_down = true;
    }
    const std::int16_t desired_wire_speed =
        ramping_down ? 0 : target_wire_speed;
    std::int16_t requested_wire_speed = 0;
    if (!recording_transport.coupleNormalWireSpeed(
            desired_wire_speed, receipt_ns, &requested_wire_speed)) {
      result.status = BenchStatus::kAdapterRejected;
      break;
    }
    const double requested_speed =
        static_cast<double>(requested_wire_speed) / 1000.0;
    if (!std::isfinite(requested_speed) || requested_wire_speed < 0 ||
        requested_wire_speed > kBenchMaximumForwardWireSpeed ||
        (result.statistics.motion_commands_submitted == 0U &&
         requested_wire_speed >= kBenchMaximumForwardWireSpeed)) {
      result.status = BenchStatus::kAdapterRejected;
      break;
    }
    VehicleExecutionCommand command;
    command.sequence_id = next_sequence++;
    command.created_monotonic_ns = receipt_ns;
    command.deadline_monotonic_ns =
        saturatedAdd(receipt_ns, config.command_valid_for_ns);
    command.signed_speed_mps = requested_speed;
    command.curvature_inv_m = 0.0;
    command.motion_enabled = true;
    command.hold = false;
    command.stop_reason = StopReason::kNone;
    const SubmitResult submitted = adapter.submit(command, receipt_ns);
    ++result.statistics.motion_commands_submitted;
    if (submitted.status != SubmissionStatus::kRecoveryPending &&
        submitted.status != SubmissionStatus::kAccepted) {
      result.status = BenchStatus::kAdapterRejected;
      break;
    }
    const AdapterCycleResult cycle = adapter.cycle(receipt_ns);
    const BenchStatus cycle_status =
        statusForCycle(cycle, recording_transport);
    if (cycle_status != BenchStatus::kCompleted) {
      result.status = cycle_status;
      break;
    }
    last_command_receipt_ns = receipt_ns;
    last_requested_speed_mps = requested_speed;
    if (!holding_target && !ramping_down &&
        requested_wire_speed == target_wire_speed &&
        cycle.action == AdapterCycleAction::kMotionWritten) {
      holding_target = true;
      hold_deadline_ns =
          saturatedAdd(receipt_ns, config.hold_duration_ns);
    }
    if (ramping_down && requested_wire_speed == 0 &&
        cycle.action == AdapterCycleAction::kMotionWritten) {
      recording_transport.setMotionPermitted(false);
      if (!adapter.setAuthorization(false, false, 0U, receipt_ns)) {
        result.status = BenchStatus::kAdapterRejected;
        break;
      }
      authorized = false;
      post_zero_observation = true;
      standstill_receipts = 0U;
      first_standstill_receipt_ns = 0;
      post_zero_deadline_ns =
          saturatedAdd(receipt_ns, kPostZeroStandstillDeadlineNs);
    }
  }

  recording_transport.setMotionPermitted(false);
  result.zero_host_write_completed = boundedZeroHostWrite(
      &adapter, &recording_transport, operations, last_good_clock_ns);
  result.authorization_revoked = !adapter.isAuthorized();
  result.delivery_unconfirmed =
      result.delivery_unconfirmed ||
      recording_transport.deliveryUnconfirmed() ||
      !result.zero_host_write_completed;
  const std::int64_t ended_ns = operations.monotonic_now_ns();
  if (ended_ns > 0 && ended_ns >= result.statistics.started_monotonic_ns) {
    result.statistics.ended_monotonic_ns = ended_ns;
  }
  if (result.status == BenchStatus::kCompleted &&
      !result.zero_host_write_completed) {
    result.status = BenchStatus::kZeroHostWriteFailed;
  }
  if (!operations.record_json_line(benchSummaryRecordJson(result))) {
    result.status = BenchStatus::kRecordError;
  }
  return result;
  } catch (...) {
    // Once a normal command may have reached the controller, evidence,
    // parser, clock and allocation failures all share the same no-record
    // bounded stop path.  A poisoned stream deliberately suppresses the stop
    // so an unknown prefix can never be extended into a second frame.
    result.status = BenchStatus::kRecordError;
    recording_transport.setMotionPermitted(false);
    result.zero_host_write_completed = boundedZeroHostWrite(
        &adapter, &recording_transport, operations, last_good_clock_ns);
    result.authorization_revoked = !adapter.isAuthorized();
    result.delivery_unconfirmed =
        result.delivery_unconfirmed ||
        recording_transport.deliveryUnconfirmed() ||
        recording_transport.writeStreamPoisoned() ||
        !result.zero_host_write_completed;
    return result;
  }
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
