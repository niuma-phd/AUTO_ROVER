#include "auto_rover_vcu_wheeltec_serial/physical_activation.hpp"

#include <limits>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

constexpr std::int64_t kMaximumActivationWriteTimeoutNs = 10000000;
constexpr std::int64_t kMaximumActivationRetryIntervalNs = 20000000;
constexpr std::uint32_t kMaximumActivationZeroAttempts = 3U;

std::int64_t saturatedAdd(std::int64_t value, std::int64_t delta) {
  if (value <= 0 || delta <= 0 ||
      value > std::numeric_limits<std::int64_t>::max() - delta) {
    return 0;
  }
  return value + delta;
}

std::int64_t safeClockNow(
    const PhysicalActivationOperations& operations) noexcept {
  try {
    return operations.monotonic_now_ns
               ? operations.monotonic_now_ns()
               : 0;
  } catch (...) {
    return 0;
  }
}

bool safeWaitUntil(const PhysicalActivationOperations& operations,
                   std::int64_t deadline_ns) noexcept {
  try {
    return deadline_ns > 0 && operations.wait_until_monotonic_ns &&
           operations.wait_until_monotonic_ns(deadline_ns);
  } catch (...) {
    return false;
  }
}

bool safeConnected(const ByteTransport* transport) noexcept {
  try {
    return transport != nullptr && transport->isConnected() &&
           transport->connectionGeneration() != 0U;
  } catch (...) {
    return false;
  }
}

IoResult safeWriteExactZero(ByteTransport* transport,
                            const CommandFrame& frame,
                            std::int64_t deadline_ns) noexcept {
  try {
    if (transport == nullptr) {
      return {TransportStatus::kInvalidArgument, 0U, 0, false};
    }
    return transport->writeAll(frame.data(), frame.size(), deadline_ns);
  } catch (...) {
    // An exception cannot prove how many bytes reached the stream.  Mark it
    // unknown so the caller never appends another frame on this generation.
    return {TransportStatus::kIoError, 0U, 0, true};
  }
}

}  // namespace

bool physicalActivationConfigIsValid(
    const PhysicalActivationConfig& config) {
  return codecLimitsAreValid(config.codec_limits) &&
         config.write_timeout_ns > 0 &&
         config.write_timeout_ns <= kMaximumActivationWriteTimeoutNs &&
         config.zero_retry_interval_ns > 0 &&
         config.zero_retry_interval_ns <=
             kMaximumActivationRetryIntervalNs &&
         config.max_zero_write_attempts > 0U &&
         config.max_zero_write_attempts <=
             kMaximumActivationZeroAttempts;
}

const char* physicalActivationStatusName(PhysicalActivationStatus status) {
  switch (status) {
    case PhysicalActivationStatus::kSuccess:
      return "success";
    case PhysicalActivationStatus::kNotPrepared:
      return "not_prepared";
    case PhysicalActivationStatus::kTransportUnavailable:
      return "transport_unavailable";
    case PhysicalActivationStatus::kClockInvalid:
      return "clock_invalid";
    case PhysicalActivationStatus::kRetryWaitFailed:
      return "retry_wait_failed";
    case PhysicalActivationStatus::kZeroRetriesExhausted:
      return "zero_retries_exhausted";
    case PhysicalActivationStatus::kWriteStreamPoisoned:
      return "write_stream_poisoned";
    case PhysicalActivationStatus::kDeadlineMissed:
      return "deadline_missed";
  }
  return "unknown";
}

PreparedPhysicalActivation::PreparedPhysicalActivation(
    const PhysicalActivationConfig& config,
    const PhysicalActivationOperations& operations)
    : config_(config), operations_(operations) {
  if (!physicalActivationConfigIsValid(config_) ||
      !operations_.monotonic_now_ns ||
      !operations_.wait_until_monotonic_ns) {
    return;
  }
  const EncodeResult encoded =
      encodeWireMotion(WireMotionCandidate{}, config_.codec_limits);
  if (!encoded.ok()) {
    return;
  }
  exact_zero_frame_ = encoded.frame;
  prepared_ = true;
}

PhysicalActivationResult PreparedPhysicalActivation::activate(
    ByteTransport* transport) const noexcept {
  PhysicalActivationResult result;
  if (!prepared_) {
    result.status = PhysicalActivationStatus::kNotPrepared;
    return result;
  }
  if (!safeConnected(transport)) {
    result.status = PhysicalActivationStatus::kTransportUnavailable;
    result.delivery_unconfirmed = true;
    return result;
  }

  std::int64_t attempt_ns = safeClockNow(operations_);
  result.started_monotonic_ns = attempt_ns;
  if (attempt_ns <= 0) {
    result.status = PhysicalActivationStatus::kClockInvalid;
    result.delivery_unconfirmed = true;
    return result;
  }

  for (std::uint32_t attempt = 0U;
       attempt < config_.max_zero_write_attempts; ++attempt) {
    if (!safeConnected(transport)) {
      result.status = PhysicalActivationStatus::kTransportUnavailable;
      result.delivery_unconfirmed = true;
      return result;
    }
    const std::int64_t write_deadline =
        saturatedAdd(attempt_ns, config_.write_timeout_ns);
    if (write_deadline <= 0) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.delivery_unconfirmed = true;
      return result;
    }

    ++result.attempts;
    result.last_io =
        safeWriteExactZero(transport, exact_zero_frame_, write_deadline);
    const bool complete_exact_zero =
        result.last_io.status == TransportStatus::kOk &&
        result.last_io.transferred == exact_zero_frame_.size() &&
        !result.last_io.delivery_unconfirmed;
    const bool unknown_or_partial = result.last_io.delivery_unconfirmed ||
        (result.last_io.transferred > 0U && !complete_exact_zero);
    if (unknown_or_partial) {
      result.status = PhysicalActivationStatus::kWriteStreamPoisoned;
      result.delivery_unconfirmed = true;
      result.write_stream_poisoned = true;
      return result;
    }

    const std::int64_t after_write_ns = safeClockNow(operations_);
    result.completed_monotonic_ns = after_write_ns;
    if (after_write_ns <= 0 || after_write_ns < attempt_ns) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.exact_zero_host_write_complete = complete_exact_zero;
      result.delivery_unconfirmed = true;
      return result;
    }
    if (after_write_ns > write_deadline) {
      result.status = PhysicalActivationStatus::kDeadlineMissed;
      result.exact_zero_host_write_complete = complete_exact_zero;
      result.delivery_unconfirmed = true;
      return result;
    }
    if (complete_exact_zero) {
      result.exact_zero_host_write_complete = true;
      result.status = PhysicalActivationStatus::kSuccess;
      return result;
    }

    if (result.last_io.status == TransportStatus::kDisconnected) {
      result.status = PhysicalActivationStatus::kTransportUnavailable;
      result.delivery_unconfirmed = true;
      return result;
    }
    if (attempt + 1U >= config_.max_zero_write_attempts) {
      break;
    }
    const std::int64_t retry_deadline =
        saturatedAdd(after_write_ns, config_.zero_retry_interval_ns);
    if (retry_deadline <= 0) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.delivery_unconfirmed = true;
      return result;
    }
    if (!safeWaitUntil(operations_, retry_deadline)) {
      result.status = PhysicalActivationStatus::kRetryWaitFailed;
      result.delivery_unconfirmed = true;
      return result;
    }
    attempt_ns = safeClockNow(operations_);
    if (attempt_ns < retry_deadline || attempt_ns < after_write_ns) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.delivery_unconfirmed = true;
      return result;
    }
  }

  result.status = PhysicalActivationStatus::kZeroRetriesExhausted;
  result.delivery_unconfirmed = true;
  return result;
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
