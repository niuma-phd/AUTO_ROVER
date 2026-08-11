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

IoResult safeWrite(ByteTransport* transport, const std::uint8_t* data,
                   std::size_t size,
                   std::int64_t deadline_ns) noexcept {
  try {
    if (transport == nullptr || data == nullptr || size == 0U) {
      return {TransportStatus::kInvalidArgument, 0U, 0, false};
    }
    return transport->writeAll(data, size, deadline_ns);
  } catch (...) {
    // An exception cannot prove how many bytes reached the stream.  Mark it
    // unknown so the caller never appends another frame on this generation.
    return {TransportStatus::kIoError, 0U, 0, true};
  }
}

}  // namespace

CommandParserResyncPadding commandParserResyncPadding() noexcept {
  CommandParserResyncPadding padding{};
  padding.fill(0U);
  return padding;
}

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
  parser_resync_padding_ = commandParserResyncPadding();
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

  bool unresolved_partial_or_unknown = false;
  for (std::uint32_t attempt = 0U;
       attempt < config_.max_zero_write_attempts; ++attempt) {
    if (!safeConnected(transport)) {
      result.status = PhysicalActivationStatus::kTransportUnavailable;
      result.delivery_unconfirmed = true;
      return result;
    }
    const std::int64_t padding_deadline =
        saturatedAdd(attempt_ns, config_.write_timeout_ns);
    if (padding_deadline <= 0) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.delivery_unconfirmed = true;
      return result;
    }

    ++result.attempts;
    result.parser_resync_padding_host_write_complete = false;
    result.exact_zero_host_write_complete = false;
    result.last_io = safeWrite(
        transport, parser_resync_padding_.data(),
        parser_resync_padding_.size(), padding_deadline);
    const bool complete_padding =
        result.last_io.status == TransportStatus::kOk &&
        result.last_io.transferred == parser_resync_padding_.size() &&
        !result.last_io.delivery_unconfirmed;
    const bool padding_unknown_or_partial =
        result.last_io.delivery_unconfirmed ||
        (result.last_io.transferred > 0U && !complete_padding);
    result.parser_resync_padding_host_write_complete = complete_padding;

    const std::int64_t after_padding_ns = safeClockNow(operations_);
    result.completed_monotonic_ns = after_padding_ns;
    if (after_padding_ns <= 0 || after_padding_ns < attempt_ns) {
      result.status = PhysicalActivationStatus::kClockInvalid;
      result.delivery_unconfirmed = true;
      result.write_stream_poisoned = padding_unknown_or_partial;
      return result;
    }
    if (after_padding_ns > padding_deadline) {
      result.status = PhysicalActivationStatus::kDeadlineMissed;
      result.delivery_unconfirmed = true;
      result.write_stream_poisoned = padding_unknown_or_partial;
      return result;
    }

    std::int64_t retry_origin_ns = after_padding_ns;
    bool sequence_complete = false;
    if (complete_padding) {
      const std::int64_t zero_deadline =
          saturatedAdd(after_padding_ns, config_.write_timeout_ns);
      if (zero_deadline <= 0) {
        result.status = PhysicalActivationStatus::kClockInvalid;
        result.delivery_unconfirmed = true;
        return result;
      }
      result.last_io = safeWrite(transport, exact_zero_frame_.data(),
                                 exact_zero_frame_.size(), zero_deadline);
      const bool complete_exact_zero =
          result.last_io.status == TransportStatus::kOk &&
          result.last_io.transferred == exact_zero_frame_.size() &&
          !result.last_io.delivery_unconfirmed;
      const bool zero_unknown_or_partial =
          result.last_io.delivery_unconfirmed ||
          (result.last_io.transferred > 0U && !complete_exact_zero);
      const std::int64_t after_zero_ns = safeClockNow(operations_);
      result.completed_monotonic_ns = after_zero_ns;
      if (after_zero_ns <= 0 || after_zero_ns < after_padding_ns) {
        result.status = PhysicalActivationStatus::kClockInvalid;
        result.exact_zero_host_write_complete = complete_exact_zero;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = zero_unknown_or_partial;
        return result;
      }
      if (after_zero_ns > zero_deadline) {
        result.status = PhysicalActivationStatus::kDeadlineMissed;
        result.exact_zero_host_write_complete = complete_exact_zero;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = zero_unknown_or_partial;
        return result;
      }
      if (complete_exact_zero) {
        result.exact_zero_host_write_complete = true;
        result.delivery_unconfirmed = false;
        result.write_stream_poisoned = false;
        result.status = PhysicalActivationStatus::kSuccess;
        return result;
      }
      if (result.last_io.status == TransportStatus::kDisconnected) {
        result.status = PhysicalActivationStatus::kTransportUnavailable;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = zero_unknown_or_partial;
        return result;
      }
      retry_origin_ns = after_zero_ns;
      unresolved_partial_or_unknown = zero_unknown_or_partial;
      if (zero_unknown_or_partial) {
        ++result.recovery_restarts;
      }
    } else {
      if (result.last_io.status == TransportStatus::kDisconnected) {
        result.status = PhysicalActivationStatus::kTransportUnavailable;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = padding_unknown_or_partial;
        return result;
      }
      unresolved_partial_or_unknown = padding_unknown_or_partial;
      if (padding_unknown_or_partial) {
        ++result.recovery_restarts;
      }
    }

    if (attempt + 1U < config_.max_zero_write_attempts) {
      const std::int64_t retry_deadline = saturatedAdd(
          retry_origin_ns, config_.zero_retry_interval_ns);
      if (retry_deadline <= 0) {
        result.status = PhysicalActivationStatus::kClockInvalid;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = unresolved_partial_or_unknown;
        return result;
      }
      if (!safeWaitUntil(operations_, retry_deadline)) {
        result.status = PhysicalActivationStatus::kRetryWaitFailed;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = unresolved_partial_or_unknown;
        return result;
      }
      attempt_ns = safeClockNow(operations_);
      if (attempt_ns < retry_deadline || attempt_ns < retry_origin_ns) {
        result.status = PhysicalActivationStatus::kClockInvalid;
        result.delivery_unconfirmed = true;
        result.write_stream_poisoned = unresolved_partial_or_unknown;
        return result;
      }
      sequence_complete = true;
    }
    if (!sequence_complete) {
      break;
    }
  }

  result.status = unresolved_partial_or_unknown
                      ? PhysicalActivationStatus::kWriteStreamPoisoned
                      : PhysicalActivationStatus::kZeroRetriesExhausted;
  result.delivery_unconfirmed = true;
  result.write_stream_poisoned = unresolved_partial_or_unknown;
  return result;
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
