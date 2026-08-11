#include "auto_rover_vcu_wheeltec_serial/adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

bool validConfig(const AdapterConfig& config, const ByteTransport* transport) {
  return transport != nullptr && config.max_command_age_ns > 0 &&
         config.fresh_commands_required > 0U &&
         config.zero_retry_interval_ns > 0 &&
         config.max_zero_write_attempts > 0U &&
         config.write_timeout_ns > 0 &&
         codecLimitsAreValid(config.codec_limits);
}

bool commandRequestsMotion(const VehicleExecutionCommand& command) {
  return command.motion_enabled && !command.hold &&
         command.stop_reason == StopReason::kNone;
}

IoResult safeWriteAll(ByteTransport* transport, const std::uint8_t* data,
                      std::size_t size, std::int64_t deadline_ns) {
  if (transport == nullptr) {
    return {TransportStatus::kInvalidArgument, 0U, 0, true};
  }
  try {
    return transport->writeAll(data, size, deadline_ns);
  } catch (...) {
    // A throwing transport cannot prove that it emitted zero bytes.  Poison
    // the generation so the runtime never concatenates another frame.
    return {TransportStatus::kIoError, 0U, 0, true};
  }
}

}  // namespace

WheeltecSerialAdapter::WheeltecSerialAdapter(const AdapterConfig& config,
                                             ByteTransport* transport)
    : config_(config),
      transport_(transport),
      configuration_valid_(validConfig(config, transport)) {
  if (configuration_valid_) {
    synchronizeConnection();
  }
}

bool WheeltecSerialAdapter::setAuthorization(
    bool protocol_enabled, bool operator_armed,
    std::uint64_t authorization_id,
    std::int64_t authorization_monotonic_ns) {
  const bool authorization_time_valid =
      authorization_monotonic_ns > 0 &&
      (last_observed_monotonic_ns_ == 0 ||
       authorization_monotonic_ns >= last_observed_monotonic_ns_);
  if (authorization_monotonic_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = authorization_monotonic_ns;
  }
  const bool connection_was_known = connection_known_;
  const std::uint64_t previous_generation = connection_generation_;
  if (!configuration_valid_ || !authorization_time_valid ||
      !synchronizeConnection()) {
    protocol_enabled_ = false;
    operator_armed_ = false;
    authorized_connection_generation_ = 0U;
    authorization_monotonic_ns_ = 0;
    clearMotionState();
    requireNewZeroEpisode();
    return false;
  }
  const bool connection_changed_during_request =
      !connection_was_known ||
      previous_generation != connection_generation_;
  if (!protocol_enabled || !operator_armed) {
    protocol_enabled_ = false;
    operator_armed_ = false;
    authorized_connection_generation_ = 0U;
    authorization_monotonic_ns_ = 0;
    clearMotionState();
    requireNewZeroEpisode();
    return !protocol_enabled && !operator_armed;
  }
  if (connection_changed_during_request || authorization_id == 0U ||
      authorization_id <= highest_authorization_id_) {
    protocol_enabled_ = false;
    operator_armed_ = false;
    authorized_connection_generation_ = 0U;
    authorization_monotonic_ns_ = 0;
    clearMotionState();
    requireNewZeroEpisode();
    return false;
  }
  highest_authorization_id_ = authorization_id;
  authorized_connection_generation_ = connection_generation_;
  authorization_monotonic_ns_ = authorization_monotonic_ns;
  protocol_enabled_ = true;
  operator_armed_ = true;
  clearMotionState();
  return true;
}

SubmitResult WheeltecSerialAdapter::submit(
    const VehicleExecutionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  if (!configuration_valid_) {
    return {SubmissionStatus::kConfigurationInvalid,
            CodecError::kInvalidLimits};
  }
  if (receipt_monotonic_ns <= 0 ||
      (last_observed_monotonic_ns_ > 0 &&
       receipt_monotonic_ns < last_observed_monotonic_ns_)) {
    invalidateAuthorizationAndMotion();
    return {SubmissionStatus::kTimestampInvalid, CodecError::kNone};
  }
  if (receipt_monotonic_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = receipt_monotonic_ns;
  }
  if (!synchronizeConnection()) {
    return {SubmissionStatus::kDisconnected, CodecError::kNone};
  }
  if (command.sequence_id == 0U || command.sequence_id <= highest_sequence_) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kSequenceInvalid, CodecError::kNone};
  }
  highest_sequence_ = command.sequence_id;

  if (receipt_monotonic_ns < 0 || command.created_monotonic_ns <= 0 ||
      command.created_monotonic_ns > receipt_monotonic_ns ||
      command.deadline_monotonic_ns <= command.created_monotonic_ns) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kTimestampInvalid, CodecError::kNone};
  }
  if (receipt_monotonic_ns > command.deadline_monotonic_ns ||
      receipt_monotonic_ns - command.created_monotonic_ns >
          config_.max_command_age_ns) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kStale, CodecError::kNone};
  }

  const EncodeResult encoded =
      encodeExecutionCommand(command, config_.codec_limits);
  if (!encoded.ok()) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kCodecRejected, encoded.error};
  }
  if (!isAuthorized()) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kNotAuthorized, CodecError::kNone};
  }
  if (command.created_monotonic_ns <= authorization_monotonic_ns_) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kPredatesAuthorization, CodecError::kNone};
  }
  if (!commandRequestsMotion(command)) {
    clearMotionState();
    requireNewZeroEpisode();
    return {SubmissionStatus::kAccepted, CodecError::kNone};
  }

  if (latest_command_valid_ &&
      (receipt_monotonic_ns > latest_command_.deadline_monotonic_ns ||
       receipt_monotonic_ns - latest_command_.created_monotonic_ns >
           config_.max_command_age_ns)) {
    clearMotionState();
  }

  latest_command_ = command;
  latest_frame_ = encoded.frame;
  latest_command_valid_ = true;
  if (consecutive_fresh_commands_ < config_.fresh_commands_required) {
    ++consecutive_fresh_commands_;
  }
  if (recoveryComplete()) {
    return {SubmissionStatus::kAccepted, CodecError::kNone};
  }
  return {SubmissionStatus::kRecoveryPending, CodecError::kNone};
}

void WheeltecSerialAdapter::requireRecoveryZeroWrite() {
  if (isAuthorized() && !recoveryComplete()) {
    requireNewZeroEpisode();
  }
}

AdapterCycleResult WheeltecSerialAdapter::cycle(
    std::int64_t now_monotonic_ns) {
  if (!configuration_valid_) {
    return {AdapterCycleAction::kConfigurationInvalid,
            TransportStatus::kInvalidArgument, true, zero_attempts_};
  }
  if (now_monotonic_ns <= 0 ||
      (last_observed_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_observed_monotonic_ns_)) {
    invalidateAuthorizationAndMotion();
    return {AdapterCycleAction::kClockInvalid,
            TransportStatus::kInvalidArgument, true, zero_attempts_};
  }
  if (now_monotonic_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = now_monotonic_ns;
  }
  if (!synchronizeConnection()) {
    return {AdapterCycleAction::kDisconnected,
            TransportStatus::kDisconnected, true, zero_attempts_};
  }

  if (motionIsFresh(now_monotonic_ns)) {
    const std::int64_t attempt_deadline_ns =
        writeDeadline(now_monotonic_ns, latest_command_.deadline_monotonic_ns);
    const IoResult result = safeWriteAll(
        transport_,
      latest_frame_.data(), latest_frame_.size(),
        attempt_deadline_ns);
    if (result.status == TransportStatus::kOk &&
        result.transferred == latest_frame_.size()) {
      zero_episode_latched_ = false;
      zero_retry_active_ = false;
      zero_retries_exhausted_ = false;
      zero_attempts_ = 0U;
      return {AdapterCycleAction::kMotionWritten, result.status,
              result.delivery_unconfirmed, 0U, result.transferred,
              result.delivery_unconfirmed, attempt_deadline_ns};
    }
    clearMotionState();
    requireNewZeroEpisode();
    if (result.status == TransportStatus::kDisconnected) {
      synchronizeConnection();
      return {AdapterCycleAction::kDisconnected, result.status, true, 0U,
              result.transferred,
              result.delivery_unconfirmed || result.transferred > 0U,
              attempt_deadline_ns};
    }
    return {AdapterCycleAction::kMotionWriteFailed, result.status, true, 0U,
            result.transferred,
            result.delivery_unconfirmed || result.transferred > 0U,
            attempt_deadline_ns};
  }

  if (latest_command_valid_ &&
      (now_monotonic_ns < latest_command_.created_monotonic_ns ||
       now_monotonic_ns > latest_command_.deadline_monotonic_ns ||
       now_monotonic_ns - latest_command_.created_monotonic_ns >
           config_.max_command_age_ns)) {
    clearMotionState();
  }
  return cycleZero(now_monotonic_ns);
}

bool WheeltecSerialAdapter::isAuthorized() const {
  return connection_known_ && protocol_enabled_ && operator_armed_ &&
         authorization_monotonic_ns_ > 0 &&
         authorized_connection_generation_ == connection_generation_;
}

bool WheeltecSerialAdapter::recoveryComplete() const {
  return isAuthorized() &&
         consecutive_fresh_commands_ >= config_.fresh_commands_required;
}

std::uint32_t WheeltecSerialAdapter::consecutiveFreshCommands() const {
  return consecutive_fresh_commands_;
}

std::uint64_t WheeltecSerialAdapter::highestObservedSequence() const {
  return highest_sequence_;
}

bool WheeltecSerialAdapter::synchronizeConnection() {
  if (transport_ == nullptr || !transport_->isConnected()) {
    if (connection_known_) {
      connection_known_ = false;
      protocol_enabled_ = false;
      operator_armed_ = false;
      authorized_connection_generation_ = 0U;
      authorization_monotonic_ns_ = 0;
      clearMotionState();
      requireNewZeroEpisode();
    }
    return false;
  }

  const std::uint64_t observed_generation =
      transport_->connectionGeneration();
  if (!connection_known_ || observed_generation != connection_generation_) {
    connection_known_ = true;
    connection_generation_ = observed_generation;
    protocol_enabled_ = false;
    operator_armed_ = false;
    authorized_connection_generation_ = 0U;
    authorization_monotonic_ns_ = 0;
    clearMotionState();
    requireNewZeroEpisode();
  }
  return true;
}

void WheeltecSerialAdapter::invalidateAuthorizationAndMotion() {
  protocol_enabled_ = false;
  operator_armed_ = false;
  authorized_connection_generation_ = 0U;
  authorization_monotonic_ns_ = 0;
  clearMotionState();
  requireNewZeroEpisode();
}

void WheeltecSerialAdapter::clearMotionState() {
  latest_command_valid_ = false;
  latest_command_ = VehicleExecutionCommand{};
  latest_frame_ = CommandFrame{};
  consecutive_fresh_commands_ = 0U;
}

void WheeltecSerialAdapter::requireNewZeroEpisode() {
  zero_episode_latched_ = false;
  zero_retry_active_ = false;
  zero_retries_exhausted_ = false;
  zero_attempts_ = 0U;
  next_zero_attempt_ns_ = 0;
}

bool WheeltecSerialAdapter::motionIsFresh(
    std::int64_t now_monotonic_ns) const {
  return latest_command_valid_ && recoveryComplete() &&
         now_monotonic_ns >= latest_command_.created_monotonic_ns &&
         now_monotonic_ns <= latest_command_.deadline_monotonic_ns &&
         now_monotonic_ns - latest_command_.created_monotonic_ns <=
             config_.max_command_age_ns;
}

AdapterCycleResult WheeltecSerialAdapter::cycleZero(
    std::int64_t now_monotonic_ns) {
  if (!zero_episode_latched_) {
    zero_episode_latched_ = true;
    zero_retry_active_ = true;
    zero_retries_exhausted_ = false;
    zero_attempts_ = 0U;
    next_zero_attempt_ns_ = now_monotonic_ns;
  }
  if (zero_retries_exhausted_) {
    return {AdapterCycleAction::kZeroRetriesExhausted,
            TransportStatus::kDeadlineExceeded, true, zero_attempts_};
  }
  if (!zero_retry_active_) {
    return {AdapterCycleAction::kNoAction, TransportStatus::kOk, false,
            zero_attempts_};
  }
  if (now_monotonic_ns < next_zero_attempt_ns_) {
    return {AdapterCycleAction::kZeroRetryPending,
            TransportStatus::kWouldBlock, true, zero_attempts_};
  }

  const EncodeResult zero = encodeWireMotion(WireMotionCandidate{},
                                              config_.codec_limits);
  if (!zero.ok()) {
    zero_retries_exhausted_ = true;
    zero_retry_active_ = false;
    return {AdapterCycleAction::kConfigurationInvalid,
            TransportStatus::kInvalidArgument, true, zero_attempts_};
  }
  ++zero_attempts_;
  const std::uint32_t attempted_zero_count = zero_attempts_;
  const std::int64_t attempt_deadline_ns = writeDeadline(
      now_monotonic_ns, std::numeric_limits<std::int64_t>::max());
  const IoResult result = safeWriteAll(
      transport_,
      zero.frame.data(), zero.frame.size(),
      attempt_deadline_ns);
  if (result.status == TransportStatus::kOk &&
      result.transferred == zero.frame.size()) {
    zero_retry_active_ = false;
    return {AdapterCycleAction::kZeroWritten, result.status,
            result.delivery_unconfirmed, zero_attempts_, result.transferred,
            result.delivery_unconfirmed, attempt_deadline_ns};
  }
  if (result.status == TransportStatus::kDisconnected) {
    synchronizeConnection();
    return {AdapterCycleAction::kDisconnected, result.status, true,
            attempted_zero_count, result.transferred,
            result.delivery_unconfirmed || result.transferred > 0U,
            attempt_deadline_ns};
  }
  if (zero_attempts_ >= config_.max_zero_write_attempts) {
    zero_retry_active_ = false;
    zero_retries_exhausted_ = true;
    return {AdapterCycleAction::kZeroRetriesExhausted, result.status, true,
            zero_attempts_, result.transferred,
            result.delivery_unconfirmed || result.transferred > 0U,
            attempt_deadline_ns};
  }
  if (now_monotonic_ns >
      std::numeric_limits<std::int64_t>::max() -
          config_.zero_retry_interval_ns) {
    next_zero_attempt_ns_ = std::numeric_limits<std::int64_t>::max();
  } else {
    next_zero_attempt_ns_ =
        now_monotonic_ns + config_.zero_retry_interval_ns;
  }
  return {AdapterCycleAction::kZeroWriteFailed, result.status, true,
          zero_attempts_, result.transferred,
          result.delivery_unconfirmed || result.transferred > 0U,
          attempt_deadline_ns};
}

std::int64_t WheeltecSerialAdapter::writeDeadline(
    std::int64_t now_monotonic_ns,
    std::int64_t command_deadline_ns) const {
  std::int64_t timeout_deadline =
      std::numeric_limits<std::int64_t>::max();
  if (now_monotonic_ns <=
      std::numeric_limits<std::int64_t>::max() - config_.write_timeout_ns) {
    timeout_deadline = now_monotonic_ns + config_.write_timeout_ns;
  }
  return std::min(timeout_deadline, command_deadline_ns);
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
