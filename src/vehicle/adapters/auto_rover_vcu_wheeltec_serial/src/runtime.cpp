#include "auto_rover_vcu_wheeltec_serial/runtime.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <limits>
#include <time.h>
#include <utility>
#include <vector>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

constexpr std::size_t kReadBufferBytes = 512U;
constexpr std::int64_t kMaximumReadTimeoutNs = 1000000;
constexpr std::int64_t kMaximumWriteTimeoutNs = 10000000;
constexpr std::int64_t kMaximumFeedbackAgeNs = 150000000;
constexpr std::int64_t kMaximumNormalWriteGapNs = 100000000;
constexpr std::int64_t kMaximumStepDurationNs = 50000000;
constexpr std::uint32_t kMaximumDrainReads = 64U;
constexpr std::uint32_t kMaximumRecoveryReceipts = 100U;
constexpr std::uint32_t kMaximumZeroWriteAttempts = 3U;
constexpr std::int64_t kMaximumZeroRetryIntervalNs = 20000000;

std::int64_t saturatedAdd(std::int64_t value, std::int64_t delta) {
  if (delta > 0 &&
      value > std::numeric_limits<std::int64_t>::max() - delta) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + delta;
}

bool cycleCompletedHostWrite(const AdapterCycleResult& cycle) {
  return (cycle.action == AdapterCycleAction::kMotionWritten ||
          cycle.action == AdapterCycleAction::kZeroWritten) &&
         cycle.transport_status == TransportStatus::kOk &&
         !cycle.delivery_unconfirmed;
}

bool cycleAttemptFailed(const AdapterCycleResult& cycle) {
  return cycle.action == AdapterCycleAction::kMotionWriteFailed ||
         cycle.action == AdapterCycleAction::kZeroWriteFailed ||
         cycle.action == AdapterCycleAction::kZeroRetriesExhausted ||
         cycle.action == AdapterCycleAction::kConfigurationInvalid ||
         cycle.action == AdapterCycleAction::kClockInvalid;
}

bool cycleMissedDeadline(const AdapterCycleResult& cycle,
                         std::int64_t completed_monotonic_ns) {
  return cycle.attempt_deadline_monotonic_ns > 0 &&
         completed_monotonic_ns > cycle.attempt_deadline_monotonic_ns;
}

bool cyclePoisonedWriteStream(const AdapterCycleResult& cycle) {
  return cycle.write_stream_poisoned ||
         (cycle.transferred_bytes > 0U &&
          cycle.transferred_bytes < kCommandFrameSize);
}

bool commandRequestsMotion(const VehicleExecutionCommand& command) {
  return command.motion_enabled && !command.hold &&
         command.stop_reason == StopReason::kNone &&
         command.signed_speed_mps > 0.0;
}

}  // namespace

bool runtimeConfigIsStructurallyValid(const RuntimeConfig& config) {
  if (!codecLimitsAreValid(config.adapter.codec_limits) ||
      config.adapter.max_command_age_ns <= 0 ||
      config.adapter.max_command_age_ns > kMaximumNormalWriteGapNs ||
      config.adapter.fresh_commands_required == 0U ||
      config.adapter.zero_retry_interval_ns <= 0 ||
      config.adapter.zero_retry_interval_ns > kMaximumZeroRetryIntervalNs ||
      config.adapter.max_zero_write_attempts == 0U ||
      config.adapter.max_zero_write_attempts > kMaximumZeroWriteAttempts ||
      config.adapter.write_timeout_ns <= 0 ||
      config.adapter.write_timeout_ns > kMaximumWriteTimeoutNs ||
      config.read_timeout_ns <= 0 ||
      config.read_timeout_ns > kMaximumReadTimeoutNs ||
      config.maximum_feedback_age_ns < config.read_timeout_ns ||
      config.maximum_feedback_age_ns > kMaximumFeedbackAgeNs ||
      config.drain_read_timeout_ns <= 0 ||
      config.drain_read_timeout_ns > kMaximumReadTimeoutNs ||
      config.maximum_drain_reads == 0U ||
      config.maximum_drain_reads > kMaximumDrainReads ||
      config.control_allowed_receipts_required < 2U ||
      config.control_allowed_receipts_required > kMaximumRecoveryReceipts ||
      config.control_allowed_minimum_span_ns <= 0 ||
      config.control_allowed_minimum_span_ns > 1000000000LL ||
      config.maximum_normal_write_gap_ns <= 0 ||
      config.maximum_normal_write_gap_ns > kMaximumNormalWriteGapNs ||
      config.maximum_step_duration_ns <= 0 ||
      config.maximum_step_duration_ns > kMaximumStepDurationNs) {
    return false;
  }
  if (config.adapter.write_timeout_ns >
      config.maximum_normal_write_gap_ns) {
    return false;
  }
  const std::int64_t combined_io_budget =
      config.adapter.write_timeout_ns + config.read_timeout_ns;
  return combined_io_budget > 0 &&
         combined_io_budget <= config.maximum_step_duration_ns &&
         config.maximum_step_duration_ns <
             config.maximum_normal_write_gap_ns;
}

bool runtimeActuationGatesAreSatisfied(const RuntimeConfig& config) {
  return config.runtime_enabled &&
         config.unverified_protocol_acknowledged &&
         config.physical_device_opt_in && config.actuation_opt_in &&
         config.readiness_gate_passed &&
         config.external_or_durable_estop_strategy_approved;
}

WheeltecSerialRuntime::WheeltecSerialRuntime(
    const RuntimeConfig& config, const RuntimeOperations& operations)
    : config_(config), operations_(operations),
      configuration_valid_(runtimeConfigIsStructurallyValid(config)) {
  if (!operations_.monotonic_now_ns) {
    operations_.monotonic_now_ns = []() { return monotonicNowNs(); };
  }
  if (!operations_.wait_until_monotonic_ns) {
    operations_.wait_until_monotonic_ns = [](std::int64_t deadline_ns) {
      if (deadline_ns <= 0) {
        return false;
      }
      timespec deadline{};
      deadline.tv_sec = static_cast<time_t>(deadline_ns / 1000000000LL);
      deadline.tv_nsec = static_cast<long>(deadline_ns % 1000000000LL);
      int status = 0;
      do {
        status = ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                   &deadline, nullptr);
      } while (status == EINTR);
      return status == 0;
    };
  }
  if (!configuration_valid_) {
    phase_ = RuntimePhase::kTerminalFault;
    diagnostic_ = "invalid_runtime_configuration";
  } else if (!config_.runtime_enabled) {
    phase_ = RuntimePhase::kDisabled;
    diagnostic_ = "actuation_disabled";
  } else if (!runtimeActuationGatesAreSatisfied(config_)) {
    phase_ = RuntimePhase::kDisabled;
    diagnostic_ = "actuation_enable_gates_incomplete";
  } else {
    phase_ = RuntimePhase::kDetached;
    diagnostic_ = "transport_detached";
  }
}

bool WheeltecSerialRuntime::attachTransport(
    ByteTransport* transport, std::int64_t now_monotonic_ns) {
  if (!configuration_valid_ || !observeCallerTime(now_monotonic_ns)) {
    enterFault("attach_clock_invalid", true);
    return false;
  }
  if (!config_.runtime_enabled) {
    transport_ = nullptr;
    adapter_.reset();
    connection_generation_ = 0U;
    phase_ = RuntimePhase::kDisabled;
    diagnostic_ = "actuation_disabled";
    return true;
  }
  const bool remountable_phase =
      phase_ == RuntimePhase::kDetached ||
      phase_ == RuntimePhase::kTerminalDisconnected ||
      phase_ == RuntimePhase::kTerminalFault;
  if (!remountable_phase) {
    if (phase_ == RuntimePhase::kShutdown) {
      diagnostic_ = "shutdown_session_cannot_remount";
      return false;
    }
    const RuntimeDelivery stopped = stopNow(now_monotonic_ns, false);
    enterFault("active_session_remount_denied",
               !stopped.delivered || stopped.delivery_unconfirmed);
    return false;
  }
  if (!runtimeActuationGatesAreSatisfied(config_) || transport == nullptr ||
      !transport->isConnected()) {
    enterFault("transport_attach_denied", false);
    return false;
  }
  const std::uint64_t generation = transport->connectionGeneration();
  if (generation == 0U || generation <= highest_connection_generation_) {
    enterFault("connection_generation_not_strictly_new", true);
    return false;
  }

  transport_ = transport;
  connection_generation_ = generation;
  highest_connection_generation_ = generation;
  adapter_.reset(new WheeltecSerialAdapter(config_.adapter, transport_));
  resetSessionEvidence();
  phase_ = RuntimePhase::kStartupZero;
  diagnostic_ = "startup_zero_required";
  return true;
}

RuntimeStepResult WheeltecSerialRuntime::step(
    std::int64_t now_monotonic_ns) {
  RuntimeStepResult result;
  result.phase = phase_;
  if (!configuration_valid_) {
    result.delivery_unconfirmed = true;
    return result;
  }
  if (phase_ == RuntimePhase::kDisabled ||
      phase_ == RuntimePhase::kShutdown ||
      phase_ == RuntimePhase::kTerminalDisconnected ||
      phase_ == RuntimePhase::kTerminalFault ||
      phase_ == RuntimePhase::kDetached) {
    return result;
  }
  if (!observeCallerTime(now_monotonic_ns)) {
    enterFault("worker_clock_rollback", true);
    result.phase = phase_;
    result.delivery_unconfirmed = true;
    return result;
  }
  const std::int64_t started_ns = clockNow();
  if (started_ns <= 0 || started_ns < now_monotonic_ns ||
      started_ns < last_observed_monotonic_ns_) {
    enterFault("worker_clock_invalid", true);
    result.phase = phase_;
    result.delivery_unconfirmed = true;
    return result;
  }
  last_observed_monotonic_ns_ = started_ns;
  if (!synchronizeTransport()) {
    result.phase = phase_;
    result.delivery_unconfirmed = true;
    return result;
  }
  switch (phase_) {
    case RuntimePhase::kStartupZero:
      return stepStartupZero(started_ns);
    case RuntimePhase::kDrainBacklog:
      return stepDrain(started_ns);
    case RuntimePhase::kAwaitingFeedback:
    case RuntimePhase::kReady:
      return stepRunning(started_ns);
    default:
      result.phase = phase_;
      return result;
  }
}

bool WheeltecSerialRuntime::setAuthorization(
    bool enabled, std::uint64_t authorization_id,
    std::int64_t now_monotonic_ns) {
  if (!configuration_valid_ || !observeCallerTime(now_monotonic_ns)) {
    enterFault("authorization_clock_invalid", true);
    return false;
  }
  const std::int64_t operation_ns = clockNow();
  if (operation_ns <= 0 || operation_ns < now_monotonic_ns ||
      operation_ns < last_observed_monotonic_ns_) {
    enterFault("authorization_clock_invalid", true);
    return false;
  }
  last_observed_monotonic_ns_ = operation_ns;
  if (!enabled) {
    const RuntimeDelivery stopped = stopNow(now_monotonic_ns, false);
    return stopped.delivered && !stopped.delivery_unconfirmed;
  }
  if (!runtimeActuationGatesAreSatisfied(config_) || adapter_ == nullptr ||
      phase_ != RuntimePhase::kReady || local_inhibit_ ||
      !feedbackRecoveryComplete() ||
      !feedbackIsFresh(operation_ns) || authorization_id == 0U ||
      authorization_id <= highest_authorization_id_ ||
      normalWriteGapExpired(saturatedAdd(
          operation_ns, config_.adapter.write_timeout_ns))) {
    const RuntimeDelivery stopped = stopNow(now_monotonic_ns, false);
    diagnostic_ = "authorization_denied";
    (void)stopped;
    return false;
  }
  if (!adapter_->setAuthorization(true, true, authorization_id,
                                  operation_ns)) {
    const RuntimeDelivery stopped = stopNow(now_monotonic_ns, false);
    diagnostic_ = "adapter_authorization_denied";
    (void)stopped;
    return false;
  }
  highest_authorization_id_ = authorization_id;
  diagnostic_ = "authorized_recovery_commands_required";
  return true;
}

RuntimeDelivery WheeltecSerialRuntime::deliver(
    const VehicleExecutionCommand& command,
    std::int64_t receipt_monotonic_ns) {
  RuntimeDelivery result;
  if (!configuration_valid_) {
    result.submission.status = SubmissionStatus::kConfigurationInvalid;
    result.delivery_unconfirmed = true;
    return result;
  }
  if (!observeCallerTime(receipt_monotonic_ns)) {
    enterFault("delivery_clock_invalid", true);
    result.submission.status = SubmissionStatus::kTimestampInvalid;
    result.delivery_unconfirmed = true;
    return result;
  }
  const std::int64_t operation_ns = clockNow();
  result.operation_started_monotonic_ns = operation_ns;
  if (operation_ns <= 0 || operation_ns < receipt_monotonic_ns ||
      operation_ns < last_observed_monotonic_ns_) {
    enterFault("delivery_clock_invalid", true);
    result.submission.status = SubmissionStatus::kTimestampInvalid;
    result.delivery_unconfirmed = true;
    return result;
  }
  last_observed_monotonic_ns_ = operation_ns;
  if (command.sequence_id == 0U ||
      command.sequence_id <= highest_observed_sequence_) {
    revokeAuthorization(operation_ns);
    if (phase_ != RuntimePhase::kReady || adapter_ == nullptr) {
      result.submission.status = SubmissionStatus::kSequenceInvalid;
      return result;
    }
    RuntimeDelivery stopped = stopNow(receipt_monotonic_ns, false);
    stopped.submission.status = SubmissionStatus::kSequenceInvalid;
    stopped.motion_accepted = false;
    return stopped;
  }
  highest_observed_sequence_ = command.sequence_id;
  if (!runtimeActuationGatesAreSatisfied(config_)) {
    result.submission.status = SubmissionStatus::kNotAuthorized;
    return result;
  }
  if (!synchronizeTransport()) {
    result.submission.status = SubmissionStatus::kDisconnected;
    result.delivery_unconfirmed = true;
    return result;
  }
  if (phase_ != RuntimePhase::kReady || local_inhibit_ ||
      !feedbackRecoveryComplete()) {
    result.submission.status = SubmissionStatus::kRecoveryPending;
    return result;
  }
  if (!feedbackIsFresh(operation_ns)) {
    revokeAuthorization(operation_ns);
    local_inhibit_ = true;
    phase_ = RuntimePhase::kAwaitingFeedback;
    diagnostic_ = "feedback_stale";
    RuntimeDelivery stopped = stopNow(receipt_monotonic_ns, false);
    stopped.submission.status = SubmissionStatus::kStale;
    stopped.motion_accepted = false;
    return stopped;
  }
  if (adapter_ == nullptr || !adapter_->isAuthorized()) {
    result.submission.status = SubmissionStatus::kNotAuthorized;
    return result;
  }
  if (normalWriteGapExpired(saturatedAdd(
          operation_ns, config_.adapter.write_timeout_ns))) {
    result = stopNow(receipt_monotonic_ns, false);
    result.command_watchdog_expired = true;
    result.submission.status = SubmissionStatus::kStale;
    diagnostic_ = "normal_write_gap_expired_before_delivery";
    return result;
  }

  result.submission = adapter_->submit(command, operation_ns);
  if (result.submission.status != SubmissionStatus::kAccepted &&
      result.submission.status != SubmissionStatus::kRecoveryPending) {
    const SubmissionStatus rejected_status = result.submission.status;
    RuntimeDelivery stopped = stopNow(receipt_monotonic_ns, false);
    stopped.submission.status = rejected_status;
    stopped.motion_accepted = false;
    return stopped;
  }
  if (result.submission.status == SubmissionStatus::kRecoveryPending) {
    // Recovery must continue emitting bounded exact zero rather than letting a
    // previously latched zero turn multiple command periods into no-op TX.
    adapter_->requireRecoveryZeroWrite();
  }
  RuntimeDelivery cycled = cycleForDelivery(
      commandRequestsMotion(command), operation_ns);
  cycled.submission = result.submission;
  return cycled;
}

RuntimeDelivery WheeltecSerialRuntime::shutdown(
    std::int64_t now_monotonic_ns) {
  return stopNow(now_monotonic_ns, true);
}

RuntimeDelivery WheeltecSerialRuntime::revokeAndStop(
    std::int64_t now_monotonic_ns) {
  return stopNow(now_monotonic_ns, false);
}

RuntimeHealth WheeltecSerialRuntime::health(
    std::int64_t now_monotonic_ns) const {
  RuntimeHealth result;
  result.configuration_valid = configuration_valid_;
  result.enabled = runtimeActuationGatesAreSatisfied(config_);
  result.connected = result.enabled && transport_ != nullptr &&
                     transport_->isConnected() && connection_generation_ != 0U &&
                     phase_ != RuntimePhase::kTerminalDisconnected &&
                     phase_ != RuntimePhase::kTerminalFault &&
                     phase_ != RuntimePhase::kShutdown;
  result.startup_zero_completed = startup_zero_completed_;
  result.backlog_drained = backlog_drained_;
  result.feedback_fresh = result.connected && feedbackIsFresh(now_monotonic_ns);
  result.feedback_recovery_complete = feedbackRecoveryComplete();
  result.actuation_permitted =
      result.feedback_fresh && result.feedback_recovery_complete &&
      !local_inhibit_ && phase_ == RuntimePhase::kReady;
  result.authorization_active =
      result.actuation_permitted && adapter_ != nullptr &&
      adapter_->isAuthorized();
  result.delivery_unconfirmed = delivery_unconfirmed_;
  result.step_overrun = step_overrun_;
  result.connection_generation = result.connected ? connection_generation_ : 0U;
  result.highest_observed_sequence = highest_observed_sequence_;
  result.highest_authorization_id = highest_authorization_id_;
  result.last_feedback_receipt_monotonic_ns =
      last_feedback_receipt_monotonic_ns_;
  result.last_successful_write_completion_ns =
      last_successful_write_completion_ns_;
  result.last_step_elapsed_ns = last_step_elapsed_ns_;
  result.phase = phase_;
  result.diagnostic = diagnostic_;
  return result;
}

bool WheeltecSerialRuntime::observeCallerTime(
    std::int64_t now_monotonic_ns) {
  if (now_monotonic_ns <= 0 ||
      (last_caller_monotonic_ns_ > 0 &&
       now_monotonic_ns < last_caller_monotonic_ns_)) {
    return false;
  }
  if (now_monotonic_ns > last_caller_monotonic_ns_) {
    last_caller_monotonic_ns_ = now_monotonic_ns;
  }
  return true;
}

std::int64_t WheeltecSerialRuntime::clockNow() const {
  try {
    return operations_.monotonic_now_ns ? operations_.monotonic_now_ns() : 0;
  } catch (...) {
    return 0;
  }
}

bool WheeltecSerialRuntime::synchronizeTransport() {
  if (transport_ == nullptr || !transport_->isConnected()) {
    enterDisconnected("transport_disconnected");
    return false;
  }
  if (transport_->connectionGeneration() != connection_generation_ ||
      connection_generation_ == 0U) {
    enterDisconnected("connection_generation_changed_without_remount");
    return false;
  }
  return true;
}

void WheeltecSerialRuntime::resetSessionEvidence() {
  parser_.reset();
  last_feedback_receipt_monotonic_ns_ = 0;
  first_allowed_receipt_monotonic_ns_ = 0;
  last_allowed_receipt_monotonic_ns_ = 0;
  consecutive_allowed_receipts_ = 0U;
  last_successful_write_completion_ns_ = 0;
  drain_reads_ = 0U;
  startup_zero_completed_ = false;
  backlog_drained_ = false;
  local_inhibit_ = true;
  delivery_unconfirmed_ = false;
  write_stream_poisoned_ = false;
  step_overrun_ = false;
  last_step_elapsed_ns_ = 0;
  latest_feedback_ = NormalizedWheeltecFeedback{};
}

void WheeltecSerialRuntime::enterDisconnected(
    const std::string& diagnostic) {
  if (adapter_ != nullptr) {
    adapter_.reset();
  }
  parser_.reset();
  last_feedback_receipt_monotonic_ns_ = 0;
  first_allowed_receipt_monotonic_ns_ = 0;
  last_allowed_receipt_monotonic_ns_ = 0;
  consecutive_allowed_receipts_ = 0U;
  local_inhibit_ = true;
  startup_zero_completed_ = false;
  backlog_drained_ = false;
  delivery_unconfirmed_ = true;
  phase_ = RuntimePhase::kTerminalDisconnected;
  diagnostic_ = diagnostic;
}

void WheeltecSerialRuntime::enterFault(const std::string& diagnostic,
                                       bool delivery_unconfirmed) {
  if (adapter_ != nullptr && last_observed_monotonic_ns_ > 0) {
    adapter_->setAuthorization(false, false, 0U,
                               last_observed_monotonic_ns_);
  }
  local_inhibit_ = true;
  consecutive_allowed_receipts_ = 0U;
  first_allowed_receipt_monotonic_ns_ = 0;
  last_allowed_receipt_monotonic_ns_ = 0;
  delivery_unconfirmed_ = delivery_unconfirmed_ || delivery_unconfirmed;
  phase_ = RuntimePhase::kTerminalFault;
  diagnostic_ = diagnostic;
}

void WheeltecSerialRuntime::revokeAuthorization(
    std::int64_t now_monotonic_ns) {
  if (adapter_ != nullptr && now_monotonic_ns > 0) {
    adapter_->setAuthorization(false, false, 0U, now_monotonic_ns);
  }
}

bool WheeltecSerialRuntime::feedbackIsFresh(
    std::int64_t now_monotonic_ns) const {
  return last_feedback_receipt_monotonic_ns_ > 0 &&
         now_monotonic_ns >= last_feedback_receipt_monotonic_ns_ &&
         now_monotonic_ns - last_feedback_receipt_monotonic_ns_ <=
             config_.maximum_feedback_age_ns;
}

bool WheeltecSerialRuntime::feedbackRecoveryComplete() const {
  return consecutive_allowed_receipts_ >=
             config_.control_allowed_receipts_required &&
         first_allowed_receipt_monotonic_ns_ > 0 &&
         last_allowed_receipt_monotonic_ns_ >=
             first_allowed_receipt_monotonic_ns_ &&
         last_allowed_receipt_monotonic_ns_ -
                 first_allowed_receipt_monotonic_ns_ >=
             config_.control_allowed_minimum_span_ns;
}

bool WheeltecSerialRuntime::normalWriteGapExpired(
    std::int64_t now_monotonic_ns) const {
  return last_successful_write_completion_ns_ > 0 &&
         (now_monotonic_ns < last_successful_write_completion_ns_ ||
          now_monotonic_ns - last_successful_write_completion_ns_ >
              config_.maximum_normal_write_gap_ns);
}

void WheeltecSerialRuntime::noteCycleCompletion(
    const AdapterCycleResult& cycle,
    std::int64_t completed_monotonic_ns) {
  if (cycleCompletedHostWrite(cycle)) {
    last_successful_write_completion_ns_ = completed_monotonic_ns;
  }
  if (cycle.delivery_unconfirmed || cycleAttemptFailed(cycle)) {
    delivery_unconfirmed_ = true;
  }
}

RuntimeStepResult WheeltecSerialRuntime::completeStep(
    RuntimeStepResult result, std::int64_t completed_monotonic_ns) {
  result.step_completed_monotonic_ns = completed_monotonic_ns;
  if (completed_monotonic_ns <= 0 ||
      completed_monotonic_ns < result.step_started_monotonic_ns ||
      completed_monotonic_ns < last_observed_monotonic_ns_) {
    enterFault("step_completion_clock_invalid", true);
    result.phase = phase_;
    result.delivery_unconfirmed = true;
    return result;
  }
  last_observed_monotonic_ns_ = completed_monotonic_ns;
  result.step_elapsed_ns =
      completed_monotonic_ns - result.step_started_monotonic_ns;
  last_step_elapsed_ns_ = result.step_elapsed_ns;
  if (result.step_elapsed_ns > config_.maximum_step_duration_ns) {
    result.step_overrun = true;
    step_overrun_ = true;
    if (phase_ != RuntimePhase::kTerminalFault &&
        phase_ != RuntimePhase::kTerminalDisconnected) {
      const RuntimeDelivery stopped = stopNow(completed_monotonic_ns, false);
      if (stopped.operation_completed_monotonic_ns > 0) {
        result.step_completed_monotonic_ns =
            stopped.operation_completed_monotonic_ns;
        result.step_elapsed_ns = result.step_completed_monotonic_ns -
                                 result.step_started_monotonic_ns;
        last_step_elapsed_ns_ = result.step_elapsed_ns;
      }
    }
    result.delivery_unconfirmed = true;
    enterFault("worker_step_overrun", true);
  }
  result.phase = phase_;
  result.delivery_unconfirmed =
      result.delivery_unconfirmed || delivery_unconfirmed_;
  return result;
}

RuntimeStepResult WheeltecSerialRuntime::stepStartupZero(
    std::int64_t started_monotonic_ns) {
  RuntimeStepResult result;
  result.step_started_monotonic_ns = started_monotonic_ns;
  result.cycle = adapter_->cycle(started_monotonic_ns);
  write_stream_poisoned_ = write_stream_poisoned_ ||
                           cyclePoisonedWriteStream(result.cycle);
  const std::int64_t after_ns = clockNow();
  noteCycleCompletion(result.cycle, after_ns);
  if (cycleMissedDeadline(result.cycle, after_ns)) {
    result.delivery_unconfirmed = true;
    enterFault("startup_zero_write_deadline_missed", true);
  } else if (cycleCompletedHostWrite(result.cycle) &&
      result.cycle.action == AdapterCycleAction::kZeroWritten) {
    startup_zero_completed_ = true;
    delivery_unconfirmed_ = false;
    phase_ = RuntimePhase::kDrainBacklog;
    diagnostic_ = "draining_opening_backlog";
  } else if (result.cycle.action == AdapterCycleAction::kDisconnected) {
    enterDisconnected("startup_zero_disconnected");
  } else if (write_stream_poisoned_) {
    // A second frame must never be appended after an unknown prefix: the
    // firmware's fixed-width receiver could interpret the next header as a
    // nonzero motion field.
    enterFault("startup_zero_partial_write", true);
  } else if (result.cycle.action ==
                 AdapterCycleAction::kZeroRetriesExhausted ||
             result.cycle.action ==
                 AdapterCycleAction::kConfigurationInvalid ||
             result.cycle.action == AdapterCycleAction::kClockInvalid) {
    enterFault("startup_zero_retries_exhausted", true);
  } else {
    // ZeroWriteFailed and ZeroRetryPending stay here.  No read is permitted
    // until a later bounded retry completes the exact-zero host write.
    diagnostic_ = "startup_zero_retry_pending";
  }
  return completeStep(result, after_ns);
}

RuntimeStepResult WheeltecSerialRuntime::stepDrain(
    std::int64_t started_monotonic_ns) {
  RuntimeStepResult result;
  result.step_started_monotonic_ns = started_monotonic_ns;
  std::array<std::uint8_t, kReadBufferBytes> buffer{};
  const std::int64_t deadline_ns =
      saturatedAdd(started_monotonic_ns, config_.drain_read_timeout_ns);
  result.read = transport_->readSome(buffer.data(), buffer.size(), deadline_ns);
  const std::int64_t after_ns = clockNow();
  ++drain_reads_;
  if (result.read.status == TransportStatus::kWouldBlock ||
      result.read.status == TransportStatus::kDeadlineExceeded) {
    backlog_drained_ = true;
    parser_.reset();
    phase_ = RuntimePhase::kAwaitingFeedback;
    diagnostic_ = "awaiting_fresh_control_allowed_feedback";
  } else if (result.read.status == TransportStatus::kDisconnected) {
    enterDisconnected("backlog_drain_disconnected");
  } else if (result.read.status != TransportStatus::kOk ||
             result.read.transferred == 0U ||
             result.read.transferred > buffer.size()) {
    enterFault("backlog_drain_io_invalid", true);
  } else if (drain_reads_ >= config_.maximum_drain_reads) {
    enterFault("backlog_drain_bound_exhausted", true);
  }
  return completeStep(result, after_ns);
}

RuntimeStepResult WheeltecSerialRuntime::stepRunning(
    std::int64_t started_monotonic_ns) {
  RuntimeStepResult result;
  result.step_started_monotonic_ns = started_monotonic_ns;
  std::array<std::uint8_t, kReadBufferBytes> buffer{};

  // Read first.  This ensures a newly arrived FlagStop inhibit or a read that
  // consumes the remaining write-gap budget is handled before normal TX.
  const std::int64_t read_deadline_ns =
      saturatedAdd(started_monotonic_ns, config_.read_timeout_ns);
  result.read = transport_->readSome(buffer.data(), buffer.size(),
                                     read_deadline_ns);
  const std::int64_t receipt_ns = clockNow();
  if (receipt_ns <= 0 || receipt_ns < started_monotonic_ns ||
      receipt_ns < last_observed_monotonic_ns_) {
    enterFault("feedback_receipt_clock_invalid", true);
    return completeStep(result, receipt_ns);
  }
  last_observed_monotonic_ns_ = receipt_ns;

  if (result.read.status == TransportStatus::kDisconnected) {
    enterDisconnected("feedback_read_disconnected");
    return completeStep(result, receipt_ns);
  }
  if (result.read.status == TransportStatus::kOk) {
    if (result.read.transferred == 0U ||
        result.read.transferred > buffer.size()) {
      const RuntimeDelivery stopped = stopNow(receipt_ns, false);
      result.delivery_unconfirmed = true;
      enterFault("feedback_read_size_invalid", true);
      return completeStep(
          result, stopped.operation_completed_monotonic_ns > 0
                      ? stopped.operation_completed_monotonic_ns
                      : receipt_ns);
    }
    const std::uint64_t invalid_stop_flags_before =
        parser_.statistics().invalid_composite_stop_flags;
    const std::vector<ParsedFeedbackFrame> frames = parser_.consumeFrames(
        buffer.data(), result.read.transferred);
    result.invalid_composite_stop_observed_in_read =
        parser_.statistics().invalid_composite_stop_flags >
        invalid_stop_flags_before;
    result.valid_frames_in_read = frames.size();
    if (!frames.empty()) {
      result.feedback_receipts_in_read = 1U;
      result.feedback_available = true;
      const FeedbackCandidate& latest = frames.back().candidate;
      latest_feedback_.signed_speed_mps = latest.forward_speed_mps;
      latest_feedback_.wheel_derived_yaw_rate_radps = latest.yaw_rate_radps;
      latest_feedback_.supply_voltage_v = latest.supply_voltage_v;
      latest_feedback_.receipt_monotonic_ns = receipt_ns;
      result.feedback = latest_feedback_;
      last_feedback_receipt_monotonic_ns_ = receipt_ns;

      bool inhibit_in_batch = false;
      for (const ParsedFeedbackFrame& frame : frames) {
        inhibit_in_batch = inhibit_in_batch ||
                           frame.candidate.control_inhibited ||
                           !frame.candidate.control_allowed;
      }
      result.inhibit_observed_in_read = inhibit_in_batch;
      if (inhibit_in_batch) {
        local_inhibit_ = true;
        consecutive_allowed_receipts_ = 0U;
        first_allowed_receipt_monotonic_ns_ = 0;
        last_allowed_receipt_monotonic_ns_ = 0;
        revokeAuthorization(receipt_ns);
        phase_ = RuntimePhase::kAwaitingFeedback;
        diagnostic_ = "composite_flagstop_inhibited";
      } else if (receipt_ns > last_allowed_receipt_monotonic_ns_) {
        if (consecutive_allowed_receipts_ == 0U) {
          first_allowed_receipt_monotonic_ns_ = receipt_ns;
        }
        last_allowed_receipt_monotonic_ns_ = receipt_ns;
        if (consecutive_allowed_receipts_ <
            config_.control_allowed_receipts_required) {
          ++consecutive_allowed_receipts_;
        }
        if (feedbackRecoveryComplete()) {
          local_inhibit_ = false;
          phase_ = RuntimePhase::kReady;
          diagnostic_ = "feedback_recovered_authorization_required";
        } else {
          local_inhibit_ = true;
          phase_ = RuntimePhase::kAwaitingFeedback;
          diagnostic_ = "feedback_recovery_pending";
        }
      }
    }
    if (result.invalid_composite_stop_observed_in_read) {
      result.inhibit_observed_in_read = true;
      local_inhibit_ = true;
      consecutive_allowed_receipts_ = 0U;
      first_allowed_receipt_monotonic_ns_ = 0;
      last_allowed_receipt_monotonic_ns_ = 0;
      revokeAuthorization(receipt_ns);
      phase_ = RuntimePhase::kAwaitingFeedback;
      diagnostic_ = "invalid_composite_flagstop_inhibited";
    }
  } else if (result.read.status != TransportStatus::kWouldBlock &&
             result.read.status != TransportStatus::kDeadlineExceeded) {
    const RuntimeDelivery stopped = stopNow(receipt_ns, false);
    result.delivery_unconfirmed = true;
    enterFault("feedback_read_io_invalid", true);
    return completeStep(
        result, stopped.operation_completed_monotonic_ns > 0
                    ? stopped.operation_completed_monotonic_ns
                    : receipt_ns);
  }

  if (!feedbackIsFresh(receipt_ns)) {
    result.feedback_watchdog_expired =
        last_feedback_receipt_monotonic_ns_ > 0;
    local_inhibit_ = true;
    consecutive_allowed_receipts_ = 0U;
    first_allowed_receipt_monotonic_ns_ = 0;
    last_allowed_receipt_monotonic_ns_ = 0;
    revokeAuthorization(receipt_ns);
    phase_ = RuntimePhase::kAwaitingFeedback;
    diagnostic_ = last_feedback_receipt_monotonic_ns_ > 0
                      ? "feedback_stale"
                      : "feedback_missing";
  }

  const bool authorized = adapter_ != nullptr && adapter_->isAuthorized();
  const std::int64_t predicted_write_completion = saturatedAdd(
      receipt_ns, config_.adapter.write_timeout_ns);
  if (authorized && normalWriteGapExpired(predicted_write_completion)) {
    result.feedback_watchdog_expired = true;
    revokeAuthorization(receipt_ns);
    diagnostic_ = "normal_write_gap_would_expire";
  }
  if (adapter_ != nullptr && !adapter_->isAuthorized()) {
    // A fresh explicit disarm creates a new zero episode, so an idle enabled
    // session continuously refreshes a recent exact-zero baseline.
    adapter_->setAuthorization(false, false, 0U, receipt_ns);
  }

  const std::int64_t previous_write_completion =
      last_successful_write_completion_ns_;
  result.cycle = adapter_->cycle(receipt_ns);
  write_stream_poisoned_ = write_stream_poisoned_ ||
                           cyclePoisonedWriteStream(result.cycle);
  const std::int64_t after_write_ns = clockNow();
  if (after_write_ns <= 0 || after_write_ns < receipt_ns ||
      after_write_ns < last_observed_monotonic_ns_) {
    enterFault("normal_write_completion_clock_invalid", true);
    result.delivery_unconfirmed = true;
    return completeStep(result, after_write_ns);
  }
  if (result.cycle.action == AdapterCycleAction::kDisconnected) {
    enterDisconnected("normal_write_disconnected");
    return completeStep(result, after_write_ns);
  }
  if (write_stream_poisoned_) {
    result.delivery_unconfirmed = true;
    enterFault("normal_write_stream_poisoned", true);
    return completeStep(result, after_write_ns);
  }
  if (cycleMissedDeadline(result.cycle, after_write_ns)) {
    result.delivery_unconfirmed = true;
    if (result.cycle.action == AdapterCycleAction::kMotionWritten) {
      result.feedback_watchdog_expired = true;
      const RuntimeDelivery stopped = stopNow(after_write_ns, false);
      enterFault("normal_motion_write_deadline_missed", true);
      return completeStep(
          result, stopped.operation_completed_monotonic_ns > 0
                      ? stopped.operation_completed_monotonic_ns
                      : after_write_ns);
    }
    enterFault("normal_zero_write_deadline_missed", true);
    return completeStep(result, after_write_ns);
  }
  if (result.cycle.action == AdapterCycleAction::kMotionWritten &&
      cycleCompletedHostWrite(result.cycle) &&
      previous_write_completion > 0 &&
      after_write_ns - previous_write_completion >
          config_.maximum_normal_write_gap_ns) {
    result.feedback_watchdog_expired = true;
    const RuntimeDelivery stopped = stopNow(after_write_ns, false);
    result.delivery_unconfirmed = !stopped.delivered;
    enterFault("normal_write_completion_gap_exceeded", true);
    return completeStep(
        result, stopped.operation_completed_monotonic_ns > 0
                    ? stopped.operation_completed_monotonic_ns
                    : after_write_ns);
  } else if (cycleAttemptFailed(result.cycle)) {
    if (result.cycle.transferred_bytes > 0U) {
      enterFault("normal_write_partial", true);
    } else {
      const RuntimeDelivery stopped = stopNow(after_write_ns, false);
      result.delivery_unconfirmed = true;
      enterFault("normal_write_failed", true);
      return completeStep(
          result, stopped.operation_completed_monotonic_ns > 0
                      ? stopped.operation_completed_monotonic_ns
                      : after_write_ns);
    }
  }
  noteCycleCompletion(result.cycle, after_write_ns);
  return completeStep(result, after_write_ns);
}

RuntimeDelivery WheeltecSerialRuntime::cycleForDelivery(
    bool motion_requested, std::int64_t started_monotonic_ns) {
  RuntimeDelivery result;
  result.operation_started_monotonic_ns = started_monotonic_ns;
  const std::int64_t before_ns = clockNow();
  if (before_ns <= 0 || before_ns < started_monotonic_ns ||
      before_ns < last_observed_monotonic_ns_) {
    enterFault("delivery_start_clock_invalid", true);
    result.delivery_unconfirmed = true;
    return result;
  }
  last_observed_monotonic_ns_ = before_ns;
  if (normalWriteGapExpired(saturatedAdd(
          before_ns, config_.adapter.write_timeout_ns))) {
    revokeAuthorization(before_ns);
    result.command_watchdog_expired = true;
  }
  const std::int64_t previous_write_completion =
      last_successful_write_completion_ns_;
  result.cycle = adapter_->cycle(before_ns);
  write_stream_poisoned_ = write_stream_poisoned_ ||
                           cyclePoisonedWriteStream(result.cycle);
  const std::int64_t after_ns = clockNow();
  result.operation_completed_monotonic_ns = after_ns;
  if (after_ns <= 0 || after_ns < before_ns ||
      after_ns < last_observed_monotonic_ns_) {
    result.delivery_unconfirmed = true;
    result.motion_accepted = false;
    result.delivered = false;
    enterFault("delivery_completion_clock_invalid", true);
    return result;
  }
  result.operation_elapsed_ns = after_ns - before_ns;
  result.delivered = cycleCompletedHostWrite(result.cycle);
  result.motion_accepted = result.delivered && motion_requested &&
                           result.cycle.action ==
                               AdapterCycleAction::kMotionWritten;
  result.stop_attempted = result.cycle.action ==
                              AdapterCycleAction::kZeroWritten ||
                          result.cycle.action ==
                              AdapterCycleAction::kZeroWriteFailed ||
                          result.cycle.action ==
                              AdapterCycleAction::kZeroRetriesExhausted;
  result.delivery_unconfirmed = result.cycle.delivery_unconfirmed ||
                                cycleAttemptFailed(result.cycle);
  if (result.cycle.action == AdapterCycleAction::kDisconnected) {
    enterDisconnected("delivery_disconnected");
    result.delivery_unconfirmed = true;
  } else if (write_stream_poisoned_) {
    result.delivery_unconfirmed = true;
    result.motion_accepted = false;
    result.delivered = false;
    enterFault("delivery_write_stream_poisoned", true);
  } else if (cycleMissedDeadline(result.cycle, after_ns)) {
    result.command_watchdog_expired = true;
    result.delivery_unconfirmed = true;
    result.motion_accepted = false;
    result.delivered = false;
    if (result.cycle.action == AdapterCycleAction::kMotionWritten) {
      const RuntimeDelivery stopped = stopNow(after_ns, false);
      result.stop_attempted = stopped.stop_attempted;
      result.operation_completed_monotonic_ns =
          stopped.operation_completed_monotonic_ns;
    }
    enterFault("delivery_write_deadline_missed", true);
  } else if (cycleAttemptFailed(result.cycle)) {
    if (result.cycle.transferred_bytes > 0U) {
      enterFault("delivery_partial_write", true);
    } else {
      const RuntimeDelivery stopped = stopNow(after_ns, false);
      result.stop_attempted = stopped.stop_attempted;
      result.delivery_unconfirmed = !stopped.delivered;
      enterFault("delivery_write_failed", !stopped.delivered);
    }
  } else if (result.cycle.action == AdapterCycleAction::kMotionWritten &&
             result.delivered && previous_write_completion > 0 &&
             after_ns - previous_write_completion >
                 config_.maximum_normal_write_gap_ns) {
    result.command_watchdog_expired = true;
    result.motion_accepted = false;
    result.delivery_unconfirmed = true;
    const RuntimeDelivery stopped = stopNow(after_ns, false);
    result.stop_attempted = stopped.stop_attempted;
    result.operation_completed_monotonic_ns =
        stopped.operation_completed_monotonic_ns;
    result.operation_elapsed_ns =
        result.operation_completed_monotonic_ns - started_monotonic_ns;
    enterFault("delivery_write_completion_gap_exceeded", true);
  } else if (result.operation_elapsed_ns >
             config_.maximum_step_duration_ns) {
    result.command_watchdog_expired = true;
    result.motion_accepted = false;
    result.delivery_unconfirmed = true;
    step_overrun_ = true;
    const RuntimeDelivery stopped = stopNow(after_ns, false);
    result.stop_attempted = stopped.stop_attempted;
    result.operation_completed_monotonic_ns =
        stopped.operation_completed_monotonic_ns;
    result.operation_elapsed_ns =
        result.operation_completed_monotonic_ns - started_monotonic_ns;
    enterFault("delivery_step_overrun", true);
  } else {
    noteCycleCompletion(result.cycle, after_ns);
  }
  if (after_ns > last_observed_monotonic_ns_) {
    last_observed_monotonic_ns_ = after_ns;
  }
  return result;
}

RuntimeDelivery WheeltecSerialRuntime::stopNow(
    std::int64_t now_monotonic_ns, bool enter_shutdown) {
  RuntimeDelivery result;
  result.stop_attempted = false;
  result.operation_started_monotonic_ns = now_monotonic_ns;
  if (!configuration_valid_ || !observeCallerTime(now_monotonic_ns)) {
    result.delivery_unconfirmed = true;
    enterFault("stop_clock_invalid", true);
    return result;
  }
  if (!runtimeActuationGatesAreSatisfied(config_)) {
    result.delivered = true;
    result.stop_attempted = true;
    if (enter_shutdown) {
      phase_ = RuntimePhase::kShutdown;
      diagnostic_ = "disabled_shutdown_complete";
    }
    return result;
  }
  if (write_stream_poisoned_) {
    result.delivery_unconfirmed = true;
    diagnostic_ = "write_stream_poisoned_new_generation_required";
    return result;
  }
  if (adapter_ == nullptr || !synchronizeTransport()) {
    result.delivery_unconfirmed = true;
    return result;
  }
  std::int64_t attempt_ns = clockNow();
  if (attempt_ns <= 0 || attempt_ns < now_monotonic_ns ||
      attempt_ns < last_observed_monotonic_ns_) {
    result.delivery_unconfirmed = true;
    enterFault("stop_start_clock_invalid", true);
    return result;
  }
  last_observed_monotonic_ns_ = attempt_ns;
  revokeAuthorization(attempt_ns);
  result.stop_attempted = true;
  for (std::uint32_t attempt = 0U;
       attempt < config_.adapter.max_zero_write_attempts; ++attempt) {
    result.cycle = adapter_->cycle(attempt_ns);
    write_stream_poisoned_ = write_stream_poisoned_ ||
                             cyclePoisonedWriteStream(result.cycle);
    const std::int64_t after_ns = clockNow();
    result.operation_completed_monotonic_ns = after_ns;
    if (after_ns <= 0 || after_ns < attempt_ns ||
        after_ns < last_observed_monotonic_ns_) {
      result.delivery_unconfirmed = true;
      result.delivered = false;
      enterFault("stop_completion_clock_invalid", true);
      return result;
    }
    last_observed_monotonic_ns_ = after_ns;
    result.operation_elapsed_ns = after_ns - now_monotonic_ns;
    result.delivered = !write_stream_poisoned_ &&
                       cycleCompletedHostWrite(result.cycle) &&
                       result.cycle.action ==
                           AdapterCycleAction::kZeroWritten;
    if (cycleMissedDeadline(result.cycle, after_ns)) {
      // The complete frame is known to be an exact zero, but the bounded stop
      // contract was missed and therefore cannot be reported as confirmed.
      result.delivery_unconfirmed = true;
      enterFault("stop_write_deadline_missed", true);
      return result;
    }
    if (result.delivered) {
      result.delivery_unconfirmed = false;
      noteCycleCompletion(result.cycle, after_ns);
      if (phase_ == RuntimePhase::kStartupZero) {
        delivery_unconfirmed_ = false;
        startup_zero_completed_ = true;
        backlog_drained_ = false;
        phase_ = RuntimePhase::kDrainBacklog;
        diagnostic_ = "draining_opening_backlog";
      }
      if (enter_shutdown) {
        local_inhibit_ = true;
        phase_ = RuntimePhase::kShutdown;
        diagnostic_ =
            "shutdown_zero_host_write_complete_ack_unavailable";
      }
      return result;
    }
    if (result.cycle.action == AdapterCycleAction::kDisconnected) {
      result.delivery_unconfirmed = true;
      enterDisconnected("stop_disconnected");
      return result;
    }
    if (write_stream_poisoned_) {
      result.delivery_unconfirmed = true;
      enterFault("stop_write_stream_poisoned_no_retry", true);
      return result;
    }
    if (result.cycle.transferred_bytes > 0U) {
      result.delivery_unconfirmed = true;
      enterFault("stop_partial_write_no_retry", true);
      return result;
    }
    if (result.cycle.action ==
            AdapterCycleAction::kZeroRetriesExhausted ||
        result.cycle.action == AdapterCycleAction::kConfigurationInvalid ||
        result.cycle.action == AdapterCycleAction::kClockInvalid) {
      break;
    }
    const std::int64_t retry_ns = saturatedAdd(
        attempt_ns, config_.adapter.zero_retry_interval_ns);
    if (!operations_.wait_until_monotonic_ns ||
        !operations_.wait_until_monotonic_ns(retry_ns)) {
      result.delivery_unconfirmed = true;
      enterFault("stop_zero_retry_wait_failed", true);
      return result;
    }
    attempt_ns = clockNow();
    if (attempt_ns < retry_ns || attempt_ns < after_ns) {
      result.delivery_unconfirmed = true;
      enterFault("stop_zero_retry_clock_invalid", true);
      return result;
    }
  }
  result.delivered = false;
  result.delivery_unconfirmed = true;
  enterFault("stop_zero_retries_exhausted", true);
  return result;
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
