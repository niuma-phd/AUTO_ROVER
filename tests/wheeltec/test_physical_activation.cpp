#include "auto_rover_vcu_wheeltec_serial/physical_activation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

struct WriteOutcome {
  wheeltec::TransportStatus status{wheeltec::TransportStatus::kOk};
  std::size_t transferred{std::numeric_limits<std::size_t>::max()};
  bool delivery_unconfirmed{false};
  std::int64_t elapsed_ns{1};
  bool throws{false};
};

class ScriptedActivationTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedActivationTransport(std::int64_t* clock)
      : clock_(clock) {}

  bool isConnected() const override { return connected; }
  std::uint64_t connectionGeneration() const override { return 1U; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t deadline_ns) override {
    const std::size_t index = write_calls++;
    if (index < frames.size()) {
      frames[index].fill(0U);
      const std::size_t copied = size < frames[index].size()
                                     ? size
                                     : frames[index].size();
      for (std::size_t byte = 0U; byte < copied; ++byte) {
        frames[index][byte] = data[byte];
      }
      deadlines[index] = deadline_ns;
      sizes[index] = size;
    }
    const WriteOutcome outcome =
        index < outcomes.size() ? outcomes[index] : WriteOutcome{};
    if (outcome.throws) {
      throw std::runtime_error("injected write exception");
    }
    if (clock_ != nullptr) {
      *clock_ += outcome.elapsed_ns;
    }
    if (outcome.status == wheeltec::TransportStatus::kDisconnected) {
      connected = false;
    }
    const std::size_t transferred =
        outcome.transferred == std::numeric_limits<std::size_t>::max()
            ? size
            : outcome.transferred;
    return {outcome.status, transferred, 0,
            outcome.delivery_unconfirmed};
  }

  wheeltec::IoResult readSome(std::uint8_t*, std::size_t,
                              std::int64_t) override {
    ++read_calls;
    return {wheeltec::TransportStatus::kIoError, 0U, 0, false};
  }

  std::int64_t* clock_{nullptr};
  bool connected{true};
  std::array<WriteOutcome, 8U> outcomes{};
  std::array<wheeltec::CommandFrame, 8U> frames{};
  std::array<std::size_t, 8U> sizes{};
  std::array<std::int64_t, 8U> deadlines{};
  std::size_t write_calls{0U};
  std::size_t read_calls{0U};
};

wheeltec::PhysicalActivationConfig config() {
  wheeltec::PhysicalActivationConfig result;
  result.codec_limits.max_forward_speed_mps = 0.50;
  result.codec_limits.max_abs_curvature_inv_m =
      wheeltec::kPhase1MaximumAbsCurvatureInvM;
  result.write_timeout_ns = 10;
  result.zero_retry_interval_ns = 5;
  result.max_zero_write_attempts = 3U;
  return result;
}

wheeltec::PhysicalActivationOperations operations(std::int64_t* clock) {
  wheeltec::PhysicalActivationOperations result;
  result.monotonic_now_ns = [clock]() { return *clock; };
  result.wait_until_monotonic_ns = [clock](std::int64_t deadline_ns) {
    if (*clock < deadline_ns) {
      *clock = deadline_ns;
    }
    return true;
  };
  return result;
}

bool isExactZero(const wheeltec::CommandFrame& frame) {
  if (frame[0U] != wheeltec::kFrameHeader ||
      frame[wheeltec::kCommandFrameSize - 1U] != wheeltec::kFrameTail) {
    return false;
  }
  for (std::size_t index = 3U; index <= 8U; ++index) {
    if (frame[index] != 0U) {
      return false;
    }
  }
  return true;
}

bool isParserResyncPadding(const ScriptedActivationTransport& transport,
                           std::size_t write_index) {
  if (write_index >= transport.frames.size() ||
      transport.sizes[write_index] !=
          wheeltec::kCommandParserResyncPaddingSize) {
    return false;
  }
  for (std::size_t index = 0U;
       index < wheeltec::kCommandParserResyncPaddingSize; ++index) {
    if (transport.frames[write_index][index] != 0U) {
      return false;
    }
  }
  return true;
}

void testPaddingResynchronizesEveryCandidateParserCount() {
  const auto padding = wheeltec::commandParserResyncPadding();
  expect(padding.size() == wheeltec::kCommandFrameSize - 1U,
         "parser resync padding covers every nonzero retained count");
  for (const std::uint8_t byte : padding) {
    expect(byte == 0U,
           "parser resync padding contains no candidate frame header");
  }

  for (std::size_t initial_count = 0U;
       initial_count < wheeltec::kCommandFrameSize; ++initial_count) {
    std::size_t count = initial_count;
    for (const std::uint8_t byte : padding) {
      if (count == 0U && byte != wheeltec::kFrameHeader) {
        continue;
      }
      ++count;
      if (count == wheeltec::kCommandFrameSize) {
        // The completion byte is 0x00, so it cannot satisfy the required
        // 0x7d tail and the reviewed candidate parser resets its count.
        expect(byte != wheeltec::kFrameTail,
               "padding cannot complete a valid retained command frame");
        count = 0U;
      }
    }
    expect(count == 0U,
           "ten zero bytes leave every candidate parser count at zero");
  }
}

void testSuccessfulActivationIsPaddingThenExactZeroAndNoRead() {
  std::int64_t clock = 100;
  ScriptedActivationTransport transport(&clock);
  const wheeltec::PreparedPhysicalActivation activation(config(),
                                                          operations(&clock));
  expect(activation.prepared(), "valid activation is prepared before open");
  const wheeltec::PhysicalActivationResult result =
      activation.activate(&transport);
  expect(result.succeeded() &&
             result.parser_resync_padding_host_write_complete &&
             result.exact_zero_host_write_complete &&
             !result.delivery_unconfirmed &&
             !result.write_stream_poisoned && result.attempts == 1U &&
             transport.write_calls == 2U && transport.read_calls == 0U &&
             isParserResyncPadding(transport, 0U) &&
             transport.sizes[1U] == wheeltec::kCommandFrameSize &&
             isExactZero(transport.frames[1U]),
         "activation starts with zero-only parser padding and then writes one exact zero command");
}

void testKnownZeroBytePaddingFailureRestartsWholeSequence() {
  std::int64_t clock = 100;
  ScriptedActivationTransport transport(&clock);
  transport.outcomes[0U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                            0U, false, 1, false};
  const wheeltec::PreparedPhysicalActivation activation(config(),
                                                          operations(&clock));
  const wheeltec::PhysicalActivationResult result =
      activation.activate(&transport);
  expect(result.succeeded() && result.attempts == 2U &&
             transport.write_calls == 3U && transport.read_calls == 0U &&
             isParserResyncPadding(transport, 0U) &&
             isParserResyncPadding(transport, 1U) &&
             isExactZero(transport.frames[2U]) &&
             transport.deadlines[1U] > transport.deadlines[0U],
         "a zero-byte padding failure gets a bounded full-sequence retry");
}

void testPartialOrUnknownProgressRestartsFromSafePadding() {
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                              4U, true, 1, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.succeeded() && !result.delivery_unconfirmed &&
               !result.write_stream_poisoned && result.attempts == 2U &&
               result.recovery_restarts == 1U &&
               transport.write_calls == 3U && transport.read_calls == 0U &&
               isParserResyncPadding(transport, 0U) &&
               isParserResyncPadding(transport, 1U) &&
               isExactZero(transport.frames[2U]),
           "partial zero padding is recoverable only by restarting with the full padding sequence");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U].throws = true;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.succeeded() && !result.delivery_unconfirmed &&
               !result.write_stream_poisoned && result.attempts == 2U &&
               result.recovery_restarts == 1U &&
               transport.write_calls == 3U && transport.read_calls == 0U &&
               isParserResyncPadding(transport, 0U) &&
               isParserResyncPadding(transport, 1U) &&
               isExactZero(transport.frames[2U]),
           "unknown padding progress is recovered without ever appending a nonzero candidate");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[1U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                              5U, true, 1, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.succeeded() && result.attempts == 2U &&
               result.recovery_restarts == 1U &&
               transport.write_calls == 4U &&
               isParserResyncPadding(transport, 0U) &&
               isExactZero(transport.frames[1U]) &&
               isParserResyncPadding(transport, 2U) &&
               isExactZero(transport.frames[3U]),
           "a partial exact-zero prefix is invalidated by a new full padding sequence before retry");
  }
}

void testEveryPartialExactZeroPrefixRestartsWholeSequence() {
  for (std::size_t prefix_size = 1U;
       prefix_size < wheeltec::kCommandFrameSize; ++prefix_size) {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[1U] = {
        wheeltec::TransportStatus::kDeadlineExceeded, prefix_size, true, 1,
        false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.succeeded() && result.attempts == 2U &&
               result.recovery_restarts == 1U &&
               transport.write_calls == 4U &&
               isParserResyncPadding(transport, 0U) &&
               isExactZero(transport.frames[1U]) &&
               isParserResyncPadding(transport, 2U) &&
               isExactZero(transport.frames[3U]),
           "every partial exact-zero prefix is invalidated by a full zero-only padding restart");
  }
}

void testDisconnectClockAndWaitFailuresFailClosed() {
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.connected = false;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kTransportUnavailable &&
               result.attempts == 0U && result.delivery_unconfirmed &&
               transport.write_calls == 0U,
           "a disconnected transport is rejected before the first padding write");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {
        wheeltec::TransportStatus::kDisconnected, 4U, true, 1, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kTransportUnavailable &&
               result.attempts == 1U && result.delivery_unconfirmed &&
               result.write_stream_poisoned &&
               transport.write_calls == 1U,
           "a partial padding disconnect fails terminally and records an unresolved stream");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[1U] = {
        wheeltec::TransportStatus::kDisconnected, 5U, true, 1, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kTransportUnavailable &&
               result.attempts == 1U && result.delivery_unconfirmed &&
               result.write_stream_poisoned &&
               transport.write_calls == 2U,
           "a partial exact-zero disconnect never retries on the disconnected generation");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U].elapsed_ns = -1;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kClockInvalid &&
               result.parser_resync_padding_host_write_complete &&
               result.delivery_unconfirmed &&
               transport.write_calls == 1U,
           "a clock rollback after padding fails closed before exact zero");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[1U].elapsed_ns = -1;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kClockInvalid &&
               result.exact_zero_host_write_complete &&
               result.delivery_unconfirmed &&
               transport.write_calls == 2U,
           "a clock rollback after exact zero cannot establish activation");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {
        wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1, false};
    wheeltec::PhysicalActivationOperations failing_wait = operations(&clock);
    failing_wait.wait_until_monotonic_ns =
        [](std::int64_t) { return false; };
    const wheeltec::PreparedPhysicalActivation activation(config(),
                                                            failing_wait);
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kRetryWaitFailed &&
               result.delivery_unconfirmed &&
               transport.write_calls == 1U,
           "a failed retry wait prevents any later write");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {
        wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1, false};
    wheeltec::PhysicalActivationOperations rollback_after_wait =
        operations(&clock);
    rollback_after_wait.wait_until_monotonic_ns =
        [&clock](std::int64_t deadline_ns) {
          clock = deadline_ns - 1;
          return true;
        };
    const wheeltec::PreparedPhysicalActivation activation(
        config(), rollback_after_wait);
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kClockInvalid &&
               result.delivery_unconfirmed &&
               transport.write_calls == 1U,
           "a retry clock that does not reach its deadline fails closed");
  }
}

void testRetryExhaustionAndLateFullZeroFailClosed() {
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                              0U, false, 11, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kDeadlineMissed &&
               !result.exact_zero_host_write_complete &&
               result.delivery_unconfirmed &&
               !result.write_stream_poisoned &&
               transport.write_calls == 1U && transport.read_calls == 0U,
           "a confirmed zero-byte padding attempt completing after its deadline is never retried");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    for (std::size_t index = 0U; index < 3U; ++index) {
      WriteOutcome& outcome = transport.outcomes[index];
      outcome = {wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1,
                 false};
    }
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kZeroRetriesExhausted &&
               !result.write_stream_poisoned &&
               result.delivery_unconfirmed && result.attempts == 3U &&
               transport.write_calls == 3U && transport.read_calls == 0U,
           "known zero-byte padding failures stop after the configured bounded sequence attempts");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[1U].elapsed_ns = 11;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kDeadlineMissed &&
               result.exact_zero_host_write_complete &&
               result.delivery_unconfirmed &&
               !result.write_stream_poisoned &&
               transport.write_calls == 2U && transport.read_calls == 0U,
           "a complete but late exact-zero after padding is known zero yet cannot activate the session");
  }
}

void testInvalidPreparationNeverTouchesTransport() {
  std::int64_t clock = 100;
  ScriptedActivationTransport transport(&clock);
  wheeltec::PhysicalActivationConfig invalid = config();
  invalid.codec_limits.max_forward_speed_mps = 0.0;
  const wheeltec::PreparedPhysicalActivation activation(
      invalid, operations(&clock));
  const wheeltec::PhysicalActivationResult result =
      activation.activate(&transport);
  expect(!activation.prepared() &&
             result.status ==
                 wheeltec::PhysicalActivationStatus::kNotPrepared &&
             transport.write_calls == 0U && transport.read_calls == 0U,
         "invalid preparation fails before any physical transport I/O");
}

}  // namespace

int main() {
  testPaddingResynchronizesEveryCandidateParserCount();
  testSuccessfulActivationIsPaddingThenExactZeroAndNoRead();
  testKnownZeroBytePaddingFailureRestartsWholeSequence();
  testPartialOrUnknownProgressRestartsFromSafePadding();
  testEveryPartialExactZeroPrefixRestartsWholeSequence();
  testDisconnectClockAndWaitFailuresFailClosed();
  testRetryExhaustionAndLateFullZeroFailClosed();
  testInvalidPreparationNeverTouchesTransport();
  if (failures != 0) {
    std::fprintf(stderr, "%d physical activation assertion(s) failed\n",
                 failures);
    return EXIT_FAILURE;
  }
  std::puts("physical activation guard tests passed");
  return EXIT_SUCCESS;
}
