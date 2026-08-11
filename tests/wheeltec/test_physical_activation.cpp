#include "auto_rover_vcu_wheeltec_serial/physical_activation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
  std::size_t transferred{wheeltec::kCommandFrameSize};
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
    return {outcome.status, outcome.transferred, 0,
            outcome.delivery_unconfirmed};
  }

  wheeltec::IoResult readSome(std::uint8_t*, std::size_t,
                              std::int64_t) override {
    ++read_calls;
    return {wheeltec::TransportStatus::kIoError, 0U, 0, false};
  }

  std::int64_t* clock_{nullptr};
  bool connected{true};
  std::array<WriteOutcome, 3U> outcomes{};
  std::array<wheeltec::CommandFrame, 3U> frames{};
  std::array<std::int64_t, 3U> deadlines{};
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

void testSuccessfulActivationIsOneExactZeroAndNoRead() {
  std::int64_t clock = 100;
  ScriptedActivationTransport transport(&clock);
  const wheeltec::PreparedPhysicalActivation activation(config(),
                                                          operations(&clock));
  expect(activation.prepared(), "valid activation is prepared before open");
  const wheeltec::PhysicalActivationResult result =
      activation.activate(&transport);
  expect(result.succeeded() && result.exact_zero_host_write_complete &&
             !result.delivery_unconfirmed &&
             !result.write_stream_poisoned && result.attempts == 1U &&
             transport.write_calls == 1U && transport.read_calls == 0U &&
             isExactZero(transport.frames[0U]),
         "activation's first and only protocol I/O is a complete exact zero");
}

void testKnownZeroByteFailureRetriesOnlyExactZero() {
  std::int64_t clock = 100;
  ScriptedActivationTransport transport(&clock);
  transport.outcomes[0U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                            0U, false, 1, false};
  const wheeltec::PreparedPhysicalActivation activation(config(),
                                                          operations(&clock));
  const wheeltec::PhysicalActivationResult result =
      activation.activate(&transport);
  expect(result.succeeded() && result.attempts == 2U &&
             transport.write_calls == 2U && transport.read_calls == 0U &&
             isExactZero(transport.frames[0U]) &&
             isExactZero(transport.frames[1U]) &&
             transport.deadlines[1U] > transport.deadlines[0U],
         "a proven zero-byte failure gets one bounded retry and every attempt is exact zero");
}

void testPartialAndThrowPoisonWithoutRetry() {
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U] = {wheeltec::TransportStatus::kDeadlineExceeded,
                              4U, true, 1, false};
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kWriteStreamPoisoned &&
               result.delivery_unconfirmed &&
               result.write_stream_poisoned &&
               transport.write_calls == 1U && transport.read_calls == 0U,
           "a partial activation write poisons the generation and is never followed by another frame");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U].throws = true;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kWriteStreamPoisoned &&
               result.delivery_unconfirmed &&
               result.write_stream_poisoned &&
               transport.write_calls == 1U && transport.read_calls == 0U,
           "an exception with unknown write progress poisons without retry");
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
           "a confirmed zero-byte attempt completing after its deadline is never retried");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    for (WriteOutcome& outcome : transport.outcomes) {
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
           "known zero-byte failures stop after the configured bounded attempts");
  }
  {
    std::int64_t clock = 100;
    ScriptedActivationTransport transport(&clock);
    transport.outcomes[0U].elapsed_ns = 11;
    const wheeltec::PreparedPhysicalActivation activation(
        config(), operations(&clock));
    const wheeltec::PhysicalActivationResult result =
        activation.activate(&transport);
    expect(result.status ==
                   wheeltec::PhysicalActivationStatus::kDeadlineMissed &&
               result.exact_zero_host_write_complete &&
               result.delivery_unconfirmed &&
               !result.write_stream_poisoned &&
               transport.write_calls == 1U && transport.read_calls == 0U,
           "a complete but late activation zero is known zero yet cannot activate the session");
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
  testSuccessfulActivationIsOneExactZeroAndNoRead();
  testKnownZeroByteFailureRetriesOnlyExactZero();
  testPartialAndThrowPoisonWithoutRetry();
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
