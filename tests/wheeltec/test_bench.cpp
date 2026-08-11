#include "auto_rover_vcu_wheeltec_serial/bench.hpp"
#include "auto_rover_vcu_wheeltec_serial/codec.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <new>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

int g_failures = 0;
int g_test_cases_run = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
  }
}

void runTest(void (*test)()) {
  ++g_test_cases_run;
  test();
}

std::uint8_t xorBytes(const std::uint8_t* bytes, std::size_t count) {
  std::uint8_t value = 0U;
  for (std::size_t index = 0U; index < count; ++index) {
    value = static_cast<std::uint8_t>(value ^ bytes[index]);
  }
  return value;
}

void putSignedBigEndian(std::int16_t value, std::uint8_t* high,
                        std::uint8_t* low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  *high = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  *low = static_cast<std::uint8_t>(bits & 0xFFU);
}

std::int16_t signedBigEndian(std::uint8_t high, std::uint8_t low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low));
  return bits <= 0x7FFFU ? static_cast<std::int16_t>(bits)
                         : static_cast<std::int16_t>(
                               static_cast<std::int32_t>(bits) - 65536);
}

wheeltec::FeedbackFrame feedback(std::int16_t forward,
                                 std::int16_t lateral = 0,
                                 std::int16_t yaw = 0,
                                 std::uint8_t composite_stop_flag = 0U) {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = composite_stop_flag;
  putSignedBigEndian(forward, &frame[2U], &frame[3U]);
  putSignedBigEndian(lateral, &frame[4U], &frame[5U]);
  putSignedBigEndian(yaw, &frame[6U], &frame[7U]);
  putSignedBigEndian(24000, &frame[20U], &frame[21U]);
  frame[22U] = xorBytes(frame.data(), 22U);
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

class ScriptedTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedTransport(std::int64_t* clock) : clock_(clock) {}

  bool isConnected() const override { return connected; }
  std::uint64_t connectionGeneration() const override { return generation; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t) override {
    const bool nonzero = size == wheeltec::kCommandFrameSize &&
                         commandWireSpeedForTransport(data, size) > 0;
    writes.emplace_back(data, data + size);
    write_receipts.push_back(clock_ == nullptr ? 0 : *clock_);
    std::int64_t write_duration_ns = default_write_duration_ns;
    if (!write_durations_ns.empty()) {
      write_duration_ns = write_durations_ns.front();
      write_durations_ns.pop_front();
    }
    if (clock_ != nullptr && write_duration_ns > 0) {
      *clock_ += write_duration_ns;
    }
    write_completions.push_back(clock_ == nullptr ? 0 : *clock_);
    wheeltec::TransportStatus status = wheeltec::TransportStatus::kOk;
    if (nonzero) {
      saw_nonzero_write_attempt = true;
    }
    if (throw_first_nonzero_write && nonzero &&
        !first_nonzero_write_threw) {
      first_nonzero_write_threw = true;
      throw std::runtime_error("injected unknown-progress write failure");
    }
    if (partial_first_nonzero_write && nonzero &&
        !first_nonzero_write_was_partial) {
      first_nonzero_write_was_partial = true;
      write_statuses.push_back(wheeltec::TransportStatus::kDeadlineExceeded);
      return {wheeltec::TransportStatus::kDeadlineExceeded, 4U, ETIMEDOUT,
              true};
    }
    const bool zero_after_nonzero = !nonzero && saw_nonzero_write_attempt;
    if (partial_first_zero_after_nonzero && zero_after_nonzero &&
        !first_zero_after_nonzero_was_partial) {
      first_zero_after_nonzero_was_partial = true;
      write_statuses.push_back(wheeltec::TransportStatus::kDeadlineExceeded);
      return {wheeltec::TransportStatus::kDeadlineExceeded, 4U, ETIMEDOUT,
              true};
    }
    if (fail_first_nonzero_write && nonzero &&
        !first_nonzero_write_failed) {
      first_nonzero_write_failed = true;
      status = wheeltec::TransportStatus::kIoError;
    } else if (!write_results.empty()) {
      status = write_results.front();
      write_results.pop_front();
    }
    write_statuses.push_back(status);
    if (status == wheeltec::TransportStatus::kDisconnected) {
      connected = false;
    }
    return {status, status == wheeltec::TransportStatus::kOk ? size : 0U,
            status == wheeltec::TransportStatus::kOk ? 0 : EIO,
            status == wheeltec::TransportStatus::kDisconnected};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t deadline_ns) override {
    if (clock_ != nullptr && deadline_ns - *clock_ <= 1000000 &&
        !drain_queued_data) {
      *clock_ = deadline_ns;
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
              false};
    }
    if (clock_ != nullptr && !immediate_normal_reads &&
        *clock_ < deadline_ns) {
      *clock_ = deadline_ns;
    }
    if (clock_ != nullptr && immediate_normal_reads &&
        !read_delays_ns.empty()) {
      const std::int64_t delay_ns = read_delays_ns.front();
      read_delays_ns.pop_front();
      if (delay_ns > 0) {
        *clock_ += delay_ns;
      }
    }
    ++normal_read_calls;
    if (clock_ != nullptr && jump_on_normal_read != 0U &&
        normal_read_calls == jump_on_normal_read) {
      *clock_ += 200000000;
    }
    if (!forced_deadline_timeouts.empty()) {
      const bool force_timeout = forced_deadline_timeouts.front();
      forced_deadline_timeouts.pop_front();
      if (force_timeout) {
        if (clock_ != nullptr && *clock_ < deadline_ns) {
          *clock_ = deadline_ns;
        }
        if (clock_ != nullptr && deadline_timeout_overshoot_ns > 0) {
          *clock_ += deadline_timeout_overshoot_ns;
        }
        ++forced_deadline_timeout_count;
        return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
                false};
      }
    }
    if (!connected) {
      return {wheeltec::TransportStatus::kDisconnected, 0U, ENODEV, true};
    }
    if (disconnect_on_read) {
      disconnect_on_read = false;
      connected = false;
      return {wheeltec::TransportStatus::kDisconnected, 0U, ENODEV, true};
    }
    if (reads.empty()) {
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
              false};
    }
    std::vector<std::uint8_t> bytes = std::move(reads.front());
    reads.pop_front();
    if (bytes.size() > capacity) {
      return {wheeltec::TransportStatus::kOk, capacity + 1U, 0, false};
    }
    std::copy(bytes.begin(), bytes.end(), data);
    return {wheeltec::TransportStatus::kOk, bytes.size(), 0, false};
  }

  std::int64_t* clock_{nullptr};
  bool connected{true};
  bool disconnect_on_read{false};
  bool drain_queued_data{false};
  bool immediate_normal_reads{false};
  bool fail_first_nonzero_write{false};
  bool first_nonzero_write_failed{false};
  bool partial_first_nonzero_write{false};
  bool first_nonzero_write_was_partial{false};
  bool throw_first_nonzero_write{false};
  bool first_nonzero_write_threw{false};
  bool partial_first_zero_after_nonzero{false};
  bool first_zero_after_nonzero_was_partial{false};
  bool saw_nonzero_write_attempt{false};
  std::uint64_t generation{1U};
  std::size_t normal_read_calls{0U};
  std::size_t jump_on_normal_read{0U};
  std::size_t forced_deadline_timeout_count{0U};
  std::int64_t default_write_duration_ns{0};
  std::int64_t deadline_timeout_overshoot_ns{0};
  std::deque<std::vector<std::uint8_t>> reads;
  std::deque<bool> forced_deadline_timeouts;
  std::deque<wheeltec::TransportStatus> write_results;
  std::deque<std::int64_t> write_durations_ns;
  std::deque<std::int64_t> read_delays_ns;
  std::vector<std::vector<std::uint8_t>> writes;
  std::vector<std::int64_t> write_receipts;
  std::vector<std::int64_t> write_completions;
  std::vector<wheeltec::TransportStatus> write_statuses;

 private:
  static std::int16_t commandWireSpeedForTransport(
      const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size != wheeltec::kCommandFrameSize) {
      return 0;
    }
    const std::uint16_t bits = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[3U]) << 8U) |
        static_cast<std::uint16_t>(data[4U]));
    return bits <= 0x7FFFU
               ? static_cast<std::int16_t>(bits)
               : static_cast<std::int16_t>(
                     static_cast<std::int32_t>(bits) - 65536);
  }
};

wheeltec::BenchConfig exactZeroConfig() {
  wheeltec::BenchConfig config;
  config.mode = wheeltec::BenchMode::kExactZero;
  config.unverified_protocol_acknowledged = true;
  config.physical_device_opt_in = true;
  config.actuation_opt_in = true;
  config.operator_confirmation_token =
      wheeltec::kBenchOperatorConfirmationToken;
  config.passive_evidence_token =
      "passive-capture-sha256:"
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  config.exact_zero_duration_ns = 600000000;
  config.read_timeout_ns = 20000000;
  config.maximum_feedback_age_ns = 60000000;
  config.initial_feedback_deadline_ns = 60000000;
  return config;
}

wheeltec::BenchConfig rampConfig() {
  wheeltec::BenchConfig config = exactZeroConfig();
  config.mode = wheeltec::BenchMode::kStraightRamp;
  config.exact_zero_evidence_token =
      "exact-zero-sha256:"
      "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789";
  config.target_speed_mps = 0.02;
  config.hold_duration_ns = 100000000;
  config.exact_zero_duration_ns = 0;
  return config;
}

wheeltec::BenchOperations operations(std::int64_t* clock,
                                     std::vector<std::string>* records) {
  wheeltec::BenchOperations result;
  result.monotonic_now_ns = [clock]() { return *clock; };
  result.wait_until_monotonic_ns = [clock](std::int64_t deadline) {
    if (*clock < deadline) {
      *clock = deadline;
    }
    return true;
  };
  result.record_json_line = [records](const std::string& record) {
    records->push_back(record);
    return true;
  };
  result.stop_requested = []() { return false; };
  return result;
}

void addFeedback(ScriptedTransport* transport, std::int16_t forward,
                 std::size_t count) {
  const wheeltec::FeedbackFrame frame = feedback(forward);
  for (std::size_t index = 0U; index < count; ++index) {
    transport->reads.emplace_back(frame.begin(), frame.end());
  }
}

bool isZeroCommand(const std::vector<std::uint8_t>& frame) {
  return frame.size() == wheeltec::kCommandFrameSize && frame[3U] == 0U &&
         frame[4U] == 0U && frame[5U] == 0U && frame[6U] == 0U &&
         frame[7U] == 0U && frame[8U] == 0U;
}

std::int16_t commandWireSpeed(const std::vector<std::uint8_t>& frame) {
  if (frame.size() != wheeltec::kCommandFrameSize) {
    return 0;
  }
  return signedBigEndian(frame[3U], frame[4U]);
}

bool strictNormalWireEnvelopeHolds(const ScriptedTransport& transport,
                                   std::size_t normal_write_count) {
  if (normal_write_count == 0U ||
      normal_write_count > transport.writes.size() ||
      normal_write_count > transport.write_receipts.size() ||
      normal_write_count > transport.write_completions.size()) {
    return false;
  }
  wheeltec::BenchWireAccelerationEnvelope replay;
  for (std::size_t index = 0U; index < normal_write_count; ++index) {
    const std::int16_t wire_speed =
        commandWireSpeed(transport.writes[index]);
    if (!replay.transitionAllowed(wire_speed,
                                  transport.write_receipts[index]) ||
        !replay.noteSuccessfulNormalWrite(
            wire_speed, transport.write_completions[index])) {
      return false;
    }
  }
  return true;
}

void testAllGatesAndEvidenceFailBeforeIo() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  wheeltec::BenchConfig config = exactZeroConfig();
  config.actuation_opt_in = false;
  wheeltec::BenchResult result =
      session.run(&transport, config, operations(&clock, &records));
  expect(result.status == wheeltec::BenchStatus::kAuthorizationDenied &&
             transport.writes.empty(),
         "missing one of three actuation gates refuses all I/O");

  config = rampConfig();
  config.exact_zero_evidence_token.clear();
  result = session.run(&transport, config, operations(&clock, &records));
  expect(result.status == wheeltec::BenchStatus::kEvidenceMissing &&
             transport.writes.empty(),
         "straight ramp requires both external evidence tokens");
}

void testIntegerWireAccelerationEnvelopeBoundariesAndFailureState() {
  constexpr std::int64_t kStartNs = 1000000000;
  for (const std::pair<std::int64_t, std::int16_t> boundary :
       {std::make_pair<std::int64_t, std::int16_t>(17000000, 3),
        std::make_pair<std::int64_t, std::int16_t>(20000000, 4),
        std::make_pair<std::int64_t, std::int16_t>(25000000, 5)}) {
    wheeltec::BenchWireAccelerationEnvelope envelope;
    expect(envelope.noteSuccessfulNormalWrite(0, kStartNs),
           "a completed regular zero establishes the integer wire baseline");
    std::int16_t coupled = -1;
    expect(envelope.coupleToward(500, kStartNs + boundary.first,
                                 &coupled) &&
               coupled == boundary.second &&
               envelope.transitionAllowed(coupled,
                                          kStartNs + boundary.first) &&
               !envelope.transitionAllowed(
                   static_cast<std::int16_t>(boundary.second + 1),
                   kStartNs + boundary.first),
           "17/20/25 ms budgets floor to 3/4/5 raw units with no quantization margin");
  }

  wheeltec::BenchWireAccelerationEnvelope measured_jitter;
  expect(measured_jitter.noteSuccessfulNormalWrite(0, kStartNs),
         "jitter test establishes a regular zero baseline");
  std::int16_t coupled = -1;
  expect(measured_jitter.coupleToward(500, kStartNs + 21470000,
                                     &coupled) &&
             coupled == 4 &&
             !measured_jitter.transitionAllowed(5,
                                                kStartNs + 21470000),
         "the observed 5-raw/21.47 ms counterexample is rejected rather than rounded up");

  wheeltec::BenchWireAccelerationEnvelope down;
  expect(down.noteSuccessfulNormalWrite(100, kStartNs),
         "ramp-down test establishes a nonzero baseline");
  expect(down.coupleToward(0, kStartNs + 25000000, &coupled) &&
             coupled == 95 &&
             down.transitionAllowed(coupled, kStartNs + 25000000) &&
             !down.transitionAllowed(94, kStartNs + 25000000),
         "the exact same integer envelope constrains regular ramp-down");

  wheeltec::BenchWireAccelerationEnvelope recovery;
  expect(recovery.noteSuccessfulNormalWrite(0, kStartNs) &&
             recovery.coupleToward(500, kStartNs + 60000000, &coupled) &&
             coupled == 12,
         "a three-command 60 ms recovery window permits exactly 12 raw units");
  const std::int16_t failed_candidate = coupled;
  expect(recovery.transitionAllowed(failed_candidate,
                                    kStartNs + 60000000) &&
             recovery.lastSuccessfulWireSpeed() == 0 &&
             recovery.lastSuccessfulCompletionNs() == kStartNs,
         "an attempted but uncompleted host write cannot change the successful-wire baseline");
  expect(recovery.coupleToward(0, kStartNs + 65000000, &coupled) &&
             coupled == 0,
         "generation after an injected failed write still uses the last completed wire state");
}

void testExactZeroOnlyWritesZeroAndRecordsBothDirections() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  addFeedback(&transport, 0, 30U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result = session.run(
      &transport, exactZeroConfig(), operations(&clock, &records));
  if (!result.completed()) {
    std::fprintf(stderr, "exact-zero debug status=%s zero=%d frames=%llu writes=%llu\n",
                 wheeltec::benchStatusName(result.status),
                 result.zero_host_write_completed ? 1 : 0,
                 static_cast<unsigned long long>(
                     result.statistics.valid_feedback_frames),
                 static_cast<unsigned long long>(
                     result.statistics.tx_host_writes_completed));
  }
  expect(result.completed() && result.zero_host_write_completed,
         "exact-zero characterization completes with a final zero host write");
  expect(!transport.writes.empty() &&
             std::all_of(transport.writes.begin(), transport.writes.end(),
                         isZeroCommand),
         "exact-zero mode cannot emit a nonzero command frame");
  const auto has_tx = std::find_if(records.begin(), records.end(),
                                   [](const std::string& line) {
                                     return line.find("\"record_type\":\"tx_result\"") !=
                                            std::string::npos;
                                   });
  const auto has_rx = std::find_if(records.begin(), records.end(),
                                   [](const std::string& line) {
                                     return line.find("\"record_type\":\"rx_result\"") !=
                                            std::string::npos;
                                   });
  expect(has_tx != records.end() && has_rx != records.end(),
         "bench evidence records TX and RX result/deadline information");
  const auto has_control_state = std::find_if(
      records.begin(), records.end(), [](const std::string& line) {
        return line.find("\"record_type\":\"feedback_frame\"") !=
                   std::string::npos &&
               line.find("\"composite_stop_flag_raw\":0") !=
                   std::string::npos &&
               line.find("\"control_allowed\":true") !=
                   std::string::npos &&
               line.find("\"control_inhibited\":false") !=
                   std::string::npos &&
               line.find("\"vcu_ack_available\":false") !=
                   std::string::npos &&
               line.find("\"specific_fault_available\":false") !=
                   std::string::npos &&
               line.find("\"command_echo_available\":false") !=
                   std::string::npos;
      });
  expect(has_control_state != records.end(),
         "bench NDJSON preserves the composite allowed state without treating it as ACK, fault detail, or command echo");
}

void testCompositeInhibitAndInvalidFlagFailClosed() {
  for (const std::uint8_t flag : {1U, 2U}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    const wheeltec::FeedbackFrame frame = feedback(0, 0, 0, flag);
    transport.reads.emplace_back(frame.begin(), frame.end());
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, exactZeroConfig(), operations(&clock, &records));
    expect(result.status == wheeltec::BenchStatus::kFeedbackInvalid &&
               result.authorization_revoked &&
               result.zero_host_write_completed &&
               !transport.writes.empty() &&
               isZeroCommand(transport.writes.back()),
           "bench characterization fails closed on composite inhibit or an invalid byte-1 domain value");
  }
}

void testStraightRampIsBoundedStraightAndRecoversThreeFreshSamples() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  addFeedback(&transport, 0, 80U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, rampConfig(), operations(&clock, &records));
  expect(result.completed() && result.zero_host_write_completed,
         "straight ramp completes and exits through bounded zero");
  expect(result.statistics.maximum_feedback_recovery_run >= 3U &&
             result.statistics.nonzero_frame_host_writes_completed > 0U,
         "motion waits for a three-fresh-feedback recovery run");
  std::vector<double> nonzero_speeds;
  for (const std::vector<std::uint8_t>& frame : transport.writes) {
    expect(frame.size() == wheeltec::kCommandFrameSize,
           "every bench TX is one complete candidate command frame");
    if (frame.size() != wheeltec::kCommandFrameSize) {
      continue;
    }
    const double speed =
        static_cast<double>(signedBigEndian(frame[3U], frame[4U])) / 1000.0;
    const double lateral =
        static_cast<double>(signedBigEndian(frame[5U], frame[6U])) / 1000.0;
    const double yaw =
        static_cast<double>(signedBigEndian(frame[7U], frame[8U])) / 1000.0;
    expect(speed >= 0.0 && speed <= wheeltec::kBenchMaximumForwardSpeedMps &&
               lateral == 0.0 && yaw == 0.0,
           "ramp TX is forward-only, straight, and under the 0.50 m/s cap");
    if (speed > 0.0) {
      nonzero_speeds.push_back(speed);
    }
  }
  expect(!nonzero_speeds.empty() &&
             nonzero_speeds.front() < wheeltec::kBenchMaximumForwardSpeedMps &&
             result.statistics.first_nonzero_speed_mps <
                 wheeltec::kBenchMaximumForwardSpeedMps,
         "the first completed nonzero host write is below 0.50 m/s");
  expect(!transport.writes.empty() && isZeroCommand(transport.writes.back()),
         "the final completed host write is exact zero");
  const std::size_t normal_write_count =
      transport.writes.empty() ? 0U : transport.writes.size() - 1U;
  expect(strictNormalWireEnvelopeHolds(transport, normal_write_count),
         "every regular up/hold/down/zero host write obeys the exact integer 0.20 m/s^2 wire envelope without tolerance");
  for (std::size_t index = 1U; index < normal_write_count; ++index) {
    expect(transport.write_receipts[index] -
                   transport.write_receipts[index - 1U] >=
               20000000,
           "regular host writes are no faster than 50 Hz");
  }
  const auto first_nonzero = std::find_if(
      transport.writes.begin(),
      transport.writes.begin() + static_cast<std::ptrdiff_t>(
                                     normal_write_count),
      [](const std::vector<std::uint8_t>& frame) {
        return commandWireSpeed(frame) > 0;
      });
  expect(first_nonzero !=
                 transport.writes.begin() + static_cast<std::ptrdiff_t>(
                                                normal_write_count) &&
             commandWireSpeed(*first_nonzero) == 12,
         "three 20 ms recovery commands produce a first completed wire speed of 12 raw units from the final pre-arm zero");
}

void testVariableWriteTimingStaysInsideWireEnvelope() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  transport.default_write_duration_ns = 3000000;
  for (std::size_t index = 0U; index < 100U; ++index) {
    transport.read_delays_ns.push_back(index % 2U == 0U ? 0 : 5000000);
  }
  addFeedback(&transport, 0, 100U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, rampConfig(), operations(&clock, &records));
  const std::size_t normal_write_count =
      transport.writes.empty() ? 0U : transport.writes.size() - 1U;
  bool saw_seventeen_ms_budget = false;
  bool saw_twenty_two_ms_budget = false;
  for (std::size_t index = 1U; index < normal_write_count; ++index) {
    const std::int64_t available_ns =
        transport.write_receipts[index] -
        transport.write_completions[index - 1U];
    saw_seventeen_ms_budget =
        saw_seventeen_ms_budget || available_ns == 17000000;
    saw_twenty_two_ms_budget =
        saw_twenty_two_ms_budget || available_ns == 22000000;
  }
  expect(result.completed() && result.zero_host_write_completed &&
             strictNormalWireEnvelopeHolds(transport, normal_write_count) &&
             saw_seventeen_ms_budget && saw_twenty_two_ms_budget,
         "injected 17-22 ms after-to-before jitter remains inside the strict wire envelope through ramp-up and ramp-down");
}

void testMaximumTargetCompletesFullStrictRamp() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  addFeedback(&transport, 0, 1000U);
  wheeltec::BenchConfig config = rampConfig();
  config.target_speed_mps = 0.50;
  config.hold_duration_ns = 100000000;
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, config, operations(&clock, &records));
  const std::size_t normal_write_count =
      transport.writes.empty() ? 0U : transport.writes.size() - 1U;
  std::int16_t maximum_wire = 0;
  bool saw_increase = false;
  bool saw_decrease = false;
  for (std::size_t index = 0U; index < normal_write_count; ++index) {
    const std::int16_t current = commandWireSpeed(transport.writes[index]);
    maximum_wire = std::max(maximum_wire, current);
    if (index > 0U) {
      const std::int16_t previous =
          commandWireSpeed(transport.writes[index - 1U]);
      saw_increase = saw_increase || current > previous;
      saw_decrease = saw_decrease || current < previous;
    }
  }
  expect(result.completed() && result.zero_host_write_completed &&
             normal_write_count > 0U && maximum_wire == 500 &&
             saw_increase && saw_decrease &&
             isZeroCommand(transport.writes[normal_write_count - 1U]) &&
             strictNormalWireEnvelopeHolds(transport, normal_write_count),
         "the 0.50 m/s boundary completes a full strict up/hold/down/regular-zero/post-zero run within the 15 s scripted session");
}

void testFailedNormalWriteDoesNotAdvanceWireBaseline() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  transport.fail_first_nonzero_write = true;
  addFeedback(&transport, 0, 100U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result = session.run(
      &transport, rampConfig(), operations(&clock, &records));
  const auto failed_nonzero = std::find_if(
      transport.writes.begin(), transport.writes.end(),
      [](const std::vector<std::uint8_t>& frame) {
        return commandWireSpeed(frame) > 0;
      });
  const std::size_t failed_index =
      failed_nonzero == transport.writes.end()
          ? transport.writes.size()
          : static_cast<std::size_t>(
                std::distance(transport.writes.begin(), failed_nonzero));
  expect(result.status == wheeltec::BenchStatus::kWriteFailed &&
             transport.first_nonzero_write_failed &&
             result.statistics.nonzero_frame_host_writes_completed == 0U &&
             failed_index < transport.write_statuses.size() &&
             transport.write_statuses[failed_index] ==
                 wheeltec::TransportStatus::kIoError &&
             result.zero_host_write_completed &&
             !transport.writes.empty() &&
             isZeroCommand(transport.writes.back()),
         "a failed regular nonzero host write never advances the completed-wire state and exits through the direct emergency-zero exception");
}

void testPartialAndThrowingNormalWritesPoisonWithoutAppendingZero() {
  for (const bool throw_unknown_progress : {false, true}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.immediate_normal_reads = true;
    transport.partial_first_nonzero_write = !throw_unknown_progress;
    transport.throw_first_nonzero_write = throw_unknown_progress;
    addFeedback(&transport, 0, 100U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, rampConfig(), operations(&clock, &records));
    const auto attempted_nonzero = std::find_if(
        transport.writes.begin(), transport.writes.end(),
        [](const std::vector<std::uint8_t>& frame) {
          return commandWireSpeed(frame) > 0;
        });
    const std::size_t attempted_index =
        attempted_nonzero == transport.writes.end()
            ? transport.writes.size()
            : static_cast<std::size_t>(std::distance(
                  transport.writes.begin(), attempted_nonzero));
    expect(result.status == wheeltec::BenchStatus::kWriteFailed &&
               result.authorization_revoked &&
               result.delivery_unconfirmed &&
               !result.zero_host_write_completed &&
               attempted_index < transport.writes.size() &&
               transport.writes.size() == attempted_index + 1U &&
               !isZeroCommand(transport.writes.back()) &&
               (throw_unknown_progress
                    ? transport.first_nonzero_write_threw
                    : transport.first_nonzero_write_was_partial),
           "a partial or throwing normal write poisons the generation and no zero frame is appended to the unknown prefix");
  }
}

void testPostMotionRecordThrowUsesNoRecordBoundedZero() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  addFeedback(&transport, 0, 100U);
  std::vector<std::string> records;
  wheeltec::BenchOperations ops = operations(&clock, &records);
  bool record_threw = false;
  ops.record_json_line = [&transport, &records,
                          &record_threw](const std::string& line) {
    const bool nonzero_already_written =
        std::any_of(transport.writes.begin(), transport.writes.end(),
                    [](const std::vector<std::uint8_t>& frame) {
                      return !isZeroCommand(frame);
                    });
    if (!record_threw && nonzero_already_written &&
        line.find("\"record_type\":\"tx_result\"") !=
            std::string::npos) {
      record_threw = true;
      throw std::bad_alloc();
    }
    records.push_back(line);
    return true;
  };
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, rampConfig(), ops);
  expect(record_threw &&
             result.status == wheeltec::BenchStatus::kRecordError &&
             result.authorization_revoked &&
             result.zero_host_write_completed &&
             !transport.writes.empty() &&
             isZeroCommand(transport.writes.back()),
         "a post-motion record exception is contained and followed by a complete no-record bounded zero");
}

void testPartialEmergencyZeroPoisonsAndIsNeverRetried() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  transport.partial_first_zero_after_nonzero = true;
  addFeedback(&transport, 0, 100U);
  std::vector<std::string> records;
  wheeltec::BenchOperations ops = operations(&clock, &records);
  bool evidence_failed = false;
  ops.record_json_line = [&transport, &records,
                          &evidence_failed](const std::string& line) {
    const bool nonzero_already_written =
        std::any_of(transport.writes.begin(), transport.writes.end(),
                    [](const std::vector<std::uint8_t>& frame) {
                      return !isZeroCommand(frame);
                    });
    if (!evidence_failed && nonzero_already_written &&
        line.find("\"record_type\":\"tx_result\"") !=
            std::string::npos) {
      evidence_failed = true;
      return false;
    }
    if (!evidence_failed) {
      records.push_back(line);
    }
    return !evidence_failed;
  };
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, rampConfig(), ops);
  const auto first_nonzero = std::find_if(
      transport.writes.begin(), transport.writes.end(),
      [](const std::vector<std::uint8_t>& frame) {
        return !isZeroCommand(frame);
      });
  const std::size_t nonzero_index =
      first_nonzero == transport.writes.end()
          ? transport.writes.size()
          : static_cast<std::size_t>(
                std::distance(transport.writes.begin(), first_nonzero));
  expect(evidence_failed && transport.first_zero_after_nonzero_was_partial &&
             result.status == wheeltec::BenchStatus::kRecordError &&
             result.authorization_revoked &&
             result.delivery_unconfirmed &&
             !result.zero_host_write_completed &&
             transport.writes.size() == nonzero_index + 2U &&
             isZeroCommand(transport.writes.back()),
         "a partial emergency-zero frame poisons the stream and is never retried or followed by another frame");
}

void testInvalidNegativeAndOverspeedFeedbackFailClosed() {
  for (const std::int16_t raw_speed : {-6, 501}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, 0, 4U);
    addFeedback(&transport, raw_speed, 1U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result =
        session.run(&transport, rampConfig(), operations(&clock, &records));
    if (result.status != wheeltec::BenchStatus::kFeedbackLimitViolation) {
      std::fprintf(stderr, "limit debug raw=%d status=%s frames=%llu\n",
                   static_cast<int>(raw_speed),
                   wheeltec::benchStatusName(result.status),
                   static_cast<unsigned long long>(
                       result.statistics.valid_feedback_frames));
    }
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.authorization_revoked,
           "negative or over-ceiling feedback revokes authorization");
    expect(!transport.writes.empty() && isZeroCommand(transport.writes.back()),
           "feedback limit failure exits with an exact-zero attempt");
  }

  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  wheeltec::FeedbackFrame bad = feedback(0);
  bad[22U] ^= 0x01U;
  transport.reads.emplace_back(bad.begin(), bad.end());
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult invalid = session.run(
      &transport, exactZeroConfig(), operations(&clock, &records));
  expect(invalid.status == wheeltec::BenchStatus::kFeedbackInvalid &&
             invalid.authorization_revoked,
         "checksum-invalid feedback fails closed");
}

void testMissingDisconnectAndClockRollbackFailClosed() {
  std::int64_t clock = 1000000000;
  ScriptedTransport missing(&clock);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  wheeltec::BenchResult result = session.run(
      &missing, exactZeroConfig(), operations(&clock, &records));
  expect(result.status == wheeltec::BenchStatus::kFeedbackMissing &&
             result.authorization_revoked,
         "missing feedback is a nonzero-exit fail-closed result");

  clock = 1000000000;
  ScriptedTransport disconnected(&clock);
  disconnected.disconnect_on_read = true;
  records.clear();
  result = session.run(&disconnected, exactZeroConfig(),
                       operations(&clock, &records));
  expect(result.status == wheeltec::BenchStatus::kDisconnected &&
             result.authorization_revoked && result.delivery_unconfirmed,
         "disconnect revokes authorization and marks delivery unconfirmed");

  clock = 1000000000;
  ScriptedTransport rollback(&clock);
  addFeedback(&rollback, 0, 8U);
  records.clear();
  std::size_t clock_calls = 0U;
  wheeltec::BenchOperations rollback_operations =
      operations(&clock, &records);
  rollback_operations.monotonic_now_ns = [&clock, &clock_calls]() {
    ++clock_calls;
    if (clock_calls == 6U) {
      clock -= 1;
    }
    return clock;
  };
  result = session.run(&rollback, exactZeroConfig(), rollback_operations);
  expect(result.status == wheeltec::BenchStatus::kClockInvalid &&
             result.authorization_revoked,
         "local monotonic rollback is never tolerated");
}

void testEvidenceRecordAndClockFailuresCannotBlockEmergencyZero() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, 0, 80U);
    std::vector<std::string> records;
    wheeltec::BenchOperations ops = operations(&clock, &records);
    bool evidence_failed = false;
    ops.record_json_line = [&transport, &records,
                            &evidence_failed](const std::string& line) {
      const bool nonzero_already_written =
          std::any_of(transport.writes.begin(), transport.writes.end(),
                      [](const std::vector<std::uint8_t>& frame) {
                        return !isZeroCommand(frame);
                      });
      if (!evidence_failed && nonzero_already_written &&
          line.find("\"record_type\":\"tx_result\"") !=
              std::string::npos) {
        evidence_failed = true;
        return false;
      }
      if (evidence_failed) {
        return false;
      }
      records.push_back(line);
      return true;
    };
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result =
        session.run(&transport, rampConfig(), ops);
    expect(result.status == wheeltec::BenchStatus::kRecordError &&
               result.zero_host_write_completed &&
               !transport.writes.empty() &&
               isZeroCommand(transport.writes.back()),
           "a post-motion evidence failure cannot block the direct bounded zero host write");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, 0, 80U);
    std::vector<std::string> records;
    wheeltec::BenchOperations ops = operations(&clock, &records);
    bool regressed_once = false;
    ops.monotonic_now_ns = [&clock, &transport, &regressed_once]() {
      const bool nonzero_already_written =
          std::any_of(transport.writes.begin(), transport.writes.end(),
                      [](const std::vector<std::uint8_t>& frame) {
                        return !isZeroCommand(frame);
                      });
      if (nonzero_already_written && !regressed_once) {
        regressed_once = true;
        return clock - 1;
      }
      return clock;
    };
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result =
        session.run(&transport, rampConfig(), ops);
    expect(result.status == wheeltec::BenchStatus::kClockInvalid &&
               result.zero_host_write_completed &&
               !transport.writes.empty() &&
               isZeroCommand(transport.writes.back()),
           "a sticky clock failure after motion cannot block the direct bounded zero host write");
  }
}

void testSchedulingGapBacklogAndRelativeOverspeedFailClosed() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.jump_on_normal_read = 16U;
    addFeedback(&transport, 0, 80U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, rampConfig(), operations(&clock, &records));
    const bool nonzero_written =
        std::any_of(transport.writes.begin(), transport.writes.end(),
                    [](const std::vector<std::uint8_t>& frame) {
                      return !isZeroCommand(frame);
                    });
    expect(result.status ==
                   wheeltec::BenchStatus::kSessionDeadlineExceeded &&
               nonzero_written && result.zero_host_write_completed &&
               isZeroCommand(transport.writes.back()),
           "a long scheduler gap after motion aborts without a catch-up speed jump");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.drain_queued_data = true;
    addFeedback(&transport, 0, 20U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, exactZeroConfig(), operations(&clock, &records));
    expect(result.status == wheeltec::BenchStatus::kFeedbackMissing &&
               result.statistics.valid_feedback_frames == 0U &&
               result.zero_host_write_completed,
           "pre-open/backlog bytes are drained and cannot satisfy fresh standstill recovery");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, 0, 9U);
    addFeedback(&transport, 100, 1U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, rampConfig(), operations(&clock, &records));
    expect(result.status ==
                   wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed &&
               isZeroCommand(transport.writes.back()),
           "feedback far above the current low-speed request triggers emergency zero even below 0.50 m/s");
  }
}

void testBoundedReadDeadlineOvershootUsesFeedbackFreshnessPolicy() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  transport.deadline_timeout_overshoot_ns = 200000;
  transport.forced_deadline_timeouts.push_back(true);
  addFeedback(&transport, 0, 40U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result = session.run(
      &transport, exactZeroConfig(), operations(&clock, &records));
  std::int64_t maximum_write_gap_ns = 0;
  for (std::size_t index = 1U; index < transport.write_receipts.size();
       ++index) {
    maximum_write_gap_ns = std::max(
        maximum_write_gap_ns,
        transport.write_receipts[index] -
            transport.write_receipts[index - 1U]);
  }
  expect(result.completed() && result.zero_host_write_completed &&
             transport.forced_deadline_timeout_count == 1U &&
             result.statistics.valid_feedback_frames >= 5U &&
             maximum_write_gap_ns < 100000000,
         "a 0.2 ms OS timeout overshoot inside the 100 ms command-gap bound is handled by the existing feedback-freshness policy");
}

wheeltec::BenchResult runRaisedWheelStartFeedbackSample(
    bool allow_asymmetry, std::int16_t forward, std::int16_t yaw,
    std::vector<std::string>* records,
    ScriptedTransport* transport, std::int64_t* clock,
    bool record_uncalibrated_motion_feedback = false,
    std::size_t pre_sample_feedback_count = 109U) {
  transport->immediate_normal_reads = true;
  addFeedback(transport, 0, pre_sample_feedback_count);
  const wheeltec::FeedbackFrame sample = feedback(forward, 0, yaw);
  transport->reads.emplace_back(sample.begin(), sample.end());
  addFeedback(transport, 0, 1000U);
  wheeltec::BenchConfig config = rampConfig();
  config.target_speed_mps = 0.50;
  config.allow_raised_wheel_start_asymmetry = allow_asymmetry;
  config.record_uncalibrated_motion_feedback =
      record_uncalibrated_motion_feedback;
  wheeltec::WheeltecBenchSession session;
  return session.run(transport, config, operations(clock, records));
}

void testUncalibratedMotionFeedbackRecordingIsExplicitAndStillBounded() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 354, 459, &records, &transport, &clock, false, 107U);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "default raised-wheel mode rejects measured tracking beyond the 0.020 m/s acceptance margin");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 354, 459, &records, &transport, &clock, true, 107U);
    const auto metadata = std::find_if(
        records.begin(), records.end(), [](const std::string& line) {
          return line.find("\"record_uncalibrated_motion_feedback\":true") !=
                     std::string::npos &&
                 line.find("\"tracking_deadband_asymmetry\":\"recorded_not_gated\"") !=
                     std::string::npos &&
                 line.find("\"acceptance_evidence\":false") !=
                     std::string::npos;
        });
    expect(result.completed() && result.zero_host_write_completed &&
               metadata != records.end(),
           "explicit calibration recording accepts measured 0.354 m/s and 0.459 rad/s feedback without claiming acceptance");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 466, -306, &records, &transport, &clock, true, 107U);
    const auto metadata = std::find_if(
        records.begin(), records.end(), [](const std::string& line) {
          return line.find("\"raw_calibration_feedback_emergency_ceiling_mps\":1") !=
                     std::string::npos &&
                 line.find("\"raw_calibration_feedback_emergency_ceiling_basis\":\"1.0_user_authorized_not_acceptance_limit\"") !=
                     std::string::npos;
        });
    expect(result.completed() && result.zero_host_write_completed &&
               metadata != records.end(),
           "raw calibration records observed 0.466 m/s and -0.306 rad/s feedback with a 0.515266 m/s derived wheel");
  }

  for (const std::pair<std::int16_t, bool> ceiling_sample :
       {std::make_pair<std::int16_t, bool>(1000, true),
        std::make_pair<std::int16_t, bool>(1001, false)}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, ceiling_sample.first, 0, &records, &transport, &clock,
        true, 107U);
    expect(result.completed() == ceiling_sample.second &&
               result.zero_host_write_completed,
           "raw calibration feedback emergency ceiling is inclusive at 1.000 m/s and rejects 1.001 m/s");
  }

  for (const std::pair<std::int16_t, std::int16_t> unsafe_sample :
       {std::make_pair<std::int16_t, std::int16_t>(-6, 0),
        std::make_pair<std::int16_t, std::int16_t>(840, -1000)}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, unsafe_sample.first, unsafe_sample.second,
        &records, &transport, &clock, true, 107U);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "calibration recording still rejects reverse or a derived wheel above 1.000 m/s");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 501, 0, &records, &transport, &clock, false, 107U);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "guarded raised-wheel mode retains the 0.500 m/s feedback ceiling");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.immediate_normal_reads = true;
    const wheeltec::FeedbackFrame cancellation = feedback(5, 0, -23);
    transport.reads.emplace_back(cancellation.begin(), cancellation.end());
    addFeedback(&transport, 0, 100U);
    std::vector<std::string> records;
    wheeltec::BenchConfig config = rampConfig();
    config.allow_raised_wheel_start_asymmetry = true;
    config.record_uncalibrated_motion_feedback = true;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, config, operations(&clock, &records));
    expect(result.status == wheeltec::BenchStatus::kStandstillViolation &&
               result.zero_host_write_completed,
           "calibration pre-arm rejects cancellation when one derived rear wheel exceeds 0.005 m/s despite bounded overall forward and yaw");
  }
}

void testRaisedWheelStartAsymmetryIsExplicitAndWheelBounded() {
  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        false, 37, 211, &records, &transport, &clock);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "default straight-ramp retains the strict yaw gate for the observed asymmetric wheel-start sample");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 373, -200, &records, &transport, &clock);
    const auto metadata = std::find_if(
        records.begin(), records.end(), [](const std::string& line) {
          return line.find("\"record_type\":\"metadata\"") !=
                     std::string::npos &&
                 line.find("\"allow_raised_wheel_start_asymmetry\":true") !=
                     std::string::npos &&
                 line.find("\"derived_rear_wheel_speed_tolerance_mps\":0.005") !=
                     std::string::npos &&
                 line.find("\"raised_wheel_start_tracking_overspeed_margin_mps\":0.02") !=
                     std::string::npos;
        });
    expect(result.completed() && result.zero_host_write_completed &&
               metadata != records.end(),
           "raised-wheel tracking evidence near request 0.393 accepts forward 0.373 m/s and yaw -0.200 rad/s while recording separate margins");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, 251, -1000, &records, &transport, &clock);
    expect(result.completed() && result.zero_host_write_completed,
           "a derived left rear wheel exactly 0.020 m/s above the last successful 0.392 m/s wire request is accepted");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, -5, 0, &records, &transport, &clock);
    expect(result.completed() && result.zero_host_write_completed,
           "raised-wheel motion accepts an overall and per-wheel one-count artifact at inclusive -0.005 m/s");
  }

  for (const std::pair<std::int16_t, std::int16_t> unsafe_sample :
       {std::make_pair<std::int16_t, std::int16_t>(-6, 0),
        std::make_pair<std::int16_t, std::int16_t>(2, 50),
        std::make_pair<std::int16_t, std::int16_t>(252, -1000),
        std::make_pair<std::int16_t, std::int16_t>(250, 1200),
        std::make_pair<std::int16_t, std::int16_t>(300, 1300)}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        true, unsafe_sample.first, unsafe_sample.second,
        &records, &transport, &clock);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "raised-wheel opt-in rejects reversal, the next wire unit above request plus 0.020, or an absolute over-ceiling rear wheel");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    std::vector<std::string> records;
    const wheeltec::BenchResult result = runRaisedWheelStartFeedbackSample(
        false, -2, 0, &records, &transport, &clock);
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "ordinary straight-ramp retains its authorized-motion -0.001 m/s reverse gate");
  }

  {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    transport.immediate_normal_reads = true;
    const wheeltec::FeedbackFrame sample = feedback(37, 0, 211);
    transport.reads.emplace_back(sample.begin(), sample.end());
    addFeedback(&transport, 0, 100U);
    std::vector<std::string> records;
    wheeltec::BenchConfig config = rampConfig();
    config.allow_raised_wheel_start_asymmetry = true;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, config, operations(&clock, &records));
    expect(result.status == wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "raised-wheel opt-in retains the strict yaw gate before motion authorization");
  }
}

void testRaisedWheelStartExtendedHoldIsOptInAndBounded() {
  wheeltec::BenchConfig config = rampConfig();
  config.hold_duration_ns = 60000000000LL;
  expect(!wheeltec::benchConfigIsValid(config),
         "ordinary straight-ramp retains the five-second hold ceiling");
  config.allow_raised_wheel_start_asymmetry = true;
  expect(wheeltec::benchConfigIsValid(config),
         "raised-wheel opt-in admits an exact sixty-second hold");
  config.record_uncalibrated_motion_feedback = true;
  expect(wheeltec::benchConfigIsValid(config),
         "uncalibrated feedback recording is valid with both raised straight-ramp opt-ins");
  config.allow_raised_wheel_start_asymmetry = false;
  expect(!wheeltec::benchConfigIsValid(config),
         "uncalibrated feedback recording is invalid without raised-wheel asymmetry opt-in");
  config.allow_raised_wheel_start_asymmetry = true;
  config.hold_duration_ns = 60000000001LL;
  expect(!wheeltec::benchConfigIsValid(config),
         "raised-wheel opt-in rejects a hold above sixty seconds");

  wheeltec::BenchConfig recording_without_raised = rampConfig();
  recording_without_raised.record_uncalibrated_motion_feedback = true;
  expect(!wheeltec::benchConfigIsValid(recording_without_raised),
         "uncalibrated feedback recording is invalid in ordinary straight-ramp even with a short hold");

  wheeltec::BenchConfig over_command_cap = rampConfig();
  over_command_cap.allow_raised_wheel_start_asymmetry = true;
  over_command_cap.record_uncalibrated_motion_feedback = true;
  over_command_cap.target_speed_mps = 0.501;
  expect(!wheeltec::benchConfigIsValid(over_command_cap),
         "raw calibration cannot raise the 0.500 m/s command and wire cap");

  wheeltec::BenchConfig exact_zero = exactZeroConfig();
  exact_zero.allow_raised_wheel_start_asymmetry = true;
  exact_zero.record_uncalibrated_motion_feedback = true;
  expect(!wheeltec::benchConfigIsValid(exact_zero),
         "raised-wheel asymmetry opt-in is invalid outside straight-ramp");

  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  transport.immediate_normal_reads = true;
  addFeedback(&transport, 0, 4000U);
  std::vector<std::string> records;
  config = rampConfig();
  config.allow_raised_wheel_start_asymmetry = true;
  config.record_uncalibrated_motion_feedback = true;
  config.target_speed_mps = 0.50;
  config.hold_duration_ns = 60000000000LL;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result = session.run(
      &transport, config, operations(&clock, &records));
  expect(result.completed() && result.zero_host_write_completed &&
             result.statistics.ended_monotonic_ns -
                     result.statistics.started_monotonic_ns <=
                 70000000000LL,
         "the 2.5 s up, 60 s hold, 2.5 s down raised-wheel run completes inside its hard seventy-second session");
}

void testBenchStandstillQuantizationThresholdIsInclusive() {
  for (const std::pair<std::int16_t, std::int16_t> accepted_sample :
       {std::make_pair<std::int16_t, std::int16_t>(5, 0),
        std::make_pair<std::int16_t, std::int16_t>(-5, 0),
        std::make_pair<std::int16_t, std::int16_t>(0, 23)}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    const wheeltec::FeedbackFrame frame = feedback(
        accepted_sample.first, 0, accepted_sample.second);
    for (std::size_t index = 0U; index < 30U; ++index) {
      transport.reads.emplace_back(frame.begin(), frame.end());
    }
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, exactZeroConfig(), operations(&clock, &records));
    const auto metadata = std::find_if(
        records.begin(), records.end(), [](const std::string& line) {
          return line.find("\"standstill_abs_forward_mps\":0.005") !=
                     std::string::npos &&
                 line.find("\"standstill_forward_threshold_basis\":\"one_type9_encoder_count_0.0037905_rounded_up_to_0.005_UNVERIFIED\"") !=
                     std::string::npos;
        });
    expect(result.completed() && result.zero_host_write_completed &&
               metadata != records.end(),
           "exact-zero accepts inclusive +/-0.005 m/s forward quantization and inclusive 0.023 rad/s yaw evidence");
  }

  for (const std::int16_t rejected_forward : {-6, 6}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    addFeedback(&transport, rejected_forward, 30U);
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, exactZeroConfig(), operations(&clock, &records));
    expect(!result.completed() && result.zero_host_write_completed,
           "exact-zero rejects forward standstill evidence at +/-0.006 m/s");
  }
}

void testStraightFeedbackSanityAndShaShape() {
  for (const std::pair<std::int16_t, std::int16_t> anomaly :
       {std::make_pair<std::int16_t, std::int16_t>(2, 0),
        std::make_pair<std::int16_t, std::int16_t>(0, 24)}) {
    std::int64_t clock = 1000000000;
    ScriptedTransport transport(&clock);
    const wheeltec::FeedbackFrame bad = feedback(0, anomaly.first,
                                                  anomaly.second);
    transport.reads.emplace_back(bad.begin(), bad.end());
    std::vector<std::string> records;
    wheeltec::WheeltecBenchSession session;
    const wheeltec::BenchResult result = session.run(
        &transport, exactZeroConfig(), operations(&clock, &records));
    expect(result.status ==
                   wheeltec::BenchStatus::kFeedbackLimitViolation &&
               result.zero_host_write_completed,
           "lateral or yaw feedback beyond the passive-evidence bench gate aborts");
  }

  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  std::vector<std::string> records;
  wheeltec::BenchConfig malformed = exactZeroConfig();
  malformed.passive_evidence_token = "passive-capture-sha256:not-a-sha";
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result = session.run(
      &transport, malformed, operations(&clock, &records));
  expect(result.status == wheeltec::BenchStatus::kEvidenceMissing &&
             transport.writes.empty() && transport.reads.empty(),
         "evidence gate accepts only an operator-attested 64 lowercase hex SHA shape before I/O");
}

void testRampDownAllowsBoundedPositiveFeedbackLag() {
  std::int64_t clock = 1000000000;
  ScriptedTransport transport(&clock);
  addFeedback(&transport, 0, 20U);
  addFeedback(&transport, 20, 3U);
  addFeedback(&transport, 0, 50U);
  std::vector<std::string> records;
  wheeltec::WheeltecBenchSession session;
  const wheeltec::BenchResult result =
      session.run(&transport, rampConfig(), operations(&clock, &records));
  expect(result.completed() && result.zero_host_write_completed &&
             !transport.writes.empty() &&
             isZeroCommand(transport.writes.back()),
         "bounded positive feedback lag during ramp-down does not cause a catch-up command and still requires final standstill");
}

void testPtyCarriesOnlyBoundedCandidateFrames() {
  const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  expect(master >= 0, "PTY master opens");
  if (master < 0) {
    return;
  }
  expect(::grantpt(master) == 0 && ::unlockpt(master) == 0,
         "PTY slave is granted and unlocked");
  char* slave_name = ::ptsname(master);
  expect(slave_name != nullptr, "PTY slave path is available");
  if (slave_name == nullptr) {
    ::close(master);
    return;
  }
  const int slave = ::open(slave_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  expect(slave >= 0, "PTY slave opens");
  if (slave < 0) {
    ::close(master);
    return;
  }
  expect(wheeltec::configureSerial115200EightNOne(slave),
         "PTY slave accepts the candidate serial settings");
  const wheeltec::FeedbackFrame frame = feedback(0);
  wheeltec::PosixFdTransport transport(slave, true, 1U, true);
  std::vector<std::string> records;
  wheeltec::BenchOperations ops;
  ops.monotonic_now_ns = []() { return wheeltec::monotonicNowNs(); };
  ops.wait_until_monotonic_ns = [](std::int64_t deadline) {
    while (wheeltec::monotonicNowNs() < deadline) {
      ::usleep(1000U);
    }
    return true;
  };
  ops.record_json_line = [&records](const std::string& line) {
    records.push_back(line);
    return true;
  };
  ops.stop_requested = []() { return false; };
  wheeltec::BenchConfig config = exactZeroConfig();
  config.exact_zero_duration_ns = 600000000;
  config.maximum_feedback_age_ns = 100000000;
  config.initial_feedback_deadline_ns = 100000000;
  wheeltec::WheeltecBenchSession session;
  const pid_t feedback_peer = ::fork();
  expect(feedback_peer >= 0, "PTY feedback peer process starts");
  if (feedback_peer == 0) {
    ::usleep(5000U);
    for (std::size_t index = 0U; index < 35U; ++index) {
      const ssize_t written = ::write(master, frame.data(), frame.size());
      if (written != static_cast<ssize_t>(frame.size())) {
        break;
      }
      ::usleep(20000U);
    }
    ::_exit(0);
  }
  const wheeltec::BenchResult result = session.run(&transport, config, ops);
  if (feedback_peer > 0) {
    int peer_status = 0;
    expect(::waitpid(feedback_peer, &peer_status, 0) == feedback_peer &&
               WIFEXITED(peer_status) && WEXITSTATUS(peer_status) == 0,
           "PTY feedback peer exits cleanly");
  }
  if (!result.completed()) {
    std::fprintf(stderr, "PTY debug status=%s zero=%d frames=%llu\n",
                 wheeltec::benchStatusName(result.status),
                 result.zero_host_write_completed ? 1 : 0,
                 static_cast<unsigned long long>(
                     result.statistics.valid_feedback_frames));
  }
  expect(result.completed(), "PTY exact-zero session completes");

  std::vector<std::uint8_t> outbound(4096U, 0U);
  const ssize_t count = ::read(master, outbound.data(), outbound.size());
  expect(count > 0 && count % static_cast<ssize_t>(wheeltec::kCommandFrameSize) == 0,
         "PTY peer observes whole candidate command frames");
  if (count > 0) {
    outbound.resize(static_cast<std::size_t>(count));
    for (std::size_t offset = 0U; offset < outbound.size();
         offset += wheeltec::kCommandFrameSize) {
      const std::vector<std::uint8_t> command(
          outbound.begin() + static_cast<std::ptrdiff_t>(offset),
          outbound.begin() + static_cast<std::ptrdiff_t>(
                                 offset + wheeltec::kCommandFrameSize));
      expect(isZeroCommand(command), "PTY exact-zero emits no motion bytes");
    }
  }
  ::close(master);
}

}  // namespace

int main() {
  runTest(testAllGatesAndEvidenceFailBeforeIo);
  runTest(testIntegerWireAccelerationEnvelopeBoundariesAndFailureState);
  runTest(testExactZeroOnlyWritesZeroAndRecordsBothDirections);
  runTest(testCompositeInhibitAndInvalidFlagFailClosed);
  runTest(testStraightRampIsBoundedStraightAndRecoversThreeFreshSamples);
  runTest(testVariableWriteTimingStaysInsideWireEnvelope);
  runTest(testMaximumTargetCompletesFullStrictRamp);
  runTest(testFailedNormalWriteDoesNotAdvanceWireBaseline);
  runTest(testPartialAndThrowingNormalWritesPoisonWithoutAppendingZero);
  runTest(testPostMotionRecordThrowUsesNoRecordBoundedZero);
  runTest(testPartialEmergencyZeroPoisonsAndIsNeverRetried);
  runTest(testInvalidNegativeAndOverspeedFeedbackFailClosed);
  runTest(testMissingDisconnectAndClockRollbackFailClosed);
  runTest(testEvidenceRecordAndClockFailuresCannotBlockEmergencyZero);
  runTest(testSchedulingGapBacklogAndRelativeOverspeedFailClosed);
  runTest(testBoundedReadDeadlineOvershootUsesFeedbackFreshnessPolicy);
  runTest(testRaisedWheelStartAsymmetryIsExplicitAndWheelBounded);
  runTest(testUncalibratedMotionFeedbackRecordingIsExplicitAndStillBounded);
  runTest(testRaisedWheelStartExtendedHoldIsOptInAndBounded);
  runTest(testBenchStandstillQuantizationThresholdIsInclusive);
  runTest(testStraightFeedbackSanityAndShaShape);
  runTest(testRampDownAllowsBoundedPositiveFeedbackLag);
  runTest(testPtyCarriesOnlyBoundedCandidateFrames);
  expect(g_test_cases_run == 23,
         "all twenty-three bench safety regression cases execute");
  if (g_failures != 0) {
    std::fprintf(stderr, "%d bench test(s) failed\n", g_failures);
    return 1;
  }
  std::fprintf(stderr, "Wheeltec bench characterization tests passed\n");
  return 0;
}
