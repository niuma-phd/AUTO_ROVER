#include "auto_rover_vcu_wheeltec_serial/feedback_capture.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

int g_failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
  }
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

wheeltec::FeedbackFrame makeFeedbackFrame(std::int16_t forward,
                                          std::int16_t lateral,
                                          std::int16_t yaw,
                                          std::uint8_t composite_stop_flag = 0U) {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = composite_stop_flag;
  putSignedBigEndian(forward, &frame[2U], &frame[3U]);
  putSignedBigEndian(lateral, &frame[4U], &frame[5U]);
  putSignedBigEndian(yaw, &frame[6U], &frame[7U]);
  putSignedBigEndian(1672, &frame[8U], &frame[9U]);
  putSignedBigEndian(-836, &frame[10U], &frame[11U]);
  putSignedBigEndian(0, &frame[12U], &frame[13U]);
  putSignedBigEndian(1000, &frame[14U], &frame[15U]);
  putSignedBigEndian(-500, &frame[16U], &frame[17U]);
  putSignedBigEndian(250, &frame[18U], &frame[19U]);
  putSignedBigEndian(24000, &frame[20U], &frame[21U]);
  frame[22U] = xorBytes(frame.data(), 22U);
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

std::string rawHex(const std::vector<std::uint8_t>& bytes) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const std::uint8_t byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

struct ReadStep {
  wheeltec::TransportStatus status{wheeltec::TransportStatus::kOk};
  std::vector<std::uint8_t> bytes;
  int os_error{0};
};

class InjectedReadTransport final : public wheeltec::ByteTransport {
 public:
  bool isConnected() const override { return connected; }
  std::uint64_t connectionGeneration() const override { return 1U; }

  wheeltec::IoResult writeAll(const std::uint8_t*, std::size_t,
                              std::int64_t) override {
    ++write_calls;
    return {wheeltec::TransportStatus::kIoError, 0U, EPERM, true};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t) override {
    ++read_calls;
    if (steps.empty()) {
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, ETIMEDOUT,
              false};
    }
    ReadStep step = std::move(steps.front());
    steps.pop_front();
    if (step.bytes.size() > capacity) {
      return {wheeltec::TransportStatus::kOk, capacity + 1U, 0, false};
    }
    std::copy(step.bytes.begin(), step.bytes.end(), data);
    if (step.status == wheeltec::TransportStatus::kDisconnected) {
      connected = false;
    }
    return {step.status, step.bytes.size(), step.os_error, false};
  }

  bool connected{true};
  std::size_t read_calls{0U};
  std::size_t write_calls{0U};
  std::deque<ReadStep> steps;
};

void testInjectedCaptureRecordsFramesAndStatistics() {
  const wheeltec::FeedbackFrame first = makeFeedbackFrame(125, 0, 25);
  const wheeltec::FeedbackFrame second = makeFeedbackFrame(250, 0, -50);
  wheeltec::FeedbackFrame bad_checksum = makeFeedbackFrame(300, 0, 60);
  bad_checksum[22U] ^= 0x01U;
  wheeltec::FeedbackFrame bad_tail = makeFeedbackFrame(400, 0, 80);
  bad_tail[23U] = 0U;

  InjectedReadTransport transport;
  ReadStep fragmented;
  fragmented.bytes = {0x00U, 0x55U};
  fragmented.bytes.insert(fragmented.bytes.end(), first.begin(),
                          first.begin() + 5);
  transport.steps.push_back(fragmented);
  ReadStep remainder;
  remainder.bytes.insert(remainder.bytes.end(), first.begin() + 5,
                         first.end());
  remainder.bytes.insert(remainder.bytes.end(), bad_checksum.begin(),
                         bad_checksum.end());
  remainder.bytes.insert(remainder.bytes.end(), bad_tail.begin(),
                         bad_tail.end());
  transport.steps.push_back(remainder);
  ReadStep valid_second;
  valid_second.bytes.insert(valid_second.bytes.end(), second.begin(),
                            second.end());
  transport.steps.push_back(valid_second);

  std::int64_t clock_ns = 100;
  std::vector<std::string> records;
  wheeltec::FeedbackCaptureOperations operations;
  operations.monotonic_now_ns = [&clock_ns]() {
    const std::int64_t value = clock_ns;
    clock_ns += 10;
    return value;
  };
  operations.record_json_line = [&records](const std::string& line) {
    records.push_back(line);
    return true;
  };
  wheeltec::FeedbackCaptureConfig config;
  config.duration_ns = 100;
  config.read_timeout_ns = 20;
  config.read_buffer_bytes = 256U;

  wheeltec::FeedbackCaptureSession session;
  const wheeltec::FeedbackCaptureResult result =
      session.run(&transport, config, operations);
  expect(result.completed(), "injected capture reaches its bounded duration");
  expect(transport.write_calls == 0U,
         "feedback capture never invokes ByteTransport::writeAll");
  expect(result.statistics.raw_bytes ==
             fragmented.bytes.size() + remainder.bytes.size() +
                 valid_second.bytes.size(),
         "capture counts every raw byte received from the transport");
  expect(result.statistics.valid_frames == 2U,
         "capture counts only complete checksum-valid frames");
  expect(result.statistics.checksum_failures >= 1U,
         "capture exposes parser checksum failures");
  expect(result.statistics.framing_failures >= 1U,
         "capture exposes parser framing failures");
  expect(result.statistics.discarded_bytes >= 2U,
         "capture exposes discarded noise and corrupt candidates");
  expect(result.statistics.read_timeouts >= 1U,
         "capture counts idle read deadlines");
  expect(result.statistics.capture_average_rate_hz > 0.0 &&
             result.statistics.interframe_rate_hz > 0.0,
         "capture reports bounded-run and inter-frame rates");
  expect(records.size() == 6U,
         "three raw chunks, two frame records, and one terminal summary are emitted");
  if (records.size() >= 6U) {
    expect(records[0U].find("\"record_type\":\"raw_chunk\"") !=
                   std::string::npos &&
               records[0U].find("\"raw_hex\":\"00557b") !=
                   std::string::npos &&
               records[1U].find("\"raw_hex\":\"" + rawHex(remainder.bytes) +
                                   "\"") != std::string::npos,
           "raw-chunk NDJSON preserves noise, fragmentation, and corrupt frames exactly");
    expect(records[2U].find("\"record_type\":\"frame\"") !=
                   std::string::npos &&
               records[2U].find("\"receipt_clock\":\"CLOCK_MONOTONIC\"") !=
                   std::string::npos &&
               records[2U].find("\"raw_hex\":\"7b00007d") !=
                   std::string::npos &&
               records[2U].find("\"forward_speed_mps\":0.125") !=
                   std::string::npos &&
               records[2U].find("\"composite_stop_flag_raw\":0") !=
                   std::string::npos &&
               records[2U].find("\"control_allowed\":true") !=
                   std::string::npos &&
               records[2U].find("\"control_inhibited\":false") !=
                   std::string::npos &&
               records[2U].find("\"vcu_ack_available\":false") !=
                   std::string::npos &&
               records[2U].find("\"specific_fault_available\":false") !=
                   std::string::npos &&
               records[2U].find("\"command_echo_available\":false") !=
                   std::string::npos,
           "frame NDJSON includes exact bytes and preserves the composite state without claiming ACK, specific fault, or command echo");
    expect(records[3U].find("\"record_type\":\"raw_chunk\"") !=
                   std::string::npos &&
               records[3U].find("\"raw_hex\":\"" +
                                   rawHex(valid_second.bytes) + "\"") !=
                   std::string::npos,
           "a later successful read is recorded before its parsed frame");
    expect(records[4U].find("\"yaw_rate_radps\":-0.050000") !=
               std::string::npos,
           "second frame NDJSON preserves a negative parsed yaw rate");
    expect(records[5U].find("\"record_type\":\"summary\"") !=
                   std::string::npos &&
               records[5U].find("\"valid_frames\":2") !=
                   std::string::npos &&
               records[5U].find("\"read_timeouts\":") !=
                   std::string::npos,
           "summary NDJSON contains machine-readable counts");
  }
}

void testCapturePreservesCompositeInhibitAndRejectsInvalidFlag() {
  const wheeltec::FeedbackFrame inhibited =
      makeFeedbackFrame(0, 0, 0, 1U);
  const wheeltec::FeedbackFrame invalid =
      makeFeedbackFrame(0, 0, 0, 2U);
  InjectedReadTransport transport;
  transport.steps.push_back(
      ReadStep{wheeltec::TransportStatus::kOk,
               std::vector<std::uint8_t>(inhibited.begin(), inhibited.end()),
               0});
  transport.steps.push_back(
      ReadStep{wheeltec::TransportStatus::kOk,
               std::vector<std::uint8_t>(invalid.begin(), invalid.end()), 0});

  std::int64_t clock_ns = 100;
  std::vector<std::string> records;
  wheeltec::FeedbackCaptureOperations operations;
  operations.monotonic_now_ns = [&clock_ns]() {
    const std::int64_t value = clock_ns;
    clock_ns += 10;
    return value;
  };
  operations.record_json_line = [&records](const std::string& line) {
    records.push_back(line);
    return true;
  };
  wheeltec::FeedbackCaptureConfig config;
  config.duration_ns = 80;
  config.read_timeout_ns = 20;
  wheeltec::FeedbackCaptureSession session;
  const wheeltec::FeedbackCaptureResult result =
      session.run(&transport, config, operations);

  const auto inhibit_record = std::find_if(
      records.begin(), records.end(), [](const std::string& line) {
        return line.find("\"record_type\":\"frame\"") != std::string::npos &&
               line.find("\"composite_stop_flag_raw\":1") !=
                   std::string::npos &&
               line.find("\"control_allowed\":false") !=
                   std::string::npos &&
               line.find("\"control_inhibited\":true") !=
                   std::string::npos;
      });
  expect(result.completed() && result.statistics.valid_frames == 1U &&
             result.statistics.framing_failures >= 1U &&
             inhibit_record != records.end(),
         "capture records a valid composite inhibit and excludes byte-1 values above one from parsed frames");
}

void testCaptureFailsClosedWithoutWriting() {
  InjectedReadTransport transport;
  std::size_t sink_calls = 0U;
  wheeltec::FeedbackCaptureOperations operations;
  operations.monotonic_now_ns = []() { return 100; };
  operations.record_json_line = [&sink_calls](const std::string&) {
    ++sink_calls;
    return true;
  };
  wheeltec::FeedbackCaptureConfig invalid;
  invalid.duration_ns = 0;
  wheeltec::FeedbackCaptureSession session;
  const wheeltec::FeedbackCaptureResult result =
      session.run(&transport, invalid, operations);
  expect(result.status == wheeltec::FeedbackCaptureStatus::kInvalidArgument &&
             transport.read_calls == 0U && transport.write_calls == 0U &&
             sink_calls == 0U,
         "invalid capture configuration touches neither transport nor sink");

  const wheeltec::FeedbackFrame frame = makeFeedbackFrame(100, 0, 0);
  transport.steps.push_back(
      ReadStep{wheeltec::TransportStatus::kOk,
               std::vector<std::uint8_t>(frame.begin(), frame.end()), 0});
  std::int64_t clock_ns = 100;
  operations.monotonic_now_ns = [&clock_ns]() {
    const std::int64_t value = clock_ns;
    clock_ns += 10;
    return value;
  };
  operations.record_json_line = [](const std::string&) { return false; };
  wheeltec::FeedbackCaptureConfig valid;
  valid.duration_ns = 100;
  const wheeltec::FeedbackCaptureResult sink_failed =
      session.run(&transport, valid, operations);
  expect(sink_failed.status == wheeltec::FeedbackCaptureStatus::kRecordError &&
             transport.write_calls == 0U,
         "record failure aborts capture without attempting a serial write");
}

struct PtyPair {
  int master{-1};
  int slave{-1};

  ~PtyPair() {
    if (master >= 0) {
      ::close(master);
    }
    if (slave >= 0) {
      ::close(slave);
    }
  }
};

bool openPtyPair(PtyPair* pair) {
  if (pair == nullptr) {
    return false;
  }
  pair->master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (pair->master < 0 || ::grantpt(pair->master) != 0 ||
      ::unlockpt(pair->master) != 0) {
    return false;
  }
  char* const slave_name = ::ptsname(pair->master);
  if (slave_name == nullptr) {
    return false;
  }
  pair->slave = ::open(slave_name, O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (pair->slave < 0) {
    return false;
  }
  termios settings{};
  if (::tcgetattr(pair->slave, &settings) != 0) {
    return false;
  }
  ::cfmakeraw(&settings);
  return ::tcsetattr(pair->slave, TCSANOW, &settings) == 0;
}

void testPtyCaptureIsReceiveOnly() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for feedback-only capture");
  if (pair.master < 0 || pair.slave < 0) {
    return;
  }
  wheeltec::PosixFdTransport transport(pair.slave, false, 9U, false);
  const std::uint8_t prohibited_byte = 0U;
  const wheeltec::IoResult prohibited_write = transport.writeAll(
      &prohibited_byte, 1U, wheeltec::monotonicNowNs() + 1000000);
  expect(prohibited_write.status == wheeltec::TransportStatus::kDisabled &&
             prohibited_write.transferred == 0U,
         "receive-only PTY transport rejects writeAll before touching the fd");
  const wheeltec::FeedbackFrame frame = makeFeedbackFrame(500, 0, 100);
  expect(::write(pair.master, frame.data(), frame.size()) ==
             static_cast<ssize_t>(frame.size()),
         "PTY peer queues one complete inbound feedback frame");

  std::vector<std::string> records;
  wheeltec::FeedbackCaptureOperations operations;
  operations.monotonic_now_ns = []() { return wheeltec::monotonicNowNs(); };
  operations.record_json_line = [&records](const std::string& line) {
    records.push_back(line);
    return true;
  };
  wheeltec::FeedbackCaptureConfig config;
  config.duration_ns = 10000000;
  config.read_timeout_ns = 2000000;
  wheeltec::FeedbackCaptureSession session;
  const wheeltec::FeedbackCaptureResult result =
      session.run(&transport, config, operations);
  expect(result.completed() && result.statistics.valid_frames == 1U,
         "PTY capture receives and parses the inbound frame");
  expect(result.statistics.read_timeouts >= 1U,
         "PTY capture observes a bounded idle timeout after the frame");
  std::uint8_t unexpected_outbound[wheeltec::kCommandFrameSize]{};
  errno = 0;
  const ssize_t outbound_count =
      ::read(pair.master, unexpected_outbound, sizeof(unexpected_outbound));
  expect(outbound_count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK),
         "PTY peer receives no outbound byte from feedback capture");
}

}  // namespace

int main() {
  testInjectedCaptureRecordsFramesAndStatistics();
  testCapturePreservesCompositeInhibitAndRejectsInvalidFlag();
  testCaptureFailsClosedWithoutWriting();
  testPtyCaptureIsReceiveOnly();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d feedback-capture test(s) failed\n", g_failures);
    return 1;
  }
  std::puts("wheeltec feedback-capture tests passed");
  return 0;
}
