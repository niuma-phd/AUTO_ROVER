#include "auto_rover_vcu_wheeltec_serial/adapter.hpp"
#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <limits>
#include <map>
#include <new>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {
std::size_t g_cpp_allocation_count = 0U;
}

void* operator new(std::size_t size) {
  ++g_cpp_allocation_count;
  void* const allocation = std::malloc(size == 0U ? 1U : size);
  if (allocation == nullptr) {
    throw std::bad_alloc();
  }
  return allocation;
}

void* operator new[](std::size_t size) {
  ++g_cpp_allocation_count;
  void* const allocation = std::malloc(size == 0U ? 1U : size);
  if (allocation == nullptr) {
    throw std::bad_alloc();
  }
  return allocation;
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  try {
    return ::operator new[](size);
  } catch (...) {
    return nullptr;
  }
}

void operator delete(void* allocation) noexcept { std::free(allocation); }
void operator delete[](void* allocation) noexcept { std::free(allocation); }
void operator delete(void* allocation, std::size_t) noexcept {
  std::free(allocation);
}
void operator delete[](void* allocation, std::size_t) noexcept {
  std::free(allocation);
}
void operator delete(void* allocation, const std::nothrow_t&) noexcept {
  ::operator delete(allocation);
}
void operator delete[](void* allocation, const std::nothrow_t&) noexcept {
  ::operator delete[](allocation);
}

namespace {

int g_failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
         message);
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
                                           bool valid_checksum = true,
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
  if (!valid_checksum) {
    frame[22U] = static_cast<std::uint8_t>(frame[22U] ^ 0x01U);
  }
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

auto_rover::VehicleExecutionCommand makeCommand(
    std::uint64_t sequence, std::int64_t created_ns,
    std::int64_t deadline_ns, double speed_mps = 0.25,
    double curvature_inv_m = 0.5) {
  auto_rover::VehicleExecutionCommand command;
  command.sequence_id = sequence;
  command.created_monotonic_ns = created_ns;
  command.deadline_monotonic_ns = deadline_ns;
  command.signed_speed_mps = speed_mps;
  command.curvature_inv_m = curvature_inv_m;
  command.motion_enabled = true;
  command.hold = false;
  command.stop_reason = auto_rover::StopReason::kNone;
  return command;
}

wheeltec::CodecLimits commissioningCodecLimits() {
  wheeltec::CodecLimits limits;
  limits.max_forward_speed_mps = 0.50;
  return limits;
}

void testCommandCodec() {
  const wheeltec::CodecLimits limits = commissioningCodecLimits();
  const auto_rover::VehicleExecutionCommand command =
      makeCommand(1U, 10, 100, 0.1239, 0.5);
  const wheeltec::EncodeResult encoded =
      wheeltec::encodeExecutionCommand(command, limits);
  expect(encoded.ok(), "valid command encodes");
  expect(encoded.frame.size() == 11U, "command frame has fixed width");
  expect(encoded.frame[0U] == 0x7BU && encoded.frame[10U] == 0x7DU,
         "command frame has fixed sentinels");
  expect(encoded.frame[1U] == 0U && encoded.frame[2U] == 0U,
         "unsupported lateral field remains zero");
  expect(encoded.frame[3U] == 0x00U && encoded.frame[4U] == 0x7BU,
         "positive speed is big-endian and quantized toward zero");
  expect(encoded.frame[5U] == 0U && encoded.frame[6U] == 0U,
         "second planar component remains zero");
  expect(encoded.frame[7U] == 0x00U && encoded.frame[8U] == 0x3DU,
         "yaw rate uses speed times curvature and truncation");
  expect(encoded.frame[9U] == xorBytes(encoded.frame.data(), 9U),
         "command checksum covers bytes zero through eight");

  const wheeltec::EncodeResult right_turn = wheeltec::encodeWireMotion(
      wheeltec::WireMotionCandidate{0.25, 0.0, -0.2509}, limits);
  expect(right_turn.ok(), "negative yaw rate encodes");
  expect(right_turn.frame[7U] == 0xFFU && right_turn.frame[8U] == 0x06U,
         "negative yaw uses signed big-endian int16 and truncates toward zero");

  const wheeltec::EncodeResult stopped = wheeltec::encodeWireMotion(
      wheeltec::WireMotionCandidate{0.0, 0.0, -0.0}, limits);
  expect(stopped.ok(), "zero speed command encodes");
  expect(stopped.frame[7U] == 0U && stopped.frame[8U] == 0U,
         "zero speed forces an exact zero yaw field");

  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.1, 0.001, 0.0}, limits)
             .error == wheeltec::CodecError::kLateralMotionUnsupported,
         "lateral motion is rejected");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{-0.1, 0.0, 0.0}, limits)
             .error == wheeltec::CodecError::kNegativeForwardSpeed,
         "reverse motion is rejected");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.501, 0.0, 0.0}, limits)
             .error == wheeltec::CodecError::kForwardSpeedLimit,
         "the explicitly configured 0.50 m/s commissioning limit is enforced");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{
                 std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
             limits)
             .error == wheeltec::CodecError::kNonFinite,
         "NaN is rejected");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{
                 0.1, 0.0, std::numeric_limits<double>::infinity()},
             limits)
             .error == wheeltec::CodecError::kNonFinite,
         "infinity is rejected");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.1, 0.0, 0.2}, limits)
             .error == wheeltec::CodecError::kCurvatureLimit,
         "derived curvature limit is enforced");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.0, 0.0, 0.001}, limits)
             .error == wheeltec::CodecError::kZeroSpeedYawRate,
         "nonzero yaw at zero speed is rejected");

  wheeltec::CodecLimits maximum_configurable_limits = limits;
  maximum_configurable_limits.max_forward_speed_mps = 5.999;
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{5.999, 0.0, 0.0},
             maximum_configurable_limits)
             .ok(),
         "a finite configured speed below the exclusive 6 m/s bound is "
         "accepted");

  expect(!wheeltec::codecLimitsAreValid(wheeltec::CodecLimits{}),
         "an omitted zero speed limit fails closed");
  expect(wheeltec::encodeWireMotion(wheeltec::WireMotionCandidate{},
                                    wheeltec::CodecLimits{})
             .ok(),
         "an omitted motion limit still permits the exact-zero emergency "
         "frame");
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.001, 0.0, 0.0},
             wheeltec::CodecLimits{})
             .error == wheeltec::CodecError::kInvalidLimits,
         "an omitted motion limit rejects every representable nonzero speed");
  wheeltec::CodecLimits invalid_speed_limit = limits;
  invalid_speed_limit.max_forward_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  expect(!wheeltec::codecLimitsAreValid(invalid_speed_limit),
         "a NaN configured speed limit is rejected");
  invalid_speed_limit.max_forward_speed_mps =
      std::numeric_limits<double>::infinity();
  expect(!wheeltec::codecLimitsAreValid(invalid_speed_limit),
         "an infinite configured speed limit is rejected");
  invalid_speed_limit.max_forward_speed_mps = 6.0;
  expect(!wheeltec::codecLimitsAreValid(invalid_speed_limit),
         "the 6 m/s configurable bound is exclusive");
  invalid_speed_limit.max_forward_speed_mps = 6.001;
  expect(!wheeltec::codecLimitsAreValid(invalid_speed_limit),
         "a configured speed above 6 m/s is rejected");
  expect(wheeltec::codecLimitsAreValid(maximum_configurable_limits),
         "5.999 m/s is a valid explicit codec speed limit");

  wheeltec::CodecLimits excessive_curvature_limit = limits;
  excessive_curvature_limit.max_abs_curvature_inv_m = 1.1;
  expect(!wheeltec::codecLimitsAreValid(excessive_curvature_limit),
         "adapter curvature is hard-capped at the 0.95 m radius");

  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{0.0009, 0.0, 0.0}, limits)
             .error == wheeltec::CodecError::kWireRange,
         "a positive speed that cannot be represented on the wire is rejected "
         "independently");

  const double adversarial_speed = 0.0019;
  const double approved_curvature =
      wheeltec::kPhase1MaximumAbsCurvatureInvM;
  expect(wheeltec::encodeWireMotion(
             wheeltec::WireMotionCandidate{
                 adversarial_speed, 0.0,
                 adversarial_speed * approved_curvature},
             limits)
             .error == wheeltec::CodecError::kCurvatureLimit,
         "post-quantization curvature cannot violate the 0.95 m radius");

  auto_rover::VehicleExecutionCommand over_curvature =
      makeCommand(2U, 10, 100, 0.2, 2.0);
  expect(wheeltec::encodeExecutionCommand(over_curvature, limits).error ==
             wheeltec::CodecError::kCurvatureLimit,
         "execution-command curvature is checked before mapping");
  auto_rover::VehicleExecutionCommand inhibited =
      makeCommand(3U, 10, 100, 0.2, 0.0);
  inhibited.motion_enabled = false;
  inhibited.hold = true;
  inhibited.stop_reason = auto_rover::StopReason::kDisarmed;
  expect(wheeltec::encodeExecutionCommand(inhibited, limits).error ==
             wheeltec::CodecError::kInhibitedMotion,
         "inhibited nonzero execution command is rejected");
  inhibited.signed_speed_mps = 0.0;
  inhibited.curvature_inv_m = 1.0;
  const wheeltec::EncodeResult inhibited_zero =
      wheeltec::encodeExecutionCommand(inhibited, limits);
  expect(inhibited_zero.ok() && inhibited_zero.frame[7U] == 0U &&
             inhibited_zero.frame[8U] == 0U,
         "inhibited zero command maps to an exact zero frame");
}

void testFeedbackCodec() {
  const wheeltec::FeedbackCandidate default_candidate;
  expect(default_candidate.composite_stop_flag_raw == 1U &&
             !default_candidate.control_allowed &&
             default_candidate.control_inhibited,
         "an unpopulated feedback candidate defaults to the fail-closed composite inhibit state");

  const wheeltec::FeedbackFrame frame = makeFeedbackFrame(-1234, 500, -250);
  const wheeltec::DecodeResult result =
      wheeltec::decodeFeedbackFrame(frame.data(), frame.size());
  expect(result.ok(), "complete feedback frame decodes");
  expectNear(result.candidate.forward_speed_mps, -1.234, 1e-12,
             "feedback forward speed is signed and scaled");
  expectNear(result.candidate.lateral_speed_mps, 0.5, 1e-12,
             "feedback lateral speed is signed and scaled");
  expectNear(result.candidate.yaw_rate_radps, -0.25, 1e-12,
             "feedback yaw rate is signed and scaled");
  expectNear(result.candidate.linear_acceleration_mps2[0U], 1672.0 / 1671.84,
             1e-12, "feedback acceleration uses observed scale");
  expectNear(result.candidate.linear_acceleration_mps2[1U], -836.0 / 1671.84,
             1e-12, "negative acceleration remains signed");
  expectNear(result.candidate.angular_velocity_radps[0U], 0.26644, 1e-12,
             "feedback gyro uses observed scale");
  expectNear(result.candidate.supply_voltage_v, 24.0, 1e-12,
             "feedback voltage uses observed scale");
  expect(!result.candidate.source_time_available &&
             result.candidate.source_time_ns == 0,
         "feedback reports that source time is unavailable");
  expect(result.candidate.composite_stop_flag_raw == 0U &&
             result.candidate.control_allowed &&
             !result.candidate.control_inhibited,
         "feedback byte 1 zero decodes as the current-cycle composite control-allowed state");

  const wheeltec::FeedbackFrame inhibited_frame =
      makeFeedbackFrame(0, 0, 0, true, 1U);
  const wheeltec::DecodeResult inhibited = wheeltec::decodeFeedbackFrame(
      inhibited_frame.data(), inhibited_frame.size());
  expect(inhibited.ok() && inhibited.candidate.composite_stop_flag_raw == 1U &&
             !inhibited.candidate.control_allowed &&
             inhibited.candidate.control_inhibited,
         "feedback byte 1 one preserves the composite control-inhibit state");

  const wheeltec::FeedbackFrame invalid_flag =
      makeFeedbackFrame(0, 0, 0, true, 2U);
  const wheeltec::DecodeResult invalid_flag_result =
      wheeltec::decodeFeedbackFrame(invalid_flag.data(), invalid_flag.size());
  expect(invalid_flag_result.error ==
                 wheeltec::CodecError::kInvalidCompositeStopFlag &&
             invalid_flag_result.candidate.composite_stop_flag_raw == 1U &&
             !invalid_flag_result.candidate.control_allowed &&
             invalid_flag_result.candidate.control_inhibited,
         "feedback byte 1 values above one are rejected and leave a fail-closed candidate");

  wheeltec::FeedbackFrame corrupt = frame;
  corrupt[0U] = 0U;
  expect(wheeltec::decodeFeedbackFrame(corrupt.data(), corrupt.size()).error ==
             wheeltec::CodecError::kInvalidHeader,
         "feedback header is checked");
  corrupt = frame;
  corrupt[23U] = 0U;
  expect(wheeltec::decodeFeedbackFrame(corrupt.data(), corrupt.size()).error ==
             wheeltec::CodecError::kInvalidTail,
         "feedback tail is checked");
  corrupt = frame;
  corrupt[22U] ^= 0x80U;
  expect(wheeltec::decodeFeedbackFrame(corrupt.data(), corrupt.size()).error ==
             wheeltec::CodecError::kChecksumMismatch,
         "feedback checksum is checked");
  expect(wheeltec::decodeFeedbackFrame(frame.data(), frame.size() - 1U).error ==
             wheeltec::CodecError::kInvalidFrameSize,
         "partial feedback frame is rejected");
}

void testRollingParser() {
  wheeltec::FeedbackStreamParser parser;
  const wheeltec::FeedbackFrame first = makeFeedbackFrame(100, 0, 10);
  const wheeltec::FeedbackFrame second = makeFeedbackFrame(200, 0, -20);

  std::vector<wheeltec::FeedbackCandidate> output =
      parser.consume(first.data(), 7U);
  expect(output.empty() && parser.bufferedBytes() == 7U,
         "parser retains a fragmented frame");
  output = parser.consume(first.data() + 7U, first.size() - 7U);
  expect(output.size() == 1U, "parser completes a fragmented frame");
  expectNear(output[0U].forward_speed_mps, 0.1, 1e-12,
             "fragmented frame value is preserved");

  std::vector<std::uint8_t> noisy{0x00U, 0x55U, 0x7DU, 0x01U};
  noisy.insert(noisy.end(), first.begin(), first.end());
  noisy.insert(noisy.end(), second.begin(), second.end());
  output = parser.consume(noisy.data(), noisy.size());
  expect(output.size() == 2U, "parser skips noise and emits adjacent frames");

  wheeltec::FeedbackFrame bad = makeFeedbackFrame(300, 0, 30, false);
  std::vector<std::uint8_t> corrupt_then_valid(bad.begin(), bad.end());
  corrupt_then_valid.insert(corrupt_then_valid.end(), second.begin(),
                            second.end());
  output = parser.consume(corrupt_then_valid.data(), corrupt_then_valid.size());
  expect(output.size() == 1U, "parser resynchronizes after a bad checksum");
  expectNear(output[0U].forward_speed_mps, 0.2, 1e-12,
             "resynchronized frame is the next valid frame");

  std::vector<std::uint8_t> false_header{0x7BU, 0x01U, 0x02U, 0x03U, 0x04U};
  false_header.insert(false_header.end(), first.begin(), first.end());
  output = parser.consume(false_header.data(), false_header.size());
  expect(output.size() == 1U,
         "parser finds a valid header inside a corrupt candidate window");
  expect(parser.statistics().checksum_failures >= 1U &&
             parser.statistics().discarded_bytes >= 1U,
         "parser records corruption and discarded noise");

  wheeltec::FeedbackFrame invalid_flag =
      makeFeedbackFrame(300, 0, 0, true, 2U);
  std::vector<std::uint8_t> invalid_flag_then_valid(invalid_flag.begin(),
                                                    invalid_flag.end());
  invalid_flag_then_valid.insert(invalid_flag_then_valid.end(), second.begin(),
                                 second.end());
  const std::uint64_t framing_before =
      parser.statistics().framing_failures;
  output = parser.consume(invalid_flag_then_valid.data(),
                          invalid_flag_then_valid.size());
  expect(output.size() == 1U && output[0U].control_allowed &&
             parser.statistics().framing_failures > framing_before,
         "parser rejects an out-of-domain composite stop flag and resynchronizes at the next valid feedback frame");

  std::vector<std::uint8_t> long_noise(4096U, 0x22U);
  output = parser.consume(long_noise.data(), long_noise.size());
  expect(output.empty() && parser.bufferedBytes() < wheeltec::kFeedbackFrameSize,
         "parser bounds retained noise");
  parser.reset();
  expect(parser.bufferedBytes() == 0U &&
             parser.statistics().valid_frames == 0U,
         "parser reset clears state and counters");
  expect(parser.consume(nullptr, 0U).empty(),
         "empty parser input is accepted without dereference");
}

void testWriteAllOperations() {
  std::int64_t now_ns = 100;
  int write_call = 0;
  int wait_call = 0;
  wheeltec::WriteOperations operations;
  operations.monotonic_now_ns = [&now_ns]() { return now_ns; };
  operations.write_bytes =
      [&write_call](int, const std::uint8_t*, std::size_t count) -> ssize_t {
    ++write_call;
    if (write_call == 1) {
      errno = EINTR;
      return -1;
    }
    if (write_call == 2) {
      return static_cast<ssize_t>(count > 2U ? 2U : count);
    }
    if (write_call == 3) {
      errno = EAGAIN;
      return -1;
    }
    return static_cast<ssize_t>(count);
  };
  operations.wait_writable =
      [&wait_call, &now_ns](int, std::int64_t) {
        ++wait_call;
        now_ns += 1;
        return 1;
      };
  const std::uint8_t bytes[5U] = {1U, 2U, 3U, 4U, 5U};
  const wheeltec::IoResult result =
      wheeltec::writeAllWithOperations(7, bytes, 5U, 200, operations);
  expect(result.status == wheeltec::TransportStatus::kOk &&
             result.transferred == 5U && write_call == 4 && wait_call == 1,
         "write-all handles interruption, short write, and would-block");

  wheeltec::WriteOperations deadline_operations;
  deadline_operations.monotonic_now_ns = []() { return 100; };
  deadline_operations.write_bytes =
      [](int, const std::uint8_t*, std::size_t) -> ssize_t {
    errno = EAGAIN;
    return -1;
  };
  deadline_operations.wait_writable = [](int, std::int64_t) { return 0; };
  const wheeltec::IoResult deadline = wheeltec::writeAllWithOperations(
      7, bytes, sizeof(bytes), 150, deadline_operations);
  expect(deadline.status == wheeltec::TransportStatus::kDeadlineExceeded &&
             deadline.transferred == 0U,
         "write-all stops at its deadline");

  wheeltec::WriteOperations disconnected_operations = deadline_operations;
  disconnected_operations.write_bytes =
      [](int, const std::uint8_t*, std::size_t) -> ssize_t {
    errno = EPIPE;
    return -1;
  };
  const wheeltec::IoResult disconnected = wheeltec::writeAllWithOperations(
      7, bytes, sizeof(bytes), 150, disconnected_operations);
  expect(disconnected.status == wheeltec::TransportStatus::kDisconnected &&
             disconnected.delivery_unconfirmed,
         "disconnect marks delivery unconfirmed");

  wheeltec::WriteOperations zero_operations = deadline_operations;
  zero_operations.write_bytes =
      [](int, const std::uint8_t*, std::size_t) -> ssize_t { return 0; };
  const wheeltec::IoResult zero = wheeltec::writeAllWithOperations(
      7, bytes, sizeof(bytes), 150, zero_operations);
  expect(zero.status == wheeltec::TransportStatus::kDisconnected &&
             zero.delivery_unconfirmed,
         "zero-length progress is treated as disconnect");

  const wheeltec::IoResult invalid = wheeltec::writeAllWithOperations(
      7, nullptr, 1U, 150, operations);
  expect(invalid.status == wheeltec::TransportStatus::kInvalidArgument,
         "write-all rejects a null nonempty buffer");
}

struct PtyPair {
  int master{-1};
  int slave{-1};
  std::string slave_path;

  ~PtyPair() {
    if (master >= 0) {
      ::close(master);
    }
    if (slave >= 0) {
      ::close(slave);
    }
  }
};

std::size_t countOpenDescriptors() noexcept {
  constexpr int kDescriptorScanLimit = 4096;
  std::size_t count = 0U;
  for (int fd = 0; fd < kDescriptorScanLimit; ++fd) {
    errno = 0;
    if (::fcntl(fd, F_GETFD) >= 0 || errno != EBADF) {
      ++count;
    }
  }
  return count;
}

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
  pair->slave_path = slave_name;
  pair->slave =
      ::open(pair->slave_path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
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

void testSerialConfigurationReadback() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for serial-setting test");
  if (pair.slave < 0) {
    return;
  }
  termios deliberately_wrong{};
  expect(::tcgetattr(pair.slave, &deliberately_wrong) == 0,
         "PTY settings can be read before configuration");
  if (::tcgetattr(pair.slave, &deliberately_wrong) != 0) {
    return;
  }
  expect(::cfsetispeed(&deliberately_wrong, B9600) == 0 &&
             ::cfsetospeed(&deliberately_wrong, B9600) == 0,
         "PTY fixture can request a non-target baud rate");
  deliberately_wrong.c_cflag |= CSTOPB;
  deliberately_wrong.c_cflag &= static_cast<tcflag_t>(~CLOCAL);
  deliberately_wrong.c_cc[VMIN] = 4;
  deliberately_wrong.c_cc[VTIME] = 2;
  expect(::tcsetattr(pair.slave, TCSANOW, &deliberately_wrong) == 0,
         "PTY fixture installs deliberately wrong settings");

  expect(wheeltec::configureSerial115200EightNOne(pair.slave),
         "serial configuration succeeds only after its internal readback");
  termios observed{};
  expect(::tcgetattr(pair.slave, &observed) == 0,
         "configured PTY settings can be read back by the test");
  expect(::cfgetispeed(&observed) == B115200 &&
             ::cfgetospeed(&observed) == B115200,
         "serial readback has exact 115200 input and output speeds");
  expect((observed.c_cflag & CSIZE) == CS8 &&
             (observed.c_cflag & (PARENB | CSTOPB)) == 0 &&
             (observed.c_cflag & (CLOCAL | CREAD)) == (CLOCAL | CREAD),
         "serial readback has exact 8N1 and local receiver flags");
#ifdef CRTSCTS
  expect((observed.c_cflag & CRTSCTS) == 0,
         "serial readback confirms hardware flow control is disabled");
#endif
  expect(observed.c_cc[VMIN] == 0 && observed.c_cc[VTIME] == 0,
         "serial readback confirms nonblocking character thresholds");
  const int read_only_fd =
      ::open(pair.slave_path.c_str(),
             O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
  expect(read_only_fd >= 0,
         "PTY can be reopened read-only for feedback configuration");
  if (read_only_fd >= 0) {
    expect(wheeltec::configureSerial115200EightNOne(read_only_fd),
           "115200 8N1 configuration and readback work on O_RDONLY feedback fd");
    ::close(read_only_fd);
  }
  expect(!wheeltec::configureSerial115200EightNOne(-1),
         "serial configuration fails closed for an invalid descriptor");
}

void testPtyTransport() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for transport test");
  if (pair.master < 0 || pair.slave < 0) {
    return;
  }
  wheeltec::PosixFdTransport transport(pair.slave, false, 7U, true);
  const int transport_flags = ::fcntl(pair.slave, F_GETFL);
  expect(transport_flags >= 0 && (transport_flags & O_NONBLOCK) != 0,
         "PTY transport enforces nonblocking I/O for deadline safety");
  const std::uint8_t outbound[4U] = {9U, 8U, 7U, 6U};
  const std::size_t allocations_before_physical_write =
      g_cpp_allocation_count;
  wheeltec::IoResult write_result = transport.writeAll(
      outbound, sizeof(outbound), wheeltec::monotonicNowNs() + 100000000);
  const std::size_t allocations_after_physical_write =
      g_cpp_allocation_count;
  expect(write_result.status == wheeltec::TransportStatus::kOk &&
             write_result.transferred == sizeof(outbound),
         "PTY transport writes a complete buffer");
  expect(allocations_after_physical_write ==
             allocations_before_physical_write,
         "physical transport write performs no C++ allocation");
  std::uint8_t observed[4U]{};
  const ssize_t observed_count = ::read(pair.master, observed, sizeof(observed));
  expect(observed_count == static_cast<ssize_t>(sizeof(observed)) &&
             std::memcmp(observed, outbound, sizeof(outbound)) == 0,
         "PTY peer observes all written bytes");

  const std::uint8_t inbound[3U] = {4U, 5U, 6U};
  expect(::write(pair.master, inbound, sizeof(inbound)) ==
             static_cast<ssize_t>(sizeof(inbound)),
         "PTY peer supplies inbound bytes");
  std::uint8_t received[8U]{};
  const wheeltec::IoResult read_result = transport.readSome(
      received, sizeof(received), wheeltec::monotonicNowNs() + 100000000);
  expect(read_result.status == wheeltec::TransportStatus::kOk &&
             read_result.transferred == sizeof(inbound) &&
             std::memcmp(received, inbound, sizeof(inbound)) == 0,
         "PTY transport reads available bytes");
  const wheeltec::IoResult no_data = transport.readSome(
      received, sizeof(received), wheeltec::monotonicNowNs() + 2000000);
  expect(no_data.status == wheeltec::TransportStatus::kDeadlineExceeded &&
             transport.isConnected(),
         "an idle PTY reaches its read deadline without a false disconnect");
  expect(transport.connectionGeneration() == 7U,
         "transport exposes its connection generation");

  wheeltec::PosixFdTransport feedback_only(pair.slave, false, 8U);
  const wheeltec::IoResult blocked_write = feedback_only.writeAll(
      outbound, sizeof(outbound),
      wheeltec::monotonicNowNs() + 100000000);
  expect(blocked_write.status == wheeltec::TransportStatus::kDisabled &&
             blocked_write.transferred == 0U &&
             !blocked_write.delivery_unconfirmed,
         "transport defaults to rejecting every write before touching the fd");
  const wheeltec::IoResult blocked_empty_write = feedback_only.writeAll(
      nullptr, 0U, wheeltec::monotonicNowNs() + 100000000);
  expect(blocked_empty_write.status == wheeltec::TransportStatus::kDisabled,
         "feedback-only transport cannot report success even for an empty write");
}

void testPhysicalBackendGate() {
  wheeltec::PhysicalDeviceOptions options;
  options.device_path = "/outside-device-tree/test-controller";
  wheeltec::PhysicalOpenResult result = wheeltec::openPhysicalSerial(options);
  expect(result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "physical backend refuses an unacknowledged request");
  options.unverified_protocol_acknowledged = true;
  result = wheeltec::openPhysicalSerial(options);
  expect(result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "protocol acknowledgement alone cannot open a physical path");
  options.unverified_protocol_acknowledged = false;
  options.physical_device_opt_in = true;
  result = wheeltec::openPhysicalSerial(options);
  expect(result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "physical-device opt-in alone cannot open a physical path");

  options.unverified_protocol_acknowledged = true;
  options.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  result = wheeltec::openPhysicalSerial(options);
  expect(result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "actuation mode requires an additional explicit actuation opt-in");
  options.access_mode = wheeltec::PhysicalAccessMode::kFeedbackOnly;
  result = wheeltec::openPhysicalSerial(options);
  expect(result.status == wheeltec::TransportStatus::kInvalidArgument &&
             !result.transport,
         "feedback-only access still requires every pinned identity field");

  options.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  options.actuation_opt_in = true;
  result = wheeltec::openPhysicalSerial(options);
  expect(!wheeltec::kPhysicalActuationReleaseEnabled &&
             result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "the compile-time release freeze rejects physical actuation before "
         "path or device access even with every runtime opt-in");
}

wheeltec::PhysicalDeviceOptions completePhysicalOptions() {
  wheeltec::PhysicalDeviceOptions options;
  options.device_path = "/dev/pts/7";
  options.unverified_protocol_acknowledged = true;
  options.physical_device_opt_in = true;
  options.access_mode = wheeltec::PhysicalAccessMode::kFeedbackOnly;
  options.expected_device_major = 136U;
  options.expected_device_minor = 7U;
  options.expected_owner_uid = 0U;
  options.expected_group_gid = 20U;
  options.expected_usb_vendor_id = "1a86";
  options.expected_usb_product_id = "7523";
  options.expected_usb_serial = "ROVER-VCU-001";
  return options;
}

wheeltec::PhysicalFileIdentity completeFileIdentity() {
  wheeltec::PhysicalFileIdentity identity;
  identity.is_symlink = false;
  identity.is_character_device = true;
  identity.filesystem_device = 42U;
  identity.inode = 99U;
  identity.device_major = 136U;
  identity.device_minor = 7U;
  identity.owner_uid = 0U;
  identity.group_gid = 20U;
  identity.permission_bits = 0660U;
  return identity;
}

void testPhysicalIdentityValidation() {
  const wheeltec::PhysicalDeviceOptions options = completePhysicalOptions();
  const wheeltec::PhysicalFileIdentity identity = completeFileIdentity();
  expect(wheeltec::physicalDeviceOptionsAreComplete(options),
         "physical identity configuration requires every pinned attribute");
  expect(wheeltec::validatePhysicalDeviceFileIdentity(
             options, identity, identity, identity, true),
         "a stable pinned character TTY identity is accepted");

  wheeltec::PhysicalDeviceOptions missing = options;
  missing.expected_usb_serial.clear();
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "an omitted USB serial fails closed before open");
  missing = options;
  missing.expected_device_minor = wheeltec::kUnspecifiedPhysicalIdentity;
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "an omitted device minor fails closed before open");
  missing = options;
  missing.expected_owner_uid = wheeltec::kUnspecifiedPhysicalIdentity;
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "an omitted expected owner fails closed before open");
  missing = options;
  missing.access_mode = wheeltec::PhysicalAccessMode::kDisabled;
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "the default access mode fails closed");
  missing = options;
  missing.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "actuation access is incomplete without its third opt-in");
  missing.actuation_opt_in = true;
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "all runtime opt-ins cannot override the physical-actuation "
         "release freeze");
  missing = options;
  missing.access_mode = static_cast<wheeltec::PhysicalAccessMode>(0xFFU);
  expect(!wheeltec::physicalDeviceOptionsAreComplete(missing),
         "an unknown physical-access enum value fails closed");

  wheeltec::PhysicalFileIdentity changed = identity;
  changed.is_symlink = true;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, identity, identity, true),
         "a symlink is rejected");
  changed = identity;
  changed.is_character_device = false;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, changed, changed, true),
         "a non-character path is rejected");
  changed = identity;
  changed.inode += 1U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, identity, changed, identity, true),
         "a path-to-fd inode race is rejected");
  changed = identity;
  changed.device_minor += 1U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, identity, changed, changed, true),
         "an unexpected device major/minor is rejected");
  changed = identity;
  changed.owner_uid += 1U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, changed, changed, true),
         "an unexpected owner is rejected");
  changed = identity;
  changed.group_gid += 1U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, changed, changed, true),
         "an unexpected group is rejected");
  changed = identity;
  changed.permission_bits |= 0002U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, changed, changed, true),
         "world-writable device permissions are rejected");
  changed = identity;
  changed.permission_bits |= 0100U;
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, changed, changed, changed, true),
         "executable device permissions are rejected");
  expect(!wheeltec::validatePhysicalDeviceFileIdentity(
             options, identity, identity, identity, false),
         "a character device that is not a TTY is rejected");
}

void testUsbIdentityAncestorLookup() {
  const std::string base =
      "/sys/devices/pci0000:00/usb1/1-1/1-1:1.0/ttyUSB0";
  const std::string usb_parent = "/sys/devices/pci0000:00/usb1/1-1";
  const std::map<std::string, std::string> attributes{
      {usb_parent + "/idVendor", "1A86\n"},
      {usb_parent + "/idProduct", "7523\n"},
      {usb_parent + "/serial", "ROVER-VCU-001\n"}};
  const wheeltec::SysfsTextReader reader =
      [&attributes](const std::string& path, std::string* value) {
        const auto found = attributes.find(path);
        if (found == attributes.end() || value == nullptr) {
          return false;
        }
        *value = found->second;
        return true;
      };
  wheeltec::UsbDeviceIdentity observed;
  expect(wheeltec::readUsbDeviceIdentityFromAncestors(base, reader,
                                                       &observed),
         "USB identity is read from one common sysfs parent after open");
  expect(wheeltec::usbDeviceIdentityMatches(completePhysicalOptions(),
                                             observed),
         "pinned VID, PID, and serial match canonicalized sysfs values");

  wheeltec::PhysicalDeviceOptions mismatch = completePhysicalOptions();
  mismatch.expected_usb_serial = "A-DIFFERENT-VCU";
  expect(!wheeltec::usbDeviceIdentityMatches(mismatch, observed),
         "a USB serial mismatch fails closed");
  mismatch = completePhysicalOptions();
  mismatch.expected_usb_vendor_id = "zzzz";
  expect(!wheeltec::physicalDeviceOptionsAreComplete(mismatch),
         "a malformed expected USB VID fails configuration validation");

  const std::map<std::string, std::string> split_attributes{
      {"/sys/devices/pci0000:00/usb1/1-1:1.0/idVendor", "1a86\n"},
      {usb_parent + "/idProduct", "7523\n"},
      {usb_parent + "/serial", "ROVER-VCU-001\n"}};
  const wheeltec::SysfsTextReader split_reader =
      [&split_attributes](const std::string& path, std::string* value) {
        const auto found = split_attributes.find(path);
        if (found == split_attributes.end() || value == nullptr) {
          return false;
        }
        *value = found->second;
        return true;
      };
  expect(!wheeltec::readUsbDeviceIdentityFromAncestors(
             base, split_reader, &observed),
         "USB attributes cannot be assembled from different ancestors");
}

void testHardenedPhysicalOpenWithPtyOnly() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for hardened-open test");
  if (pair.slave < 0 || pair.slave_path.empty()) {
    return;
  }
  struct stat metadata {};
  expect(::fstat(pair.slave, &metadata) == 0,
         "PTY metadata is available for pinned-identity test");
  if (::fstat(pair.slave, &metadata) != 0) {
    return;
  }
  wheeltec::PhysicalDeviceOptions options = completePhysicalOptions();
  options.device_path = pair.slave_path;
  options.expected_device_major =
      static_cast<std::uint64_t>(::major(metadata.st_rdev));
  options.expected_device_minor =
      static_cast<std::uint64_t>(::minor(metadata.st_rdev));
  options.expected_owner_uid = static_cast<std::uint64_t>(metadata.st_uid);
  options.expected_group_gid = static_cast<std::uint64_t>(metadata.st_gid);
  const wheeltec::PhysicalOpenResult result =
      wheeltec::openPhysicalSerial(options);
  expect(result.status != wheeltec::TransportStatus::kOk &&
             !result.transport,
         "a PTY still fails closed because it has no pinned USB sysfs identity");

  options.device_path = "/dev/fd/0";
  const wheeltec::PhysicalOpenResult symlink_component =
      wheeltec::openPhysicalSerial(options);
  expect(symlink_component.status ==
                 wheeltec::TransportStatus::kInvalidArgument &&
             !symlink_component.transport,
         "a symlink in the /dev path is rejected before open");
}

wheeltec::PhysicalPreparationOperations injectedUsbPreparation(
    const std::map<std::string, std::string>& attributes,
    int* resolver_calls = nullptr) {
  wheeltec::PhysicalPreparationOperations operations;
  operations.resolve_sysfs_device_path =
      [resolver_calls](const std::string& link, std::string* resolved) {
        if (resolver_calls != nullptr) {
          ++(*resolver_calls);
        }
        if (resolved == nullptr ||
            link.compare(0U, 14U, "/sys/dev/char/") != 0) {
          return false;
        }
        *resolved =
            "/sys/devices/pci0000:00/usb1/1-1/1-1:1.0/ttyUSB0";
        return true;
      };
  operations.read_sysfs_text =
      [&attributes](const std::string& path, std::string* value) {
        const auto found = attributes.find(path);
        if (found == attributes.end() || value == nullptr) {
          return false;
        }
        *value = found->second;
        return true;
      };
  return operations;
}

wheeltec::PhysicalDeviceOptions optionsForPty(
    const PtyPair& pair, const struct stat& metadata) {
  wheeltec::PhysicalDeviceOptions options = completePhysicalOptions();
  options.device_path = pair.slave_path;
  options.expected_device_major =
      static_cast<std::uint64_t>(::major(metadata.st_rdev));
  options.expected_device_minor =
      static_cast<std::uint64_t>(::minor(metadata.st_rdev));
  options.expected_owner_uid = static_cast<std::uint64_t>(metadata.st_uid);
  options.expected_group_gid = static_cast<std::uint64_t>(metadata.st_gid);
  return options;
}

wheeltec::PhysicalFileIdentity fileIdentityFromStatForTest(
    const struct stat& metadata) {
  wheeltec::PhysicalFileIdentity identity;
  identity.is_symlink = S_ISLNK(metadata.st_mode);
  identity.is_character_device = S_ISCHR(metadata.st_mode);
  identity.filesystem_device =
      static_cast<std::uint64_t>(metadata.st_dev);
  identity.inode = static_cast<std::uint64_t>(metadata.st_ino);
  identity.device_major =
      static_cast<std::uint64_t>(::major(metadata.st_rdev));
  identity.device_minor =
      static_cast<std::uint64_t>(::minor(metadata.st_rdev));
  identity.owner_uid = static_cast<std::uint64_t>(metadata.st_uid);
  identity.group_gid = static_cast<std::uint64_t>(metadata.st_gid);
  identity.permission_bits =
      static_cast<std::uint32_t>(metadata.st_mode & 07777);
  return identity;
}

struct SequencedPathIdentityRead {
  const char* expected_path{nullptr};
  wheeltec::PhysicalFileIdentity before_open;
  wheeltec::PhysicalFileIdentity after_open;
  int calls{0};
};

bool readSequencedPathIdentity(
    const char* path, wheeltec::PhysicalFileIdentity* identity,
    void* opaque) noexcept {
  SequencedPathIdentityRead* const sequence =
      static_cast<SequencedPathIdentityRead*>(opaque);
  if (path == nullptr || identity == nullptr || sequence == nullptr ||
      sequence->expected_path == nullptr ||
      std::strcmp(path, sequence->expected_path) != 0 ||
      sequence->calls < 0 || sequence->calls > 1) {
    errno = EINVAL;
    return false;
  }
  *identity = sequence->calls == 0 ? sequence->before_open
                                   : sequence->after_open;
  ++sequence->calls;
  return true;
}

void testPreparedPhysicalOpenIsAllocationFreeAndOwnsLock() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for prepared-open test");
  if (pair.slave < 0 || pair.slave_path.empty()) {
    return;
  }
  struct stat metadata {};
  expect(::fstat(pair.slave, &metadata) == 0,
         "PTY metadata is available for prepared-open test");
  if (::fstat(pair.slave, &metadata) != 0) {
    return;
  }
  const wheeltec::PhysicalDeviceOptions options =
      optionsForPty(pair, metadata);
  const std::string usb_parent = "/sys/devices/pci0000:00/usb1/1-1";
  const std::map<std::string, std::string> attributes{
      {usb_parent + "/idVendor", "1a86\n"},
      {usb_parent + "/idProduct", "7523\n"},
      {usb_parent + "/serial", "ROVER-VCU-001\n"}};
  const wheeltec::PhysicalPreparationOperations operations =
      injectedUsbPreparation(attributes);

  wheeltec::PhysicalPreparationOperations allocation_failure = operations;
  allocation_failure.resolve_sysfs_device_path =
      [](const std::string&, std::string*) -> bool {
        throw std::bad_alloc();
      };
  const std::size_t descriptors_before_bad_alloc = countOpenDescriptors();
  wheeltec::PreparedPhysicalSerialOpen failed_preparation(
      options, allocation_failure);
  const std::size_t descriptors_after_bad_alloc = countOpenDescriptors();
  expect(!failed_preparation.prepared() &&
             failed_preparation.preparationStatus() ==
                 wheeltec::TransportStatus::kIoError &&
             failed_preparation.preparationError() == ENOMEM &&
             descriptors_after_bad_alloc == descriptors_before_bad_alloc,
         "a preparation allocation exception is converted before raw open "
         "and cannot leak a descriptor");

  wheeltec::PreparedPhysicalSerialOpen lock_rejected(options, operations);
  expect(lock_rejected.prepared() &&
             lock_rejected.preparationStatus() ==
                 wheeltec::TransportStatus::kOk &&
             lock_rejected.preparationError() == 0,
         "all string, sysfs, identity, and transport allocation completes "
         "during preparation");
  expect(::flock(pair.slave, LOCK_EX | LOCK_NB) == 0,
         "PTY fixture acquires a cooperative pre-existing owner lock");
  const std::size_t descriptors_before_rejected_open =
      countOpenDescriptors();
  const std::size_t allocations_before_rejected_open =
      g_cpp_allocation_count;
  wheeltec::PhysicalOpenResult rejected = lock_rejected.open();
  const std::size_t allocations_after_rejected_open =
      g_cpp_allocation_count;
  const std::size_t descriptors_after_rejected_open =
      countOpenDescriptors();
  expect(rejected.status == wheeltec::TransportStatus::kIoError &&
             (rejected.os_error == EWOULDBLOCK ||
              rejected.os_error == EAGAIN) &&
             !rejected.transport,
         "a cooperative existing owner lock rejects the second physical "
         "open before exclusivity or serial I/O");
  expect(descriptors_after_rejected_open == descriptors_before_rejected_open,
         "post-open lock rejection closes the guarded descriptor");
  expect(allocations_after_rejected_open ==
             allocations_before_rejected_open,
         "the post-open rejection path performs no C++ allocation");
  expect(::flock(pair.slave, LOCK_UN) == 0,
         "PTY fixture releases its cooperative owner lock");

  wheeltec::PreparedPhysicalSerialOpen first(options, operations);
  wheeltec::PreparedPhysicalSerialOpen second(options, operations);
  expect(first.prepared() && second.prepared(),
         "two contenders can finish all preparation before either raw open");
  const std::size_t allocations_before_successful_open =
      g_cpp_allocation_count;
  wheeltec::PhysicalOpenResult opened = first.open();
  const std::size_t allocations_after_successful_open =
      g_cpp_allocation_count;
  expect(opened.status == wheeltec::TransportStatus::kOk &&
             opened.transport && opened.transport->isConnected(),
         "prepared feedback-only PTY open adopts the validated owned fd");
  expect(allocations_after_successful_open ==
             allocations_before_successful_open,
         "raw open through fd adoption performs no C++ allocation");

  wheeltec::PhysicalOpenResult competing = second.open();
  expect(competing.status != wheeltec::TransportStatus::kOk &&
             !competing.transport,
         "a second fully prepared owner cannot enter an active generation");
  wheeltec::PhysicalOpenResult repeated = first.open();
  expect(repeated.status == wheeltec::TransportStatus::kInvalidArgument &&
             repeated.os_error == EALREADY && !repeated.transport,
         "a prepared physical open is one-shot even after success");
}

void testPreparedPhysicalOpenRejectsPostOpenPathIdentityChange() {
  PtyPair pair;
  expect(openPtyPair(&pair),
         "PTY pair opens for post-open identity-race test");
  if (pair.slave < 0 || pair.slave_path.empty()) {
    return;
  }
  struct stat metadata {};
  if (::fstat(pair.slave, &metadata) != 0) {
    expect(false,
           "PTY metadata is available for post-open identity-race test");
    return;
  }

  const wheeltec::PhysicalDeviceOptions options =
      optionsForPty(pair, metadata);
  const std::string usb_parent = "/sys/devices/pci0000:00/usb1/1-1";
  const std::map<std::string, std::string> attributes{
      {usb_parent + "/idVendor", "1a86\n"},
      {usb_parent + "/idProduct", "7523\n"},
      {usb_parent + "/serial", "ROVER-VCU-001\n"}};
  wheeltec::PhysicalPreparationOperations operations =
      injectedUsbPreparation(attributes);
  SequencedPathIdentityRead sequence;
  sequence.expected_path = pair.slave_path.c_str();
  sequence.before_open = fileIdentityFromStatForTest(metadata);
  sequence.after_open = sequence.before_open;
  ++sequence.after_open.inode;
  operations.read_path_identity = &readSequencedPathIdentity;
  operations.path_identity_context = &sequence;

  wheeltec::PreparedPhysicalSerialOpen prepared(options, operations);
  expect(prepared.prepared() && sequence.calls == 1,
         "preparation records the first path identity observation");
  const std::size_t descriptors_before_open = countOpenDescriptors();
  const std::size_t allocations_before_open = g_cpp_allocation_count;
  const wheeltec::PhysicalOpenResult result = prepared.open();
  const std::size_t allocations_after_open = g_cpp_allocation_count;
  const std::size_t descriptors_after_open = countOpenDescriptors();
  expect(sequence.calls == 2,
         "production open re-reads the no-symlink path identity after fstat");
  expect(result.status == wheeltec::TransportStatus::kInvalidArgument &&
             result.os_error == EPERM && !result.transport,
         "a post-open path identity change rejects the production open");
  expect(descriptors_after_open == descriptors_before_open,
         "post-open identity rejection closes the guarded descriptor");
  expect(allocations_after_open == allocations_before_open,
         "post-open identity validation performs no C++ allocation");

  const int probe_fd =
      ::open(pair.slave_path.c_str(),
             O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
  expect(probe_fd >= 0,
         "post-open identity rejection occurs before TIOCEXCL");
  if (probe_fd >= 0) {
    ::close(probe_fd);
  }
}

void testActuationReleaseFreezePrecedesEveryOpenStep() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY pair opens for release-freeze test");
  if (pair.slave < 0 || pair.slave_path.empty()) {
    return;
  }
  struct stat metadata {};
  if (::fstat(pair.slave, &metadata) != 0) {
    expect(false, "PTY metadata is available for release-freeze test");
    return;
  }
  wheeltec::PhysicalDeviceOptions options = optionsForPty(pair, metadata);
  options.access_mode = wheeltec::PhysicalAccessMode::kActuation;
  options.actuation_opt_in = true;
  const std::map<std::string, std::string> attributes;
  int resolver_calls = 0;
  const wheeltec::PhysicalPreparationOperations operations =
      injectedUsbPreparation(attributes, &resolver_calls);
  const std::size_t descriptors_before = countOpenDescriptors();
  wheeltec::PreparedPhysicalSerialOpen prepared(options, operations);
  wheeltec::PhysicalOpenResult result = prepared.open();
  const std::size_t descriptors_after = countOpenDescriptors();
  expect(!wheeltec::kPhysicalActuationReleaseEnabled && !prepared.prepared() &&
             prepared.preparationStatus() ==
                 wheeltec::TransportStatus::kDisabled &&
             result.status == wheeltec::TransportStatus::kDisabled &&
             !result.transport,
         "compile-time release freeze rejects an otherwise fully opted-in "
         "actuation request");
  expect(resolver_calls == 0 && descriptors_after == descriptors_before,
         "actuation freeze occurs before sysfs callbacks and raw device open");

  const int probe_fd =
      ::open(pair.slave_path.c_str(),
             O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW);
  expect(probe_fd >= 0,
         "actuation rejection did not leave TIOCEXCL on the PTY");
  if (probe_fd >= 0) {
    ::close(probe_fd);
  }
}

class FakeTransport final : public wheeltec::ByteTransport {
 public:
  bool isConnected() const override { return connected_; }
  std::uint64_t connectionGeneration() const override { return generation_; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t) override {
    ++write_calls;
    writes.emplace_back(data, data + size);
    wheeltec::TransportStatus status = wheeltec::TransportStatus::kOk;
    if (!outcomes.empty()) {
      status = outcomes.front();
      outcomes.pop_front();
    }
    if (status == wheeltec::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    return {status, status == wheeltec::TransportStatus::kOk ? size : 0U, 0,
            status == wheeltec::TransportStatus::kDisconnected};
  }

  wheeltec::IoResult readSome(std::uint8_t*, std::size_t,
                              std::int64_t) override {
    return {wheeltec::TransportStatus::kWouldBlock, 0U, EAGAIN, false};
  }

  void disconnect() { connected_ = false; }
  void reconnect() {
    connected_ = true;
    ++generation_;
  }

  bool connected_{true};
  std::uint64_t generation_{1U};
  std::size_t write_calls{0U};
  std::vector<std::vector<std::uint8_t>> writes;
  std::deque<wheeltec::TransportStatus> outcomes;
};

wheeltec::AdapterConfig adapterConfig(std::uint32_t fresh_count) {
  wheeltec::AdapterConfig config;
  config.codec_limits = commissioningCodecLimits();
  config.max_command_age_ns = 50;
  config.fresh_commands_required = fresh_count;
  config.zero_retry_interval_ns = 10;
  config.max_zero_write_attempts = 3U;
  config.write_timeout_ns = 5;
  return config;
}

void testAdapterRecoveryAndWatchdog() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(2U), &transport);
  expect(!adapter.isAuthorized() && !adapter.recoveryComplete(),
         "adapter starts disabled and recovery-inhibited");
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "initial authorization has a fresh identity");

  wheeltec::SubmitResult submit =
      adapter.submit(makeCommand(1U, 90, 180), 100);
  expect(submit.status == wheeltec::SubmissionStatus::kRecoveryPending &&
             adapter.consecutiveFreshCommands() == 1U,
         "first fresh command remains recovery-inhibited");
  wheeltec::AdapterCycleResult cycle = adapter.cycle(100);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWritten,
         "recovery inhibition emits an explicit zero frame");
  expect(!transport.writes.empty() && transport.writes.back().size() == 11U &&
             transport.writes.back()[3U] == 0U &&
             transport.writes.back()[4U] == 0U &&
             transport.writes.back()[7U] == 0U &&
             transport.writes.back()[8U] == 0U,
         "recovery zero has exact zero speed and yaw fields");

  submit = adapter.submit(makeCommand(2U, 95, 180), 100);
  expect(submit.status == wheeltec::SubmissionStatus::kAccepted &&
             adapter.recoveryComplete(),
         "required consecutive fresh commands complete recovery");
  cycle = adapter.cycle(100);
  expect(cycle.action == wheeltec::AdapterCycleAction::kMotionWritten,
         "recovered adapter writes the latest motion command");

  cycle = adapter.cycle(181);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWritten,
         "expired command causes an explicit watchdog zero");
  const std::size_t after_timeout = transport.write_calls;
  cycle = adapter.cycle(182);
  expect(cycle.action == wheeltec::AdapterCycleAction::kNoAction &&
             transport.write_calls == after_timeout,
         "one successful zero satisfies the current inhibit episode");
}

void testAdapterSubmissionRejections() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(2U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "submission fixture is explicitly authorized");
  expect(adapter.submit(makeCommand(1U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kRecoveryPending,
         "baseline fresh command is accepted");
  expect(adapter.submit(makeCommand(1U, 91, 181), 100).status ==
             wheeltec::SubmissionStatus::kSequenceInvalid,
         "duplicate sequence is rejected");
  expect(adapter.consecutiveFreshCommands() == 0U,
         "duplicate sequence breaks the fresh-command run");
  expect(adapter.submit(makeCommand(2U, 101, 180), 100).status ==
             wheeltec::SubmissionStatus::kTimestampInvalid,
         "future-created command is rejected");
  expect(adapter.submit(makeCommand(3U, 10, 40), 100).status ==
             wheeltec::SubmissionStatus::kStale,
         "over-age command is rejected");
  expect(adapter.submit(makeCommand(4U, 90, 99), 100).status ==
             wheeltec::SubmissionStatus::kStale,
         "past-deadline command is rejected");
  expect(adapter.submit(makeCommand(5U, 90, 180, 0.6, 0.0), 100).status ==
             wheeltec::SubmissionStatus::kCodecRejected,
         "adapter rejects commands outside codec limits");
  auto_rover::VehicleExecutionCommand malformed =
      makeCommand(6U, 90, 90, 0.2, 0.0);
  expect(adapter.submit(malformed, 90).status ==
             wheeltec::SubmissionStatus::kTimestampInvalid,
         "non-increasing command deadline is rejected");
  expect(adapter.submit(makeCommand(7U, 0, 100), 10).status ==
             wheeltec::SubmissionStatus::kTimestampInvalid,
         "zero creation time is rejected");
}

void testAuthorizationTimeFailsClosed() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(1U), &transport);
  expect(!adapter.setAuthorization(true, true, 1U, 0) &&
             !adapter.isAuthorized(),
         "authorization requires a valid local monotonic transition time");
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "a valid authorization time establishes the command epoch");
  expect(adapter.submit(makeCommand(1U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "a command created after the authorization epoch is accepted");
  expect(!adapter.setAuthorization(true, true, 2U, 99) &&
             !adapter.isAuthorized() && !adapter.recoveryComplete(),
         "a regressed authorization clock revokes motion and cannot move the epoch backward");
}

void testCycleClockRollbackRevokesAuthorization() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(1U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "cycle-clock fixture establishes an authorization epoch");
  expect(adapter.submit(makeCommand(1U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "cycle-clock fixture completes recovery");
  expect(adapter.cycle(100).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "cycle-clock fixture first writes motion");

  const std::size_t writes_before_rollback = transport.write_calls;
  const auto regressed = adapter.cycle(99);
  expect(regressed.action == wheeltec::AdapterCycleAction::kClockInvalid &&
             regressed.delivery_unconfirmed &&
             transport.write_calls == writes_before_rollback &&
             !adapter.isAuthorized() && !adapter.recoveryComplete() &&
             adapter.consecutiveFreshCommands() == 0U,
         "a positive cycle-clock rollback writes nothing and revokes authorization and recovery");

  const auto nonpositive = adapter.cycle(-1);
  expect(nonpositive.action == wheeltec::AdapterCycleAction::kClockInvalid &&
             nonpositive.delivery_unconfirmed &&
             transport.write_calls == writes_before_rollback &&
             !adapter.isAuthorized() && !adapter.recoveryComplete(),
         "a non-positive cycle clock writes nothing and remains authorization-inhibited");

  const auto recovered_clock = adapter.cycle(101);
  expect(recovered_clock.action ==
             wheeltec::AdapterCycleAction::kZeroWritten &&
             transport.write_calls == writes_before_rollback + 1U &&
             !adapter.isAuthorized() && !adapter.recoveryComplete(),
         "clock recovery emits zero and cannot resume cached motion under the old authorization");
}

void testSubmitClockRollbackRequiresNewAuthorizationAndRecovery() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(2U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "submit-clock fixture establishes an authorization epoch");
  expect(adapter.submit(makeCommand(1U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kRecoveryPending &&
             adapter.consecutiveFreshCommands() == 1U,
         "first fresh command begins submit-clock recovery");

  expect(adapter.submit(makeCommand(2U, 91, 180), 99).status ==
             wheeltec::SubmissionStatus::kTimestampInvalid &&
             !adapter.isAuthorized() && !adapter.recoveryComplete() &&
             adapter.consecutiveFreshCommands() == 0U,
         "a regressed command receipt cannot accumulate recovery and revokes authorization");
  expect(adapter.submit(makeCommand(2U, 91, 180), 0).status ==
             wheeltec::SubmissionStatus::kTimestampInvalid &&
             !adapter.isAuthorized() && !adapter.recoveryComplete(),
         "a non-positive command receipt remains fail-closed");
  expect(adapter.cycle(101).action ==
             wheeltec::AdapterCycleAction::kZeroWritten,
         "submit-clock failure starts an explicit zero episode");

  expect(adapter.setAuthorization(true, true, 2U, 102),
         "submit-clock recovery requires a new authorization epoch");
  expect(adapter.submit(makeCommand(2U, 103, 180), 103).status ==
             wheeltec::SubmissionStatus::kRecoveryPending &&
             adapter.consecutiveFreshCommands() == 1U,
         "first post-authorization command remains recovery-inhibited");
  expect(adapter.submit(makeCommand(3U, 104, 180), 104).status ==
             wheeltec::SubmissionStatus::kAccepted &&
             adapter.recoveryComplete(),
         "only a full run of post-authorization fresh commands restores recovery");
  expect(adapter.cycle(104).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "motion resumes only after the new authorization and fresh run");
}

void testRecoveryRunBreaksOnFreshnessGap() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(2U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "recovery fixture is explicitly authorized");
  expect(adapter.submit(makeCommand(1U, 90, 110), 100).status ==
             wheeltec::SubmissionStatus::kRecoveryPending,
         "first recovery command begins a fresh run");
  expect(adapter.submit(makeCommand(2U, 190, 250), 200).status ==
             wheeltec::SubmissionStatus::kRecoveryPending &&
             adapter.consecutiveFreshCommands() == 1U,
         "a gap beyond the prior deadline restarts the fresh run");
}

void testBoundedZeroRetries() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(1U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "zero-retry fixture is explicitly authorized");
  expect(adapter.submit(makeCommand(1U, 90, 120), 100).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "single fresh command completes configured recovery");
  expect(adapter.cycle(100).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "motion is written before watchdog expiry");

  transport.outcomes.push_back(wheeltec::TransportStatus::kDeadlineExceeded);
  transport.outcomes.push_back(wheeltec::TransportStatus::kDeadlineExceeded);
  transport.outcomes.push_back(wheeltec::TransportStatus::kDeadlineExceeded);
  const std::size_t before_zero = transport.write_calls;
  wheeltec::AdapterCycleResult cycle = adapter.cycle(121);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWriteFailed &&
             cycle.zero_attempts == 1U &&
             transport.write_calls == before_zero + 1U,
         "first zero failure is recorded");
  cycle = adapter.cycle(125);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroRetryPending &&
             transport.write_calls == before_zero + 1U,
         "zero retry respects its interval");
  cycle = adapter.cycle(131);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWriteFailed &&
             cycle.zero_attempts == 2U,
         "second bounded zero attempt occurs after interval");
  cycle = adapter.cycle(141);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroRetriesExhausted &&
             cycle.zero_attempts == 3U,
         "zero retries stop at the configured bound");
  const std::size_t at_bound = transport.write_calls;
  cycle = adapter.cycle(200);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroRetriesExhausted &&
             transport.write_calls == at_bound,
         "no zero writes occur after retry exhaustion");
}

void testZeroDisconnectReportsAttempt() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(1U), &transport);
  transport.outcomes.push_back(wheeltec::TransportStatus::kDisconnected);
  const wheeltec::AdapterCycleResult cycle = adapter.cycle(10);
  expect(cycle.action == wheeltec::AdapterCycleAction::kDisconnected &&
             cycle.delivery_unconfirmed && cycle.zero_attempts == 1U,
         "a disconnecting zero write reports its attempted delivery");
}

void testExplicitDisarmWritesZero() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(1U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "disarm fixture is explicitly authorized");
  expect(adapter.submit(makeCommand(1U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "fresh command arms the single-sample test adapter");
  expect(adapter.cycle(100).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "motion precedes the explicit disarm");
  expect(adapter.setAuthorization(false, false, 0U, 101),
         "explicit disarm is accepted without an arm identity");
  const wheeltec::AdapterCycleResult cycle = adapter.cycle(101);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWritten &&
             !adapter.isAuthorized(),
         "explicit disarm emits a zero frame for orderly shutdown");
}

void testDisconnectAndReconnectInterlock() {
  FakeTransport transport;
  wheeltec::WheeltecSerialAdapter adapter(adapterConfig(2U), &transport);
  expect(adapter.setAuthorization(true, true, 1U, 80),
         "disconnect fixture is explicitly authorized");
  expect(adapter.submit(makeCommand(10U, 90, 180), 100).status ==
             wheeltec::SubmissionStatus::kRecoveryPending,
         "first pre-disconnect command is fresh");
  expect(adapter.submit(makeCommand(11U, 95, 180), 100).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "second pre-disconnect command completes recovery");
  expect(adapter.cycle(100).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "pre-disconnect motion is sent");

  transport.disconnect();
  wheeltec::AdapterCycleResult cycle = adapter.cycle(101);
  expect(cycle.action == wheeltec::AdapterCycleAction::kDisconnected &&
             cycle.delivery_unconfirmed && !adapter.recoveryComplete(),
         "disconnect clears motion state and marks delivery unconfirmed");
  transport.reconnect();
  expect(!adapter.setAuthorization(true, true, 1U, 102) &&
             !adapter.isAuthorized(),
         "a stale true authorization cannot cross a new connection generation");
  cycle = adapter.cycle(102);
  expect(cycle.action == wheeltec::AdapterCycleAction::kZeroWritten &&
             !adapter.isAuthorized(),
         "new connection stays inhibited and emits zero");
  expect(adapter.submit(makeCommand(12U, 103, 180), 103).status ==
             wheeltec::SubmissionStatus::kNotAuthorized,
         "reconnect requires explicit re-enable and re-arm");

  expect(adapter.setAuthorization(true, true, 2U, 104),
         "reconnect requires a new explicit authorization identity");
  expect(adapter.submit(makeCommand(11U, 105, 180), 105).status ==
             wheeltec::SubmissionStatus::kSequenceInvalid,
         "reconnect cannot replay an old sequence");
  expect(adapter.submit(makeCommand(13U, 100, 180), 105).status ==
             wheeltec::SubmissionStatus::kPredatesAuthorization &&
             adapter.consecutiveFreshCommands() == 0U,
         "a previously unseen pre-reconnect command cannot enter recovery after reauthorization");
  expect(adapter.submit(makeCommand(14U, 100, 180), 105).status ==
             wheeltec::SubmissionStatus::kPredatesAuthorization &&
             adapter.consecutiveFreshCommands() == 0U,
         "repeated pre-reconnect commands cannot complete recovery");
  expect(adapter.cycle(105).action ==
             wheeltec::AdapterCycleAction::kZeroWritten,
         "pre-authorization command replay remains motion-inhibited");
  expect(adapter.submit(makeCommand(15U, 105, 180), 105).status ==
             wheeltec::SubmissionStatus::kRecoveryPending,
         "first post-reconnect fresh command remains inhibited");
  expect(adapter.submit(makeCommand(16U, 106, 180), 106).status ==
             wheeltec::SubmissionStatus::kAccepted,
         "second post-reconnect fresh command restores eligibility");
  expect(adapter.cycle(106).action ==
             wheeltec::AdapterCycleAction::kMotionWritten,
         "motion resumes only after reauthorization and recovery");
}

}  // namespace

int main() {
  testCommandCodec();
  testFeedbackCodec();
  testRollingParser();
  testWriteAllOperations();
  testSerialConfigurationReadback();
  testPtyTransport();
  testPhysicalBackendGate();
  testPhysicalIdentityValidation();
  testUsbIdentityAncestorLookup();
  testHardenedPhysicalOpenWithPtyOnly();
  testPreparedPhysicalOpenIsAllocationFreeAndOwnsLock();
  testPreparedPhysicalOpenRejectsPostOpenPathIdentityChange();
  testActuationReleaseFreezePrecedesEveryOpenStep();
  testAdapterRecoveryAndWatchdog();
  testAdapterSubmissionRejections();
  testAuthorizationTimeFailsClosed();
  testCycleClockRollbackRevokesAuthorization();
  testSubmitClockRollbackRequiresNewAuthorizationAndRecovery();
  testRecoveryRunBreaksOnFreshnessGap();
  testBoundedZeroRetries();
  testZeroDisconnectReportsAttempt();
  testExplicitDisarmWritesZero();
  testDisconnectAndReconnectInterlock();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d wheeltec test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("wheeltec tests passed\n");
  return 0;
}
