#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

namespace auto_rover {
namespace wheeltec_serial {

constexpr std::size_t kMaximumFeedbackCaptureReadBytes = 4096U;

enum class FeedbackCaptureStatus : std::uint8_t {
  kCompleted = 0,
  kInvalidArgument,
  kClockInvalid,
  kDisconnected,
  kTransportError,
  kRecordError,
};

struct FeedbackCaptureConfig {
  std::int64_t duration_ns{0};
  std::int64_t read_timeout_ns{100000000};
  std::size_t read_buffer_bytes{256U};
};

struct FeedbackCaptureStatistics {
  std::uint64_t raw_bytes{0U};
  std::uint64_t read_calls{0U};
  std::uint64_t read_timeouts{0U};
  std::uint64_t valid_frames{0U};
  std::uint64_t checksum_failures{0U};
  std::uint64_t framing_failures{0U};
  std::uint64_t discarded_bytes{0U};
  std::size_t trailing_buffered_bytes{0U};
  std::int64_t started_monotonic_ns{0};
  std::int64_t ended_monotonic_ns{0};
  std::int64_t first_frame_receipt_monotonic_ns{0};
  std::int64_t last_frame_receipt_monotonic_ns{0};
  double capture_average_rate_hz{0.0};
  double interframe_rate_hz{0.0};
};

struct FeedbackCaptureOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<bool(const std::string&)> record_json_line;
};

struct FeedbackCaptureResult {
  FeedbackCaptureStatus status{FeedbackCaptureStatus::kInvalidArgument};
  TransportStatus terminal_transport_status{TransportStatus::kOk};
  int os_error{0};
  FeedbackCaptureStatistics statistics{};

  bool completed() const { return status == FeedbackCaptureStatus::kCompleted; }
};

const char* feedbackCaptureStatusName(FeedbackCaptureStatus status);
const char* transportStatusName(TransportStatus status);
std::string feedbackRawChunkRecordJson(const std::uint8_t* data,
                                       std::size_t size,
                                       std::int64_t receipt_monotonic_ns);
std::string feedbackFrameRecordJson(const ParsedFeedbackFrame& frame,
                                    std::int64_t receipt_monotonic_ns);
std::string feedbackSummaryRecordJson(const FeedbackCaptureResult& result);

class FeedbackCaptureSession {
 public:
  FeedbackCaptureResult run(ByteTransport* transport,
                            const FeedbackCaptureConfig& config,
                            const FeedbackCaptureOperations& operations);
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
