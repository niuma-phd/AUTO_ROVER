#include "auto_rover_vcu_wheeltec_serial/feedback_capture.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

constexpr const char* kCaptureSchema =
    "auto_rover.wheeltec.feedback_capture.v1";

bool captureConfigIsValid(const FeedbackCaptureConfig& config) {
  return config.duration_ns > 0 && config.read_timeout_ns > 0 &&
         config.read_buffer_bytes > 0U &&
         config.read_buffer_bytes <= kMaximumFeedbackCaptureReadBytes;
}

std::int64_t boundedDeadline(std::int64_t now_ns,
                             std::int64_t timeout_ns,
                             std::int64_t session_deadline_ns) {
  if (timeout_ns > std::numeric_limits<std::int64_t>::max() - now_ns) {
    return session_deadline_ns;
  }
  return std::min(now_ns + timeout_ns, session_deadline_ns);
}

std::string rawHex(const std::uint8_t* data, std::size_t size) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0U; index < size; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return stream.str();
}

void appendDouble(std::ostringstream* stream, double value) {
  *stream << std::setprecision(17) << value;
}

void updateParserStatistics(const FeedbackStreamParser& parser,
                            FeedbackCaptureStatistics* statistics) {
  const ParserStatistics& parser_statistics = parser.statistics();
  statistics->valid_frames = parser_statistics.valid_frames;
  statistics->checksum_failures = parser_statistics.checksum_failures;
  statistics->framing_failures = parser_statistics.framing_failures;
  statistics->discarded_bytes = parser_statistics.discarded_bytes;
  statistics->trailing_buffered_bytes = parser.bufferedBytes();
}

void updateRates(FeedbackCaptureStatistics* statistics) {
  const std::int64_t elapsed_ns =
      statistics->ended_monotonic_ns - statistics->started_monotonic_ns;
  if (elapsed_ns > 0) {
    statistics->capture_average_rate_hz =
        static_cast<double>(statistics->valid_frames) * 1.0e9 /
        static_cast<double>(elapsed_ns);
  }
  const std::int64_t interframe_elapsed_ns =
      statistics->last_frame_receipt_monotonic_ns -
      statistics->first_frame_receipt_monotonic_ns;
  if (statistics->valid_frames >= 2U && interframe_elapsed_ns > 0) {
    statistics->interframe_rate_hz =
        static_cast<double>(statistics->valid_frames - 1U) * 1.0e9 /
        static_cast<double>(interframe_elapsed_ns);
  }
}

}  // namespace

const char* feedbackCaptureStatusName(FeedbackCaptureStatus status) {
  switch (status) {
    case FeedbackCaptureStatus::kCompleted:
      return "completed";
    case FeedbackCaptureStatus::kInvalidArgument:
      return "invalid_argument";
    case FeedbackCaptureStatus::kClockInvalid:
      return "clock_invalid";
    case FeedbackCaptureStatus::kDisconnected:
      return "disconnected";
    case FeedbackCaptureStatus::kTransportError:
      return "transport_error";
    case FeedbackCaptureStatus::kRecordError:
      return "record_error";
  }
  return "unknown";
}

const char* transportStatusName(TransportStatus status) {
  switch (status) {
    case TransportStatus::kOk:
      return "ok";
    case TransportStatus::kWouldBlock:
      return "would_block";
    case TransportStatus::kDeadlineExceeded:
      return "deadline_exceeded";
    case TransportStatus::kDisconnected:
      return "disconnected";
    case TransportStatus::kDisabled:
      return "disabled";
    case TransportStatus::kInvalidArgument:
      return "invalid_argument";
    case TransportStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

std::string feedbackRawChunkRecordJson(const std::uint8_t* data,
                                       std::size_t size,
                                       std::int64_t receipt_monotonic_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kCaptureSchema
         << "\",\"record_type\":\"raw_chunk\","
         << "\"receipt_clock\":\"CLOCK_MONOTONIC\","
         << "\"receipt_monotonic_ns\":" << receipt_monotonic_ns
         << ",\"byte_count\":" << size << ",\"raw_hex\":\"";
  if (data != nullptr) {
    stream << rawHex(data, size);
  }
  stream << "\"}";
  return stream.str();
}

std::string feedbackFrameRecordJson(const ParsedFeedbackFrame& frame,
                                    std::int64_t receipt_monotonic_ns) {
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kCaptureSchema
         << "\",\"record_type\":\"frame\","
         << "\"receipt_clock\":\"CLOCK_MONOTONIC\","
         << "\"receipt_monotonic_ns\":" << receipt_monotonic_ns
         << ",\"raw_hex\":\""
         << rawHex(frame.raw_frame.data(), frame.raw_frame.size())
         << "\",\"parsed\":{";
  stream << "\"composite_stop_flag_raw\":"
         << static_cast<unsigned int>(
                frame.candidate.composite_stop_flag_raw)
         << ",\"control_allowed\":"
         << (frame.candidate.control_allowed ? "true" : "false")
         << ",\"control_inhibited\":"
         << (frame.candidate.control_inhibited ? "true" : "false")
         << ",\"vcu_ack_available\":false,"
         << "\"specific_fault_available\":false,"
         << "\"command_echo_available\":false,"
         << "\"forward_speed_mps\":";
  appendDouble(&stream, frame.candidate.forward_speed_mps);
  stream << ",\"lateral_speed_mps\":";
  appendDouble(&stream, frame.candidate.lateral_speed_mps);
  stream << ",\"yaw_rate_radps\":";
  appendDouble(&stream, frame.candidate.yaw_rate_radps);
  stream << ",\"linear_acceleration_mps2\":[";
  for (std::size_t index = 0U;
       index < frame.candidate.linear_acceleration_mps2.size(); ++index) {
    if (index != 0U) {
      stream << ',';
    }
    appendDouble(&stream,
                 frame.candidate.linear_acceleration_mps2[index]);
  }
  stream << "],\"angular_velocity_radps\":[";
  for (std::size_t index = 0U;
       index < frame.candidate.angular_velocity_radps.size(); ++index) {
    if (index != 0U) {
      stream << ',';
    }
    appendDouble(&stream, frame.candidate.angular_velocity_radps[index]);
  }
  stream << "],\"supply_voltage_v\":";
  appendDouble(&stream, frame.candidate.supply_voltage_v);
  stream << ",\"source_time_available\":"
         << (frame.candidate.source_time_available ? "true" : "false")
         << ",\"source_time_ns\":" << frame.candidate.source_time_ns
         << "}}";
  return stream.str();
}

std::string feedbackSummaryRecordJson(const FeedbackCaptureResult& result) {
  const FeedbackCaptureStatistics& statistics = result.statistics;
  std::ostringstream stream;
  stream << "{\"schema\":\"" << kCaptureSchema
         << "\",\"record_type\":\"summary\",\"status\":\""
         << feedbackCaptureStatusName(result.status)
         << "\",\"terminal_transport_status\":\""
         << transportStatusName(result.terminal_transport_status)
         << "\",\"os_error\":" << result.os_error
         << ",\"statistics\":{\"raw_bytes\":" << statistics.raw_bytes
         << ",\"read_calls\":" << statistics.read_calls
         << ",\"read_timeouts\":" << statistics.read_timeouts
         << ",\"valid_frames\":" << statistics.valid_frames
         << ",\"checksum_failures\":" << statistics.checksum_failures
         << ",\"framing_failures\":" << statistics.framing_failures
         << ",\"discarded_bytes\":" << statistics.discarded_bytes
         << ",\"trailing_buffered_bytes\":"
         << statistics.trailing_buffered_bytes
         << ",\"started_monotonic_ns\":"
         << statistics.started_monotonic_ns
         << ",\"ended_monotonic_ns\":" << statistics.ended_monotonic_ns
         << ",\"first_frame_receipt_monotonic_ns\":"
         << statistics.first_frame_receipt_monotonic_ns
         << ",\"last_frame_receipt_monotonic_ns\":"
         << statistics.last_frame_receipt_monotonic_ns
         << ",\"capture_average_rate_hz\":";
  appendDouble(&stream, statistics.capture_average_rate_hz);
  stream << ",\"interframe_rate_hz\":";
  appendDouble(&stream, statistics.interframe_rate_hz);
  stream << "}}";
  return stream.str();
}

FeedbackCaptureResult FeedbackCaptureSession::run(
    ByteTransport* transport, const FeedbackCaptureConfig& config,
    const FeedbackCaptureOperations& operations) {
  FeedbackCaptureResult result;
  if (transport == nullptr || !captureConfigIsValid(config) ||
      !operations.monotonic_now_ns || !operations.record_json_line) {
    return result;
  }
  if (!transport->isConnected()) {
    result.status = FeedbackCaptureStatus::kDisconnected;
    result.terminal_transport_status = TransportStatus::kDisconnected;
    return result;
  }

  const std::int64_t started_ns = operations.monotonic_now_ns();
  result.statistics.started_monotonic_ns = started_ns;
  result.statistics.ended_monotonic_ns = started_ns;
  if (started_ns <= 0 ||
      config.duration_ns >
          std::numeric_limits<std::int64_t>::max() - started_ns) {
    result.status = FeedbackCaptureStatus::kClockInvalid;
    return result;
  }
  const std::int64_t session_deadline_ns = started_ns + config.duration_ns;
  std::int64_t last_clock_ns = started_ns;
  FeedbackStreamParser parser;
  std::array<std::uint8_t, kMaximumFeedbackCaptureReadBytes> buffer{};
  result.status = FeedbackCaptureStatus::kCompleted;

  for (;;) {
    const std::int64_t before_read_ns = operations.monotonic_now_ns();
    if (before_read_ns <= 0 || before_read_ns < last_clock_ns) {
      result.status = FeedbackCaptureStatus::kClockInvalid;
      result.statistics.ended_monotonic_ns = before_read_ns;
      break;
    }
    if (before_read_ns >= session_deadline_ns) {
      result.statistics.ended_monotonic_ns = before_read_ns;
      break;
    }
    const std::int64_t read_deadline_ns = boundedDeadline(
        before_read_ns, config.read_timeout_ns, session_deadline_ns);
    ++result.statistics.read_calls;
    const IoResult read_result = transport->readSome(
        buffer.data(), config.read_buffer_bytes, read_deadline_ns);
    const std::int64_t receipt_ns = operations.monotonic_now_ns();
    if (receipt_ns <= 0 || receipt_ns < before_read_ns) {
      result.status = FeedbackCaptureStatus::kClockInvalid;
      result.statistics.ended_monotonic_ns = receipt_ns;
      break;
    }
    last_clock_ns = receipt_ns;
    result.statistics.ended_monotonic_ns = receipt_ns;
    result.terminal_transport_status = read_result.status;
    result.os_error = read_result.os_error;

    if (read_result.status == TransportStatus::kOk) {
      if (read_result.transferred == 0U ||
          read_result.transferred > config.read_buffer_bytes) {
        result.status = FeedbackCaptureStatus::kTransportError;
        break;
      }
      result.statistics.raw_bytes += read_result.transferred;
      if (!operations.record_json_line(feedbackRawChunkRecordJson(
              buffer.data(), read_result.transferred, receipt_ns))) {
        result.status = FeedbackCaptureStatus::kRecordError;
        break;
      }
      const std::vector<ParsedFeedbackFrame> frames = parser.consumeFrames(
          buffer.data(), read_result.transferred);
      updateParserStatistics(parser, &result.statistics);
      for (const ParsedFeedbackFrame& frame : frames) {
        if (result.statistics.first_frame_receipt_monotonic_ns == 0) {
          result.statistics.first_frame_receipt_monotonic_ns = receipt_ns;
        }
        result.statistics.last_frame_receipt_monotonic_ns = receipt_ns;
        if (!operations.record_json_line(
                feedbackFrameRecordJson(frame, receipt_ns))) {
          result.status = FeedbackCaptureStatus::kRecordError;
          break;
        }
      }
      if (result.status != FeedbackCaptureStatus::kCompleted) {
        break;
      }
      continue;
    }
    if (read_result.status == TransportStatus::kDeadlineExceeded ||
        read_result.status == TransportStatus::kWouldBlock) {
      ++result.statistics.read_timeouts;
      continue;
    }
    if (read_result.status == TransportStatus::kDisconnected) {
      result.status = FeedbackCaptureStatus::kDisconnected;
      break;
    }
    result.status = FeedbackCaptureStatus::kTransportError;
    break;
  }

  updateParserStatistics(parser, &result.statistics);
  updateRates(&result.statistics);
  if (result.status != FeedbackCaptureStatus::kRecordError &&
      !operations.record_json_line(feedbackSummaryRecordJson(result))) {
    result.status = FeedbackCaptureStatus::kRecordError;
  }
  return result;
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
