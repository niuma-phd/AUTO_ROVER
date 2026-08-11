#include "auto_rover_vcu_wheeltec_serial/stream_parser.hpp"

#include <algorithm>

namespace auto_rover {
namespace wheeltec_serial {

std::vector<ParsedFeedbackFrame> FeedbackStreamParser::consumeFrames(
    const std::uint8_t* data, std::size_t size) {
  std::vector<ParsedFeedbackFrame> output;
  if (size == 0U) {
    return output;
  }
  if (data == nullptr) {
    statistics_.discarded_bytes += size;
    return output;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  while (!buffer_.empty()) {
    const auto header = std::find(buffer_.begin(), buffer_.end(), kFrameHeader);
    if (header == buffer_.end()) {
      statistics_.discarded_bytes += buffer_.size();
      buffer_.clear();
      break;
    }
    if (header != buffer_.begin()) {
      const std::size_t discard =
          static_cast<std::size_t>(std::distance(buffer_.begin(), header));
      statistics_.discarded_bytes += discard;
      buffer_.erase(buffer_.begin(), header);
    }
    if (buffer_.size() < kFeedbackFrameSize) {
      break;
    }

    const DecodeResult decoded =
        decodeFeedbackFrame(buffer_.data(), kFeedbackFrameSize);
    if (decoded.ok()) {
      ParsedFeedbackFrame parsed;
      std::copy_n(buffer_.begin(), kFeedbackFrameSize,
                  parsed.raw_frame.begin());
      parsed.candidate = decoded.candidate;
      output.push_back(parsed);
      ++statistics_.valid_frames;
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() +
                        static_cast<std::ptrdiff_t>(kFeedbackFrameSize));
      continue;
    }
    if (decoded.error == CodecError::kChecksumMismatch) {
      ++statistics_.checksum_failures;
    } else if (decoded.error ==
               CodecError::kInvalidCompositeStopFlag) {
      ++statistics_.invalid_composite_stop_flags;
      ++statistics_.framing_failures;
    } else {
      ++statistics_.framing_failures;
    }
    ++statistics_.discarded_bytes;
    buffer_.erase(buffer_.begin());
  }
  return output;
}

std::vector<FeedbackCandidate> FeedbackStreamParser::consume(
    const std::uint8_t* data, std::size_t size) {
  const std::vector<ParsedFeedbackFrame> frames = consumeFrames(data, size);
  std::vector<FeedbackCandidate> output;
  output.reserve(frames.size());
  for (const ParsedFeedbackFrame& frame : frames) {
    output.push_back(frame.candidate);
  }
  return output;
}

void FeedbackStreamParser::reset() {
  buffer_.clear();
  statistics_ = ParserStatistics{};
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
