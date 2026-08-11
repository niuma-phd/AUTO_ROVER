#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "auto_rover_vcu_wheeltec_serial/codec.hpp"

namespace auto_rover {
namespace wheeltec_serial {

struct ParserStatistics {
  std::uint64_t valid_frames{0U};
  std::uint64_t checksum_failures{0U};
  std::uint64_t framing_failures{0U};
  std::uint64_t invalid_composite_stop_flags{0U};
  std::uint64_t discarded_bytes{0U};
};

struct ParsedFeedbackFrame {
  FeedbackFrame raw_frame{};
  FeedbackCandidate candidate{};
};

class FeedbackStreamParser {
 public:
  std::vector<ParsedFeedbackFrame> consumeFrames(const std::uint8_t* data,
                                                 std::size_t size);
  std::vector<FeedbackCandidate> consume(const std::uint8_t* data,
                                         std::size_t size);
  void reset();

  std::size_t bufferedBytes() const { return buffer_.size(); }
  const ParserStatistics& statistics() const { return statistics_; }

 private:
  std::vector<std::uint8_t> buffer_;
  ParserStatistics statistics_;
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
