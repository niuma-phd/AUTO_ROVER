#pragma once

#include <cstdint>
#include <functional>

#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"

namespace auto_rover {
namespace wheeltec_serial {

// This guard is prepared before a physical serial path is opened.  Its
// activate() transaction performs no read or evidence callback: the first
// protocol I/O after open is an exact-zero command frame.
struct PhysicalActivationConfig {
  CodecLimits codec_limits{};
  std::int64_t write_timeout_ns{10000000};
  std::int64_t zero_retry_interval_ns{20000000};
  std::uint32_t max_zero_write_attempts{3U};
};

struct PhysicalActivationOperations {
  std::function<std::int64_t()> monotonic_now_ns;
  std::function<bool(std::int64_t)> wait_until_monotonic_ns;
};

enum class PhysicalActivationStatus : std::uint8_t {
  kSuccess = 0,
  kNotPrepared,
  kTransportUnavailable,
  kClockInvalid,
  kRetryWaitFailed,
  kZeroRetriesExhausted,
  kWriteStreamPoisoned,
  kDeadlineMissed,
};

struct PhysicalActivationResult {
  PhysicalActivationStatus status{PhysicalActivationStatus::kNotPrepared};
  IoResult last_io{};
  bool exact_zero_host_write_complete{false};
  bool delivery_unconfirmed{false};
  bool write_stream_poisoned{false};
  std::uint32_t attempts{0U};
  std::int64_t started_monotonic_ns{0};
  std::int64_t completed_monotonic_ns{0};

  bool succeeded() const {
    return status == PhysicalActivationStatus::kSuccess &&
           exact_zero_host_write_complete && !delivery_unconfirmed &&
           !write_stream_poisoned;
  }
};

bool physicalActivationConfigIsValid(
    const PhysicalActivationConfig& config);
const char* physicalActivationStatusName(PhysicalActivationStatus status);

class PreparedPhysicalActivation {
 public:
  PreparedPhysicalActivation(
      const PhysicalActivationConfig& config,
      const PhysicalActivationOperations& operations);

  bool prepared() const { return prepared_; }
  PhysicalActivationResult activate(ByteTransport* transport) const noexcept;

 private:
  PhysicalActivationConfig config_{};
  PhysicalActivationOperations operations_{};
  CommandFrame exact_zero_frame_{};
  bool prepared_{false};
};

}  // namespace wheeltec_serial
}  // namespace auto_rover
