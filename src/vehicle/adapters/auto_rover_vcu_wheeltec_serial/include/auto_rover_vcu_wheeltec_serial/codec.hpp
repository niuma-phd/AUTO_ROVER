#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "auto_rover_core/types.hpp"

namespace auto_rover {
namespace wheeltec_serial {

constexpr std::uint8_t kFrameHeader = 0x7BU;
constexpr std::uint8_t kFrameTail = 0x7DU;
constexpr std::size_t kCommandFrameSize = 11U;
constexpr std::size_t kFeedbackFrameSize = 24U;
// Application configurations must remain strictly below this bound.  The
// Phase-1 commissioning value is supplied by bringup rather than embedded in
// the protocol library.
constexpr double kMaximumConfigurableForwardSpeedMps = 6.0;
constexpr double kPhase1MaximumAbsCurvatureInvM = 1.0 / 0.95;

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;
using FeedbackFrame = std::array<std::uint8_t, kFeedbackFrameSize>;

struct CodecLimits {
  // Zero deliberately makes an omitted application limit fail closed.
  double max_forward_speed_mps{0.0};
  double max_abs_curvature_inv_m{kPhase1MaximumAbsCurvatureInvM};
};

struct WireMotionCandidate {
  double forward_speed_mps{0.0};
  double lateral_speed_mps{0.0};
  double yaw_rate_radps{0.0};
};

enum class CodecError : std::uint8_t {
  kNone = 0,
  kNonFinite,
  kInvalidLimits,
  kNegativeForwardSpeed,
  kLateralMotionUnsupported,
  kForwardSpeedLimit,
  kCurvatureLimit,
  kZeroSpeedYawRate,
  kWireRange,
  kInhibitedMotion,
  kInvalidFrameSize,
  kInvalidHeader,
  kInvalidTail,
  kChecksumMismatch,
  kInvalidCompositeStopFlag,
};

struct EncodeResult {
  CodecError error{CodecError::kNone};
  CommandFrame frame{};

  bool ok() const { return error == CodecError::kNone; }
};

struct FeedbackCandidate {
  // Firmware byte 1 is the current-cycle composite FlagStop value.  It is
  // binary: zero means the low-voltage/physical-enable/self-check/software-stop
  // aggregate currently permits control; one means at least one source
  // inhibits control.  It is not an ACK, a specific fault code, or a command
  // echo.  Defaults are deliberately fail closed for unpopulated candidates.
  std::uint8_t composite_stop_flag_raw{1U};
  bool control_allowed{false};
  bool control_inhibited{true};
  double forward_speed_mps{0.0};
  double lateral_speed_mps{0.0};
  double yaw_rate_radps{0.0};
  std::array<double, 3U> linear_acceleration_mps2{};
  std::array<double, 3U> angular_velocity_radps{};
  double supply_voltage_v{0.0};
  bool source_time_available{false};
  std::int64_t source_time_ns{0};
};

struct DecodeResult {
  CodecError error{CodecError::kNone};
  FeedbackCandidate candidate{};

  bool ok() const { return error == CodecError::kNone; }
};

bool codecLimitsAreValid(const CodecLimits& limits);
EncodeResult encodeWireMotion(const WireMotionCandidate& candidate,
                              const CodecLimits& limits);
EncodeResult encodeExecutionCommand(const VehicleExecutionCommand& command,
                                    const CodecLimits& limits);
DecodeResult decodeFeedbackFrame(const std::uint8_t* data, std::size_t size);

}  // namespace wheeltec_serial
}  // namespace auto_rover
