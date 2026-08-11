#include "auto_rover_vcu_wheeltec_serial/codec.hpp"

#include <cmath>
#include <limits>

namespace auto_rover {
namespace wheeltec_serial {
namespace {

constexpr double kWireScale = 1000.0;
constexpr double kAccelerationScale = 1671.84;
constexpr double kGyroscopeScale = 0.00026644;

bool quantizeTowardZero(double value, std::int16_t* output) {
  if (output == nullptr || !std::isfinite(value)) {
    return false;
  }
  const double quantized = std::trunc(value * kWireScale);
  if (!std::isfinite(quantized) ||
      quantized < static_cast<double>(std::numeric_limits<std::int16_t>::min()) ||
      quantized > static_cast<double>(std::numeric_limits<std::int16_t>::max())) {
    return false;
  }
  *output = static_cast<std::int16_t>(quantized);
  return true;
}

void putSignedBigEndian(std::int16_t value, std::uint8_t* high,
                        std::uint8_t* low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  *high = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  *low = static_cast<std::uint8_t>(bits & 0xFFU);
}

std::int16_t readSignedBigEndian(std::uint8_t high, std::uint8_t low) {
  const std::uint16_t bits =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(high) << 8U) |
                                 static_cast<std::uint16_t>(low));
  const std::int32_t value =
      bits <= static_cast<std::uint16_t>(
                  std::numeric_limits<std::int16_t>::max())
          ? static_cast<std::int32_t>(bits)
          : static_cast<std::int32_t>(bits) - 65536;
  return static_cast<std::int16_t>(value);
}

std::uint16_t readUnsignedBigEndian(std::uint8_t high, std::uint8_t low) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low));
}

std::uint8_t xorBytes(const std::uint8_t* data, std::size_t size) {
  std::uint8_t checksum = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    checksum = static_cast<std::uint8_t>(checksum ^ data[index]);
  }
  return checksum;
}

EncodeResult failure(CodecError error) {
  EncodeResult result;
  result.error = error;
  return result;
}

}  // namespace

bool codecLimitsAreValid(const CodecLimits& limits) {
  return std::isfinite(limits.max_forward_speed_mps) &&
         limits.max_forward_speed_mps > 0.0 &&
         limits.max_forward_speed_mps <
             kMaximumConfigurableForwardSpeedMps &&
         std::isfinite(limits.max_abs_curvature_inv_m) &&
         limits.max_abs_curvature_inv_m > 0.0 &&
         limits.max_abs_curvature_inv_m <=
             kPhase1MaximumAbsCurvatureInvM;
}

EncodeResult encodeWireMotion(const WireMotionCandidate& candidate,
                              const CodecLimits& limits) {
  const bool exact_zero = candidate.forward_speed_mps == 0.0 &&
                          candidate.lateral_speed_mps == 0.0 &&
                          candidate.yaw_rate_radps == 0.0;
  // An exact zero is a protocol-level safe-state frame and remains available
  // to emergency shutdown paths even when motion limits are missing.  Invalid
  // limits can never authorize a nonzero wire command, and the adapter rejects
  // such a configuration before accepting commands.
  if (!codecLimitsAreValid(limits) && !exact_zero) {
    return failure(CodecError::kInvalidLimits);
  }
  if (!std::isfinite(candidate.forward_speed_mps) ||
      !std::isfinite(candidate.lateral_speed_mps) ||
      !std::isfinite(candidate.yaw_rate_radps)) {
    return failure(CodecError::kNonFinite);
  }
  if (candidate.forward_speed_mps < 0.0) {
    return failure(CodecError::kNegativeForwardSpeed);
  }
  if (candidate.lateral_speed_mps != 0.0) {
    return failure(CodecError::kLateralMotionUnsupported);
  }
  if (!exact_zero &&
      candidate.forward_speed_mps > limits.max_forward_speed_mps) {
    return failure(CodecError::kForwardSpeedLimit);
  }
  if (candidate.forward_speed_mps == 0.0) {
    if (candidate.yaw_rate_radps != 0.0) {
      return failure(CodecError::kZeroSpeedYawRate);
    }
  } else {
    const double curvature =
        candidate.yaw_rate_radps / candidate.forward_speed_mps;
    if (!std::isfinite(curvature) ||
        std::abs(curvature) > limits.max_abs_curvature_inv_m) {
      return failure(CodecError::kCurvatureLimit);
    }
  }

  std::int16_t forward_wire = 0;
  std::int16_t lateral_wire = 0;
  std::int16_t yaw_wire = 0;
  if (!quantizeTowardZero(candidate.forward_speed_mps, &forward_wire) ||
      !quantizeTowardZero(candidate.lateral_speed_mps, &lateral_wire) ||
      !quantizeTowardZero(candidate.yaw_rate_radps, &yaw_wire)) {
    return failure(CodecError::kWireRange);
  }
  if (candidate.forward_speed_mps > 0.0 && forward_wire == 0) {
    return failure(CodecError::kWireRange);
  }
  if (forward_wire == 0) {
    if (yaw_wire != 0) {
      return failure(CodecError::kZeroSpeedYawRate);
    }
  } else {
    const double quantized_curvature =
        static_cast<double>(yaw_wire) /
        static_cast<double>(forward_wire);
    if (!std::isfinite(quantized_curvature) ||
        std::abs(quantized_curvature) >
            limits.max_abs_curvature_inv_m) {
      return failure(CodecError::kCurvatureLimit);
    }
  }

  EncodeResult result;
  result.frame[0U] = kFrameHeader;
  result.frame[1U] = 0U;
  result.frame[2U] = 0U;
  putSignedBigEndian(forward_wire, &result.frame[3U], &result.frame[4U]);
  putSignedBigEndian(lateral_wire, &result.frame[5U], &result.frame[6U]);
  putSignedBigEndian(yaw_wire, &result.frame[7U], &result.frame[8U]);
  result.frame[9U] = xorBytes(result.frame.data(), 9U);
  result.frame[10U] = kFrameTail;
  return result;
}

EncodeResult encodeExecutionCommand(const VehicleExecutionCommand& command,
                                    const CodecLimits& limits) {
  if (!codecLimitsAreValid(limits)) {
    return failure(CodecError::kInvalidLimits);
  }
  if (!std::isfinite(command.signed_speed_mps) ||
      !std::isfinite(command.curvature_inv_m)) {
    return failure(CodecError::kNonFinite);
  }
  if (command.signed_speed_mps < 0.0) {
    return failure(CodecError::kNegativeForwardSpeed);
  }
  if (command.signed_speed_mps > limits.max_forward_speed_mps) {
    return failure(CodecError::kForwardSpeedLimit);
  }
  if (std::abs(command.curvature_inv_m) >
      limits.max_abs_curvature_inv_m) {
    return failure(CodecError::kCurvatureLimit);
  }
  const bool motion_permitted = command.motion_enabled && !command.hold &&
                                command.stop_reason == StopReason::kNone;
  if (!motion_permitted && command.signed_speed_mps != 0.0) {
    return failure(CodecError::kInhibitedMotion);
  }
  const double speed = motion_permitted ? command.signed_speed_mps : 0.0;
  const double yaw_rate =
      speed == 0.0 ? 0.0 : speed * command.curvature_inv_m;
  return encodeWireMotion(WireMotionCandidate{speed, 0.0, yaw_rate}, limits);
}

DecodeResult decodeFeedbackFrame(const std::uint8_t* data, std::size_t size) {
  DecodeResult result;
  if (data == nullptr || size != kFeedbackFrameSize) {
    result.error = CodecError::kInvalidFrameSize;
    return result;
  }
  if (data[0U] != kFrameHeader) {
    result.error = CodecError::kInvalidHeader;
    return result;
  }
  if (data[23U] != kFrameTail) {
    result.error = CodecError::kInvalidTail;
    return result;
  }
  if (xorBytes(data, 22U) != data[22U]) {
    result.error = CodecError::kChecksumMismatch;
    return result;
  }
  if (data[1U] > 1U) {
    result.error = CodecError::kInvalidCompositeStopFlag;
    return result;
  }

  result.candidate.composite_stop_flag_raw = data[1U];
  result.candidate.control_allowed = data[1U] == 0U;
  result.candidate.control_inhibited = data[1U] == 1U;
  result.candidate.forward_speed_mps =
      static_cast<double>(readSignedBigEndian(data[2U], data[3U])) /
      kWireScale;
  result.candidate.lateral_speed_mps =
      static_cast<double>(readSignedBigEndian(data[4U], data[5U])) /
      kWireScale;
  result.candidate.yaw_rate_radps =
      static_cast<double>(readSignedBigEndian(data[6U], data[7U])) /
      kWireScale;
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const std::size_t offset = 8U + 2U * axis;
    result.candidate.linear_acceleration_mps2[axis] =
        static_cast<double>(readSignedBigEndian(data[offset], data[offset + 1U])) /
        kAccelerationScale;
  }
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    const std::size_t offset = 14U + 2U * axis;
    result.candidate.angular_velocity_radps[axis] =
        static_cast<double>(readSignedBigEndian(data[offset], data[offset + 1U])) *
        kGyroscopeScale;
  }
  result.candidate.supply_voltage_v =
      static_cast<double>(readUnsignedBigEndian(data[20U], data[21U])) /
      kWireScale;
  result.candidate.source_time_available = false;
  result.candidate.source_time_ns = 0;
  return result;
}

}  // namespace wheeltec_serial
}  // namespace auto_rover
