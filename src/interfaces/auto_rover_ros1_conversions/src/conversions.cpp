#include "auto_rover_ros1_conversions/conversions.hpp"

#include <algorithm>
#include <cstdint>

#include <ros/duration.h>
#include <ros/time.h>

#include "auto_rover_core/geometry.hpp"

namespace auto_rover_ros1 {
namespace {

constexpr std::uint32_t kSupportedSchemaVersion = 1U;
constexpr std::uint32_t kEgoKnownValidMask =
    auto_rover::EgoState::kPoseCovarianceValid |
    auto_rover::EgoState::kTwistValid |
    auto_rover::EgoState::kTwistCovarianceValid;
constexpr std::uint32_t kChassisKnownValidMask =
    auto_rover::ChassisState::kMeasuredSpeedValid |
    auto_rover::ChassisState::kYawRateValid |
    auto_rover::ChassisState::kSteeringAngleValid |
    auto_rover::ChassisState::kGearValid |
    auto_rover::ChassisState::kControlEnabledValid |
    auto_rover::ChassisState::kFaultValid |
    auto_rover::ChassisState::kVoltageValid;

bool recognizedTimeSource(std::uint8_t value) {
  return value >= static_cast<std::uint8_t>(auto_rover::TimeSource::kSampleTime) &&
         value <= static_cast<std::uint8_t>(auto_rover::TimeSource::kReceiptTime);
}

bool recognizedDirection(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(auto_rover::Direction::kReverse);
}

bool recognizedGear(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(auto_rover::GearState::kOther);
}

bool recognizedFault(std::uint8_t value) {
  return value <=
         static_cast<std::uint8_t>(auto_rover::FaultState::kEmergencyStop);
}

bool recognizedSafetyMode(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(
                      auto_rover::SafetyMode::kEmergencyStopLatched);
}

bool recognizedStopReason(std::uint8_t value) {
  return value <=
         static_cast<std::uint8_t>(auto_rover::StopReason::kRecoveryPending);
}

ros::Time toRosTime(std::int64_t nanoseconds) {
  ros::Time output;
  if (nanoseconds > 0) {
    output.fromNSec(static_cast<std::uint64_t>(nanoseconds));
  }
  return output;
}

ros::Duration toRosDuration(std::int64_t nanoseconds) {
  ros::Duration output;
  if (nanoseconds > 0) {
    output.fromNSec(nanoseconds);
  }
  return output;
}

std::int64_t toNanoseconds(const ros::Time& value) {
  return static_cast<std::int64_t>(value.toNSec());
}

std::int64_t toNanoseconds(const ros::Duration& value) {
  return value.toNSec();
}

geometry_msgs::Vector3 toRos(const auto_rover::Vector3& value) {
  geometry_msgs::Vector3 output;
  output.x = value.x;
  output.y = value.y;
  output.z = value.z;
  return output;
}

auto_rover::Vector3 toCore(const geometry_msgs::Vector3& message) {
  return {message.x, message.y, message.z};
}

geometry_msgs::Point toRosPoint(const auto_rover::Vector3& value) {
  geometry_msgs::Point output;
  output.x = value.x;
  output.y = value.y;
  output.z = value.z;
  return output;
}

auto_rover::Vector3 toCore(const geometry_msgs::Point& message) {
  return {message.x, message.y, message.z};
}

geometry_msgs::Quaternion toRos(const auto_rover::Quaternion& value) {
  geometry_msgs::Quaternion output;
  output.x = value.x;
  output.y = value.y;
  output.z = value.z;
  output.w = value.w;
  return output;
}

auto_rover::Quaternion toCore(const geometry_msgs::Quaternion& message) {
  return {message.x, message.y, message.z, message.w};
}

std_msgs::Header header(std::int64_t stamp_ns, const std::string& frame_id) {
  std_msgs::Header output;
  output.stamp = toRosTime(stamp_ns);
  output.frame_id = frame_id;
  return output;
}

}  // namespace

auto_rover_interfaces::EgoState toRos(const auto_rover::EgoState& value) {
  auto_rover_interfaces::EgoState output;
  output.header = header(value.stamp_ns, value.frame_id);
  output.schema_version = 1U;
  output.state_id = value.state_id;
  output.time_source = static_cast<std::uint8_t>(value.time_source);
  output.reference_frame = value.reference_frame;
  output.pose.pose.position = toRosPoint(value.pose.position);
  output.pose.pose.orientation = toRos(value.pose.orientation);
  std::copy(value.pose_covariance.begin(), value.pose_covariance.end(),
            output.pose.covariance.begin());
  output.twist.twist.linear = toRos(value.twist.linear);
  output.twist.twist.angular = toRos(value.twist.angular);
  std::copy(value.twist_covariance.begin(), value.twist_covariance.end(),
            output.twist.covariance.begin());
  output.valid_mask = value.valid_mask;
  output.source_id = value.source_id;
  output.valid = value.valid;
  return output;
}

auto_rover::EgoState toCore(const auto_rover_interfaces::EgoState& message) {
  auto_rover::EgoState output;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.frame_id = message.header.frame_id;
  output.state_id = message.state_id;
  output.time_source = static_cast<auto_rover::TimeSource>(message.time_source);
  output.reference_frame = message.reference_frame;
  output.pose.position = toCore(message.pose.pose.position);
  output.pose.orientation = toCore(message.pose.pose.orientation);
  std::copy(message.pose.covariance.begin(), message.pose.covariance.end(),
            output.pose_covariance.begin());
  output.twist.linear = toCore(message.twist.twist.linear);
  output.twist.angular = toCore(message.twist.twist.angular);
  std::copy(message.twist.covariance.begin(), message.twist.covariance.end(),
            output.twist_covariance.begin());
  output.valid_mask = message.valid_mask;
  output.source_id = message.source_id;
  output.valid = message.valid &&
                 message.schema_version == kSupportedSchemaVersion &&
                 recognizedTimeSource(message.time_source) &&
                 (message.valid_mask & ~kEgoKnownValidMask) == 0U;
  return output;
}

auto_rover_interfaces::RoutePlan toRos(const auto_rover::RoutePlan& value) {
  auto_rover_interfaces::RoutePlan output;
  output.header = header(value.stamp_ns, value.frame_id);
  output.schema_version = value.schema_version;
  output.route_id = value.route_id;
  output.plan_version = value.plan_version;
  output.loop = value.loop;
  output.waypoints.reserve(value.waypoints.size());
  for (const auto& point : value.waypoints) {
    auto_rover_interfaces::RouteWaypoint converted;
    converted.x_m = point.x_m;
    converted.y_m = point.y_m;
    converted.yaw_rad = auto_rover::normalizeAngle(point.yaw_rad);
    converted.speed_mps = point.speed_mps;
    output.waypoints.push_back(converted);
  }
  return output;
}

auto_rover::RoutePlan toCore(const auto_rover_interfaces::RoutePlan& message) {
  auto_rover::RoutePlan output;
  output.schema_version = message.schema_version;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.frame_id = message.header.frame_id;
  output.route_id = message.route_id;
  output.plan_version = message.plan_version;
  output.loop = message.loop;
  output.waypoints.reserve(message.waypoints.size());
  for (const auto& point : message.waypoints) {
    output.waypoints.push_back(
        {point.x_m, point.y_m, auto_rover::normalizeAngle(point.yaw_rad),
         point.speed_mps});
  }
  return output;
}

auto_rover_interfaces::Trajectory toRos(const auto_rover::Trajectory& value) {
  auto_rover_interfaces::Trajectory output;
  output.header = header(value.stamp_ns, value.frame_id);
  output.schema_version = 1U;
  output.trajectory_id = value.trajectory_id;
  output.route_id = value.route_id;
  output.plan_version = value.plan_version;
  output.vehicle_profile_id = value.vehicle_profile_id;
  output.valid_for = toRosDuration(value.valid_for_ns);
  output.completion_behavior =
      static_cast<std::uint8_t>(value.completion_behavior);
  output.valid = value.valid;
  output.points.reserve(value.points.size());
  for (const auto& point : value.points) {
    auto_rover_interfaces::TrajectoryPoint converted;
    converted.x_m = point.x_m;
    converted.y_m = point.y_m;
    converted.yaw_rad = auto_rover::normalizeAngle(point.yaw_rad);
    converted.curvature_inv_m = point.curvature_inv_m;
    converted.target_speed_mps = point.target_speed_mps;
    converted.arc_length_m = point.arc_length_m;
    converted.direction = static_cast<std::uint8_t>(point.direction);
    output.points.push_back(converted);
  }
  return output;
}

auto_rover::Trajectory toCore(
    const auto_rover_interfaces::Trajectory& message) {
  auto_rover::Trajectory output;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.frame_id = message.header.frame_id;
  output.trajectory_id = message.trajectory_id;
  output.route_id = message.route_id;
  output.plan_version = message.plan_version;
  output.vehicle_profile_id = message.vehicle_profile_id;
  output.valid_for_ns = toNanoseconds(message.valid_for);
  output.completion_behavior =
      static_cast<auto_rover::CompletionBehavior>(message.completion_behavior);
  bool enums_recognized =
      message.completion_behavior == static_cast<std::uint8_t>(
                                         auto_rover::CompletionBehavior::kStopAndHold);
  output.points.reserve(message.points.size());
  for (const auto& point : message.points) {
    enums_recognized = enums_recognized && recognizedDirection(point.direction);
    output.points.push_back(
        {point.x_m, point.y_m, auto_rover::normalizeAngle(point.yaw_rad),
         point.curvature_inv_m,
         point.target_speed_mps, point.arc_length_m,
         static_cast<auto_rover::Direction>(point.direction)});
  }
  output.valid = message.valid &&
                 message.schema_version == kSupportedSchemaVersion &&
                 enums_recognized;
  return output;
}

auto_rover_interfaces::MotionReference toRos(
    const auto_rover::MotionReference& value) {
  auto_rover_interfaces::MotionReference output;
  output.header = header(value.stamp_ns, value.frame_id);
  output.schema_version = 1U;
  output.command_id = value.command_id;
  output.producer_generation_id = value.producer_generation_id;
  output.trajectory_id = value.trajectory_id;
  output.direction = static_cast<std::uint8_t>(value.direction);
  output.target_speed_mps = value.target_speed_mps;
  output.target_curvature_inv_m = value.target_curvature_inv_m;
  output.valid_for = toRosDuration(value.valid_for_ns);
  output.route_complete = value.route_complete;
  output.valid = value.valid;
  return output;
}

auto_rover::MotionReference toCore(
    const auto_rover_interfaces::MotionReference& message) {
  auto_rover::MotionReference output;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.frame_id = message.header.frame_id;
  output.command_id = message.command_id;
  output.producer_generation_id = message.producer_generation_id;
  output.trajectory_id = message.trajectory_id;
  output.direction = static_cast<auto_rover::Direction>(message.direction);
  output.target_speed_mps = message.target_speed_mps;
  output.target_curvature_inv_m = message.target_curvature_inv_m;
  output.valid_for_ns = toNanoseconds(message.valid_for);
  output.route_complete = message.route_complete;
  output.valid = message.valid &&
                 message.schema_version == kSupportedSchemaVersion &&
                 recognizedDirection(message.direction);
  return output;
}

auto_rover_interfaces::ChassisState toRos(
    const auto_rover::ChassisState& value) {
  auto_rover_interfaces::ChassisState output;
  output.header = header(value.stamp_ns, value.frame_id);
  output.schema_version = 1U;
  output.state_id = value.state_id;
  output.time_source = static_cast<std::uint8_t>(value.time_source);
  output.measured_speed_mps = value.measured_speed_mps;
  output.yaw_rate_radps = value.yaw_rate_radps;
  output.steering_tire_angle_rad = value.steering_tire_angle_rad;
  output.gear_state = static_cast<std::uint8_t>(value.gear_state);
  output.control_enabled = value.control_enabled;
  output.fault_state = static_cast<std::uint8_t>(value.fault_state);
  output.supply_voltage_v = value.supply_voltage_v;
  output.valid_mask = value.valid_mask;
  output.source_id = value.source_id;
  output.valid = value.valid;
  return output;
}

auto_rover::ChassisState toCore(
    const auto_rover_interfaces::ChassisState& message) {
  auto_rover::ChassisState output;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.frame_id = message.header.frame_id;
  output.state_id = message.state_id;
  output.time_source = static_cast<auto_rover::TimeSource>(message.time_source);
  output.measured_speed_mps = message.measured_speed_mps;
  output.yaw_rate_radps = message.yaw_rate_radps;
  output.steering_tire_angle_rad = message.steering_tire_angle_rad;
  output.gear_state = static_cast<auto_rover::GearState>(message.gear_state);
  output.control_enabled = message.control_enabled;
  output.fault_state = static_cast<auto_rover::FaultState>(message.fault_state);
  output.supply_voltage_v = message.supply_voltage_v;
  output.valid_mask = message.valid_mask;
  output.source_id = message.source_id;
  const bool known_mask =
      (message.valid_mask & ~kChassisKnownValidMask) == 0U;
  const bool valid_gear =
      recognizedGear(message.gear_state) &&
      (((message.valid_mask & auto_rover::ChassisState::kGearValid) == 0U) ||
       message.gear_state != static_cast<std::uint8_t>(
                                 auto_rover::GearState::kUnknown));
  const bool valid_fault =
      recognizedFault(message.fault_state) &&
      (((message.valid_mask & auto_rover::ChassisState::kFaultValid) == 0U) ||
       message.fault_state != static_cast<std::uint8_t>(
                                  auto_rover::FaultState::kUnknown));
  output.valid = message.valid &&
                 message.schema_version == kSupportedSchemaVersion &&
                 recognizedTimeSource(message.time_source) && known_mask &&
                 valid_gear && valid_fault &&
                 (!message.control_enabled ||
                  (message.valid_mask &
                   auto_rover::ChassisState::kControlEnabledValid) != 0U);
  return output;
}

auto_rover_interfaces::SafetyState toRos(
    const auto_rover::SafetyState& value) {
  auto_rover_interfaces::SafetyState output;
  output.header = header(value.stamp_ns, std::string());
  output.schema_version = 1U;
  output.state_id = value.state_id;
  output.mode = static_cast<std::uint8_t>(value.mode);
  output.latch_generation = value.latch_generation;
  output.valid_for = toRosDuration(value.valid_for_ns);
  output.reason_codes.reserve(value.reasons.size());
  for (const auto reason : value.reasons) {
    output.reason_codes.push_back(static_cast<std::uint8_t>(reason));
  }
  output.valid = value.valid;
  return output;
}

auto_rover::SafetyState toCore(
    const auto_rover_interfaces::SafetyState& message) {
  auto_rover::SafetyState output;
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.state_id = message.state_id;
  output.mode = static_cast<auto_rover::SafetyMode>(message.mode);
  output.latch_generation = message.latch_generation;
  output.valid_for_ns = toNanoseconds(message.valid_for);
  bool enums_recognized = recognizedSafetyMode(message.mode);
  output.reasons.reserve(message.reason_codes.size());
  for (const auto reason : message.reason_codes) {
    enums_recognized = enums_recognized && recognizedStopReason(reason);
    output.reasons.push_back(static_cast<auto_rover::StopReason>(reason));
  }
  output.valid = message.valid &&
                 message.schema_version == kSupportedSchemaVersion &&
                 enums_recognized;
  return output;
}

auto_rover_interfaces::EmergencyStop toRos(
    const auto_rover::EmergencyStop& value) {
  auto_rover_interfaces::EmergencyStop output;
  output.header = header(value.stamp_ns, std::string());
  output.schema_version = 1U;
  output.request_id = value.request_id;
  output.source_id = value.source_id;
  output.asserted = value.asserted;
  return output;
}

auto_rover::EmergencyStop toCore(
    const auto_rover_interfaces::EmergencyStop& message) {
  auto_rover::EmergencyStop output;
  if (message.schema_version != kSupportedSchemaVersion) {
    return output;
  }
  output.stamp_ns = toNanoseconds(message.header.stamp);
  output.request_id = message.request_id;
  output.source_id = message.source_id;
  output.asserted = message.asserted;
  return output;
}

}  // namespace auto_rover_ros1
