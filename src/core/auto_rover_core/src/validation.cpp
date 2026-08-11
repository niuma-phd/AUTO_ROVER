#include "auto_rover_core/validation.hpp"

#include <algorithm>
#include <cmath>

#include "auto_rover_core/geometry.hpp"

namespace auto_rover {
namespace {

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::uint32_t kEgoKnownValidMask =
    EgoState::kPoseCovarianceValid | EgoState::kTwistValid |
    EgoState::kTwistCovarianceValid;
constexpr std::uint32_t kChassisKnownValidMask =
    ChassisState::kMeasuredSpeedValid | ChassisState::kYawRateValid |
    ChassisState::kSteeringAngleValid | ChassisState::kGearValid |
    ChassisState::kControlEnabledValid | ChassisState::kFaultValid |
    ChassisState::kVoltageValid;

bool nonEmpty(const std::string& value) { return !value.empty(); }

bool finiteCovariance(const std::array<double, 36U>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return isFinite(value); });
}

bool quaternionIsNormalized(const Quaternion& value) {
  if (!isFinite(value)) {
    return false;
  }
  const double norm_squared = value.x * value.x + value.y * value.y +
                              value.z * value.z + value.w * value.w;
  return std::abs(norm_squared - 1.0) <= 1e-6;
}

bool angleIsNormalized(double value) {
  return isFinite(value) && value >= -kPi && value < kPi;
}

bool isKnownTimeSource(TimeSource value) {
  return value == TimeSource::kSampleTime ||
         value == TimeSource::kPublishTime ||
         value == TimeSource::kReceiptTime;
}

bool isKnownGearState(GearState value) {
  return value == GearState::kPark || value == GearState::kReverse ||
         value == GearState::kNeutral || value == GearState::kDrive ||
         value == GearState::kLow || value == GearState::kOther;
}

bool isKnownFaultState(FaultState value) {
  return value == FaultState::kOk || value == FaultState::kDegraded ||
         value == FaultState::kFault ||
         value == FaultState::kEmergencyStop;
}

bool isKnownSafetyMode(SafetyMode value) {
  return value == SafetyMode::kBootInhibited ||
         value == SafetyMode::kDisarmed ||
         value == SafetyMode::kRecoveryInhibited ||
         value == SafetyMode::kArmed ||
         value == SafetyMode::kFaultInhibited ||
         value == SafetyMode::kEmergencyStopLatched;
}

bool isKnownStopReason(StopReason value) {
  switch (value) {
    case StopReason::kNone:
    case StopReason::kActuationDisabled:
    case StopReason::kDisarmed:
    case StopReason::kInvalidInput:
    case StopReason::kStaleLocalization:
    case StopReason::kStaleTrajectory:
    case StopReason::kStaleMotionReference:
    case StopReason::kStaleChassisState:
    case StopReason::kLimitViolation:
    case StopReason::kUnsupportedDirection:
    case StopReason::kEmergencyStop:
    case StopReason::kBackendDisconnected:
    case StopReason::kBackendWatchdog:
    case StopReason::kRouteInvalid:
    case StopReason::kRecoveryPending:
      return true;
  }
  return false;
}

ValidationResult validateDirection(Direction direction,
                                   const VehicleProfile& profile) {
  if (direction == Direction::kForward) {
    return ValidationResult::success();
  }
  if (direction == Direction::kReverse && profile.reverse_supported) {
    return ValidationResult::success();
  }
  return ValidationResult::failure("unsupported direction");
}

}  // namespace

double maxAbsCurvature(const VehicleProfile& profile) {
  if (!isFinite(profile.min_turning_radius_m) ||
      profile.min_turning_radius_m <= 0.0) {
    return 0.0;
  }
  return 1.0 / profile.min_turning_radius_m;
}

ValidationResult validateVehicleProfile(const VehicleProfile& profile) {
  if (profile.schema_version != 1U) {
    return ValidationResult::failure("unsupported vehicle profile schema");
  }
  if (!nonEmpty(profile.profile_id) || !nonEmpty(profile.reference_frame)) {
    return ValidationResult::failure("vehicle profile identity is missing");
  }
  if (profile.kinematic_model != KinematicModel::kAckermannBicycle) {
    return ValidationResult::failure("unsupported kinematic model");
  }
  if (profile.direction_capability !=
          DirectionCapability::kSignedSpeedDirection &&
      profile.direction_capability !=
          DirectionCapability::kConfirmedGearDirection) {
    return ValidationResult::failure("direction capability is not selected");
  }
  if (!isFinite(profile.wheelbase_m) || profile.wheelbase_m <= 0.0 ||
      !isFinite(profile.max_forward_speed_mps) ||
      profile.max_forward_speed_mps <= 0.0 ||
      !isFinite(profile.max_longitudinal_accel_mps2) ||
      profile.max_longitudinal_accel_mps2 <= 0.0 ||
      !isFinite(profile.min_turning_radius_m) ||
      profile.min_turning_radius_m <= 0.0) {
    return ValidationResult::failure("vehicle profile has invalid geometry or limits");
  }
  return ValidationResult::success();
}

ValidationResult validateNucPhase1Profile(const VehicleProfile& profile) {
  const ValidationResult general = validateVehicleProfile(profile);
  if (!general.ok) {
    return general;
  }
  if (profile.reference_frame != "rear_axle_center") {
    return ValidationResult::failure("NUC profile requires rear axle control point");
  }
  if (profile.reverse_supported) {
    return ValidationResult::failure("NUC phase-1 profile does not support reverse");
  }
  if (profile.direction_capability !=
      DirectionCapability::kSignedSpeedDirection) {
    return ValidationResult::failure(
        "NUC phase-1 profile requires signed-speed direction capability");
  }
  if (profile.max_forward_speed_mps > 0.50 + kTolerance ||
      profile.max_longitudinal_accel_mps2 > 0.20 + kTolerance ||
      profile.min_turning_radius_m + kTolerance < 0.95) {
    return ValidationResult::failure("NUC phase-1 approved limit exceeded");
  }
  return ValidationResult::success();
}

ValidationResult validateEgoState(const EgoState& ego,
                                  const std::string& expected_frame,
                                  const std::string& expected_reference_frame) {
  if (!ego.valid) {
    return ValidationResult::failure("ego state is invalid");
  }
  if (ego.stamp_ns <= 0 || ego.state_id == 0U || !nonEmpty(ego.source_id)) {
    return ValidationResult::failure("ego state identity or time is invalid");
  }
  if (ego.frame_id != expected_frame ||
      ego.reference_frame != expected_reference_frame) {
    return ValidationResult::failure("ego state frame mismatch");
  }
  if (!isKnownTimeSource(ego.time_source) || !isFinite(ego.pose) ||
      !quaternionIsNormalized(ego.pose.orientation)) {
    return ValidationResult::failure("ego state pose or time source is invalid");
  }
  if ((ego.valid_mask & ~kEgoKnownValidMask) != 0U ||
      ((ego.valid_mask & EgoState::kTwistCovarianceValid) != 0U &&
       (ego.valid_mask & EgoState::kTwistValid) == 0U)) {
    return ValidationResult::failure("ego state validity mask is unsupported");
  }
  if ((ego.valid_mask & EgoState::kTwistValid) != 0U &&
      !isFinite(ego.twist)) {
    return ValidationResult::failure("ego state valid twist is non-finite");
  }
  if ((ego.valid_mask & EgoState::kPoseCovarianceValid) != 0U &&
      !finiteCovariance(ego.pose_covariance)) {
    return ValidationResult::failure("ego pose covariance is non-finite");
  }
  if ((ego.valid_mask & EgoState::kTwistCovarianceValid) != 0U &&
      !finiteCovariance(ego.twist_covariance)) {
    return ValidationResult::failure("ego twist covariance is non-finite");
  }
  return ValidationResult::success();
}

ValidationResult validateRoutePlan(const RoutePlan& route,
                                   const VehicleProfile& profile,
                                   const std::string& expected_frame) {
  const ValidationResult profile_result = validateNucPhase1Profile(profile);
  if (!profile_result.ok) {
    return profile_result;
  }
  if (route.schema_version != 1U || route.stamp_ns <= 0 ||
      !nonEmpty(route.route_id) || route.plan_version == 0U) {
    return ValidationResult::failure("route identity, schema, or time is invalid");
  }
  if (route.frame_id != expected_frame) {
    return ValidationResult::failure("route frame mismatch");
  }
  if (route.loop) {
    return ValidationResult::failure("loop routes are disabled for commissioning");
  }
  if (route.waypoints.size() < 2U) {
    return ValidationResult::failure("route needs at least two waypoints");
  }
  for (std::size_t index = 0U; index < route.waypoints.size(); ++index) {
    const RouteWaypoint& point = route.waypoints[index];
    if (!isFinite(point.x_m) || !isFinite(point.y_m) ||
        !angleIsNormalized(point.yaw_rad) || !isFinite(point.speed_mps)) {
      return ValidationResult::failure("route waypoint is non-finite");
    }
    if (point.speed_mps < 0.0 ||
        point.speed_mps > profile.max_forward_speed_mps + kTolerance) {
      return ValidationResult::failure("route waypoint speed is outside profile");
    }
    if (index > 0U) {
      const RouteWaypoint& previous = route.waypoints[index - 1U];
      if (distance2d(previous.x_m, previous.y_m, point.x_m, point.y_m) <=
          kTolerance) {
        return ValidationResult::failure("adjacent route waypoints are duplicates");
      }
    }
  }
  return ValidationResult::success();
}

ValidationResult validateTrajectory(const Trajectory& trajectory,
                                    const VehicleProfile& profile,
                                    const std::string& expected_frame) {
  const ValidationResult profile_result = validateNucPhase1Profile(profile);
  if (!profile_result.ok) {
    return profile_result;
  }
  if (!trajectory.valid || trajectory.stamp_ns <= 0 ||
      trajectory.valid_for_ns <= 0 || !nonEmpty(trajectory.trajectory_id) ||
      !nonEmpty(trajectory.route_id) || trajectory.plan_version == 0U ||
      trajectory.vehicle_profile_id != profile.profile_id ||
      trajectory.completion_behavior != CompletionBehavior::kStopAndHold) {
    return ValidationResult::failure("trajectory identity or validity is invalid");
  }
  if (trajectory.frame_id != expected_frame || trajectory.points.size() < 2U) {
    return ValidationResult::failure("trajectory frame or point count is invalid");
  }
  const double curvature_limit = maxAbsCurvature(profile);
  for (std::size_t index = 0U; index < trajectory.points.size(); ++index) {
    const TrajectoryPoint& point = trajectory.points[index];
    if (!isFinite(point.x_m) || !isFinite(point.y_m) ||
        !angleIsNormalized(point.yaw_rad) ||
        !isFinite(point.curvature_inv_m) ||
        !isFinite(point.target_speed_mps) || !isFinite(point.arc_length_m)) {
      return ValidationResult::failure("trajectory point is non-finite");
    }
    const ValidationResult direction = validateDirection(point.direction, profile);
    if (!direction.ok) {
      return direction;
    }
    if (std::abs(point.curvature_inv_m) > curvature_limit + kTolerance ||
        point.target_speed_mps < 0.0 ||
        point.target_speed_mps > profile.max_forward_speed_mps + kTolerance ||
        point.arc_length_m < 0.0) {
      return ValidationResult::failure("trajectory point exceeds profile");
    }
    if (index > 0U &&
        point.arc_length_m <= trajectory.points[index - 1U].arc_length_m) {
      return ValidationResult::failure("trajectory arc length is not increasing");
    }
  }
  return ValidationResult::success();
}

ValidationResult validateMotionReference(const MotionReference& motion,
                                         const VehicleProfile& profile) {
  const ValidationResult profile_result = validateNucPhase1Profile(profile);
  if (!profile_result.ok) {
    return profile_result;
  }
  if (!motion.valid || motion.stamp_ns <= 0 || motion.command_id == 0U ||
      !nonEmpty(motion.producer_generation_id) ||
      !nonEmpty(motion.trajectory_id) || motion.valid_for_ns <= 0 ||
      motion.frame_id != profile.reference_frame) {
    return ValidationResult::failure("motion reference identity or validity is invalid");
  }
  const ValidationResult direction = validateDirection(motion.direction, profile);
  if (!direction.ok) {
    return direction;
  }
  if (!isFinite(motion.target_speed_mps) ||
      !isFinite(motion.target_curvature_inv_m) ||
      motion.target_speed_mps < 0.0 ||
      motion.target_speed_mps > profile.max_forward_speed_mps + kTolerance ||
      std::abs(motion.target_curvature_inv_m) >
          maxAbsCurvature(profile) + kTolerance) {
    return ValidationResult::failure("motion reference exceeds profile");
  }
  return ValidationResult::success();
}

ValidationResult validateChassisState(const ChassisState& chassis,
                                      const VehicleProfile& profile,
                                      bool measured_speed_required) {
  const ValidationResult profile_result = validateNucPhase1Profile(profile);
  if (!profile_result.ok) {
    return profile_result;
  }
  if (!chassis.valid || chassis.stamp_ns <= 0 || chassis.state_id == 0U ||
      chassis.frame_id != profile.reference_frame ||
      !isKnownTimeSource(chassis.time_source) ||
      !nonEmpty(chassis.source_id)) {
    return ValidationResult::failure("chassis state identity or validity is invalid");
  }
  if ((chassis.valid_mask & ~kChassisKnownValidMask) != 0U) {
    return ValidationResult::failure("chassis state validity mask is unsupported");
  }
  if (measured_speed_required &&
      (chassis.valid_mask & ChassisState::kMeasuredSpeedValid) == 0U) {
    return ValidationResult::failure("measured speed is unavailable");
  }
  if ((chassis.valid_mask & ChassisState::kMeasuredSpeedValid) != 0U &&
      !isFinite(chassis.measured_speed_mps)) {
    return ValidationResult::failure("measured speed is non-finite");
  }
  if ((chassis.valid_mask & ChassisState::kYawRateValid) != 0U &&
      !isFinite(chassis.yaw_rate_radps)) {
    return ValidationResult::failure("yaw rate is non-finite");
  }
  if ((chassis.valid_mask & ChassisState::kSteeringAngleValid) != 0U &&
      !isFinite(chassis.steering_tire_angle_rad)) {
    return ValidationResult::failure("steering angle is non-finite");
  }
  if ((chassis.valid_mask & ChassisState::kVoltageValid) != 0U &&
      !isFinite(chassis.supply_voltage_v)) {
    return ValidationResult::failure("voltage is non-finite");
  }
  if ((chassis.valid_mask & ChassisState::kGearValid) != 0U &&
      (!isKnownGearState(chassis.gear_state) ||
       chassis.gear_state != GearState::kDrive)) {
    return ValidationResult::failure(
        "chassis gear is not the substantiated forward drive gear");
  }
  if (chassis.control_enabled &&
      (chassis.valid_mask & ChassisState::kControlEnabledValid) == 0U) {
    return ValidationResult::failure("control enable is not substantiated");
  }
  if ((chassis.valid_mask & ChassisState::kFaultValid) != 0U &&
      !isKnownFaultState(chassis.fault_state)) {
    return ValidationResult::failure("chassis fault state is unsupported");
  }
  return ValidationResult::success();
}

ValidationResult validateSafetyState(const SafetyState& safety) {
  if (!safety.valid || safety.stamp_ns <= 0 || safety.state_id == 0U ||
      safety.valid_for_ns <= 0 || !isKnownSafetyMode(safety.mode)) {
    return ValidationResult::failure(
        "safety state identity, mode, or validity is invalid");
  }
  if (!std::all_of(safety.reasons.begin(), safety.reasons.end(),
                   [](StopReason reason) { return isKnownStopReason(reason); })) {
    return ValidationResult::failure("safety state reason is unsupported");
  }
  return ValidationResult::success();
}

ValidationResult validateExecutionCommand(
    const VehicleExecutionCommand& command, const VehicleProfile& profile,
    std::int64_t now_monotonic_ns) {
  const ValidationResult profile_result = validateNucPhase1Profile(profile);
  if (!profile_result.ok) {
    return profile_result;
  }
  if (command.sequence_id == 0U || command.created_monotonic_ns <= 0 ||
      command.deadline_monotonic_ns <= command.created_monotonic_ns ||
      now_monotonic_ns < command.created_monotonic_ns ||
      now_monotonic_ns > command.deadline_monotonic_ns) {
    return ValidationResult::failure("execution command deadline is invalid");
  }
  if (!isFinite(command.signed_speed_mps) ||
      !isFinite(command.curvature_inv_m) || command.signed_speed_mps < 0.0 ||
      command.signed_speed_mps > profile.max_forward_speed_mps + kTolerance ||
      std::abs(command.curvature_inv_m) >
          maxAbsCurvature(profile) + kTolerance) {
    return ValidationResult::failure("execution command exceeds profile");
  }
  if ((!command.motion_enabled || command.hold) &&
      std::abs(command.signed_speed_mps) > kTolerance) {
    return ValidationResult::failure("inhibited command requests motion");
  }
  return ValidationResult::success();
}

bool isFresh(std::int64_t receipt_monotonic_ns,
             std::int64_t now_monotonic_ns,
             std::int64_t freshness_limit_ns) {
  return receipt_monotonic_ns > 0 && freshness_limit_ns > 0 &&
         now_monotonic_ns >= receipt_monotonic_ns &&
         now_monotonic_ns - receipt_monotonic_ns <= freshness_limit_ns;
}

bool withinDeclaredValidity(std::int64_t production_ns,
                            std::int64_t now_ns,
                            std::int64_t valid_for_ns) {
  return production_ns > 0 && valid_for_ns > 0 && now_ns >= production_ns &&
         now_ns - production_ns <= valid_for_ns;
}

}  // namespace auto_rover
