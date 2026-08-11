#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

auto_rover::VehicleProfile validProfile() {
  auto_rover::VehicleProfile profile;
  profile.schema_version = 1U;
  profile.profile_id = "nuc_senior_akm_v1";
  profile.kinematic_model = auto_rover::KinematicModel::kAckermannBicycle;
  profile.direction_capability =
      auto_rover::DirectionCapability::kSignedSpeedDirection;
  profile.reference_frame = "rear_axle_center";
  profile.wheelbase_m = 0.3187;
  profile.max_forward_speed_mps = 0.50;
  profile.max_longitudinal_accel_mps2 = 0.20;
  profile.min_turning_radius_m = 0.95;
  profile.reverse_supported = false;
  return profile;
}

auto_rover::EgoState validEgo() {
  auto_rover::EgoState ego;
  ego.stamp_ns = 1000000000LL;
  ego.frame_id = "camera_init";
  ego.state_id = 1U;
  ego.time_source = auto_rover::TimeSource::kPublishTime;
  ego.reference_frame = "rear_axle_center";
  ego.pose.orientation.w = 1.0;
  ego.source_id = "localization_provider_generation_1";
  ego.valid = true;
  return ego;
}

auto_rover::RoutePlan validRoute() {
  auto_rover::RoutePlan route;
  route.schema_version = 1U;
  route.stamp_ns = 1000000000LL;
  route.frame_id = "camera_init";
  route.route_id = "commissioning_route_01";
  route.plan_version = 1U;
  route.loop = false;
  route.waypoints.push_back({0.0, 0.0, 0.0, 0.50});
  route.waypoints.push_back({2.0, 0.0, 0.0, 0.50});
  return route;
}

auto_rover::Trajectory validTrajectory() {
  auto_rover::Trajectory trajectory;
  trajectory.stamp_ns = 1000000000LL;
  trajectory.frame_id = "camera_init";
  trajectory.trajectory_id = "commissioning_route_01:1:1";
  trajectory.route_id = "commissioning_route_01";
  trajectory.plan_version = 1U;
  trajectory.vehicle_profile_id = "nuc_senior_akm_v1";
  trajectory.valid_for_ns = 1000000000LL;
  trajectory.completion_behavior =
      auto_rover::CompletionBehavior::kStopAndHold;
  trajectory.valid = true;
  trajectory.points.push_back(
      {0.0, 0.0, 0.0, 0.0, 0.50, 0.0, auto_rover::Direction::kForward});
  trajectory.points.push_back(
      {2.0, 0.0, 0.0, 0.0, 0.50, 2.0, auto_rover::Direction::kForward});
  return trajectory;
}

auto_rover::ChassisState validChassis() {
  auto_rover::ChassisState chassis;
  chassis.stamp_ns = 1000000000LL;
  chassis.frame_id = "rear_axle_center";
  chassis.state_id = 1U;
  chassis.time_source = auto_rover::TimeSource::kReceiptTime;
  chassis.measured_speed_mps = 0.0;
  chassis.valid_mask = auto_rover::ChassisState::kMeasuredSpeedValid;
  chassis.source_id = "fake_vcu";
  chassis.valid = true;
  return chassis;
}

auto_rover::SafetyState validSafety() {
  auto_rover::SafetyState safety;
  safety.stamp_ns = 1000000000LL;
  safety.state_id = 1U;
  safety.mode = auto_rover::SafetyMode::kDisarmed;
  safety.valid_for_ns = 100000000LL;
  safety.reasons = {auto_rover::StopReason::kDisarmed};
  safety.valid = true;
  return safety;
}

void testGeometry() {
  constexpr double kPi = 3.14159265358979323846;
  const auto_rover::Quaternion yaw_left =
      auto_rover::quaternionFromYaw(kPi / 2.0);
  auto_rover::Pose3 world_source;
  world_source.position.x = 1.0;
  world_source.position.y = 2.0;
  world_source.orientation = yaw_left;

  auto_rover::Pose3 source_rear;
  source_rear.position.x = -0.25;
  source_rear.orientation.w = 1.0;
  const auto_rover::Pose3 world_rear =
      auto_rover::composePoses(world_source, source_rear);

  expectNear(world_rear.position.x, 1.0, 1e-12,
             "extrinsic translation rotates into the world frame");
  expectNear(world_rear.position.y, 1.75, 1e-12,
             "world_T_rear uses world_T_source times source_T_rear");
  expectNear(auto_rover::yawFromQuaternion(world_rear.orientation), kPi / 2.0,
             1e-12, "pose composition preserves yaw");
  expectNear(auto_rover::normalizeAngle(3.0 * kPi), -kPi, 1e-12,
             "angles normalize to the documented half-open interval");

  auto_rover::Quaternion scaled;
  scaled.z = 1.0;
  scaled.w = 1.0;
  auto_rover::Quaternion normalized;
  expect(auto_rover::normalizeQuaternion(scaled, &normalized),
         "finite non-zero quaternion normalizes");
  expectNear(normalized.z, std::sqrt(0.5), 1e-12,
             "quaternion normalization is deterministic");
  auto_rover::Quaternion zero;
  expect(!auto_rover::normalizeQuaternion(zero, &normalized),
         "zero quaternion fails closed");
}

void testVehicleProfileValidation() {
  auto profile = validProfile();
  expect(auto_rover::validateVehicleProfile(profile).ok,
         "approved initial profile validates");
  expectNear(auto_rover::maxAbsCurvature(profile), 1.0 / 0.95, 1e-12,
             "runtime radius is the sole curvature source");

  profile.min_turning_radius_m = 0.80;
  expect(auto_rover::validateVehicleProfile(profile).ok,
         "validation is structural and does not silently rewrite configuration");
  profile = validProfile();
  profile.max_forward_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  expect(!auto_rover::validateVehicleProfile(profile).ok,
         "non-finite profile limits are rejected");
  profile = validProfile();
  profile.reference_frame.clear();
  expect(!auto_rover::validateVehicleProfile(profile).ok,
         "missing control reference fails closed");
  profile = validProfile();
  profile.reverse_supported = true;
  expect(!auto_rover::validateNucPhase1Profile(profile).ok,
         "selected phase-1 vehicle profile is forward-only");
  profile = validProfile();
  profile.direction_capability =
      auto_rover::DirectionCapability::kConfirmedGearDirection;
  expect(!auto_rover::validateNucPhase1Profile(profile).ok,
         "NUC phase-1 cannot silently switch direction capability policy");
}

void testContractValidation() {
  const auto profile = validProfile();
  auto ego = validEgo();
  expect(auto_rover::validateEgoState(ego, "camera_init",
                                      "rear_axle_center")
             .ok,
         "finite rear-axle pose in the expected frame validates");
  ego.pose.position.x = std::numeric_limits<double>::infinity();
  expect(!auto_rover::validateEgoState(ego, "camera_init",
                                       "rear_axle_center")
              .ok,
         "infinite localization is rejected");
  ego = validEgo();
  ego.time_source = static_cast<auto_rover::TimeSource>(255U);
  expect(!auto_rover::validateEgoState(ego, "camera_init",
                                       "rear_axle_center")
              .ok,
         "unsupported localization time-source enum is rejected");
  ego = validEgo();
  ego.valid_mask = 1U << 31U;
  expect(!auto_rover::validateEgoState(ego, "camera_init",
                                       "rear_axle_center")
              .ok,
         "unknown localization validity bits are rejected");
  ego = validEgo();
  ego.valid_mask = auto_rover::EgoState::kTwistCovarianceValid;
  expect(!auto_rover::validateEgoState(ego, "camera_init",
                                       "rear_axle_center")
              .ok,
         "twist covariance cannot be substantiated without twist");

  auto route = validRoute();
  expect(auto_rover::validateRoutePlan(route, profile, "camera_init").ok,
         "valid commissioning route validates");
  route.waypoints[1].speed_mps = 0.5000001;
  expect(!auto_rover::validateRoutePlan(route, profile, "camera_init").ok,
         "route speed cannot exceed 0.50 m/s");
  route = validRoute();
  route.waypoints[1].x_m = route.waypoints[0].x_m;
  route.waypoints[1].y_m = route.waypoints[0].y_m;
  expect(!auto_rover::validateRoutePlan(route, profile, "camera_init").ok,
         "duplicate adjacent waypoint positions are rejected");
  route = validRoute();
  route.waypoints[1].yaw_rad = 6.28318530717958647692;
  expect(!auto_rover::validateRoutePlan(route, profile, "camera_init").ok,
         "route yaw must be normalized at an algorithm boundary");

  auto trajectory = validTrajectory();
  expect(auto_rover::validateTrajectory(trajectory, profile, "camera_init").ok,
         "finite ordered forward trajectory validates");
  trajectory.points[1].curvature_inv_m = 1.0 / 0.95 + 1e-6;
  expect(!auto_rover::validateTrajectory(trajectory, profile, "camera_init").ok,
         "trajectory curvature beyond 0.95 m radius is rejected");
  trajectory = validTrajectory();
  trajectory.points[1].yaw_rad = 3.14159265358979323846;
  expect(!auto_rover::validateTrajectory(trajectory, profile, "camera_init")
              .ok,
         "trajectory yaw must use the documented half-open interval");

  auto_rover::MotionReference motion;
  motion.stamp_ns = 1000000000LL;
  motion.frame_id = "rear_axle_center";
  motion.command_id = 1U;
  motion.producer_generation_id = "core-test-tracker-generation-1";
  motion.trajectory_id = trajectory.trajectory_id;
  motion.direction = auto_rover::Direction::kForward;
  motion.target_speed_mps = 0.1;
  motion.target_curvature_inv_m = 0.2;
  motion.valid_for_ns = 100000000LL;
  motion.valid = true;
  expect(auto_rover::validateMotionReference(motion, profile).ok,
         "finite forward motion reference validates");
  motion.producer_generation_id.clear();
  expect(!auto_rover::validateMotionReference(motion, profile).ok,
         "missing tracker producer generation fails closed");
  motion.producer_generation_id = "core-test-tracker-generation-1";
  motion.direction = auto_rover::Direction::kReverse;
  expect(!auto_rover::validateMotionReference(motion, profile).ok,
         "reverse request fails closed");
  motion.direction = auto_rover::Direction::kForward;
  motion.target_speed_mps = -0.01;
  expect(!auto_rover::validateMotionReference(motion, profile).ok,
         "negative commanded speed remains invalid for the forward-only profile");
  motion.target_speed_mps = 0.0;
  motion.target_curvature_inv_m = 0.5;
  expect(auto_rover::validateMotionReference(motion, profile).ok,
         "zero speed may retain curvature before protocol conversion");

  auto_rover::VehicleExecutionCommand execution;
  execution.sequence_id = 1U;
  execution.created_monotonic_ns = 1000000000LL;
  execution.deadline_monotonic_ns = 1100000000LL;
  execution.signed_speed_mps = 0.1;
  execution.motion_enabled = true;
  execution.hold = false;
  execution.stop_reason = auto_rover::StopReason::kNone;
  expect(auto_rover::validateExecutionCommand(execution, profile,
                                               1050000000LL)
             .ok,
         "finite forward execution command validates");
  execution.signed_speed_mps = -0.001;
  expect(!auto_rover::validateExecutionCommand(execution, profile,
                                                1050000000LL)
              .ok,
         "negative execution speed remains invalid for the forward-only profile");

  auto chassis = validChassis();
  expect(auto_rover::validateChassisState(chassis, profile, true).ok,
         "finite forward chassis feedback validates");
  chassis.time_source = static_cast<auto_rover::TimeSource>(255U);
  expect(!auto_rover::validateChassisState(chassis, profile, true).ok,
         "unsupported chassis time-source enum is rejected");
  chassis = validChassis();
  chassis.measured_speed_mps = -0.001;
  expect(auto_rover::validateChassisState(chassis, profile, true).ok,
         "small finite negative measured speed is valid signed feedback");
  chassis.measured_speed_mps = -3.75;
  expect(auto_rover::validateChassisState(chassis, profile, true).ok,
         "larger finite negative measured speed remains valid signed feedback");
  chassis.measured_speed_mps = -std::numeric_limits<double>::infinity();
  expect(!auto_rover::validateChassisState(chassis, profile, true).ok,
         "negative non-finite measured speed still fails closed");
  const auto gears = {auto_rover::GearState::kPark,
                      auto_rover::GearState::kReverse,
                      auto_rover::GearState::kNeutral,
                      auto_rover::GearState::kLow,
                      auto_rover::GearState::kOther,
                      auto_rover::GearState::kUnknown,
                      static_cast<auto_rover::GearState>(255U)};
  for (const auto gear : gears) {
    chassis = validChassis();
    chassis.valid_mask |= auto_rover::ChassisState::kGearValid;
    chassis.gear_state = gear;
    expect(!auto_rover::validateChassisState(chassis, profile, true).ok,
           "every substantiated non-Drive gear fails closed");
  }
  chassis = validChassis();
  chassis.valid_mask |= auto_rover::ChassisState::kGearValid;
  chassis.gear_state = auto_rover::GearState::kDrive;
  expect(auto_rover::validateChassisState(chassis, profile, true).ok,
         "substantiated Drive gear is the sole accepted forward gear");
  chassis = validChassis();
  chassis.valid_mask |= auto_rover::ChassisState::kFaultValid;
  chassis.fault_state = static_cast<auto_rover::FaultState>(255U);
  expect(!auto_rover::validateChassisState(chassis, profile, true).ok,
         "unsupported chassis fault enum is rejected");

  auto safety = validSafety();
  expect(auto_rover::validateSafetyState(safety).ok,
         "known safety state validates");
  safety.mode = static_cast<auto_rover::SafetyMode>(255U);
  expect(!auto_rover::validateSafetyState(safety).ok,
         "unsupported safety mode enum is rejected");
  safety = validSafety();
  safety.reasons.push_back(static_cast<auto_rover::StopReason>(255U));
  expect(!auto_rover::validateSafetyState(safety).ok,
         "unsupported safety reason enum is rejected");
}

void testFreshness() {
  expect(auto_rover::isFresh(1000000000LL, 1100000000LL, 100000000LL),
         "age equal to timeout remains fresh");
  expect(!auto_rover::isFresh(1000000000LL, 1100000001LL, 100000000LL),
         "age beyond timeout is stale");
  expect(!auto_rover::isFresh(1100000000LL, 1000000000LL, 100000000LL),
         "monotonic regression fails closed");
  expect(!auto_rover::isFresh(1000000000LL, 1000000000LL, 0LL),
         "non-positive freshness limit is invalid");
  expect(auto_rover::withinDeclaredValidity(1000000000LL, 1100000000LL,
                                            100000000LL),
         "declared lifetime is inclusive");
  expect(!auto_rover::withinDeclaredValidity(1000000000LL, 1100000001LL,
                                             100000000LL),
         "expired producer lifetime is rejected");
}

}  // namespace

int main() {
  testGeometry();
  testVehicleProfileValidation();
  testContractValidation();
  testFreshness();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "core contract tests passed\n";
  return EXIT_SUCCESS;
}
