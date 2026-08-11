#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace auto_rover {

enum class TimeSource : std::uint8_t {
  kUnknown = 0,
  kSampleTime = 1,
  kPublishTime = 2,
  kReceiptTime = 3,
};

enum class KinematicModel : std::uint8_t {
  kUnknown = 0,
  kAckermannBicycle = 1,
};

enum class Direction : std::uint8_t {
  kUnknown = 0,
  kForward = 1,
  kReverse = 2,
};

enum class DirectionCapability : std::uint8_t {
  kUnknown = 0,
  kSignedSpeedDirection = 1,
  kConfirmedGearDirection = 2,
};

enum class CompletionBehavior : std::uint8_t {
  kUnknown = 0,
  kStopAndHold = 1,
};

enum class GearState : std::uint8_t {
  kUnknown = 0,
  kPark = 1,
  kReverse = 2,
  kNeutral = 3,
  kDrive = 4,
  kLow = 5,
  kOther = 6,
};

enum class FaultState : std::uint8_t {
  kUnknown = 0,
  kOk = 1,
  kDegraded = 2,
  kFault = 3,
  kEmergencyStop = 4,
};

enum class SafetyMode : std::uint8_t {
  kBootInhibited = 0,
  kDisarmed = 1,
  kRecoveryInhibited = 2,
  kArmed = 3,
  kFaultInhibited = 4,
  kEmergencyStopLatched = 5,
};

enum class StopReason : std::uint8_t {
  kNone = 0,
  kActuationDisabled = 1,
  kDisarmed = 2,
  kInvalidInput = 3,
  kStaleLocalization = 4,
  kStaleTrajectory = 5,
  kStaleMotionReference = 6,
  kStaleChassisState = 7,
  kLimitViolation = 8,
  kUnsupportedDirection = 9,
  kEmergencyStop = 10,
  kBackendDisconnected = 11,
  kBackendWatchdog = 12,
  kRouteInvalid = 13,
  kRecoveryPending = 14,
};

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{0.0};
};

struct Pose3 {
  Vector3 position;
  Quaternion orientation;
};

struct Twist3 {
  Vector3 linear;
  Vector3 angular;
};

struct EgoState {
  enum ValidMask : std::uint32_t {
    kPoseCovarianceValid = 1U << 0U,
    kTwistValid = 1U << 1U,
    kTwistCovarianceValid = 1U << 2U,
  };

  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::uint64_t state_id{0U};
  TimeSource time_source{TimeSource::kUnknown};
  std::string reference_frame;
  Pose3 pose;
  std::array<double, 36U> pose_covariance{};
  Twist3 twist;
  std::array<double, 36U> twist_covariance{};
  std::uint32_t valid_mask{0U};
  std::string source_id;
  bool valid{false};
};

struct RouteWaypoint {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double speed_mps{0.0};
};

struct RoutePlan {
  std::uint32_t schema_version{0U};
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::string route_id;
  std::uint64_t plan_version{0U};
  bool loop{false};
  std::vector<RouteWaypoint> waypoints;
};

struct TrajectoryPoint {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double curvature_inv_m{0.0};
  double target_speed_mps{0.0};
  double arc_length_m{0.0};
  Direction direction{Direction::kUnknown};
};

struct Trajectory {
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::string trajectory_id;
  std::string route_id;
  std::uint64_t plan_version{0U};
  std::string vehicle_profile_id;
  std::int64_t valid_for_ns{0};
  CompletionBehavior completion_behavior{CompletionBehavior::kUnknown};
  bool valid{false};
  std::vector<TrajectoryPoint> points;
};

struct MotionReference {
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::uint64_t command_id{0U};
  std::string producer_generation_id;
  std::string trajectory_id;
  Direction direction{Direction::kUnknown};
  double target_speed_mps{0.0};
  double target_curvature_inv_m{0.0};
  std::int64_t valid_for_ns{0};
  bool route_complete{false};
  bool valid{false};
};

struct ChassisState {
  enum ValidMask : std::uint32_t {
    kMeasuredSpeedValid = 1U << 0U,
    kYawRateValid = 1U << 1U,
    kSteeringAngleValid = 1U << 2U,
    kGearValid = 1U << 3U,
    kControlEnabledValid = 1U << 4U,
    kFaultValid = 1U << 5U,
    kVoltageValid = 1U << 6U,
  };

  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::uint64_t state_id{0U};
  TimeSource time_source{TimeSource::kUnknown};
  double measured_speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double steering_tire_angle_rad{0.0};
  GearState gear_state{GearState::kUnknown};
  bool control_enabled{false};
  FaultState fault_state{FaultState::kUnknown};
  double supply_voltage_v{0.0};
  std::uint32_t valid_mask{0U};
  std::string source_id;
  bool valid{false};
};

struct VehicleProfile {
  std::uint32_t schema_version{0U};
  std::string profile_id;
  KinematicModel kinematic_model{KinematicModel::kUnknown};
  DirectionCapability direction_capability{DirectionCapability::kUnknown};
  std::string reference_frame;
  double wheelbase_m{0.0};
  double max_forward_speed_mps{0.0};
  double max_longitudinal_accel_mps2{0.0};
  double min_turning_radius_m{0.0};
  bool reverse_supported{false};
};

struct SafetyState {
  std::int64_t stamp_ns{0};
  std::uint64_t state_id{0U};
  SafetyMode mode{SafetyMode::kBootInhibited};
  std::uint64_t latch_generation{0U};
  std::int64_t valid_for_ns{0};
  std::vector<StopReason> reasons;
  bool valid{false};
};

struct EmergencyStop {
  std::int64_t stamp_ns{0};
  std::string request_id;
  std::string source_id;
  bool asserted{false};
};

struct VehicleExecutionCommand {
  std::uint64_t sequence_id{0U};
  std::int64_t created_monotonic_ns{0};
  std::int64_t deadline_monotonic_ns{0};
  double signed_speed_mps{0.0};
  double curvature_inv_m{0.0};
  bool motion_enabled{false};
  bool hold{true};
  StopReason stop_reason{StopReason::kDisarmed};
};

template <typename T>
struct Received {
  T value;
  std::int64_t receipt_monotonic_ns{0};
};

struct ValidationResult {
  bool ok{false};
  std::string reason;

  static ValidationResult success() { return {true, std::string()}; }
  static ValidationResult failure(const std::string& message) {
    return {false, message};
  }
};

}  // namespace auto_rover
