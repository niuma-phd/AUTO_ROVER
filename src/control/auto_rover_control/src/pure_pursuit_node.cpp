#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>

#include <unistd.h>

#include <auto_rover_interfaces/ChassisState.h>
#include <auto_rover_interfaces/EgoState.h>
#include <auto_rover_interfaces/MotionReference.h>
#include <auto_rover_interfaces/Trajectory.h>
#include <ros/ros.h>

#include "auto_rover_control/pure_pursuit.hpp"
#include "auto_rover_core/validation.hpp"
#include "auto_rover_ros1_conversions/conversions.hpp"

namespace {

template <typename Value>
bool requireParameter(const ros::NodeHandle& private_node,
                      const std::string& name, Value* output,
                      std::string* reason) {
  if (output == nullptr || reason == nullptr) {
    return false;
  }
  if (!private_node.getParam(name, *output)) {
    *reason = "required parameter is missing or has the wrong type: " + name;
    return false;
  }
  return true;
}

bool positiveInt64Parameter(const ros::NodeHandle& private_node,
                            const std::string& name, std::int64_t* output,
                            std::string* reason) {
  int value = 0;
  if (!requireParameter(private_node, name, &value, reason)) {
    return false;
  }
  if (value <= 0) {
    *reason = "required nanosecond parameter must be positive: " + name;
    return false;
  }
  *output = static_cast<std::int64_t>(value);
  return true;
}

bool loadVehicleProfile(const ros::NodeHandle& private_node,
                        auto_rover::VehicleProfile* profile,
                        std::string* reason) {
  int schema_version = 0;
  std::string kinematic_model;
  std::string direction_capability;
  if (profile == nullptr || reason == nullptr) {
    return false;
  }
  if (!requireParameter(private_node, "vehicle_profile/schema_version",
                        &schema_version, reason) ||
      !requireParameter(private_node, "vehicle_profile/profile_id",
                        &profile->profile_id, reason) ||
      !requireParameter(private_node, "vehicle_profile/kinematic_model",
                        &kinematic_model, reason) ||
      !requireParameter(private_node, "vehicle_profile/direction_capability",
                        &direction_capability, reason) ||
      !requireParameter(private_node, "vehicle_profile/reference_frame",
                        &profile->reference_frame, reason) ||
      !requireParameter(private_node, "vehicle_profile/wheelbase_m",
                        &profile->wheelbase_m, reason) ||
      !requireParameter(private_node,
                        "vehicle_profile/max_forward_speed_mps",
                        &profile->max_forward_speed_mps, reason) ||
      !requireParameter(private_node,
                        "vehicle_profile/max_longitudinal_accel_mps2",
                        &profile->max_longitudinal_accel_mps2, reason) ||
      !requireParameter(private_node,
                        "vehicle_profile/min_turning_radius_m",
                        &profile->min_turning_radius_m, reason) ||
      !requireParameter(private_node, "vehicle_profile/reverse_supported",
                        &profile->reverse_supported, reason)) {
    return false;
  }
  if (schema_version < 0) {
    *reason = "vehicle profile schema version must not be negative";
    return false;
  }
  profile->schema_version = static_cast<std::uint32_t>(schema_version);
  if (kinematic_model != "ackermann_bicycle") {
    *reason = "vehicle profile kinematic_model must be ackermann_bicycle";
    return false;
  }
  profile->kinematic_model = auto_rover::KinematicModel::kAckermannBicycle;
  if (direction_capability != "signed_speed_direction") {
    *reason =
        "vehicle profile direction_capability must be signed_speed_direction";
    return false;
  }
  profile->direction_capability =
      auto_rover::DirectionCapability::kSignedSpeedDirection;
  const auto validation = auto_rover::validateNucPhase1Profile(*profile);
  if (!validation.ok) {
    *reason = validation.reason;
    return false;
  }
  return true;
}

std::int64_t toSignedNanoseconds(const ros::SteadyTime& time) {
  const std::uint64_t value = time.toNSec();
  if (value == 0U ||
      value > static_cast<std::uint64_t>(
                  std::numeric_limits<std::int64_t>::max())) {
    return 0;
  }
  return static_cast<std::int64_t>(value);
}

std::int64_t toSignedNanoseconds(const ros::Time& time) {
  const std::uint64_t value = time.toNSec();
  if (value == 0U ||
      value > static_cast<std::uint64_t>(
                  std::numeric_limits<std::int64_t>::max())) {
    return 0;
  }
  return static_cast<std::int64_t>(value);
}

std::string makeProducerGenerationId() {
  const std::uint64_t wall_generation_ns = ros::WallTime::now().toNSec();
  const std::uint64_t steady_generation_ns = ros::SteadyTime::now().toNSec();
  const pid_t process_id = getpid();
  if (wall_generation_ns == 0U || steady_generation_ns == 0U ||
      process_id <= 0) {
    return {};
  }
  return std::to_string(wall_generation_ns) + "-" +
         std::to_string(steady_generation_ns) + "-pid" +
         std::to_string(static_cast<long long>(process_id));
}

struct PurePursuitNodeConfig {
  auto_rover_control::PurePursuitConfig tracker;
  auto_rover::VehicleProfile vehicle_profile;
  double publish_period_s{0.0};
};

bool loadConfig(const ros::NodeHandle& private_node,
                PurePursuitNodeConfig* config, std::string* reason) {
  if (config == nullptr || reason == nullptr) {
    return false;
  }
  if (!requireParameter(private_node, "world_frame",
                        &config->tracker.world_frame, reason) ||
      !requireParameter(private_node, "control_frame",
                        &config->tracker.control_frame, reason) ||
      !requireParameter(private_node, "lookahead_min_m",
                        &config->tracker.lookahead_min_m, reason) ||
      !requireParameter(private_node, "lookahead_max_m",
                        &config->tracker.lookahead_max_m, reason) ||
      !requireParameter(private_node, "lookahead_speed_gain_s",
                        &config->tracker.lookahead_speed_gain_s, reason) ||
      !requireParameter(private_node, "goal_position_tolerance_m",
                        &config->tracker.goal_position_tolerance_m, reason) ||
      !requireParameter(private_node, "standstill_speed_threshold_mps",
                        &config->tracker.standstill_speed_threshold_mps,
                        reason) ||
      !positiveInt64Parameter(private_node, "localization_freshness_ns",
                              &config->tracker.localization_freshness_ns,
                              reason) ||
      !positiveInt64Parameter(private_node, "trajectory_freshness_ns",
                              &config->tracker.trajectory_freshness_ns,
                              reason) ||
      !positiveInt64Parameter(private_node, "chassis_freshness_ns",
                              &config->tracker.chassis_freshness_ns, reason) ||
      !positiveInt64Parameter(private_node, "motion_valid_for_ns",
                              &config->tracker.motion_valid_for_ns, reason) ||
      !requireParameter(private_node, "publish_period_s",
                        &config->publish_period_s, reason) ||
      !loadVehicleProfile(private_node, &config->vehicle_profile, reason)) {
    return false;
  }
  if (config->tracker.world_frame != "camera_init") {
    *reason = "world_frame must be camera_init";
    return false;
  }
  if (config->tracker.control_frame !=
      config->vehicle_profile.reference_frame) {
    *reason = "control_frame must match the vehicle profile reference frame";
    return false;
  }
  const auto validation =
      auto_rover_control::validatePurePursuitConfig(config->tracker);
  if (!validation.ok) {
    *reason = validation.reason;
    return false;
  }
  if (!std::isfinite(config->publish_period_s) ||
      config->publish_period_s <= 0.0) {
    *reason = "publish_period_s must be finite and positive";
    return false;
  }
  return true;
}

class PurePursuitNode {
 public:
  PurePursuitNode(ros::NodeHandle node, PurePursuitNodeConfig config)
      : node_(std::move(node)),
        tracker_(config.tracker, config.vehicle_profile) {
    ego_subscriber_ = node_.subscribe("ego_state", 1,
                                      &PurePursuitNode::egoCallback, this);
    trajectory_subscriber_ =
        node_.subscribe("trajectory", 1,
                        &PurePursuitNode::trajectoryCallback, this);
    chassis_subscriber_ =
        node_.subscribe("chassis_state", 1,
                        &PurePursuitNode::chassisCallback, this);
    publisher_ = node_.advertise<auto_rover_interfaces::MotionReference>("motion_reference", 1);
    timer_ = node_.createTimer(ros::Duration(config.publish_period_s),
                               &PurePursuitNode::publishCallback, this);
  }

 private:
  void egoCallback(const auto_rover_interfaces::EgoState::ConstPtr& message) {
    const std::int64_t receipt_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    ego_.value = auto_rover_ros1::toCore(*message);
    ego_.receipt_monotonic_ns = receipt_monotonic_ns;
  }

  void trajectoryCallback(
      const auto_rover_interfaces::Trajectory::ConstPtr& message) {
    const std::int64_t receipt_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    trajectory_.value = auto_rover_ros1::toCore(*message);
    trajectory_.receipt_monotonic_ns = receipt_monotonic_ns;
  }

  void chassisCallback(
      const auto_rover_interfaces::ChassisState::ConstPtr& message) {
    const std::int64_t receipt_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    chassis_.value = auto_rover_ros1::toCore(*message);
    chassis_.receipt_monotonic_ns = receipt_monotonic_ns;
  }

  void publishCallback(const ros::TimerEvent&) {
    auto_rover_control::TrackingInput input;
    input.ego = ego_;
    input.trajectory = trajectory_;
    input.chassis = chassis_;
    input.now_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    input.now_ros_ns = toSignedNanoseconds(ros::Time::now());

    const auto result = tracker_.update(input);
    if (!result.reference.valid) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "Pure Pursuit output is invalid: "
                                   << result.reason);
    }
    publisher_.publish(auto_rover_ros1::toRos(result.reference));
  }

  ros::NodeHandle node_;
  auto_rover_control::PurePursuit tracker_;
  auto_rover::Received<auto_rover::EgoState> ego_;
  auto_rover::Received<auto_rover::Trajectory> trajectory_;
  auto_rover::Received<auto_rover::ChassisState> chassis_;
  ros::Subscriber ego_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber chassis_subscriber_;
  ros::Publisher publisher_;
  ros::Timer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle node;
  const ros::NodeHandle private_node("~");

  PurePursuitNodeConfig config;
  config.tracker.producer_generation_id = makeProducerGenerationId();
  if (config.tracker.producer_generation_id.empty()) {
    ROS_FATAL("Pure Pursuit could not establish a producer generation");
    return EXIT_FAILURE;
  }
  std::string reason;
  if (!loadConfig(private_node, &config, &reason)) {
    ROS_FATAL_STREAM("Pure Pursuit configuration rejected: " << reason);
    return EXIT_FAILURE;
  }

  PurePursuitNode wrapper(node, std::move(config));
  ros::spin();
  return EXIT_SUCCESS;
}
