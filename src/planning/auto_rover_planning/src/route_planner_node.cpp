#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>

#include <auto_rover_interfaces/ReloadRoute.h>
#include <auto_rover_interfaces/Trajectory.h>
#include <ros/ros.h>

#include "auto_rover_core/validation.hpp"
#include "auto_rover_planning/route_repository.hpp"
#include "auto_rover_planning/trajectory_generator.hpp"
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

std::int64_t toSignedNanoseconds(const ros::Time& time) {
  const std::uint64_t value = time.toNSec();
  if (value == 0U ||
      value > static_cast<std::uint64_t>(
                  std::numeric_limits<std::int64_t>::max())) {
    return 0;
  }
  return static_cast<std::int64_t>(value);
}

struct RoutePlannerConfig {
  std::string waypoints_file;
  std::string expected_frame_id;
  auto_rover::planning::TrajectoryGeneratorConfig generator;
  auto_rover::VehicleProfile vehicle_profile;
  double publish_period_s{0.0};
};

bool loadConfig(const ros::NodeHandle& private_node, RoutePlannerConfig* config,
                std::string* reason) {
  if (config == nullptr || reason == nullptr) {
    return false;
  }
  if (!requireParameter(private_node, "waypoints_file",
                        &config->waypoints_file, reason) ||
      !requireParameter(private_node, "expected_frame_id",
                        &config->expected_frame_id, reason) ||
      !requireParameter(private_node, "sampling_resolution_m",
                        &config->generator.sampling_resolution_m, reason) ||
      !positiveInt64Parameter(private_node, "trajectory_valid_for_ns",
                              &config->generator.valid_for_ns, reason) ||
      !requireParameter(private_node, "publish_period_s",
                        &config->publish_period_s, reason) ||
      !loadVehicleProfile(private_node, &config->vehicle_profile, reason)) {
    return false;
  }
  if (config->waypoints_file.empty()) {
    *reason = "waypoints_file must not be empty";
    return false;
  }
  if (config->expected_frame_id != "camera_init") {
    *reason = "expected_frame_id must be camera_init";
    return false;
  }
  if (!std::isfinite(config->generator.sampling_resolution_m) ||
      config->generator.sampling_resolution_m <= 0.0) {
    *reason = "sampling_resolution_m must be finite and positive";
    return false;
  }
  if (!std::isfinite(config->publish_period_s) ||
      config->publish_period_s <= 0.0) {
    *reason = "publish_period_s must be finite and positive";
    return false;
  }
  return true;
}

class RoutePlannerNode {
 public:
  RoutePlannerNode(ros::NodeHandle node, RoutePlannerConfig config)
      : node_(std::move(node)),
        config_(std::move(config)),
        repository_(config_.vehicle_profile, config_.expected_frame_id),
        generator_(config_.generator),
        active_trajectory_(makeInvalidTrajectory(
            0, config_.expected_frame_id, config_.vehicle_profile.profile_id)) {
    publisher_ =
        node_.advertise<auto_rover_interfaces::Trajectory>("trajectory", 1);
    reload_service_ = node_.advertiseService(
        "reload_route", &RoutePlannerNode::reloadCallback, this);
    timer_ = node_.createTimer(ros::Duration(config_.publish_period_s),
                               &RoutePlannerNode::publishCallback, this);

    const auto result = reloadRoute();
    if (!result.ok) {
      ROS_ERROR_STREAM("initial route load rejected: " << result.reason);
    }
  }

 private:
  static auto_rover::Trajectory makeInvalidTrajectory(
      std::int64_t stamp_ns, const std::string& frame_id,
      const std::string& vehicle_profile_id) {
    auto_rover::Trajectory trajectory;
    trajectory.stamp_ns = stamp_ns;
    trajectory.frame_id = frame_id;
    trajectory.trajectory_id = "invalid_route";
    trajectory.vehicle_profile_id = vehicle_profile_id;
    trajectory.completion_behavior =
        auto_rover::CompletionBehavior::kStopAndHold;
    trajectory.valid = false;
    return trajectory;
  }

  void invalidateActive(const std::string& reason, std::int64_t stamp_ns) {
    has_active_trajectory_ = false;
    active_trajectory_ = makeInvalidTrajectory(
        stamp_ns, config_.expected_frame_id,
        config_.vehicle_profile.profile_id);
    last_reason_ = reason;
  }

  auto_rover::ValidationResult reloadRoute() {
    const std::int64_t load_stamp_ns = toSignedNanoseconds(ros::Time::now());
    const auto loaded = repository_.reloadFromFile(config_.waypoints_file,
                                                   load_stamp_ns);
    if (!loaded.ok) {
      invalidateActive(loaded.reason, load_stamp_ns);
      return loaded;
    }

    auto generated = generator_.generate(repository_.activeRoute(),
                                         config_.vehicle_profile,
                                         load_stamp_ns);
    if (!generated.validation.ok) {
      invalidateActive(generated.validation.reason, load_stamp_ns);
      return generated.validation;
    }
    active_trajectory_ = std::move(generated.trajectory);
    has_active_trajectory_ = true;
    last_reason_.clear();
    return auto_rover::ValidationResult::success();
  }

  bool reloadCallback(auto_rover_interfaces::ReloadRoute::Request&,
                      auto_rover_interfaces::ReloadRoute::Response& response) {
    const auto result = reloadRoute();
    response.success = result.ok;
    response.reason = result.reason;
    if (result.ok) {
      response.route_id = active_trajectory_.route_id;
      response.plan_version = active_trajectory_.plan_version;
    } else {
      response.route_id.clear();
      response.plan_version = 0U;
      ROS_ERROR_STREAM("route reload rejected: " << result.reason);
    }
    return true;
  }

  void publishCallback(const ros::TimerEvent&) {
    const std::int64_t now_ros_ns = toSignedNanoseconds(ros::Time::now());
    auto_rover::Trajectory published = makeInvalidTrajectory(
        now_ros_ns, config_.expected_frame_id,
        config_.vehicle_profile.profile_id);
    if (has_active_trajectory_ && now_ros_ns > 0) {
      published = active_trajectory_;
      published.stamp_ns = now_ros_ns;
    } else if (!last_reason_.empty()) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "publishing invalid trajectory: "
                                   << last_reason_);
    }
    publisher_.publish(auto_rover_ros1::toRos(published));
  }

  ros::NodeHandle node_;
  RoutePlannerConfig config_;
  auto_rover::planning::RouteRepository repository_;
  auto_rover::planning::HermiteTrajectoryGenerator generator_;
  auto_rover::Trajectory active_trajectory_;
  bool has_active_trajectory_{false};
  std::string last_reason_{"no route has been loaded"};
  ros::Publisher publisher_;
  ros::ServiceServer reload_service_;
  ros::Timer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle node;
  const ros::NodeHandle private_node("~");

  RoutePlannerConfig config;
  std::string reason;
  if (!loadConfig(private_node, &config, &reason)) {
    ROS_FATAL_STREAM("route planner configuration rejected: " << reason);
    return EXIT_FAILURE;
  }

  RoutePlannerNode wrapper(node, std::move(config));
  ros::spin();
  return EXIT_SUCCESS;
}
