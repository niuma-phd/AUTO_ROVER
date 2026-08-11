#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <unistd.h>

#include <auto_rover_interfaces/ArmVehicle.h>
#include <auto_rover_interfaces/AssertEmergencyStop.h>
#include <auto_rover_interfaces/EgoState.h>
#include <auto_rover_interfaces/EmergencyStop.h>
#include <auto_rover_interfaces/MotionReference.h>
#include <auto_rover_interfaces/ResetEmergencyStop.h>
#include <auto_rover_interfaces/Trajectory.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "auto_rover_core/validation.hpp"
#include "auto_rover_ros1_conversions/conversions.hpp"
#include "auto_rover_vehicle/vehicle_execution_core.hpp"

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

bool secondsParameter(const ros::NodeHandle& private_node,
                      const std::string& name, std::int64_t* output_ns,
                      double* output_s, std::string* reason) {
  double seconds = 0.0;
  if (output_ns == nullptr || output_s == nullptr || reason == nullptr ||
      !requireParameter(private_node, name, &seconds, reason)) {
    return false;
  }
  const long double nanoseconds =
      static_cast<long double>(seconds) * 1000000000.0L;
  if (!std::isfinite(seconds) || seconds <= 0.0 ||
      !std::isfinite(nanoseconds) || nanoseconds < 1.0L ||
      nanoseconds > static_cast<long double>(
                        std::numeric_limits<std::int64_t>::max())) {
    *reason = "required seconds parameter is outside the supported range: " +
              name;
    return false;
  }
  *output_ns = static_cast<std::int64_t>(nanoseconds);
  *output_s = seconds;
  return true;
}

bool positiveCountParameter(const ros::NodeHandle& private_node,
                            const std::string& name, int* output,
                            std::string* reason) {
  if (!requireParameter(private_node, name, output, reason)) {
    return false;
  }
  if (*output <= 0) {
    *reason = "required recovery count must be positive: " + name;
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

std::string makeProcessGenerationId() {
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

bool loadVehicleProfile(const ros::NodeHandle& private_node,
                        auto_rover::VehicleProfile* profile,
                        std::string* reason) {
  if (profile == nullptr || reason == nullptr) {
    return false;
  }
  int schema_version = 0;
  std::string kinematic_model;
  std::string direction_capability;
  if (!requireParameter(private_node, "vehicle_profile/schema_version",
                        &schema_version, reason) ||
      !requireParameter(private_node, "vehicle_profile/profile_id",
                        &profile->profile_id, reason) ||
      !requireParameter(private_node, "vehicle_profile/kinematic_model",
                        &kinematic_model, reason) ||
      !requireParameter(private_node,
                        "vehicle_profile/direction_capability",
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

struct VehicleExecutionNodeConfig {
  auto_rover_vehicle::VehicleExecutionCoreConfig core;
  double feedback_period_s{0.0};
};

bool loadConfig(const ros::NodeHandle& private_node,
                VehicleExecutionNodeConfig* config,
                std::string* reason) {
  if (config == nullptr || reason == nullptr) {
    return false;
  }

  config->core.safety_supervisor.actuation_enabled = false;
  bool configured_actuation_enabled = false;
  if (private_node.getParam("safety_supervisor/actuation_enabled",
                            configured_actuation_enabled)) {
    config->core.safety_supervisor.actuation_enabled =
        configured_actuation_enabled;
  }

  int safety_recovery_count = 0;
  int guard_recovery_count = 0;
  int fake_recovery_count = 0;
  double unused_seconds = 0.0;
  if (!loadVehicleProfile(private_node, &config->core.vehicle_profile,
                          reason) ||
      !positiveCountParameter(
          private_node, "safety_supervisor/fresh_recovery_count",
          &safety_recovery_count, reason) ||
      !secondsParameter(private_node,
                        "safety_supervisor/state_valid_for_s",
                        &config->core.safety_supervisor.state_valid_for_ns,
                        &unused_seconds, reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/reset_service_enabled",
                        &config->core.safety_supervisor.reset_service_enabled,
                        reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/authorized_reset_operator_id",
                        &config->core.authorized_reset_operator_id, reason) ||
      !requireParameter(private_node, "command_guard/world_frame",
                        &config->core.command_guard.world_frame, reason) ||
      !secondsParameter(
          private_node, "command_guard/localization_freshness_s",
          &config->core.command_guard.localization_freshness_ns,
          &unused_seconds, reason) ||
      !secondsParameter(private_node,
                        "command_guard/trajectory_freshness_s",
                        &config->core.command_guard.trajectory_freshness_ns,
                        &unused_seconds, reason) ||
      !secondsParameter(private_node,
                        "command_guard/motion_freshness_s",
                        &config->core.command_guard.motion_freshness_ns,
                        &unused_seconds, reason) ||
      !secondsParameter(private_node,
                        "command_guard/chassis_freshness_s",
                        &config->core.command_guard.chassis_freshness_ns,
                        &unused_seconds, reason) ||
      !secondsParameter(private_node,
                        "command_guard/safety_freshness_s",
                        &config->core.command_guard.safety_freshness_ns,
                        &unused_seconds, reason) ||
      !positiveCountParameter(private_node,
                              "command_guard/fresh_recovery_count",
                              &guard_recovery_count, reason) ||
      !secondsParameter(private_node,
                        "vehicle_manager/command_valid_for_s",
                        &config->core.vehicle_manager.command_valid_for_ns,
                        &unused_seconds, reason) ||
      !secondsParameter(private_node, "fake_vcu/command_watchdog_s",
                        &config->core.fake_vcu.command_watchdog_ns,
                        &unused_seconds, reason) ||
      !secondsParameter(private_node, "fake_vcu/feedback_period_s",
                        &config->core.fake_vcu.feedback_period_ns,
                        &config->feedback_period_s, reason) ||
      !positiveCountParameter(private_node,
                              "fake_vcu/fresh_recovery_count",
                              &fake_recovery_count, reason) ||
      !requireParameter(private_node, "fake_vcu/max_accel_mps2",
                        &config->core.fake_vcu.max_accel_mps2, reason)) {
    return false;
  }

  config->core.safety_supervisor.fresh_recovery_count =
      static_cast<std::size_t>(safety_recovery_count);
  config->core.command_guard.fresh_recovery_count =
      static_cast<std::size_t>(guard_recovery_count);
  config->core.fake_vcu.fresh_recovery_count =
      static_cast<std::uint32_t>(fake_recovery_count);
  if (config->core.command_guard.world_frame != "camera_init") {
    *reason = "command_guard/world_frame must be camera_init";
    return false;
  }
  const auto validation =
      auto_rover_vehicle::validateVehicleExecutionCoreConfig(config->core);
  if (!validation.ok) {
    *reason = validation.reason;
    return false;
  }
  return true;
}

class VehicleExecutionNode {
 public:
  VehicleExecutionNode(ros::NodeHandle node,
                       ros::NodeHandle private_node,
                       const VehicleExecutionNodeConfig& config)
      : node_(std::move(node)),
        private_node_(std::move(private_node)),
        runtime_(config.core) {
    const std::int64_t now_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    const std::int64_t now_ros_ns =
        toSignedNanoseconds(ros::Time::now());
    if (!runtime_.initialize(now_monotonic_ns, now_ros_ns)) {
      const auto stopped = runtime_.shutdown(monotonicNow());
      if (stopped.delivery_unconfirmed) {
        ROS_ERROR_STREAM("Fake backend startup stop delivery is unconfirmed: "
                         << stopped.diagnostic);
      }
      throw std::runtime_error("vehicle execution core initialization failed");
    }

    ego_subscriber_ = node_.subscribe("ego_state", 1,
                                      &VehicleExecutionNode::egoCallback,
                                      this);
    trajectory_subscriber_ = node_.subscribe(
        "trajectory", 1, &VehicleExecutionNode::trajectoryCallback, this);
    motion_subscriber_ = node_.subscribe(
        "motion_reference", 1,
        &VehicleExecutionNode::motionReferenceCallback, this);
    // Emergency-stop assertions are events, not latest-value motion commands.
    // Keep a bounded burst queue and expose an acknowledged service below.
    emergency_stop_subscriber_ = node_.subscribe(
        "emergency_stop", 10,
        &VehicleExecutionNode::emergencyStopCallback, this);

    chassis_publisher_ =
        node_.advertise<auto_rover_interfaces::ChassisState>("chassis_state", 1);
    safety_publisher_ =
        node_.advertise<auto_rover_interfaces::SafetyState>("safety_state", 1);
    arm_service_ = node_.advertiseService(
        "arm_vehicle", &VehicleExecutionNode::armCallback, this);
    assert_emergency_stop_service_ = node_.advertiseService(
        "assert_emergency_stop",
        &VehicleExecutionNode::assertEmergencyStopCallback, this);
    reset_service_ = node_.advertiseService(
        "reset_emergency_stop", &VehicleExecutionNode::resetCallback, this);
    fake_connected_service_ = private_node_.advertiseService(
        "fake_connected", &VehicleExecutionNode::fakeConnectedCallback,
        this);
    fake_faulted_service_ = private_node_.advertiseService(
        "fake_faulted", &VehicleExecutionNode::fakeFaultedCallback, this);
    fake_drop_feedback_service_ = private_node_.advertiseService(
        "fake_drop_feedback",
        &VehicleExecutionNode::fakeDropFeedbackCallback, this);
    timer_ = node_.createSteadyTimer(
        ros::WallDuration(config.feedback_period_s),
        &VehicleExecutionNode::cycleCallback, this);
  }

  ~VehicleExecutionNode() {
    const auto stopped = runtime_.shutdown(monotonicNow());
    if (stopped.delivery_unconfirmed) {
      ROS_ERROR_STREAM("Fake backend shutdown stop delivery is unconfirmed: "
                       << stopped.diagnostic);
    }
  }

 private:
  static std::int64_t monotonicNow() {
    return toSignedNanoseconds(ros::SteadyTime::now());
  }

  static std::int64_t rosNow() {
    return toSignedNanoseconds(ros::Time::now());
  }

  void egoCallback(
      const auto_rover_interfaces::EgoState::ConstPtr& message) {
    runtime_.updateEgoState(auto_rover_ros1::toCore(*message),
                            monotonicNow());
  }

  void trajectoryCallback(
      const auto_rover_interfaces::Trajectory::ConstPtr& message) {
    runtime_.updateTrajectory(auto_rover_ros1::toCore(*message),
                              monotonicNow());
  }

  void motionReferenceCallback(
      const auto_rover_interfaces::MotionReference::ConstPtr& message) {
    runtime_.updateMotionReference(auto_rover_ros1::toCore(*message),
                                   monotonicNow());
  }

  void emergencyStopCallback(
      const auto_rover_interfaces::EmergencyStop::ConstPtr& message) {
    const auto result = runtime_.handleEmergencyStop(
        auto_rover_ros1::toCore(*message), monotonicNow(), rosNow());
    safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM(
          "Emergency-stop latch "
          << (result.success ? "accepted" : "processed")
          << ", but fake-backend zero delivery is unconfirmed: "
          << result.stop_delivery.diagnostic);
    }
    if (!result.success) {
      ROS_ERROR_STREAM("Emergency-stop assertion rejected after execution "
                       "was revoked: "
                       << result.reason);
    }
  }

  void cycleCallback(const ros::SteadyTimerEvent&) {
    const auto result = runtime_.cycle(monotonicNow(), rosNow());
    if (result.chassis_available) {
      chassis_publisher_.publish(auto_rover_ros1::toRos(result.chassis));
    }
    safety_publisher_.publish(auto_rover_ros1::toRos(result.safety));
    if (!result.health_clear) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Vehicle execution inhibited: " << result.health_diagnostic);
    }
    if (result.delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Fake backend host delivery is unconfirmed: "
                   << result.delivery.diagnostic);
    }
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Fake-backend zero delivery is unconfirmed: "
                   << result.stop_delivery.diagnostic);
    }
  }

  bool armCallback(auto_rover_interfaces::ArmVehicle::Request& request,
                   auto_rover_interfaces::ArmVehicle::Response& response) {
    const auto result = runtime_.requestArm(
        request.operator_id, request.safety_generation, request.arm,
        monotonicNow(), rosNow());
    response.success = result.success;
    response.reason = result.reason;
    response.safety_mode = static_cast<std::uint8_t>(result.state.mode);
    safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM("Arm/disarm request completed with unconfirmed "
                       "fake-backend zero delivery: "
                       << result.stop_delivery.diagnostic);
    }
    return true;
  }

  bool assertEmergencyStopCallback(
      auto_rover_interfaces::AssertEmergencyStop::Request& request,
      auto_rover_interfaces::AssertEmergencyStop::Response& response) {
    auto_rover::EmergencyStop assertion;
    if (request.schema_version == 1U) {
      assertion.stamp_ns = toSignedNanoseconds(request.source_stamp);
      assertion.request_id = request.request_id;
      assertion.source_id = request.source_id;
      assertion.asserted = true;
    }
    const auto result = runtime_.handleEmergencyStop(
        assertion, monotonicNow(), rosNow());
    response.success = result.success;
    response.reason = result.reason;
    response.safety_mode =
        static_cast<std::uint8_t>(result.state.mode);
    response.latch_generation = result.state.latch_generation;
    safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM(
          "Emergency-stop latch "
          << (result.success ? "accepted" : "processed")
          << ", but fake-backend zero delivery is unconfirmed: "
          << result.stop_delivery.diagnostic);
    }
    return true;
  }

  bool resetCallback(
      auto_rover_interfaces::ResetEmergencyStop::Request& request,
      auto_rover_interfaces::ResetEmergencyStop::Response& response) {
    const auto result = runtime_.resetEmergencyStop(
        request.operator_id, request.latch_generation,
        request.conditions_cleared_acknowledged, monotonicNow(), rosNow());
    response.success = result.success;
    response.reason = result.reason;
    response.safety_mode = static_cast<std::uint8_t>(result.state.mode);
    response.current_latch_generation = result.state.latch_generation;
    safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM("Emergency-stop reset completed with unconfirmed "
                       "fake-backend zero delivery: "
                       << result.stop_delivery.diagnostic);
    }
    return true;
  }

  bool fakeConnectedCallback(std_srvs::SetBool::Request& request,
                             std_srvs::SetBool::Response& response) {
    const auto result = runtime_.setFakeConnected(
        request.data, monotonicNow(), rosNow());
    response.success = result.success;
    response.message = result.reason;
    return true;
  }

  bool fakeFaultedCallback(std_srvs::SetBool::Request& request,
                           std_srvs::SetBool::Response& response) {
    const auto result = runtime_.setFakeFaulted(
        request.data, monotonicNow(), rosNow());
    response.success = result.success;
    response.message = result.reason;
    return true;
  }

  bool fakeDropFeedbackCallback(std_srvs::SetBool::Request& request,
                                std_srvs::SetBool::Response& response) {
    const auto result = runtime_.setFakeDropFeedback(
        request.data, monotonicNow(), rosNow());
    response.success = result.success;
    response.message = result.reason;
    return true;
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  auto_rover_vehicle::VehicleExecutionCore runtime_;
  ros::Subscriber ego_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber motion_subscriber_;
  ros::Subscriber emergency_stop_subscriber_;
  ros::Publisher chassis_publisher_;
  ros::Publisher safety_publisher_;
  ros::ServiceServer arm_service_;
  ros::ServiceServer assert_emergency_stop_service_;
  ros::ServiceServer reset_service_;
  ros::ServiceServer fake_connected_service_;
  ros::ServiceServer fake_faulted_service_;
  ros::ServiceServer fake_drop_feedback_service_;
  ros::SteadyTimer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vehicle_execution");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  VehicleExecutionNodeConfig config;
  config.core.fake_vcu.process_generation_id = makeProcessGenerationId();
  if (config.core.fake_vcu.process_generation_id.empty()) {
    ROS_FATAL("Vehicle execution could not establish a process generation");
    return EXIT_FAILURE;
  }
  std::string reason;
  if (!loadConfig(private_node, &config, &reason)) {
    ROS_FATAL_STREAM("Vehicle execution configuration rejected: " << reason);
    return EXIT_FAILURE;
  }

  try {
    VehicleExecutionNode wrapper(node, private_node, config);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Vehicle execution startup rejected: " << error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
