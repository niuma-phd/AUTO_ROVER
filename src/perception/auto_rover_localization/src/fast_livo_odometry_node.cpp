#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>

#include <unistd.h>

#include <auto_rover_interfaces/EgoState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "auto_rover_localization/fast_livo_odometry_adapter_core.hpp"
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

struct LocalizationNodeConfig {
  std::string source_topic;
  auto_rover::localization::FastLivoOdometryAdapterConfig adapter;
};

bool loadConfig(
    const ros::NodeHandle& private_node, LocalizationNodeConfig* config,
    std::string* reason) {
  if (config == nullptr || reason == nullptr) {
    return false;
  }

  std::string timestamp_semantics;
  double freshness_limit_s = 0.0;
  if (!requireParameter(private_node, "source/topic",
                        &config->source_topic, reason) ||
      !requireParameter(private_node, "source/expected_frame_id",
                        &config->adapter.expected_frame_id, reason) ||
      !requireParameter(private_node, "source/expected_child_frame_id",
                        &config->adapter.expected_child_frame_id, reason) ||
      !requireParameter(private_node, "source/revision",
                        &config->adapter.source_revision, reason) ||
      !requireParameter(private_node, "source/timestamp_semantics",
                        &timestamp_semantics, reason) ||
      !requireParameter(private_node, "source/freshness_limit_s",
                        &freshness_limit_s, reason) ||
      !requireParameter(private_node, "extrinsic/known",
                        &config->adapter.extrinsic_known, reason) ||
      !requireParameter(private_node, "extrinsic/translation/x",
                        &config->adapter.source_T_rear.position.x, reason) ||
      !requireParameter(private_node, "extrinsic/translation/y",
                        &config->adapter.source_T_rear.position.y, reason) ||
      !requireParameter(private_node, "extrinsic/translation/z",
                        &config->adapter.source_T_rear.position.z, reason) ||
      !requireParameter(private_node, "extrinsic/rotation/x",
                        &config->adapter.source_T_rear.orientation.x, reason) ||
      !requireParameter(private_node, "extrinsic/rotation/y",
                        &config->adapter.source_T_rear.orientation.y, reason) ||
      !requireParameter(private_node, "extrinsic/rotation/z",
                        &config->adapter.source_T_rear.orientation.z, reason) ||
      !requireParameter(private_node, "extrinsic/rotation/w",
                        &config->adapter.source_T_rear.orientation.w, reason)) {
    return false;
  }
  if (timestamp_semantics != "PUBLISH_TIME") {
    *reason = "timestamp_semantics must be PUBLISH_TIME";
    return false;
  }
  if (config->source_topic.empty()) {
    *reason = "source/topic must not be empty";
    return false;
  }
  const long double freshness_limit_ns =
      static_cast<long double>(freshness_limit_s) * 1000000000.0L;
  if (!std::isfinite(freshness_limit_s) || freshness_limit_s <= 0.0 ||
      !std::isfinite(freshness_limit_ns) || freshness_limit_ns < 1.0 ||
      freshness_limit_ns >
          static_cast<long double>(
              std::numeric_limits<std::int64_t>::max())) {
    *reason = "source/freshness_limit_s is outside the supported range";
    return false;
  }
  config->adapter.freshness_limit_ns =
      static_cast<std::int64_t>(freshness_limit_ns);
  config->adapter.timestamp_semantics = auto_rover::TimeSource::kPublishTime;
  return true;
}

class FastLivoOdometryNode {
 public:
  FastLivoOdometryNode(
      ros::NodeHandle node, const LocalizationNodeConfig& config)
      : node_(std::move(node)), adapter_(config.adapter) {
    publisher_ =
        node_.advertise<auto_rover_interfaces::EgoState>("ego_state", 1);
    subscriber_ = node_.subscribe(config.source_topic, 1,
                                  &FastLivoOdometryNode::odometryCallback,
                                  this);
  }

 private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const ros::SteadyTime receipt_time = ros::SteadyTime::now();
    auto_rover::localization::SourceOdometry source;
    source.provider_stamp_ns = toSignedNanoseconds(message->header.stamp);
    source.frame_id = message->header.frame_id;
    source.child_frame_id = message->child_frame_id;
    source.pose.position.x = message->pose.pose.position.x;
    source.pose.position.y = message->pose.pose.position.y;
    source.pose.position.z = message->pose.pose.position.z;
    source.pose.orientation.x = message->pose.pose.orientation.x;
    source.pose.orientation.y = message->pose.pose.orientation.y;
    source.pose.orientation.z = message->pose.pose.orientation.z;
    source.pose.orientation.w = message->pose.pose.orientation.w;

    const auto result = adapter_.adapt(
        source, toSignedNanoseconds(receipt_time),
        toSignedNanoseconds(ros::SteadyTime::now()));
    if (!result.ego_state.valid) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "FAST-LIVO2 odometry rejected: "
                                   << result.reason);
    }
    publisher_.publish(auto_rover_ros1::toRos(result.ego_state));
  }

  ros::NodeHandle node_;
  auto_rover::localization::FastLivoOdometryAdapterCore adapter_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_livo_odometry_adapter");
  ros::NodeHandle node;
  const ros::NodeHandle private_node("~");

  LocalizationNodeConfig config;
  std::string reason;
  if (!loadConfig(private_node, &config, &reason)) {
    ROS_FATAL_STREAM("FAST-LIVO2 adapter configuration rejected: " << reason);
    return EXIT_FAILURE;
  }

  config.adapter.process_generation_id = makeProcessGenerationId();
  if (config.adapter.process_generation_id.empty()) {
    ROS_FATAL("FAST-LIVO2 adapter could not establish a process generation");
    return EXIT_FAILURE;
  }

  FastLivoOdometryNode wrapper(node, config);
  ros::spin();
  return EXIT_SUCCESS;
}
