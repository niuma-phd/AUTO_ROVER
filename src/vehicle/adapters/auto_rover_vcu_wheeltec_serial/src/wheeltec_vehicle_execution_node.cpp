#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
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

#include "auto_rover_core/validation.hpp"
#include "auto_rover_ros1_conversions/conversions.hpp"
#include "auto_rover_vcu_wheeltec_serial/physical_activation.hpp"
#include "auto_rover_vcu_wheeltec_serial/runtime.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"
#include "auto_rover_vcu_wheeltec_serial/vehicle_backend.hpp"
#include "auto_rover_vehicle/vehicle_execution_core.hpp"

namespace {

constexpr double kMaximumCyclePeriodS = 0.05;
constexpr double kLimitTolerance = 1e-12;

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

bool positiveNanosecondsParameter(const ros::NodeHandle& private_node,
                                  const std::string& name,
                                  std::int64_t* output,
                                  std::string* reason) {
  int value = 0;
  if (output == nullptr || !requireParameter(private_node, name, &value,
                                              reason)) {
    return false;
  }
  if (value <= 0) {
    *reason = "required nanoseconds parameter must be positive: " + name;
    return false;
  }
  *output = static_cast<std::int64_t>(value);
  return true;
}

bool positiveCountParameter(const ros::NodeHandle& private_node,
                            const std::string& name,
                            std::uint32_t* output,
                            std::string* reason) {
  int value = 0;
  if (output == nullptr || !requireParameter(private_node, name, &value,
                                              reason)) {
    return false;
  }
  if (value <= 0) {
    *reason = "required count parameter must be positive: " + name;
    return false;
  }
  *output = static_cast<std::uint32_t>(value);
  return true;
}

bool secondsParameter(const ros::NodeHandle& private_node,
                      const std::string& name, std::int64_t* output_ns,
                      std::string* reason) {
  double seconds = 0.0;
  if (output_ns == nullptr ||
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

bool executionInputStringIsBounded(const std::string& value) {
  return value.size() <=
         auto_rover_vehicle::kMaximumExecutionIdentifierBytes;
}

bool waitUntilMonotonicNs(std::int64_t deadline_ns) {
  if (deadline_ns <= 0) {
    return false;
  }
  const std::chrono::steady_clock::time_point deadline{
      std::chrono::nanoseconds(deadline_ns)};
  std::this_thread::sleep_until(deadline);
  return std::chrono::steady_clock::now() >= deadline;
}

bool executionInputIsBounded(
    const auto_rover_interfaces::EgoState& message) {
  return executionInputStringIsBounded(message.header.frame_id) &&
         executionInputStringIsBounded(message.reference_frame) &&
         executionInputStringIsBounded(message.source_id);
}

bool executionInputIsBounded(
    const auto_rover_interfaces::Trajectory& message) {
  return message.points.size() <=
             auto_rover_vehicle::kMaximumExecutionTrajectoryPoints &&
         executionInputStringIsBounded(message.header.frame_id) &&
         executionInputStringIsBounded(message.trajectory_id) &&
         executionInputStringIsBounded(message.route_id) &&
         executionInputStringIsBounded(message.vehicle_profile_id);
}

bool executionInputIsBounded(
    const auto_rover_interfaces::MotionReference& message) {
  return executionInputStringIsBounded(message.header.frame_id) &&
         executionInputStringIsBounded(message.producer_generation_id) &&
         executionInputStringIsBounded(message.trajectory_id);
}

bool executionInputIsBounded(
    const auto_rover_interfaces::EmergencyStop& message) {
  return executionInputStringIsBounded(message.header.frame_id) &&
         executionInputStringIsBounded(message.request_id) &&
         executionInputStringIsBounded(message.source_id);
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

struct WheeltecExecutionNodeConfig {
  auto_rover_vehicle::VehicleExecutionCommonConfig core;
  auto_rover::wheeltec_serial::WheeltecVehicleBackendConfig backend;
  auto_rover::wheeltec_serial::PhysicalDeviceOptions physical;
  bool real_device_enabled{false};
  bool actuation_enabled{false};
  double cycle_period_s{0.0};
  std::int64_t cycle_period_ns{0};
};

bool loadCommonExecutionConfig(const ros::NodeHandle& private_node,
                               WheeltecExecutionNodeConfig* config,
                               std::string* reason) {
  int safety_recovery_count = 0;
  int guard_recovery_count = 0;
  if (config == nullptr || reason == nullptr ||
      !loadVehicleProfile(private_node, &config->core.vehicle_profile,
                          reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/actuation_enabled",
                        &config->actuation_enabled, reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/fresh_recovery_count",
                        &safety_recovery_count, reason) ||
      !secondsParameter(private_node,
                        "safety_supervisor/state_valid_for_s",
                        &config->core.safety_supervisor.state_valid_for_ns,
                        reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/reset_service_enabled",
                        &config->core.safety_supervisor.reset_service_enabled,
                        reason) ||
      !requireParameter(private_node,
                        "safety_supervisor/authorized_reset_operator_id",
                        &config->core.authorized_reset_operator_id, reason) ||
      !requireParameter(private_node, "command_guard/world_frame",
                        &config->core.command_guard.world_frame, reason) ||
      !secondsParameter(private_node,
                        "command_guard/localization_freshness_s",
                        &config->core.command_guard.localization_freshness_ns,
                        reason) ||
      !secondsParameter(private_node,
                        "command_guard/trajectory_freshness_s",
                        &config->core.command_guard.trajectory_freshness_ns,
                        reason) ||
      !secondsParameter(private_node, "command_guard/motion_freshness_s",
                        &config->core.command_guard.motion_freshness_ns,
                        reason) ||
      !secondsParameter(private_node, "command_guard/chassis_freshness_s",
                        &config->core.command_guard.chassis_freshness_ns,
                        reason) ||
      !secondsParameter(private_node, "command_guard/safety_freshness_s",
                        &config->core.command_guard.safety_freshness_ns,
                        reason) ||
      !requireParameter(private_node, "command_guard/fresh_recovery_count",
                        &guard_recovery_count, reason) ||
      !secondsParameter(private_node, "vehicle_manager/command_valid_for_s",
                        &config->core.vehicle_manager.command_valid_for_ns,
                        reason)) {
    return false;
  }
  if (safety_recovery_count <= 0 || guard_recovery_count <= 0) {
    *reason = "safety and guard recovery counts must be positive";
    return false;
  }
  config->core.safety_supervisor.actuation_enabled =
      config->actuation_enabled;
  config->core.safety_supervisor.fresh_recovery_count =
      static_cast<std::size_t>(safety_recovery_count);
  config->core.command_guard.fresh_recovery_count =
      static_cast<std::size_t>(guard_recovery_count);
  if (config->core.command_guard.world_frame != "camera_init") {
    *reason = "command_guard/world_frame must be camera_init";
    return false;
  }
  const auto validation =
      auto_rover_vehicle::validateVehicleExecutionCommonConfig(config->core);
  if (!validation.ok) {
    *reason = validation.reason;
    return false;
  }
  return true;
}

bool loadPhysicalIdentity(const ros::NodeHandle& private_node,
                          WheeltecExecutionNodeConfig* config,
                          std::string* reason) {
  int device_major = -1;
  int device_minor = -1;
  int owner_uid = -1;
  int group_gid = -1;
  int baud = 0;
  std::string serial_format;
  std::string command_layout;
  std::string feedback_layout;
  if (!requireParameter(private_node, "physical_backend/device_path",
                        &config->physical.device_path, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_device_major",
                        &device_major, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_device_minor",
                        &device_minor, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_owner_uid",
                        &owner_uid, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_group_gid",
                        &group_gid, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_usb_vendor_id",
                        &config->physical.expected_usb_vendor_id, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_usb_product_id",
                        &config->physical.expected_usb_product_id, reason) ||
      !requireParameter(private_node,
                        "physical_backend/expected_usb_serial",
                        &config->physical.expected_usb_serial, reason) ||
      !requireParameter(private_node, "physical_backend/baud", &baud,
                        reason) ||
      !requireParameter(
          private_node, "physical_backend/serial_format_source_assumption",
          &serial_format, reason) ||
      !requireParameter(
          private_node, "physical_backend/command_layout_source_assumption",
          &command_layout, reason) ||
      !requireParameter(
          private_node, "physical_backend/feedback_layout_source_assumption",
          &feedback_layout, reason)) {
    return false;
  }
  if (baud != 115200 || serial_format != "8N1_no_flow" ||
      command_layout != "wheeltec_11_byte_candidate" ||
      feedback_layout != "wheeltec_24_byte_candidate") {
    *reason = "physical serial candidate contract does not match 115200 8N1";
    return false;
  }
  if (config->real_device_enabled &&
      (device_major < 0 || device_minor < 0 || owner_uid < 0 ||
       group_gid < 0)) {
    *reason = "enabled physical backend requires pinned non-negative identity";
    return false;
  }
  if (device_major >= 0) {
    config->physical.expected_device_major =
        static_cast<std::uint64_t>(device_major);
  }
  if (device_minor >= 0) {
    config->physical.expected_device_minor =
        static_cast<std::uint64_t>(device_minor);
  }
  if (owner_uid >= 0) {
    config->physical.expected_owner_uid =
        static_cast<std::uint64_t>(owner_uid);
  }
  if (group_gid >= 0) {
    config->physical.expected_group_gid =
        static_cast<std::uint64_t>(group_gid);
  }
  return true;
}

bool loadWheeltecConfig(const ros::NodeHandle& private_node,
                        WheeltecExecutionNodeConfig* config,
                        std::string* reason) {
  using auto_rover::wheeltec_serial::PhysicalAccessMode;
  if (config == nullptr || reason == nullptr ||
      !loadCommonExecutionConfig(private_node, config, reason) ||
      !requireParameter(private_node, "real_device_enabled",
                        &config->real_device_enabled, reason) ||
      !requireParameter(private_node, "actuation_enabled",
                        &config->actuation_enabled, reason) ||
      !requireParameter(private_node, "unverified_protocol_acknowledged",
                        &config->backend.runtime
                             .unverified_protocol_acknowledged,
                        reason) ||
      !requireParameter(private_node, "physical_device_opt_in",
                        &config->backend.runtime.physical_device_opt_in,
                        reason) ||
      !requireParameter(private_node, "actuation_opt_in",
                        &config->backend.runtime.actuation_opt_in, reason) ||
      !requireParameter(private_node, "readiness_gate_passed",
                        &config->backend.runtime.readiness_gate_passed,
                        reason) ||
      !requireParameter(
          private_node, "external_or_durable_estop_strategy_approved",
          &config->backend.runtime
               .external_or_durable_estop_strategy_approved,
          reason) ||
      !requireParameter(private_node, "codec/max_forward_speed_mps",
                        &config->backend.runtime.adapter.codec_limits
                             .max_forward_speed_mps,
                        reason) ||
      !requireParameter(private_node, "codec/max_abs_curvature_inv_m",
                        &config->backend.runtime.adapter.codec_limits
                             .max_abs_curvature_inv_m,
                        reason) ||
      !requireParameter(private_node, "runtime/cycle_period_s",
                        &config->cycle_period_s, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/max_command_age_ns",
          &config->backend.runtime.adapter.max_command_age_ns, reason) ||
      !positiveCountParameter(
          private_node, "runtime/fresh_commands_required",
          &config->backend.runtime.adapter.fresh_commands_required, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/zero_retry_interval_ns",
          &config->backend.runtime.adapter.zero_retry_interval_ns, reason) ||
      !positiveCountParameter(
          private_node, "runtime/max_zero_write_attempts",
          &config->backend.runtime.adapter.max_zero_write_attempts, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/read_timeout_ns",
          &config->backend.runtime.read_timeout_ns, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/drain_read_timeout_ns",
          &config->backend.runtime.drain_read_timeout_ns, reason) ||
      !positiveCountParameter(
          private_node, "runtime/maximum_drain_reads",
          &config->backend.runtime.maximum_drain_reads, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/maximum_feedback_age_ns",
          &config->backend.runtime.maximum_feedback_age_ns, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/write_timeout_ns",
          &config->backend.runtime.adapter.write_timeout_ns, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/maximum_normal_write_gap_ns",
          &config->backend.runtime.maximum_normal_write_gap_ns, reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/maximum_step_duration_ns",
          &config->backend.runtime.maximum_step_duration_ns, reason) ||
      !positiveCountParameter(
          private_node, "runtime/control_allowed_receipts_required",
          &config->backend.runtime.control_allowed_receipts_required,
          reason) ||
      !positiveNanosecondsParameter(
          private_node, "runtime/control_allowed_minimum_span_ns",
          &config->backend.runtime.control_allowed_minimum_span_ns,
          reason) ||
      !loadPhysicalIdentity(private_node, config, reason)) {
    return false;
  }

  config->backend.runtime.runtime_enabled = config->real_device_enabled;
  config->physical.unverified_protocol_acknowledged =
      config->backend.runtime.unverified_protocol_acknowledged;
  config->physical.physical_device_opt_in =
      config->backend.runtime.physical_device_opt_in;
  config->physical.actuation_opt_in =
      config->backend.runtime.actuation_opt_in;
  config->physical.access_mode = config->real_device_enabled
                                     ? PhysicalAccessMode::kActuation
                                     : PhysicalAccessMode::kDisabled;

  if (config->actuation_enabled !=
      config->core.safety_supervisor.actuation_enabled) {
    *reason =
        "top-level and safety-supervisor actuation gates must match";
    return false;
  }

  if (!std::isfinite(config->cycle_period_s) ||
      config->cycle_period_s <= 0.0 ||
      config->cycle_period_s > kMaximumCyclePeriodS) {
    *reason = "runtime/cycle_period_s is outside (0, 0.05]";
    return false;
  }
  const long double cycle_period_ns =
      static_cast<long double>(config->cycle_period_s) * 1000000000.0L;
  if (!std::isfinite(cycle_period_ns) || cycle_period_ns < 1.0L) {
    *reason = "runtime/cycle_period_s must represent at least one nanosecond";
    return false;
  }
  config->cycle_period_ns = static_cast<std::int64_t>(cycle_period_ns);
  if (config->cycle_period_ns >
      config->backend.runtime.maximum_normal_write_gap_ns / 2) {
    *reason =
        "two execution-worker periods must fit within the normal write gap";
    return false;
  }
  if (!auto_rover::wheeltec_serial::runtimeConfigIsStructurallyValid(
          config->backend.runtime)) {
    *reason = "wheeltec runtime configuration is structurally invalid";
    return false;
  }
  if (std::abs(config->backend.runtime.adapter.codec_limits
                   .max_forward_speed_mps -
               config->core.vehicle_profile.max_forward_speed_mps) >
      kLimitTolerance) {
    *reason =
        "codec speed limit must equal the selected vehicle-profile limit";
    return false;
  }
  const double profile_curvature =
      auto_rover::maxAbsCurvature(config->core.vehicle_profile);
  if (config->backend.runtime.adapter.codec_limits
          .max_abs_curvature_inv_m >
      profile_curvature + kLimitTolerance) {
    *reason = "codec curvature limit exceeds the vehicle-profile limit";
    return false;
  }

  const bool all_runtime_gates =
      auto_rover::wheeltec_serial::runtimeActuationGatesAreSatisfied(
          config->backend.runtime);
  if (!config->real_device_enabled) {
    if (config->actuation_enabled ||
        config->backend.runtime.unverified_protocol_acknowledged ||
        config->backend.runtime.physical_device_opt_in ||
        config->backend.runtime.actuation_opt_in ||
        config->backend.runtime.readiness_gate_passed ||
        config->backend.runtime
            .external_or_durable_estop_strategy_approved) {
      *reason =
          "partial physical-backend enablement is rejected before device open";
      return false;
    }
  } else if (!auto_rover::wheeltec_serial::
                  kPhysicalActuationReleaseEnabled) {
    *reason =
        "physical actuation is release-frozen before device preparation/open";
    return false;
  } else if (!config->actuation_enabled || !all_runtime_gates ||
             !auto_rover::wheeltec_serial::physicalDeviceOptionsAreComplete(
                 config->physical)) {
    *reason =
        "enabled physical backend requires every actuation and identity gate";
    return false;
  }

  config->backend.chassis_frame_id =
      config->core.vehicle_profile.reference_frame;
  return true;
}

class WheeltecVehicleExecutionNode {
 public:
  WheeltecVehicleExecutionNode(
      ros::NodeHandle node, ros::NodeHandle private_node,
      const WheeltecExecutionNodeConfig& config)
      : node_(std::move(node)),
        private_node_(std::move(private_node)),
        worker_period_(config.cycle_period_ns),
        real_device_enabled_(config.real_device_enabled) {
    auto_rover::wheeltec_serial::PhysicalActivationConfig activation_config;
    activation_config.codec_limits =
        config.backend.runtime.adapter.codec_limits;
    activation_config.write_timeout_ns =
        config.backend.runtime.adapter.write_timeout_ns;
    activation_config.zero_retry_interval_ns =
        config.backend.runtime.adapter.zero_retry_interval_ns;
    activation_config.max_zero_write_attempts =
        config.backend.runtime.adapter.max_zero_write_attempts;
    auto_rover::wheeltec_serial::PhysicalActivationOperations
        activation_operations;
    activation_operations.monotonic_now_ns = []() { return monotonicNow(); };
    activation_operations.wait_until_monotonic_ns = waitUntilMonotonicNs;
    physical_activation_.reset(
        new auto_rover::wheeltec_serial::PreparedPhysicalActivation(
            activation_config, activation_operations));
    if (!physical_activation_->prepared()) {
      throw std::runtime_error(
          "Wheeltec physical activation guard preparation failed");
    }

    auto_rover::wheeltec_serial::WheeltecVehicleBackendOperations operations;
    operations.runtime.monotonic_now_ns = []() { return monotonicNow(); };
    operations.ros_now_ns = []() { return rosNow(); };
    std::unique_ptr<auto_rover::wheeltec_serial::WheeltecVehicleBackend>
        concrete_backend(
        new auto_rover::wheeltec_serial::WheeltecVehicleBackend(
            config.backend, nullptr, operations));
    wheeltec_backend_ = concrete_backend.get();
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(
        std::move(concrete_backend));
    runtime_.reset(new auto_rover_vehicle::VehicleExecutionCore(
        config.core, std::move(backend)));

    ego_subscriber_ = node_.subscribe(
        "ego_state", 1, &WheeltecVehicleExecutionNode::egoCallback, this);
    trajectory_subscriber_ = node_.subscribe(
        "trajectory", 1,
        &WheeltecVehicleExecutionNode::trajectoryCallback, this);
    motion_subscriber_ = node_.subscribe(
        "motion_reference", 1,
        &WheeltecVehicleExecutionNode::motionReferenceCallback, this);
    emergency_stop_subscriber_ = node_.subscribe(
        "emergency_stop", 10,
        &WheeltecVehicleExecutionNode::emergencyStopCallback, this);

    chassis_publisher_ = node_.advertise<auto_rover_interfaces::ChassisState>(
        "chassis_state", 1);
    safety_publisher_ = node_.advertise<auto_rover_interfaces::SafetyState>(
        "safety_state", 1);
    arm_service_ = node_.advertiseService(
        "arm_vehicle", &WheeltecVehicleExecutionNode::armCallback, this);
    assert_emergency_stop_service_ = node_.advertiseService(
        "assert_emergency_stop",
        &WheeltecVehicleExecutionNode::assertEmergencyStopCallback, this);
    reset_service_ = node_.advertiseService(
        "reset_emergency_stop",
        &WheeltecVehicleExecutionNode::resetCallback, this);
    publication_timer_ = node_.createWallTimer(
        ros::WallDuration(config.cycle_period_s),
        &WheeltecVehicleExecutionNode::publicationCallback, this, false,
        false);
  }

  bool activatePhysical(
      std::unique_ptr<auto_rover::wheeltec_serial::ByteTransport> transport,
      std::int64_t now_monotonic_ns, std::int64_t now_ros_ns) noexcept {
    if (activated_ || !real_device_enabled_ || transport == nullptr ||
        transport_ != nullptr || physical_activation_ == nullptr ||
        wheeltec_backend_ == nullptr || now_monotonic_ns <= 0 ||
        now_ros_ns <= 0) {
      return false;
    }
    transport_ = std::move(transport);
    try {
      activation_result_ = physical_activation_->activate(transport_.get());
      activation_stream_poisoned_ =
          activation_result_.write_stream_poisoned;
      if (!activation_result_.succeeded()) {
        stopAfterActivationFailure();
        return false;
      }
      if (!wheeltec_backend_->setTransportForInitialization(
              transport_.get())) {
        stopAfterActivationFailure();
        return false;
      }
      backend_activation_started_ = true;
      if (!initializeAndStart(now_monotonic_ns, now_ros_ns)) {
        stopAfterActivationFailure();
        return false;
      }
      return true;
    } catch (...) {
      stopAfterActivationFailure();
      return false;
    }
  }

  bool activateDisabled(std::int64_t now_monotonic_ns,
                        std::int64_t now_ros_ns) noexcept {
    if (activated_ || real_device_enabled_ || transport_ != nullptr ||
        now_monotonic_ns <= 0 || now_ros_ns <= 0) {
      return false;
    }
    try {
      if (!initializeAndStart(now_monotonic_ns, now_ros_ns)) {
        stopAfterActivationFailure();
        return false;
      }
      return true;
    } catch (...) {
      stopAfterActivationFailure();
      return false;
    }
  }

  const auto_rover::wheeltec_serial::PhysicalActivationResult&
  activationResult() const noexcept {
    return activation_result_;
  }

  ~WheeltecVehicleExecutionNode() noexcept {
    try {
      // Stop accepting ROS work, stop and join the watchdog worker, then run
      // the poison-aware bounded stop while the physical transport is still
      // owned.  ROS graph teardown is deliberately last: it must not be able
      // to prevent the physical stop attempt.
      accepting_ros_callbacks_.store(false);
      stop_worker_.store(true);
      worker_wakeup_.notify_all();
      if (worker_.joinable()) {
        worker_.join();
      }
    } catch (...) {
      // Continue to the bounded core shutdown even if thread teardown reports
      // an OS-level failure.  No direct serial fallback is safe after a
      // potentially partial frame.
    }
    try {
      if (runtime_ != nullptr) {
        std::lock_guard<std::mutex> lock(core_mutex_);
        const auto stopped = runtime_->shutdown(monotonicNow());
        if (stopped.delivery_unconfirmed) {
          ROS_ERROR_STREAM(
              "Wheeltec backend shutdown stop delivery is unconfirmed: "
              << stopped.diagnostic);
        }
      }
    } catch (...) {
      // Never bypass the poison-aware runtime with a direct frame from a
      // destructor.  A partial frame may already be present on this stream.
      try {
        ROS_ERROR(
            "Wheeltec backend shutdown raised after the bounded stop path");
      } catch (...) {
      }
    }
    try {
      ego_subscriber_.shutdown();
      trajectory_subscriber_.shutdown();
      motion_subscriber_.shutdown();
      emergency_stop_subscriber_.shutdown();
      arm_service_.shutdown();
      assert_emergency_stop_service_.shutdown();
      reset_service_.shutdown();
      publication_timer_.stop();
    } catch (...) {
      // ROS graph teardown cannot invalidate the already-completed physical
      // shutdown sequence and must never escape a noexcept destructor.
    }
    reportWorkerTerminalIfNeeded();
  }

 private:
  static std::int64_t monotonicNow() {
    return toSignedNanoseconds(ros::SteadyTime::now());
  }

  static std::int64_t rosNow() {
    return toSignedNanoseconds(ros::Time::now());
  }

  bool initializeAndStart(std::int64_t now_monotonic_ns,
                          std::int64_t now_ros_ns) {
    if (runtime_ == nullptr ||
        !runtime_->initialize(now_monotonic_ns, now_ros_ns)) {
      return false;
    }
    accepting_ros_callbacks_.store(true);
    stop_worker_.store(false);
    try {
      worker_ = std::thread(&WheeltecVehicleExecutionNode::workerLoop, this);
      publication_timer_.start();
    } catch (...) {
      accepting_ros_callbacks_.store(false);
      stop_worker_.store(true);
      worker_wakeup_.notify_all();
      if (worker_.joinable()) {
        worker_.join();
      }
      return false;
    }
    activated_ = true;
    return true;
  }

  void stopAfterActivationFailure() noexcept {
    accepting_ros_callbacks_.store(false);
    stop_worker_.store(true);
    worker_wakeup_.notify_all();
    try {
      if (worker_.joinable()) {
        worker_.join();
      }
    } catch (...) {
    }
    try {
      publication_timer_.stop();
    } catch (...) {
    }

    if (activation_stream_poisoned_) {
      worker_terminal_delivery_unconfirmed_.store(true);
      return;
    }
    if (backend_activation_started_ && runtime_ != nullptr) {
      try {
        const auto stopped = runtime_->shutdown(monotonicNow());
        worker_terminal_delivery_unconfirmed_.store(
            stopped.delivery_unconfirmed);
      } catch (...) {
        worker_terminal_delivery_unconfirmed_.store(true);
      }
      return;
    }
    if (activation_result_.succeeded() && physical_activation_ != nullptr &&
        transport_ != nullptr) {
      activation_result_ = physical_activation_->activate(transport_.get());
      activation_stream_poisoned_ =
          activation_result_.write_stream_poisoned;
      worker_terminal_delivery_unconfirmed_.store(
          !activation_result_.succeeded());
    }
  }

  bool rosCallbackMayEnter() const {
    return accepting_ros_callbacks_.load();
  }

  enum class WorkerTerminalReason : std::uint8_t {
    kNone = 0,
    kDeadlineMiss,
    kCycleOverrun,
    kMailboxFailure,
    kRosCallbackException,
    kUnhandledException,
  };

  void terminalWorkerStopLocked(WorkerTerminalReason reason) noexcept {
    bool delivery_unconfirmed = true;
    try {
      const auto stopped = runtime_->shutdown(monotonicNow());
      delivery_unconfirmed = stopped.delivery_unconfirmed;
    } catch (...) {
      // Do not append an unchecked zero frame here: the runtime may have
      // marked this connection generation poisoned after a partial write.
      delivery_unconfirmed = true;
    }
    worker_terminal_delivery_unconfirmed_.store(delivery_unconfirmed);
    worker_terminal_reason_.store(reason);
    accepting_ros_callbacks_.store(false);
    // The bounded stop has already run. This wakes ros::spin so destruction
    // can close the still-owned transport; it is not part of the watchdog.
    try {
      ros::requestShutdown();
    } catch (...) {
    }
  }

  void terminalWorkerStop(WorkerTerminalReason reason) noexcept {
    try {
      std::lock_guard<std::mutex> lock(core_mutex_);
      terminalWorkerStopLocked(reason);
    } catch (...) {
      worker_terminal_delivery_unconfirmed_.store(true);
      worker_terminal_reason_.store(reason);
      accepting_ros_callbacks_.store(false);
      try {
        ros::requestShutdown();
      } catch (...) {
      }
    }
  }

  enum class MailboxStoreResult : std::uint8_t {
    kStored = 0,
    kContended,
    kLate,
  };

  MailboxStoreResult tryStoreCycleResult(
      auto_rover_vehicle::VehicleExecutionCycleResult result,
      const std::chrono::steady_clock::time_point latest_completion) {
    std::unique_lock<std::mutex> lock(publication_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return MailboxStoreResult::kContended;
    }
    pending_cycle_result_ = std::move(result);
    cycle_result_pending_ = true;
    if (std::chrono::steady_clock::now() >= latest_completion) {
      // Do not publish a result that may still describe the pre-stop armed
      // cycle after the physical worker has declared a timing failure.
      cycle_result_pending_ = false;
      return MailboxStoreResult::kLate;
    }
    return MailboxStoreResult::kStored;
  }

  void workerLoop() noexcept {
    try {
      using Clock = std::chrono::steady_clock;
      auto next_deadline = Clock::now() + worker_period_;
      for (;;) {
        {
          std::unique_lock<std::mutex> wait_lock(worker_wait_mutex_);
          if (worker_wakeup_.wait_until(
                  wait_lock, next_deadline,
                  [this]() { return stop_worker_.load(); })) {
            return;
          }
        }

        const auto cycle_started = Clock::now();
        // A whole scheduled period was missed. Do not issue a late motion
        // refresh; make the backend terminal and synchronously attempt zero.
        if (cycle_started >= next_deadline + worker_period_) {
          terminalWorkerStop(WorkerTerminalReason::kDeadlineMiss);
          return;
        }

        auto_rover_vehicle::VehicleExecutionCycleResult result;
        {
          std::lock_guard<std::mutex> lock(core_mutex_);
          // A ROS callback may have occupied the core mutex after the first
          // deadline check.  Never refresh a motion command after waiting an
          // entire additional worker period for that lock.
          if (Clock::now() >= next_deadline + worker_period_) {
            terminalWorkerStopLocked(WorkerTerminalReason::kDeadlineMiss);
            return;
          }
          try {
            result = runtime_->cycle(monotonicNow(), rosNow());
          } catch (...) {
            terminalWorkerStopLocked(
                WorkerTerminalReason::kUnhandledException);
            return;
          }
          if (Clock::now() >= next_deadline + worker_period_) {
            // Keep the core lock until the terminal stop completes so a ROS
            // service cannot interleave another authorization transition.
            terminalWorkerStopLocked(WorkerTerminalReason::kCycleOverrun);
            return;
          }
          try {
            // The ROS publication thread must never be able to block the
            // physical watchdog worker.  Failure to acquire or populate the
            // one-slot mailbox is terminal while the core lock is still held.
            const MailboxStoreResult mailbox_result = tryStoreCycleResult(
                std::move(result), next_deadline + worker_period_);
            if (mailbox_result == MailboxStoreResult::kContended) {
              terminalWorkerStopLocked(WorkerTerminalReason::kMailboxFailure);
              return;
            }
            if (mailbox_result == MailboxStoreResult::kLate) {
              terminalWorkerStopLocked(WorkerTerminalReason::kCycleOverrun);
              return;
            }
          } catch (...) {
            terminalWorkerStopLocked(WorkerTerminalReason::kMailboxFailure);
            return;
          }
        }
        next_deadline += worker_period_;
      }
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kUnhandledException);
    }
  }

  void publicationCallback(const ros::WallTimerEvent&) {
    auto_rover_vehicle::VehicleExecutionCycleResult result;
    bool available = false;
    {
      std::lock_guard<std::mutex> lock(publication_mutex_);
      if (cycle_result_pending_) {
        result = std::move(pending_cycle_result_);
        cycle_result_pending_ = false;
        available = true;
      }
    }
    if (available) {
      publishCycleResult(result);
    }
    reportWorkerTerminalIfNeeded();
  }

  void reportWorkerTerminalIfNeeded() noexcept {
    if (worker_terminal_reported_.exchange(true)) {
      return;
    }
    const WorkerTerminalReason reason = worker_terminal_reason_.load();
    if (reason == WorkerTerminalReason::kNone) {
      worker_terminal_reported_.store(false);
      return;
    }
    try {
      ROS_ERROR_STREAM(
          "Wheeltec monotonic execution worker terminated with reason code "
          << static_cast<unsigned int>(reason));
      if (worker_terminal_delivery_unconfirmed_.load()) {
        ROS_ERROR(
            "Wheeltec worker terminal physical zero delivery is unconfirmed");
      }
    } catch (...) {
    }
  }

  void egoCallback(const auto_rover_interfaces::EgoState::ConstPtr& message) {
    if (!rosCallbackMayEnter()) {
      return;
    }
    try {
      if (message == nullptr || !executionInputIsBounded(*message)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return;
      }
      const auto value = auto_rover_ros1::toCore(*message);
      std::lock_guard<std::mutex> lock(core_mutex_);
      if (rosCallbackMayEnter()) {
        runtime_->updateEgoState(value, monotonicNow());
      }
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
    }
  }

  void trajectoryCallback(
      const auto_rover_interfaces::Trajectory::ConstPtr& message) {
    if (!rosCallbackMayEnter()) {
      return;
    }
    try {
      if (message == nullptr || !executionInputIsBounded(*message)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return;
      }
      const auto value = auto_rover_ros1::toCore(*message);
      std::lock_guard<std::mutex> lock(core_mutex_);
      if (rosCallbackMayEnter()) {
        runtime_->updateTrajectory(value, monotonicNow());
      }
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
    }
  }

  void motionReferenceCallback(
      const auto_rover_interfaces::MotionReference::ConstPtr& message) {
    if (!rosCallbackMayEnter()) {
      return;
    }
    try {
      if (message == nullptr || !executionInputIsBounded(*message)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return;
      }
      const auto value = auto_rover_ros1::toCore(*message);
      std::lock_guard<std::mutex> lock(core_mutex_);
      if (rosCallbackMayEnter()) {
        runtime_->updateMotionReference(value, monotonicNow());
      }
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
    }
  }

  void emergencyStopCallback(
      const auto_rover_interfaces::EmergencyStop::ConstPtr& message) {
    if (!rosCallbackMayEnter()) {
      return;
    }
    try {
      if (message == nullptr || !executionInputIsBounded(*message)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return;
      }
      const auto assertion = auto_rover_ros1::toCore(*message);
      auto_rover_vehicle::VehicleExecutionServiceResult result;
      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        if (!rosCallbackMayEnter()) {
          return;
        }
        result = runtime_->handleEmergencyStop(assertion, monotonicNow(),
                                               rosNow());
      }
      safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
      if (result.stop_delivery.delivery_unconfirmed) {
        ROS_ERROR_STREAM(
            "Emergency-stop latch "
            << (result.success ? "accepted" : "processed")
            << ", but Wheeltec physical zero delivery is unconfirmed: "
            << result.stop_delivery.diagnostic);
      }
      if (!result.success) {
        ROS_ERROR_STREAM("Emergency-stop assertion rejected after execution "
                         "was revoked: "
                         << result.reason);
      }
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
    }
  }

  void publishCycleResult(
      const auto_rover_vehicle::VehicleExecutionCycleResult& result) {
    if (result.chassis_available) {
      chassis_publisher_.publish(auto_rover_ros1::toRos(result.chassis));
    }
    safety_publisher_.publish(auto_rover_ros1::toRos(result.safety));
    if (!result.health_clear) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Wheeltec vehicle execution inhibited: "
                   << result.health_diagnostic);
    }
    if (result.delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Wheeltec backend host delivery is unconfirmed: "
                   << result.delivery.diagnostic);
    }
    if (result.stop_delivery.delivery_unconfirmed) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Wheeltec physical zero delivery is unconfirmed: "
                   << result.stop_delivery.diagnostic);
    }
  }

  bool armCallback(auto_rover_interfaces::ArmVehicle::Request& request,
                   auto_rover_interfaces::ArmVehicle::Response& response) {
    if (!rosCallbackMayEnter()) {
      response.success = false;
      response.reason = "Wheeltec execution node is shutting down";
      return true;
    }
    try {
      if (!executionInputStringIsBounded(request.operator_id)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return false;
      }
      auto_rover_vehicle::VehicleExecutionServiceResult result;
      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        if (!rosCallbackMayEnter()) {
          response.success = false;
          response.reason = "Wheeltec execution node is shutting down";
          return true;
        }
        result = runtime_->requestArm(
            request.operator_id, request.safety_generation, request.arm,
            monotonicNow(), rosNow());
      }
      response.success = result.success;
      response.reason = result.reason;
      response.safety_mode = static_cast<std::uint8_t>(result.state.mode);
      safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
      if (result.stop_delivery.delivery_unconfirmed) {
        ROS_ERROR_STREAM("Arm/disarm request completed with unconfirmed "
                         "Wheeltec physical zero delivery: "
                         << result.stop_delivery.diagnostic);
      }
      return true;
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
      return false;
    }
  }

  bool assertEmergencyStopCallback(
      auto_rover_interfaces::AssertEmergencyStop::Request& request,
      auto_rover_interfaces::AssertEmergencyStop::Response& response) {
    if (!rosCallbackMayEnter()) {
      response.success = false;
      response.reason = "Wheeltec execution node is shutting down";
      return true;
    }
    try {
      if (!executionInputStringIsBounded(request.request_id) ||
          !executionInputStringIsBounded(request.source_id)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return false;
      }
      auto_rover::EmergencyStop assertion;
      if (request.schema_version == 1U) {
        assertion.stamp_ns = toSignedNanoseconds(request.source_stamp);
        assertion.request_id = request.request_id;
        assertion.source_id = request.source_id;
        assertion.asserted = true;
      }
      auto_rover_vehicle::VehicleExecutionServiceResult result;
      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        if (!rosCallbackMayEnter()) {
          response.success = false;
          response.reason = "Wheeltec execution node is shutting down";
          return true;
        }
        result = runtime_->handleEmergencyStop(assertion, monotonicNow(),
                                               rosNow());
      }
      response.success = result.success;
      response.reason = result.reason;
      response.safety_mode = static_cast<std::uint8_t>(result.state.mode);
      response.latch_generation = result.state.latch_generation;
      safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
      if (result.stop_delivery.delivery_unconfirmed) {
        ROS_ERROR_STREAM(
            "Emergency-stop latch "
            << (result.success ? "accepted" : "processed")
            << ", but Wheeltec physical zero delivery is unconfirmed: "
            << result.stop_delivery.diagnostic);
      }
      return true;
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
      return false;
    }
  }

  bool resetCallback(
      auto_rover_interfaces::ResetEmergencyStop::Request& request,
      auto_rover_interfaces::ResetEmergencyStop::Response& response) {
    if (!rosCallbackMayEnter()) {
      response.success = false;
      response.reason = "Wheeltec execution node is shutting down";
      return true;
    }
    try {
      if (!executionInputStringIsBounded(request.operator_id)) {
        terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
        return false;
      }
      auto_rover_vehicle::VehicleExecutionServiceResult result;
      {
        std::lock_guard<std::mutex> lock(core_mutex_);
        if (!rosCallbackMayEnter()) {
          response.success = false;
          response.reason = "Wheeltec execution node is shutting down";
          return true;
        }
        result = runtime_->resetEmergencyStop(
            request.operator_id, request.latch_generation,
            request.conditions_cleared_acknowledged, monotonicNow(), rosNow());
      }
      response.success = result.success;
      response.reason = result.reason;
      response.safety_mode = static_cast<std::uint8_t>(result.state.mode);
      response.current_latch_generation = result.state.latch_generation;
      safety_publisher_.publish(auto_rover_ros1::toRos(result.state));
      if (result.stop_delivery.delivery_unconfirmed) {
        ROS_ERROR_STREAM("Emergency-stop reset completed with unconfirmed "
                         "Wheeltec physical zero delivery: "
                         << result.stop_delivery.diagnostic);
      }
      return true;
    } catch (...) {
      terminalWorkerStop(WorkerTerminalReason::kRosCallbackException);
      return false;
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::unique_ptr<
      auto_rover::wheeltec_serial::PreparedPhysicalActivation>
      physical_activation_;
  std::unique_ptr<auto_rover::wheeltec_serial::ByteTransport> transport_;
  std::unique_ptr<auto_rover_vehicle::VehicleExecutionCore> runtime_;
  auto_rover::wheeltec_serial::WheeltecVehicleBackend* wheeltec_backend_{
      nullptr};
  const std::chrono::nanoseconds worker_period_;
  const bool real_device_enabled_{false};
  auto_rover::wheeltec_serial::PhysicalActivationResult activation_result_{};
  bool activation_stream_poisoned_{false};
  bool backend_activation_started_{false};
  bool activated_{false};
  std::mutex core_mutex_;
  std::mutex worker_wait_mutex_;
  std::mutex publication_mutex_;
  std::condition_variable worker_wakeup_;
  std::atomic<bool> stop_worker_{false};
  std::atomic<bool> accepting_ros_callbacks_{false};
  std::atomic<WorkerTerminalReason> worker_terminal_reason_{
      WorkerTerminalReason::kNone};
  std::atomic<bool> worker_terminal_delivery_unconfirmed_{true};
  std::atomic<bool> worker_terminal_reported_{false};
  std::thread worker_;
  auto_rover_vehicle::VehicleExecutionCycleResult pending_cycle_result_;
  bool cycle_result_pending_{false};
  ros::Subscriber ego_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber motion_subscriber_;
  ros::Subscriber emergency_stop_subscriber_;
  ros::Publisher chassis_publisher_;
  ros::Publisher safety_publisher_;
  ros::ServiceServer arm_service_;
  ros::ServiceServer assert_emergency_stop_service_;
  ros::ServiceServer reset_service_;
  ros::WallTimer publication_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheeltec_vehicle_execution");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  WheeltecExecutionNodeConfig config;
  std::string reason;
  if (!loadWheeltecConfig(private_node, &config, &reason)) {
    ROS_FATAL_STREAM(
        "Wheeltec vehicle execution configuration rejected before device "
        "open: "
        << reason);
    return EXIT_FAILURE;
  }
  const std::string generation = makeProcessGenerationId();
  if (generation.empty()) {
    ROS_FATAL("Wheeltec backend could not establish a process generation");
    return EXIT_FAILURE;
  }
  config.backend.source_id =
      "wheeltec_serial_candidate_v1#process=" + generation;

  try {
    // Construct the backend/core, activation frame, ROS graph resources,
    // mailbox and timer before acquiring a physical command channel.
    WheeltecVehicleExecutionNode wrapper(node, private_node, config);
    const std::int64_t preopen_monotonic_ns =
        toSignedNanoseconds(ros::SteadyTime::now());
    const std::int64_t preopen_ros_ns =
        toSignedNanoseconds(ros::Time::now());
    if (preopen_monotonic_ns <= 0 || preopen_ros_ns <= 0) {
      ROS_FATAL(
          "Wheeltec physical serial open rejected because startup clocks "
          "are invalid");
      return EXIT_FAILURE;
    }

    if (config.real_device_enabled) {
      // String/sysfs identity work and transport storage allocation are
      // completed before raw ::open.  open() is the noexcept, one-shot
      // syscall-only boundary consumed immediately by the exact-zero guard.
      auto_rover::wheeltec_serial::PreparedPhysicalSerialOpen prepared_open(
          config.physical);
      auto opened = prepared_open.open();
      if (opened.status !=
              auto_rover::wheeltec_serial::TransportStatus::kOk ||
          opened.transport == nullptr) {
        ROS_FATAL_STREAM("Wheeltec physical serial open rejected: status="
                         << static_cast<int>(opened.status)
                         << " os_error=" << opened.os_error);
        return EXIT_FAILURE;
      }
      // No logging, metadata, read, allocation, or ROS construction is placed
      // between successful open and this prepared no-record exact-zero guard.
      if (!wrapper.activatePhysical(std::move(opened.transport),
                                    preopen_monotonic_ns,
                                    preopen_ros_ns)) {
        const auto& activation = wrapper.activationResult();
        ROS_FATAL_STREAM(
            "Wheeltec physical activation failed after bounded exact-zero: "
            << auto_rover::wheeltec_serial::physicalActivationStatusName(
                   activation.status)
            << " write_stream_poisoned="
            << (activation.write_stream_poisoned ? "true" : "false")
            << " delivery_unconfirmed="
            << (activation.delivery_unconfirmed ? "true" : "false"));
        return EXIT_FAILURE;
      }
    } else if (!wrapper.activateDisabled(preopen_monotonic_ns,
                                         preopen_ros_ns)) {
      ROS_FATAL("Wheeltec disabled backend initialization failed");
      return EXIT_FAILURE;
    }
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Wheeltec vehicle execution startup rejected: "
                     << error.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
