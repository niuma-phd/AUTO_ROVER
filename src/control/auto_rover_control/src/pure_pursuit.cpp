#include "auto_rover_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"

namespace auto_rover_control {
namespace {

constexpr double kTolerance = 1e-9;
constexpr double kNanosecondsPerSecond = 1000000000.0;

bool positiveFinite(double value) {
  return auto_rover::isFinite(value) && value > 0.0;
}

}  // namespace

auto_rover::ValidationResult validatePurePursuitConfig(
    const PurePursuitConfig& config) {
  if (config.world_frame.empty() || config.control_frame.empty() ||
      config.producer_generation_id.empty()) {
    return auto_rover::ValidationResult::failure(
        "control frames and producer generation must be configured");
  }
  if (!positiveFinite(config.lookahead_min_m) ||
      !positiveFinite(config.lookahead_max_m) ||
      config.lookahead_min_m > config.lookahead_max_m ||
      !auto_rover::isFinite(config.lookahead_speed_gain_s) ||
      config.lookahead_speed_gain_s < 0.0 ||
      !positiveFinite(config.goal_position_tolerance_m) ||
      !auto_rover::isFinite(config.standstill_speed_threshold_mps) ||
      config.standstill_speed_threshold_mps < 0.0) {
    return auto_rover::ValidationResult::failure(
        "control geometry configuration is invalid");
  }
  if (config.localization_freshness_ns <= 0 ||
      config.trajectory_freshness_ns <= 0 ||
      config.chassis_freshness_ns <= 0 || config.motion_valid_for_ns <= 0) {
    return auto_rover::ValidationResult::failure(
        "control time limits must be positive");
  }
  return auto_rover::ValidationResult::success();
}

PurePursuit::PurePursuit(PurePursuitConfig config,
                         auto_rover::VehicleProfile vehicle_profile)
    : config_(std::move(config)), vehicle_profile_(std::move(vehicle_profile)) {}

void PurePursuit::reset() {
  active_trajectory_id_.clear();
  progress_index_ = 0U;
  last_update_monotonic_ns_ = 0;
  last_commanded_speed_mps_ = 0.0;
  control_enable_observed_ = false;
}

auto_rover::MotionReference PurePursuit::makeReference(
    const TrackingInput& input, double speed_mps, double curvature_inv_m,
    bool route_complete, bool valid) {
  auto_rover::MotionReference output;
  output.stamp_ns = input.now_ros_ns;
  output.frame_id = config_.control_frame;
  output.command_id = next_command_id_++;
  output.producer_generation_id = config_.producer_generation_id;
  output.trajectory_id = input.trajectory.value.trajectory_id.empty()
                             ? "invalid_trajectory"
                             : input.trajectory.value.trajectory_id;
  output.direction = auto_rover::Direction::kForward;
  output.target_speed_mps = speed_mps;
  output.target_curvature_inv_m = curvature_inv_m;
  output.valid_for_ns = config_.motion_valid_for_ns;
  output.route_complete = route_complete;
  output.valid = valid;
  return output;
}

TrackingResult PurePursuit::invalidResult(const TrackingInput& input,
                                          const std::string& reason) {
  last_commanded_speed_mps_ = 0.0;
  last_update_monotonic_ns_ = input.now_monotonic_ns;
  control_enable_observed_ = false;
  TrackingResult output;
  output.reference = makeReference(input, 0.0, 0.0, false, false);
  output.reason = reason;
  output.nearest_index = progress_index_;
  output.target_index = progress_index_;
  return output;
}

double PurePursuit::rateLimitSpeed(double desired_speed_mps,
                                   std::int64_t now_monotonic_ns) {
  desired_speed_mps = std::max(0.0, std::min(desired_speed_mps,
                                             vehicle_profile_.max_forward_speed_mps));
  if (last_update_monotonic_ns_ <= 0 ||
      now_monotonic_ns <= last_update_monotonic_ns_) {
    last_update_monotonic_ns_ = now_monotonic_ns;
    last_commanded_speed_mps_ = 0.0;
    return 0.0;
  }
  const double elapsed_seconds =
      static_cast<double>(now_monotonic_ns - last_update_monotonic_ns_) /
      kNanosecondsPerSecond;
  const double maximum_change =
      vehicle_profile_.max_longitudinal_accel_mps2 * elapsed_seconds;
  const double lower = std::max(0.0, last_commanded_speed_mps_ - maximum_change);
  const double upper =
      std::min(vehicle_profile_.max_forward_speed_mps,
               last_commanded_speed_mps_ + maximum_change);
  last_commanded_speed_mps_ = std::max(lower, std::min(desired_speed_mps, upper));
  last_update_monotonic_ns_ = now_monotonic_ns;
  return last_commanded_speed_mps_;
}

auto_rover::ValidationResult PurePursuit::observeRequiredStateOrder(
    const std::string& label, const std::string& source_id,
    std::uint64_t state_id, std::int64_t stamp_ns,
    std::int64_t receipt_monotonic_ns,
    RequiredStateOrder* order) {
  if (!order->initialized) {
    order->initialized = true;
    order->source_id = source_id;
    order->state_id = state_id;
    order->stamp_ns = stamp_ns;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::success();
  }
  if (source_id != order->source_id) {
    if (order->retired_source_ids.count(source_id) != 0U) {
      return auto_rover::ValidationResult::failure(
          label + " source identity was already retired");
    }
    if (receipt_monotonic_ns <= order->receipt_monotonic_ns) {
      return auto_rover::ValidationResult::failure(
          label + " receipt did not advance for a new source identity");
    }
    order->retired_source_ids.insert(order->source_id);
    order->source_id = source_id;
    order->state_id = state_id;
    order->stamp_ns = stamp_ns;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    order->compromised = false;
    return auto_rover::ValidationResult::success();
  }
  if (order->compromised) {
    return auto_rover::ValidationResult::failure(
        label + " source identity is compromised; a new identity is required");
  }
  if (receipt_monotonic_ns < order->receipt_monotonic_ns) {
    order->compromised = true;
    return auto_rover::ValidationResult::failure(
        label + " receipt time rolled back");
  }
  if (state_id < order->state_id) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " state identity rolled back");
  }
  if (stamp_ns < order->stamp_ns) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " source stamp rolled back");
  }
  if (state_id == order->state_id && stamp_ns == order->stamp_ns) {
    if (receipt_monotonic_ns == order->receipt_monotonic_ns) {
      return auto_rover::ValidationResult::success();
    }
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " semantic sample was replayed with a new receipt");
  }
  if (state_id == order->state_id) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " state identity was reused with a different stamp");
  }
  if (stamp_ns == order->stamp_ns) {
    order->compromised = true;
    order->receipt_monotonic_ns = receipt_monotonic_ns;
    return auto_rover::ValidationResult::failure(
        label + " source stamp did not advance with state identity");
  }
  if (receipt_monotonic_ns == order->receipt_monotonic_ns) {
    order->compromised = true;
    return auto_rover::ValidationResult::failure(
        label + " semantic state changed without a new receipt");
  }
  order->state_id = state_id;
  order->stamp_ns = stamp_ns;
  order->receipt_monotonic_ns = receipt_monotonic_ns;
  return auto_rover::ValidationResult::success();
}

auto_rover::ValidationResult PurePursuit::observeTrajectoryOrder(
    const auto_rover::Trajectory& trajectory) {
  if (!trajectory_order_.initialized) {
    trajectory_order_.initialized = true;
    trajectory_order_.trajectory_id = trajectory.trajectory_id;
    trajectory_order_.route_id = trajectory.route_id;
    trajectory_order_.plan_version = trajectory.plan_version;
    trajectory_order_.stamp_ns = trajectory.stamp_ns;
    trajectory_order_.highest_plan_versions[trajectory.route_id] =
        trajectory.plan_version;
    return auto_rover::ValidationResult::success();
  }
  if (trajectory.trajectory_id != trajectory_order_.trajectory_id) {
    if (trajectory_order_.retired_trajectory_ids.count(
            trajectory.trajectory_id) != 0U) {
      return auto_rover::ValidationResult::failure(
          "trajectory generation identity was already retired");
    }
    const auto previous = trajectory_order_.highest_plan_versions.find(
        trajectory.route_id);
    if (previous != trajectory_order_.highest_plan_versions.end() &&
        trajectory.plan_version <= previous->second) {
      return auto_rover::ValidationResult::failure(
          "trajectory plan version did not advance for a new generation");
    }
    trajectory_order_.retired_trajectory_ids.insert(
        trajectory_order_.trajectory_id);
    trajectory_order_.trajectory_id = trajectory.trajectory_id;
    trajectory_order_.route_id = trajectory.route_id;
    trajectory_order_.plan_version = trajectory.plan_version;
    trajectory_order_.stamp_ns = trajectory.stamp_ns;
    trajectory_order_.highest_plan_versions[trajectory.route_id] =
        trajectory.plan_version;
    trajectory_order_.compromised = false;
    return auto_rover::ValidationResult::success();
  }
  if (trajectory_order_.compromised) {
    return auto_rover::ValidationResult::failure(
        "trajectory generation identity is compromised; a new identity is required");
  }
  if (trajectory.route_id != trajectory_order_.route_id ||
      trajectory.plan_version != trajectory_order_.plan_version) {
    trajectory_order_.compromised = true;
    return auto_rover::ValidationResult::failure(
        "trajectory route or plan identity changed within one generation");
  }
  if (trajectory.stamp_ns < trajectory_order_.stamp_ns) {
    trajectory_order_.compromised = true;
    return auto_rover::ValidationResult::failure(
        "trajectory source stamp rolled back");
  }
  trajectory_order_.stamp_ns = trajectory.stamp_ns;
  return auto_rover::ValidationResult::success();
}

TrackingResult PurePursuit::update(const TrackingInput& input) {
  const auto_rover::ValidationResult config_result =
      validatePurePursuitConfig(config_);
  if (!config_result.ok) {
    return invalidResult(input, config_result.reason);
  }
  const auto_rover::ValidationResult profile_result =
      auto_rover::validateNucPhase1Profile(vehicle_profile_);
  if (!profile_result.ok) {
    return invalidResult(input, profile_result.reason);
  }
  if (input.now_monotonic_ns <= 0 || input.now_ros_ns <= 0) {
    return invalidResult(input, "current time is invalid");
  }
  if (!auto_rover::isFresh(input.ego.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.localization_freshness_ns)) {
    return invalidResult(input, "localization is stale");
  }
  if (!auto_rover::isFresh(input.trajectory.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.trajectory_freshness_ns)) {
    return invalidResult(input, "trajectory is stale");
  }
  if (!auto_rover::isFresh(input.chassis.receipt_monotonic_ns,
                           input.now_monotonic_ns,
                           config_.chassis_freshness_ns)) {
    return invalidResult(input, "chassis state is stale");
  }
  if (!auto_rover::withinDeclaredValidity(input.trajectory.value.stamp_ns,
                                          input.now_ros_ns,
                                          input.trajectory.value.valid_for_ns)) {
    return invalidResult(input, "trajectory declared validity expired");
  }
  const auto_rover::ValidationResult ego_result = auto_rover::validateEgoState(
      input.ego.value, config_.world_frame, config_.control_frame);
  if (!ego_result.ok) {
    return invalidResult(input, ego_result.reason);
  }
  const auto_rover::ValidationResult trajectory_result =
      auto_rover::validateTrajectory(input.trajectory.value, vehicle_profile_,
                                     config_.world_frame);
  if (!trajectory_result.ok) {
    return invalidResult(input, trajectory_result.reason);
  }
  const auto_rover::ValidationResult chassis_result =
      auto_rover::validateChassisState(input.chassis.value, vehicle_profile_, true);
  if (!chassis_result.ok) {
    return invalidResult(input, chassis_result.reason);
  }

  auto_rover::ValidationResult order_result = observeRequiredStateOrder(
      "ego", input.ego.value.source_id, input.ego.value.state_id,
      input.ego.value.stamp_ns, input.ego.receipt_monotonic_ns,
      &ego_order_);
  if (!order_result.ok) {
    return invalidResult(input, order_result.reason);
  }
  order_result = observeTrajectoryOrder(input.trajectory.value);
  if (!order_result.ok) {
    return invalidResult(input, order_result.reason);
  }
  order_result = observeRequiredStateOrder(
      "chassis", input.chassis.value.source_id,
      input.chassis.value.state_id, input.chassis.value.stamp_ns,
      input.chassis.receipt_monotonic_ns,
      &chassis_order_);
  if (!order_result.ok) {
    return invalidResult(input, order_result.reason);
  }

  const auto& trajectory = input.trajectory.value;
  if (active_trajectory_id_ != trajectory.trajectory_id) {
    active_trajectory_id_ = trajectory.trajectory_id;
    progress_index_ = 0U;
    last_update_monotonic_ns_ = 0;
    last_commanded_speed_mps_ = 0.0;
  }

  const bool control_enable_substantiated =
      (input.chassis.value.valid_mask &
       auto_rover::ChassisState::kControlEnabledValid) != 0U;
  if (!control_enable_substantiated) {
    return invalidResult(input, "chassis control-enable state is unavailable");
  }
  if (!input.chassis.value.control_enabled) {
    last_commanded_speed_mps_ = 0.0;
    last_update_monotonic_ns_ = input.now_monotonic_ns;
    control_enable_observed_ = false;
    TrackingResult output;
    output.reference = makeReference(input, 0.0, 0.0, false, true);
    output.reason = "vehicle control is disabled; holding zero";
    return output;
  }
  if (!control_enable_observed_) {
    last_commanded_speed_mps_ = 0.0;
    last_update_monotonic_ns_ = input.now_monotonic_ns;
    control_enable_observed_ = true;
    TrackingResult output;
    output.reference = makeReference(input, 0.0, 0.0, false, true);
    output.reason = "vehicle control enabled; establishing zero ramp origin";
    return output;
  }

  const double ego_x = input.ego.value.pose.position.x;
  const double ego_y = input.ego.value.pose.position.y;
  double nearest_distance = std::numeric_limits<double>::infinity();
  std::size_t nearest_index = progress_index_;
  for (std::size_t index = progress_index_; index < trajectory.points.size();
       ++index) {
    const double distance = auto_rover::distance2d(
        ego_x, ego_y, trajectory.points[index].x_m,
        trajectory.points[index].y_m);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  progress_index_ = nearest_index;

  const auto& final_point = trajectory.points.back();
  const double goal_distance =
      auto_rover::distance2d(ego_x, ego_y, final_point.x_m, final_point.y_m);
  const double remaining_distance =
      std::max(0.0, final_point.arc_length_m -
                        trajectory.points[nearest_index].arc_length_m);
  const bool position_reached =
      goal_distance <= config_.goal_position_tolerance_m;
  const bool standstill =
      std::abs(input.chassis.value.measured_speed_mps) <=
      config_.standstill_speed_threshold_mps;

  if (position_reached && standstill) {
    last_commanded_speed_mps_ = 0.0;
    last_update_monotonic_ns_ = input.now_monotonic_ns;
    TrackingResult output;
    output.reference = makeReference(input, 0.0, 0.0, true, true);
    output.reason = "route complete; stop and hold";
    output.nearest_index = nearest_index;
    output.target_index = trajectory.points.size() - 1U;
    output.remaining_distance_m = remaining_distance;
    return output;
  }

  if (position_reached) {
    const double limited_speed =
        rateLimitSpeed(0.0, input.now_monotonic_ns);
    TrackingResult output;
    output.reference = makeReference(input, limited_speed, 0.0, false, true);
    output.reason = "goal position reached; decelerating to confirmed standstill";
    output.nearest_index = nearest_index;
    output.target_index = trajectory.points.size() - 1U;
    output.remaining_distance_m = remaining_distance;
    return output;
  }

  const double measured_speed =
      std::abs(input.chassis.value.measured_speed_mps);
  const double lookahead = std::max(
      config_.lookahead_min_m,
      std::min(config_.lookahead_max_m,
               config_.lookahead_min_m +
                   config_.lookahead_speed_gain_s * measured_speed));
  const double target_arc =
      trajectory.points[nearest_index].arc_length_m + lookahead;
  std::size_t target_index = nearest_index;
  while (target_index + 1U < trajectory.points.size() &&
         trajectory.points[target_index].arc_length_m < target_arc) {
    ++target_index;
  }

  const double yaw =
      auto_rover::yawFromQuaternion(input.ego.value.pose.orientation);
  if (!auto_rover::isFinite(yaw)) {
    return invalidResult(input, "ego yaw is invalid");
  }
  const double delta_x = trajectory.points[target_index].x_m - ego_x;
  const double delta_y = trajectory.points[target_index].y_m - ego_y;
  const double local_x = std::cos(yaw) * delta_x + std::sin(yaw) * delta_y;
  const double local_y = -std::sin(yaw) * delta_x + std::cos(yaw) * delta_y;
  const double distance_squared = delta_x * delta_x + delta_y * delta_y;
  if (!auto_rover::isFinite(distance_squared) ||
      distance_squared <= kTolerance) {
    return invalidResult(input, "lookahead geometry is degenerate");
  }
  if (!auto_rover::isFinite(local_x) || local_x <= kTolerance) {
    return invalidResult(
        input, "forward-only lookahead target is not ahead of the vehicle");
  }
  const double curvature = 2.0 * local_y / distance_squared;
  if (!auto_rover::isFinite(curvature) ||
      std::abs(curvature) >
          auto_rover::maxAbsCurvature(vehicle_profile_) + kTolerance) {
    return invalidResult(input, "Pure Pursuit curvature exceeds vehicle profile");
  }

  double desired_speed = trajectory.points[target_index].target_speed_mps;
  const double stopping_speed =
      std::sqrt(std::max(0.0, 2.0 *
                                 vehicle_profile_.max_longitudinal_accel_mps2 *
                                 remaining_distance));
  desired_speed = std::min(desired_speed, stopping_speed);
  const double limited_speed =
      rateLimitSpeed(desired_speed, input.now_monotonic_ns);

  TrackingResult output;
  output.reference =
      makeReference(input, limited_speed, curvature, false, true);
  output.reason = position_reached ? "waiting for confirmed standstill"
                                   : "tracking";
  output.nearest_index = nearest_index;
  output.target_index = target_index;
  output.remaining_distance_m = remaining_distance;
  return output;
}

}  // namespace auto_rover_control
