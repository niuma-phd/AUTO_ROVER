#include "auto_rover_planning/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"
#include "auto_rover_planning/resource_limits.hpp"

namespace auto_rover {
namespace planning {
namespace {

constexpr std::size_t kFeasibilityProbeIntervals = 1024U;
constexpr double kDerivativeNormSquaredFloor = 1e-12;
constexpr double kArcIncrementFloorM = 1e-12;
constexpr double kCurvatureTolerance = 1e-9;
constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct HermiteSample {
  double x_m{0.0};
  double y_m{0.0};
  double derivative_x{0.0};
  double derivative_y{0.0};
  double second_derivative_x{0.0};
  double second_derivative_y{0.0};
};

struct SegmentGeometry {
  RouteWaypoint start;
  RouteWaypoint finish;
  double tangent_start_x{0.0};
  double tangent_start_y{0.0};
  double tangent_finish_x{0.0};
  double tangent_finish_y{0.0};
};

void hashByte(std::uint8_t value, std::uint64_t* hash) {
  *hash ^= static_cast<std::uint64_t>(value);
  *hash *= kFnvPrime;
}

void hashUnsigned(std::uint64_t value, std::uint64_t* hash) {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    hashByte(static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU),
             hash);
  }
}

void hashString(const std::string& value, std::uint64_t* hash) {
  hashUnsigned(static_cast<std::uint64_t>(value.size()), hash);
  for (const unsigned char character : value) {
    hashByte(character, hash);
  }
}

void hashDouble(double value, std::uint64_t* hash) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "trajectory identity requires 64-bit double");
  const double canonical = value == 0.0 ? 0.0 : value;
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &canonical, sizeof(bits));
  hashUnsigned(bits, hash);
}

std::string trajectoryFingerprint(const RoutePlan& route,
                                  const VehicleProfile& profile,
                                  const TrajectoryGeneratorConfig& config) {
  std::uint64_t hash = kFnvOffsetBasis;
  hashString("auto_rover_yaw_hermite_v1", &hash);
  hashUnsigned(route.schema_version, &hash);
  hashString(route.frame_id, &hash);
  hashString(route.route_id, &hash);
  hashUnsigned(route.plan_version, &hash);
  hashUnsigned(route.loop ? 1U : 0U, &hash);
  hashUnsigned(static_cast<std::uint64_t>(route.waypoints.size()), &hash);
  for (const RouteWaypoint& waypoint : route.waypoints) {
    hashDouble(waypoint.x_m, &hash);
    hashDouble(waypoint.y_m, &hash);
    hashDouble(waypoint.yaw_rad, &hash);
    hashDouble(waypoint.speed_mps, &hash);
  }
  hashUnsigned(profile.schema_version, &hash);
  hashString(profile.profile_id, &hash);
  hashUnsigned(static_cast<std::uint8_t>(profile.kinematic_model), &hash);
  hashUnsigned(static_cast<std::uint8_t>(profile.direction_capability), &hash);
  hashString(profile.reference_frame, &hash);
  hashDouble(profile.wheelbase_m, &hash);
  hashDouble(profile.max_forward_speed_mps, &hash);
  hashDouble(profile.max_longitudinal_accel_mps2, &hash);
  hashDouble(profile.min_turning_radius_m, &hash);
  hashUnsigned(profile.reverse_supported ? 1U : 0U, &hash);
  hashDouble(config.sampling_resolution_m, &hash);
  hashUnsigned(static_cast<std::uint64_t>(config.valid_for_ns), &hash);

  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

TrajectoryGenerationResult failure(Trajectory trajectory,
                                   const std::string& reason) {
  trajectory.valid = false;
  trajectory.points.clear();
  TrajectoryGenerationResult result;
  result.trajectory = std::move(trajectory);
  result.validation = ValidationResult::failure(reason);
  return result;
}

HermiteSample evaluate(const SegmentGeometry& segment, double parameter) {
  const double parameter_squared = parameter * parameter;
  const double parameter_cubed = parameter_squared * parameter;

  const double h00 = 2.0 * parameter_cubed - 3.0 * parameter_squared + 1.0;
  const double h10 = parameter_cubed - 2.0 * parameter_squared + parameter;
  const double h01 = -2.0 * parameter_cubed + 3.0 * parameter_squared;
  const double h11 = parameter_cubed - parameter_squared;

  const double dh00 = 6.0 * parameter_squared - 6.0 * parameter;
  const double dh10 = 3.0 * parameter_squared - 4.0 * parameter + 1.0;
  const double dh01 = -6.0 * parameter_squared + 6.0 * parameter;
  const double dh11 = 3.0 * parameter_squared - 2.0 * parameter;

  const double ddh00 = 12.0 * parameter - 6.0;
  const double ddh10 = 6.0 * parameter - 4.0;
  const double ddh01 = -12.0 * parameter + 6.0;
  const double ddh11 = 6.0 * parameter - 2.0;

  HermiteSample sample;
  sample.x_m = h00 * segment.start.x_m +
               h10 * segment.tangent_start_x + h01 * segment.finish.x_m +
               h11 * segment.tangent_finish_x;
  sample.y_m = h00 * segment.start.y_m +
               h10 * segment.tangent_start_y + h01 * segment.finish.y_m +
               h11 * segment.tangent_finish_y;
  sample.derivative_x =
      dh00 * segment.start.x_m + dh10 * segment.tangent_start_x +
      dh01 * segment.finish.x_m + dh11 * segment.tangent_finish_x;
  sample.derivative_y =
      dh00 * segment.start.y_m + dh10 * segment.tangent_start_y +
      dh01 * segment.finish.y_m + dh11 * segment.tangent_finish_y;
  sample.second_derivative_x =
      ddh00 * segment.start.x_m + ddh10 * segment.tangent_start_x +
      ddh01 * segment.finish.x_m + ddh11 * segment.tangent_finish_x;
  sample.second_derivative_y =
      ddh00 * segment.start.y_m + ddh10 * segment.tangent_start_y +
      ddh01 * segment.finish.y_m + ddh11 * segment.tangent_finish_y;
  return sample;
}

bool finite(const HermiteSample& sample) {
  return isFinite(sample.x_m) && isFinite(sample.y_m) &&
         isFinite(sample.derivative_x) && isFinite(sample.derivative_y) &&
         isFinite(sample.second_derivative_x) &&
         isFinite(sample.second_derivative_y);
}

ValidationResult curvature(const HermiteSample& sample, double* output) {
  if (output == nullptr || !finite(sample)) {
    return ValidationResult::failure("Hermite sample is non-finite");
  }
  const double derivative_norm_squared =
      sample.derivative_x * sample.derivative_x +
      sample.derivative_y * sample.derivative_y;
  if (!isFinite(derivative_norm_squared) ||
      derivative_norm_squared <= kDerivativeNormSquaredFloor) {
    return ValidationResult::failure("Hermite curve has a zero tangent");
  }
  const double denominator =
      derivative_norm_squared * std::sqrt(derivative_norm_squared);
  const double numerator =
      sample.derivative_x * sample.second_derivative_y -
      sample.derivative_y * sample.second_derivative_x;
  const double value = numerator / denominator;
  if (!isFinite(value)) {
    return ValidationResult::failure("Hermite curvature is non-finite");
  }
  *output = value;
  return ValidationResult::success();
}

SegmentGeometry makeSegment(const RouteWaypoint& start,
                            const RouteWaypoint& finish) {
  const double scale = distance2d(start.x_m, start.y_m, finish.x_m, finish.y_m);
  SegmentGeometry segment;
  segment.start = start;
  segment.finish = finish;
  segment.tangent_start_x = scale * std::cos(start.yaw_rad);
  segment.tangent_start_y = scale * std::sin(start.yaw_rad);
  segment.tangent_finish_x = scale * std::cos(finish.yaw_rad);
  segment.tangent_finish_y = scale * std::sin(finish.yaw_rad);
  return segment;
}

double derivativeMagnitudeBound(const SegmentGeometry& segment) {
  const double middle_x = 3.0 * (segment.finish.x_m - segment.start.x_m) -
                          segment.tangent_start_x -
                          segment.tangent_finish_x;
  const double middle_y = 3.0 * (segment.finish.y_m - segment.start.y_m) -
                          segment.tangent_start_y -
                          segment.tangent_finish_y;
  return std::max(
      {std::hypot(segment.tangent_start_x, segment.tangent_start_y),
       std::hypot(middle_x, middle_y),
       std::hypot(segment.tangent_finish_x, segment.tangent_finish_y)});
}

ValidationResult inspectSegment(const SegmentGeometry& segment,
                                double curvature_limit,
                                double* estimated_length_m) {
  if (estimated_length_m == nullptr) {
    return ValidationResult::failure("segment length output is missing");
  }
  *estimated_length_m = 0.0;
  HermiteSample previous = evaluate(segment, 0.0);
  double previous_curvature = 0.0;
  ValidationResult result = curvature(previous, &previous_curvature);
  if (!result.ok) {
    return result;
  }
  if (std::abs(previous_curvature) >
      curvature_limit + kCurvatureTolerance) {
    return ValidationResult::failure("Hermite curvature exceeds vehicle profile");
  }

  for (std::size_t index = 1U; index <= kFeasibilityProbeIntervals; ++index) {
    const double parameter = static_cast<double>(index) /
                             static_cast<double>(kFeasibilityProbeIntervals);
    const HermiteSample current = evaluate(segment, parameter);
    double current_curvature = 0.0;
    result = curvature(current, &current_curvature);
    if (!result.ok) {
      return result;
    }
    if (std::abs(current_curvature) >
        curvature_limit + kCurvatureTolerance) {
      return ValidationResult::failure(
          "Hermite curvature exceeds vehicle profile");
    }
    const double increment =
        distance2d(previous.x_m, previous.y_m, current.x_m, current.y_m);
    if (!isFinite(increment)) {
      return ValidationResult::failure("Hermite arc length is non-finite");
    }
    *estimated_length_m += increment;
    previous = current;
  }
  if (!isFinite(*estimated_length_m) ||
      *estimated_length_m <= kArcIncrementFloorM) {
    return ValidationResult::failure("Hermite segment has zero arc length");
  }
  return ValidationResult::success();
}

Trajectory makeTrajectoryMetadata(const RoutePlan& route,
                                  const VehicleProfile& profile,
                                  const TrajectoryGeneratorConfig& config,
                                  std::int64_t generation_stamp_ns,
                                  std::int64_t valid_for_ns) {
  Trajectory trajectory;
  trajectory.stamp_ns = generation_stamp_ns;
  trajectory.frame_id = route.frame_id;
  trajectory.trajectory_id =
      route.route_id + ":" + std::to_string(route.plan_version) + ":" +
      "fnv1a64-" + trajectoryFingerprint(route, profile, config);
  trajectory.route_id = route.route_id;
  trajectory.plan_version = route.plan_version;
  trajectory.vehicle_profile_id = profile.profile_id;
  trajectory.valid_for_ns = valid_for_ns;
  trajectory.completion_behavior = CompletionBehavior::kStopAndHold;
  return trajectory;
}

}  // namespace

HermiteTrajectoryGenerator::HermiteTrajectoryGenerator(
    TrajectoryGeneratorConfig config)
    : config_(config) {}

TrajectoryGenerationResult HermiteTrajectoryGenerator::generate(
    const RoutePlan& route, const VehicleProfile& profile,
    std::int64_t generation_stamp_ns) const {
  Trajectory trajectory;
  if (!isFinite(config_.sampling_resolution_m) ||
      config_.sampling_resolution_m <= 0.0) {
    return failure(std::move(trajectory),
                   "sampling resolution must be finite and positive");
  }
  if (config_.valid_for_ns <= 0) {
    return failure(std::move(trajectory),
                   "trajectory valid duration must be positive");
  }
  if (generation_stamp_ns <= 0) {
    return failure(std::move(trajectory),
                   "trajectory generation time must be positive");
  }

  if (route.frame_id.size() > kMaximumFrameIdBytes) {
    return failure(std::move(trajectory),
                   "frame_id exceeds maximum UTF-8 byte count");
  }
  if (route.route_id.size() > kMaximumRouteIdBytes) {
    return failure(std::move(trajectory),
                   "route_id exceeds maximum UTF-8 byte count");
  }
  if (route.waypoints.size() > kMaximumWaypointCount) {
    return failure(std::move(trajectory),
                   "waypoint list exceeds maximum point count");
  }
  if (profile.profile_id.size() > kMaximumProfileIdBytes) {
    return failure(std::move(trajectory),
                   "profile_id exceeds maximum UTF-8 byte count");
  }

  const ValidationResult route_result =
      validateRoutePlan(route, profile, route.frame_id);
  if (!route_result.ok) {
    return failure(std::move(trajectory), route_result.reason);
  }

  trajectory = makeTrajectoryMetadata(route, profile, config_,
                                      generation_stamp_ns,
                                      config_.valid_for_ns);
  if (trajectory.trajectory_id.size() > kMaximumTrajectoryIdBytes) {
    return failure(std::move(trajectory),
                   "generated trajectory_id exceeds maximum byte count");
  }

  const double curvature_limit = maxAbsCurvature(profile);
  std::size_t total_points = 1U;
  std::vector<SegmentGeometry> segments;
  std::vector<std::size_t> interval_counts;
  segments.reserve(route.waypoints.size() - 1U);
  interval_counts.reserve(route.waypoints.size() - 1U);

  for (std::size_t index = 1U; index < route.waypoints.size(); ++index) {
    const SegmentGeometry segment =
        makeSegment(route.waypoints[index - 1U], route.waypoints[index]);
    double estimated_length_m = 0.0;
    const ValidationResult inspection =
        inspectSegment(segment, curvature_limit, &estimated_length_m);
    if (!inspection.ok) {
      return failure(std::move(trajectory), inspection.reason);
    }
    const double raw_intervals =
        std::ceil(std::max(estimated_length_m,
                           derivativeMagnitudeBound(segment)) /
                  config_.sampling_resolution_m);
    if (!isFinite(raw_intervals) || raw_intervals < 1.0 ||
        raw_intervals >
            static_cast<double>(kMaximumTrajectoryPointCount)) {
      return failure(std::move(trajectory),
                     "sampling resolution produces excessive point count");
    }
    const std::size_t interval_count =
        std::max<std::size_t>(2U, static_cast<std::size_t>(raw_intervals));
    if (interval_count > kMaximumTrajectoryPointCount - total_points) {
      return failure(std::move(trajectory),
                     "trajectory exceeds maximum point count");
    }
    total_points += interval_count;
    segments.push_back(segment);
    interval_counts.push_back(interval_count);
  }

  trajectory.points.reserve(total_points);
  double accumulated_arc_length_m = 0.0;
  for (std::size_t segment_index = 0U; segment_index < segments.size();
       ++segment_index) {
    const SegmentGeometry& segment = segments[segment_index];
    const std::size_t interval_count = interval_counts[segment_index];
    const std::size_t first_sample = segment_index == 0U ? 0U : 1U;
    for (std::size_t sample_index = first_sample;
         sample_index <= interval_count; ++sample_index) {
      const double parameter = static_cast<double>(sample_index) /
                               static_cast<double>(interval_count);
      const HermiteSample sample = evaluate(segment, parameter);
      double sample_curvature = 0.0;
      const ValidationResult curvature_result =
          curvature(sample, &sample_curvature);
      if (!curvature_result.ok ||
          std::abs(sample_curvature) >
              curvature_limit + kCurvatureTolerance) {
        return failure(std::move(trajectory),
                       curvature_result.ok
                           ? "sampled curvature exceeds vehicle profile"
                           : curvature_result.reason);
      }

      if (!trajectory.points.empty()) {
        const TrajectoryPoint& previous = trajectory.points.back();
        const double increment =
            distance2d(previous.x_m, previous.y_m, sample.x_m, sample.y_m);
        if (!isFinite(increment) || increment <= kArcIncrementFloorM) {
          return failure(std::move(trajectory),
                         "sampled trajectory has duplicate points");
        }
        accumulated_arc_length_m += increment;
      }

      TrajectoryPoint point;
      point.x_m = sample.x_m;
      point.y_m = sample.y_m;
      point.yaw_rad =
          normalizeAngle(std::atan2(sample.derivative_y, sample.derivative_x));
      point.curvature_inv_m = sample_curvature;
      point.target_speed_mps =
          segment.start.speed_mps +
          parameter * (segment.finish.speed_mps - segment.start.speed_mps);
      point.arc_length_m = accumulated_arc_length_m;
      point.direction = Direction::kForward;
      trajectory.points.push_back(point);
    }
  }

  trajectory.valid = true;
  const ValidationResult generated =
      validateTrajectory(trajectory, profile, route.frame_id);
  if (!generated.ok) {
    return failure(std::move(trajectory), generated.reason);
  }

  TrajectoryGenerationResult result;
  result.trajectory = std::move(trajectory);
  result.validation = ValidationResult::success();
  return result;
}

}  // namespace planning
}  // namespace auto_rover
