#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include "auto_rover_planning/route_repository.hpp"
#include "auto_rover_planning/resource_limits.hpp"
#include "auto_rover_planning/trajectory_generator.hpp"
#include "auto_rover_planning/waypoint_loader.hpp"

namespace {

void expect(bool condition, const std::string& message) {
  EXPECT_TRUE(condition) << message;
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

void expectReasonContains(const auto_rover::ValidationResult& result,
                          const std::string& fragment,
                          const std::string& message) {
  expect(!result.ok && result.reason.find(fragment) != std::string::npos,
         message + " reason=" + result.reason);
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

std::string validYaml(std::uint64_t plan_version = 1U,
                      const std::string& route_id =
                          "commissioning_route_01") {
  return "schema_version: 1\n"
         "frame_id: camera_init\n"
         "route_id: " +
         route_id + "\n" +
         "plan_version: " + std::to_string(plan_version) + "\n" +
         "loop: false\n"
         "waypoints:\n"
         "  - {x_m: 0.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n"
         "  - {x_m: 2.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n";
}

std::string yamlWithFrameAndRoute(const std::string& frame_id,
                                  const std::string& route_id) {
  return "schema_version: 1\n"
         "frame_id: " +
         frame_id + "\n" + "route_id: " + route_id + "\n" +
         "plan_version: 1\n"
         "loop: false\n"
         "waypoints:\n"
         "  - {x_m: 0.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n"
         "  - {x_m: 2.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}\n";
}

std::string yamlWithWaypointCount(std::size_t waypoint_count) {
  std::ostringstream output;
  output << "schema_version: 1\n"
            "frame_id: camera_init\n"
            "route_id: resource_boundary\n"
            "plan_version: 1\n"
            "loop: false\n"
            "waypoints:\n";
  for (std::size_t index = 0U; index < waypoint_count; ++index) {
    output << "  - {x_m: " << index
           << ", y_m: 0, yaw_rad: 0, speed_mps: 0.50}\n";
  }
  return output.str();
}

auto_rover::RoutePlan straightRoute() {
  auto_rover::RoutePlan route;
  route.schema_version = 1U;
  route.stamp_ns = 1000000000LL;
  route.frame_id = "camera_init";
  route.route_id = "straight";
  route.plan_version = 7U;
  route.loop = false;
  route.waypoints.push_back({0.0, 0.0, 0.0, 0.50});
  route.waypoints.push_back({2.0, 0.0, 0.0, 0.50});
  return route;
}

void testStrictWaypointLoaderAcceptsValidInput() {
  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");
  const auto result = loader.loadString(validYaml(), 123456789LL);
  expect(result.validation.ok, "strict loader accepts the documented schema");
  expect(result.route.schema_version == 1U, "schema version is preserved");
  expect(result.route.stamp_ns == 123456789LL,
         "injected load time becomes route stamp");
  expect(result.route.frame_id == "camera_init", "frame is preserved");
  expect(result.route.route_id == "commissioning_route_01",
         "route identity is preserved");
  expect(result.route.plan_version == 1U, "plan version is preserved");
  expect(!result.route.loop, "commissioning route is non-looping");
  expect(result.route.waypoints.size() == 2U,
         "all waypoints are loaded");
  expectNear(result.route.waypoints.back().speed_mps, 0.50, 1e-12,
             "commissioning target is not rewritten as a deadband");

  std::string wrapped_yaw = validYaml();
  const std::size_t yaw_position = wrapped_yaw.find("yaw_rad: 0.0");
  wrapped_yaw.replace(yaw_position, std::string("yaw_rad: 0.0").size(),
                      "yaw_rad: 6.283185307179586");
  const auto normalized = loader.loadString(wrapped_yaw, 123456790LL);
  expect(normalized.validation.ok,
         "finite waypoint yaw is normalized at the YAML boundary");
  expectNear(normalized.route.waypoints.front().yaw_rad, 0.0, 1e-12,
             "wrapped YAML yaw normalizes into [-pi, pi)");
}

void testStrictWaypointLoaderRejectsStructureAndNumbers() {
  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");

  expectReasonContains(
      loader
          .loadString("schema_version: 1\n"
                      "schema_version: 1\n"
                      "frame_id: camera_init\n"
                      "route_id: route\n"
                      "plan_version: 1\n"
                      "loop: false\n"
                      "waypoints:\n"
                      "  - {x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n"
                      "  - {x_m: 1, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n",
                      1LL)
          .validation,
      "duplicate", "duplicate top-level keys are rejected");

  expectReasonContains(
      loader
          .loadString(validYaml() + "unexpected: value\n", 1LL)
          .validation,
      "unknown", "unknown top-level keys are rejected");

  expectReasonContains(
      loader
          .loadString("schema_version: 1\n"
                      "frame_id: camera_init\n"
                      "route_id: route\n"
                      "plan_version: 1\n"
                      "loop: false\n"
                      "waypoints:\n"
                      "  - {x_m: 0, x_m: 1, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n"
                      "  - {x_m: 2, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n",
                      1LL)
          .validation,
      "duplicate", "duplicate waypoint keys are rejected");

  expectReasonContains(
      loader
          .loadString("schema_version: 1\n"
                      "frame_id: camera_init\n"
                      "route_id: route\n"
                      "plan_version: 1\n"
                      "loop: false\n"
                      "waypoints:\n"
                      "  - {x_m: 0, y_m: 0, yaw_rad: 0, speed_mps: .nan}\n"
                      "  - {x_m: 1, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n",
                      1LL)
          .validation,
      "number", "non-finite YAML numbers are rejected");

  expectReasonContains(
      loader
          .loadString("schema_version: 1\n"
                      "frame_id: camera_init\n"
                      "route_id: route\n"
                      "plan_version: 1\n"
                      "loop: false\n"
                      "waypoints:\n"
                      "  - {x_m: 0m, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n"
                      "  - {x_m: 1, y_m: 0, yaw_rad: 0, speed_mps: 0.1}\n",
                      1LL)
          .validation,
      "number", "numbers with trailing units are rejected");

  expectReasonContains(
      loader
          .loadString("schema_version: 1\n"
                      "frame_id: camera_init\n"
                      "route_id: route\n"
                      "plan_version: 1\n"
                      "loop: false\n"
                      "waypoints: []\n",
                      1LL)
          .validation,
      "waypoint", "empty waypoint lists are rejected");
}

void testStrictWaypointLoaderRejectsContractViolations() {
  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");

  std::string wrong_frame = validYaml();
  const std::size_t frame_position = wrong_frame.find("camera_init");
  wrong_frame.replace(frame_position, std::string("camera_init").size(),
                      "map");
  expectReasonContains(loader.loadString(wrong_frame, 1LL).validation, "frame",
                       "wrong route frame is rejected");

  std::string excessive_speed = validYaml();
  const std::size_t speed_position = excessive_speed.find("speed_mps: 0.50");
  excessive_speed.replace(speed_position, std::string("speed_mps: 0.50").size(),
                          "speed_mps: 0.51");
  expectReasonContains(loader.loadString(excessive_speed, 1LL).validation,
                       "speed", "speed above the vehicle profile is rejected");

  std::string negative_speed = validYaml();
  const std::size_t negative_position = negative_speed.find("speed_mps: 0.50");
  negative_speed.replace(negative_position,
                         std::string("speed_mps: 0.50").size(),
                         "speed_mps: -0.01");
  expectReasonContains(loader.loadString(negative_speed, 1LL).validation,
                       "speed", "negative speed is rejected");

  std::string duplicate_point = validYaml();
  const std::size_t second_x = duplicate_point.rfind("x_m: 2.0");
  duplicate_point.replace(second_x, std::string("x_m: 2.0").size(),
                          "x_m: 0.0");
  expectReasonContains(loader.loadString(duplicate_point, 1LL).validation,
                       "duplicate", "duplicate waypoint positions are rejected");

  std::string bad_schema = validYaml();
  bad_schema.replace(bad_schema.find("schema_version: 1"),
                     std::string("schema_version: 1").size(),
                     "schema_version: 2");
  expectReasonContains(loader.loadString(bad_schema, 1LL).validation, "schema",
                       "unsupported schema version is rejected");

  expectReasonContains(loader.loadString(validYaml(0U), 1LL).validation,
                       "version", "zero plan version is rejected");
  expectReasonContains(loader.loadString(validYaml(), 0LL).validation, "time",
                       "non-positive injected load time is rejected");
}

void testFileLoader() {
  const std::string path = "/tmp/auto_rover_waypoints_" +
                           std::to_string(static_cast<long long>(::getpid())) +
                           ".yaml";
  {
    std::ofstream output(path);
    output << validYaml();
  }
  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");
  expect(loader.loadFile(path, 10LL).validation.ok,
         "file entry point loads the same strict schema");
  expect(std::remove(path.c_str()) == 0, "temporary waypoint fixture is removed");
  expectReasonContains(loader.loadFile(path, 10LL).validation, "file",
                       "missing waypoint file fails closed");
}

void testWaypointLoaderEnforcesIdentityAndScalarBudgets() {
  using auto_rover::planning::kMaximumFrameIdBytes;
  using auto_rover::planning::kMaximumNumericScalarBytes;
  using auto_rover::planning::kMaximumRouteIdBytes;

  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");
  const std::string exact_route_id(kMaximumRouteIdBytes, 'r');
  expect(loader.loadString(validYaml(1U, exact_route_id), 1LL).validation.ok,
         "route identity at its UTF-8 byte budget is accepted");
  expectReasonContains(
      loader
          .loadString(validYaml(1U, exact_route_id + "r"), 1LL)
          .validation,
      "route_id exceeds", "route identity over its byte budget is rejected");

  const std::string two_byte_utf8 = "\xC2\xA2";
  std::string exact_multibyte_route_id;
  for (std::size_t index = 0U; index < kMaximumRouteIdBytes / 2U; ++index) {
    exact_multibyte_route_id += two_byte_utf8;
  }
  expect(exact_multibyte_route_id.size() == kMaximumRouteIdBytes,
         "multibyte identity fixture reaches the byte budget exactly");
  expect(loader
             .loadString(validYaml(1U, exact_multibyte_route_id), 2LL)
             .validation.ok,
         "identity budget is measured in UTF-8 bytes, not code points");
  expectReasonContains(
      loader
          .loadString(validYaml(1U,
                                exact_multibyte_route_id + two_byte_utf8),
                      3LL)
          .validation,
      "route_id exceeds",
      "one additional multibyte code point exceeds the byte budget");

  const std::string exact_frame_id(kMaximumFrameIdBytes, 'f');
  const auto_rover::planning::WaypointYamlLoader exact_frame_loader(
      validProfile(), exact_frame_id);
  expect(exact_frame_loader
             .loadString(yamlWithFrameAndRoute(exact_frame_id, "route"), 4LL)
             .validation.ok,
         "frame identity at its UTF-8 byte budget is accepted");
  expectReasonContains(
      exact_frame_loader
          .loadString(
              yamlWithFrameAndRoute(exact_frame_id + "f", "route"), 5LL)
          .validation,
      "frame_id exceeds", "frame identity over its byte budget is rejected");

  const std::string exact_numeric_scalar =
      "0." + std::string(kMaximumNumericScalarBytes - 2U, '0');
  std::string exact_numeric_yaml = validYaml();
  exact_numeric_yaml.replace(exact_numeric_yaml.find("x_m: 0.0"),
                             std::string("x_m: 0.0").size(),
                             "x_m: " + exact_numeric_scalar);
  expect(loader.loadString(exact_numeric_yaml, 6LL).validation.ok,
         "finite numeric scalar at its byte budget is accepted");
  std::string excessive_numeric_yaml = validYaml();
  excessive_numeric_yaml.replace(
      excessive_numeric_yaml.find("x_m: 0.0"),
      std::string("x_m: 0.0").size(),
      "x_m: " + exact_numeric_scalar + "0");
  expectReasonContains(loader.loadString(excessive_numeric_yaml, 7LL).validation,
                       "numeric scalar exceeds",
                       "numeric scalar over its byte budget is rejected");

  const std::string exact_key(auto_rover::planning::kMaximumYamlKeyBytes, 'k');
  expectReasonContains(
      loader.loadString(validYaml() + exact_key + ": value\n", 8LL)
          .validation,
      "unknown", "a key at the byte budget reaches strict-schema validation");
  const std::string excessive_key = exact_key + "k";
  expectReasonContains(
      loader.loadString(validYaml() + excessive_key + ": value\n", 9LL)
          .validation,
      "map key exceeds", "oversized YAML keys fail before error expansion");

  std::string excessive_boolean_yaml = validYaml();
  excessive_boolean_yaml.replace(excessive_boolean_yaml.find("loop: false"),
                                  std::string("loop: false").size(),
                                  "loop: falsex");
  expectReasonContains(
      loader.loadString(excessive_boolean_yaml, 10LL).validation,
      "boolean scalar exceeds",
      "boolean scalar over its syntax-sized budget is rejected");
}

void testWaypointLoaderEnforcesDocumentAndWaypointBudgets() {
  using auto_rover::planning::kMaximumWaypointCount;
  using auto_rover::planning::kMaximumWaypointYamlBytes;

  const auto_rover::planning::WaypointYamlLoader loader(validProfile(),
                                                         "camera_init");
  std::string exact_document = validYaml();
  exact_document += '#';
  exact_document.append(kMaximumWaypointYamlBytes - exact_document.size(),
                        'x');
  expect(exact_document.size() == kMaximumWaypointYamlBytes,
         "document fixture reaches the byte budget exactly");
  expect(loader.loadString(exact_document, 1LL).validation.ok,
         "string document at the byte budget is parsed");

  const std::string excessive_document = exact_document + "x";
  expectReasonContains(loader.loadString(excessive_document, 2LL).validation,
                       "document exceeds",
                       "oversized string input is rejected before parsing");

  const std::string path =
      "/tmp/auto_rover_waypoint_budget_" +
      std::to_string(static_cast<long long>(::getpid())) + ".yaml";
  {
    std::ofstream output(path, std::ios::binary);
    output << exact_document;
  }
  expect(loader.loadFile(path, 3LL).validation.ok,
         "file document at the byte budget is parsed");
  {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << excessive_document;
  }
  expectReasonContains(loader.loadFile(path, 4LL).validation,
                       "document exceeds",
                       "oversized file is rejected before YAML parsing");
  expect(std::remove(path.c_str()) == 0,
         "resource-budget file fixture is removed");

  const std::string maximum_waypoints =
      yamlWithWaypointCount(kMaximumWaypointCount);
  expect(maximum_waypoints.size() < kMaximumWaypointYamlBytes,
         "maximum waypoint fixture remains within the document budget");
  expect(loader.loadString(maximum_waypoints, 5LL).validation.ok,
         "waypoint sequence at its count budget is accepted");
  expectReasonContains(
      loader
          .loadString(yamlWithWaypointCount(kMaximumWaypointCount + 1U), 6LL)
          .validation,
      "waypoint list exceeds",
      "waypoint sequence over its count budget is rejected before iteration");
}

void testRouteRepositoryInvalidatesOnEveryFailedReload() {
  auto_rover::planning::RouteRepository repository(validProfile(),
                                                    "camera_init");
  expect(repository.reloadFromString(validYaml(1U), 1LL).ok,
         "first route becomes active");
  expect(repository.hasActiveRoute(), "successful route is marked active");
  expect(repository.activeRoute().plan_version == 1U,
         "active route exposes accepted version");

  expect(repository.reloadFromString(validYaml(2U), 2LL).ok,
         "higher version of the same route replaces active route");
  expect(repository.activeRoute().plan_version == 2U,
         "replacement is visible atomically");

  expectReasonContains(repository.reloadFromString(validYaml(2U), 3LL),
                       "increase", "version replay is rejected");
  expect(!repository.hasActiveRoute(),
         "failed replacement invalidates active route execution");
  expect(repository.activeRoute().waypoints.empty(),
         "failed replacement does not expose old executable points");

  expect(repository.reloadFromString(validYaml(3U), 4LL).ok,
         "history permits only a later version after invalidation");
  expectReasonContains(
      repository.reloadFromString(validYaml(4U, "other_route"), 5LL),
      "route_id", "replacement cannot silently change route identity");
  expect(!repository.hasActiveRoute(),
         "route identity failure also invalidates execution");

  expect(repository.reloadFromString(validYaml(5U), 6LL).ok,
         "same route can recover with a higher version");
  expectReasonContains(repository.reloadFromString("not: [valid", 7LL),
                       "YAML", "malformed reload is reported");
  expect(!repository.hasActiveRoute(),
         "malformed reload cannot leave the old route active");
}

void testStraightHermiteTrajectoryIsDeterministic() {
  auto_rover::planning::TrajectoryGeneratorConfig config;
  config.sampling_resolution_m = 0.25;
  config.valid_for_ns = 200000000LL;
  const auto_rover::planning::HermiteTrajectoryGenerator generator(config);
  const auto first = generator.generate(straightRoute(), validProfile(),
                                        2000000000LL);
  const auto second = generator.generate(straightRoute(), validProfile(),
                                         2000000000LL);
  const auto restarted = generator.generate(straightRoute(), validProfile(),
                                            3000000000LL);

  expect(first.validation.ok, "straight Hermite trajectory is feasible");
  expect(second.validation.ok, "deterministic regeneration remains feasible");
  expect(first.trajectory.valid, "successful trajectory is explicitly valid");
  expect(first.trajectory.completion_behavior ==
             auto_rover::CompletionBehavior::kStopAndHold,
         "trajectory completion is stop and hold");
  expect(first.trajectory.valid_for_ns == config.valid_for_ns,
         "configured validity horizon is preserved");
  expect(first.trajectory.trajectory_id == second.trajectory.trajectory_id,
         "same inputs produce the same trajectory identity");
  expect(first.trajectory.trajectory_id == restarted.trajectory.trajectory_id &&
             first.trajectory.stamp_ns != restarted.trajectory.stamp_ns,
         "process restart time does not change an immutable generation identity");
  auto changed_route = straightRoute();
  changed_route.waypoints.back().speed_mps = 0.40;
  const auto changed = generator.generate(changed_route, validProfile(),
                                          3000000000LL);
  expect(changed.validation.ok &&
             changed.trajectory.trajectory_id !=
                 first.trajectory.trajectory_id,
         "same route version with different executable content has a new identity");
  expect(first.trajectory.points.size() == second.trajectory.points.size(),
         "same inputs produce the same sample count");
  expect(first.trajectory.points.size() == 9U,
         "two metres at 0.25 metre resolution has deterministic samples");

  for (std::size_t index = 0U; index < first.trajectory.points.size(); ++index) {
    const auto& point = first.trajectory.points[index];
    const auto& repeated = second.trajectory.points[index];
    expectNear(point.x_m, repeated.x_m, 0.0,
               "deterministic samples have identical x");
    expectNear(point.y_m, 0.0, 1e-12, "straight trajectory remains on axis");
    expectNear(point.yaw_rad, 0.0, 1e-12, "straight yaw remains zero");
    expectNear(point.curvature_inv_m, 0.0, 1e-12,
               "straight curvature remains zero");
    expect(point.direction == auto_rover::Direction::kForward,
           "every sample is forward");
    if (index > 0U) {
      expect(point.arc_length_m >
                 first.trajectory.points[index - 1U].arc_length_m,
             "trajectory arc length strictly increases");
    }
  }
}

void testHermiteUsesWaypointYawAndChecksCurvature() {
  auto route = straightRoute();
  route.route_id = "quarter_turn";
  route.waypoints[1].x_m = 2.0;
  route.waypoints[1].y_m = 2.0;
  route.waypoints[1].yaw_rad = 1.57079632679489661923;

  auto_rover::planning::TrajectoryGeneratorConfig config;
  config.sampling_resolution_m = 0.10;
  config.valid_for_ns = 100000000LL;
  const auto_rover::planning::HermiteTrajectoryGenerator generator(config);
  const auto result = generator.generate(route, validProfile(), 3000000000LL);
  expect(result.validation.ok, "gentle Hermite turn is feasible");
  expectNear(result.trajectory.points.front().yaw_rad, 0.0, 1e-12,
             "first Hermite tangent matches first waypoint yaw");
  expectNear(result.trajectory.points.back().yaw_rad,
             1.57079632679489661923, 1e-12,
             "last Hermite tangent matches last waypoint yaw");
  for (const auto& point : result.trajectory.points) {
    expect(std::abs(point.curvature_inv_m) <= 1.0 / 0.95 + 1e-9,
           "every turn sample obeys the operational radius");
  }
  for (std::size_t index = 1U; index < result.trajectory.points.size();
       ++index) {
    const auto& previous = result.trajectory.points[index - 1U];
    const auto& current = result.trajectory.points[index];
    expect(std::hypot(current.x_m - previous.x_m,
                      current.y_m - previous.y_m) <=
               config.sampling_resolution_m + 1e-12,
           "sample spacing does not exceed configured resolution");
  }

  route.route_id = "infeasible_turn";
  route.waypoints[1].x_m = 0.20;
  route.waypoints[1].y_m = 0.20;
  const auto infeasible =
      generator.generate(route, validProfile(), 3000000001LL);
  expectReasonContains(infeasible.validation, "curvature",
                       "tight Hermite geometry is rejected");
  expect(!infeasible.trajectory.valid,
         "infeasible trajectory is explicitly invalid");
  expect(infeasible.trajectory.points.empty(),
         "infeasible trajectory contains no executable points");
}

void testTrajectoryGeneratorRejectsInvalidInputs() {
  auto_rover::planning::TrajectoryGeneratorConfig config;
  config.sampling_resolution_m = 0.0;
  config.valid_for_ns = 1LL;
  auto_rover::planning::HermiteTrajectoryGenerator generator(config);
  expectReasonContains(
      generator.generate(straightRoute(), validProfile(), 1LL).validation,
      "sampling", "zero sampling resolution is rejected");

  config.sampling_resolution_m = 0.1;
  config.valid_for_ns = 0LL;
  generator = auto_rover::planning::HermiteTrajectoryGenerator(config);
  expectReasonContains(
      generator.generate(straightRoute(), validProfile(), 1LL).validation,
      "valid", "non-positive trajectory lifetime is rejected");

  config.valid_for_ns = 1LL;
  generator = auto_rover::planning::HermiteTrajectoryGenerator(config);
  auto route = straightRoute();
  route.waypoints[1].speed_mps =
      std::numeric_limits<double>::infinity();
  const auto invalid_route = generator.generate(route, validProfile(), 1LL);
  expect(!invalid_route.validation.ok, "invalid route cannot generate a trajectory");
  expect(invalid_route.trajectory.points.empty(),
         "invalid route produces no executable points");

  expectReasonContains(
      generator.generate(straightRoute(), validProfile(), 0LL).validation,
      "time", "non-positive trajectory generation time is rejected");
}

void testTrajectoryGeneratorEnforcesPlanningResourceBudgets() {
  using auto_rover::planning::kMaximumFrameIdBytes;
  using auto_rover::planning::kMaximumProfileIdBytes;
  using auto_rover::planning::kMaximumRouteIdBytes;
  using auto_rover::planning::kMaximumTrajectoryPointCount;
  using auto_rover::planning::kMaximumWaypointCount;

  auto_rover::planning::TrajectoryGeneratorConfig config;
  config.sampling_resolution_m = 1.0;
  config.valid_for_ns = 1LL;
  const auto_rover::planning::HermiteTrajectoryGenerator generator(config);

  auto exact_points_route = straightRoute();
  exact_points_route.waypoints.back().x_m =
      static_cast<double>(kMaximumTrajectoryPointCount - 1U);
  const auto exact_points =
      generator.generate(exact_points_route, validProfile(), 1LL);
  expect(exact_points.validation.ok,
         "trajectory at the execution point budget is accepted");
  expect(exact_points.trajectory.points.size() ==
             kMaximumTrajectoryPointCount,
         "the trajectory point budget includes both segment endpoints");

  auto maximum_waypoint_route = straightRoute();
  maximum_waypoint_route.route_id = "maximum_waypoint_route";
  maximum_waypoint_route.waypoints.clear();
  maximum_waypoint_route.waypoints.reserve(kMaximumWaypointCount);
  for (std::size_t index = 0U; index < kMaximumWaypointCount; ++index) {
    maximum_waypoint_route.waypoints.push_back(
        {static_cast<double>(index), 0.0, 0.0, 0.50});
  }
  const auto maximum_waypoints =
      generator.generate(maximum_waypoint_route, validProfile(), 2LL);
  expect(maximum_waypoints.validation.ok,
         "maximum waypoint input fits when every segment uses two intervals");
  expect(maximum_waypoints.trajectory.points.size() ==
             2U * kMaximumWaypointCount - 1U,
         "2,048 waypoints produce the expected minimum 4,095 points");

  auto excessive_points_route = exact_points_route;
  excessive_points_route.waypoints.back().x_m =
      static_cast<double>(kMaximumTrajectoryPointCount);
  const auto excessive_points =
      generator.generate(excessive_points_route, validProfile(), 3LL);
  expectReasonContains(excessive_points.validation, "maximum point count",
                       "one output point over the budget is rejected");
  expect(!excessive_points.trajectory.valid &&
             excessive_points.trajectory.points.empty(),
         "over-budget trajectory output fails closed and remains empty");

  auto exact_identity_route = straightRoute();
  exact_identity_route.route_id.assign(kMaximumRouteIdBytes, 'r');
  exact_identity_route.plan_version =
      std::numeric_limits<std::uint64_t>::max();
  const auto exact_identity =
      generator.generate(exact_identity_route, validProfile(), 4LL);
  expect(exact_identity.validation.ok,
         "worst-case generated identity within its budget is accepted");
  expect(exact_identity.trajectory.trajectory_id.size() ==
             auto_rover::planning::kMaximumTrajectoryIdBytes,
         "route budget accounts for separators, version, and fingerprint");

  auto excessive_route_id = straightRoute();
  excessive_route_id.route_id.assign(kMaximumRouteIdBytes + 1U, 'r');
  expectReasonContains(
      generator.generate(excessive_route_id, validProfile(), 5LL).validation,
      "route_id exceeds",
      "programmatic routes cannot bypass the route identity budget");

  auto excessive_frame_id = straightRoute();
  excessive_frame_id.frame_id.assign(kMaximumFrameIdBytes + 1U, 'f');
  expectReasonContains(
      generator.generate(excessive_frame_id, validProfile(), 6LL).validation,
      "frame_id exceeds",
      "programmatic routes cannot bypass the frame identity budget");

  auto excessive_waypoints = straightRoute();
  excessive_waypoints.waypoints.resize(kMaximumWaypointCount + 1U);
  expectReasonContains(
      generator.generate(excessive_waypoints, validProfile(), 7LL).validation,
      "waypoint list exceeds",
      "programmatic routes cannot bypass the waypoint count budget");

  auto excessive_profile = validProfile();
  excessive_profile.profile_id.assign(kMaximumProfileIdBytes + 1U, 'p');
  expectReasonContains(
      generator.generate(straightRoute(), excessive_profile, 8LL).validation,
      "profile_id exceeds",
      "generated trajectories cannot carry an oversized profile identity");
}

}  // namespace

TEST(WaypointYamlLoaderTest, AcceptsValidInput) {
  testStrictWaypointLoaderAcceptsValidInput();
}

TEST(WaypointYamlLoaderTest, RejectsStructureAndNumbers) {
  testStrictWaypointLoaderRejectsStructureAndNumbers();
}

TEST(WaypointYamlLoaderTest, RejectsContractViolations) {
  testStrictWaypointLoaderRejectsContractViolations();
}

TEST(WaypointYamlLoaderTest, LoadsFiles) { testFileLoader(); }

TEST(WaypointYamlLoaderTest, EnforcesIdentityAndScalarBudgets) {
  testWaypointLoaderEnforcesIdentityAndScalarBudgets();
}

TEST(WaypointYamlLoaderTest, EnforcesDocumentAndWaypointBudgets) {
  testWaypointLoaderEnforcesDocumentAndWaypointBudgets();
}

TEST(RouteRepositoryTest, InvalidatesOnEveryFailedReload) {
  testRouteRepositoryInvalidatesOnEveryFailedReload();
}

TEST(HermiteTrajectoryGeneratorTest, StraightRouteIsDeterministic) {
  testStraightHermiteTrajectoryIsDeterministic();
}

TEST(HermiteTrajectoryGeneratorTest, UsesYawAndChecksCurvature) {
  testHermiteUsesWaypointYawAndChecksCurvature();
}

TEST(HermiteTrajectoryGeneratorTest, RejectsInvalidInputs) {
  testTrajectoryGeneratorRejectsInvalidInputs();
}

TEST(HermiteTrajectoryGeneratorTest, EnforcesPlanningResourceBudgets) {
  testTrajectoryGeneratorEnforcesPlanningResourceBudgets();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
