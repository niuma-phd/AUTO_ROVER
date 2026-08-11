#include "auto_rover_planning/waypoint_loader.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"
#include "auto_rover_planning/resource_limits.hpp"

namespace auto_rover {
namespace planning {
namespace {

constexpr double kDuplicatePositionToleranceM = 1e-9;

RouteLoadResult failure(const std::string& reason) {
  RouteLoadResult result;
  result.validation = ValidationResult::failure(reason);
  return result;
}

ValidationResult validateMapKeys(const YAML::Node& node,
                                 const std::set<std::string>& allowed,
                                 const std::set<std::string>& required,
                                 const std::string& context) {
  if (!node.IsMap()) {
    return ValidationResult::failure(context + " must be a map");
  }
  std::set<std::string> seen;
  for (const auto& entry : node) {
    if (!entry.first.IsScalar()) {
      return ValidationResult::failure(context + " has a non-scalar key");
    }
    const std::string key = entry.first.Scalar();
    if (key.size() > kMaximumYamlKeyBytes) {
      return ValidationResult::failure(context +
                                       " map key exceeds maximum byte count");
    }
    if (!seen.insert(key).second) {
      return ValidationResult::failure(context + " has duplicate key: " + key);
    }
    if (allowed.count(key) == 0U) {
      return ValidationResult::failure(context + " has unknown key: " + key);
    }
  }
  for (const std::string& key : required) {
    if (seen.count(key) == 0U) {
      return ValidationResult::failure(context + " is missing key: " + key);
    }
  }
  return ValidationResult::success();
}

ValidationResult parseNonEmptyString(const YAML::Node& node,
                                     const std::string& name,
                                     std::size_t maximum_bytes,
                                     std::string* output) {
  if (output == nullptr || !node.IsScalar()) {
    return ValidationResult::failure(name + " must be a string");
  }
  const std::string value = node.Scalar();
  if (value.size() > maximum_bytes) {
    return ValidationResult::failure(
        name + " exceeds maximum UTF-8 byte count");
  }
  if (value.empty() || value.find_first_not_of(" \t\r\n") == std::string::npos) {
    return ValidationResult::failure(name + " must not be empty");
  }
  *output = value;
  return ValidationResult::success();
}

ValidationResult parseUnsigned(const YAML::Node& node, const std::string& name,
                               std::uint64_t maximum,
                               std::uint64_t* output) {
  if (output == nullptr || !node.IsScalar()) {
    return ValidationResult::failure(name + " must be an unsigned integer");
  }
  const std::string text = node.Scalar();
  if (text.size() > kMaximumNumericScalarBytes) {
    return ValidationResult::failure(name +
                                     " numeric scalar exceeds maximum byte count");
  }
  if (text.empty() ||
      text.find_first_not_of("0123456789") != std::string::npos) {
    return ValidationResult::failure(name + " has invalid version syntax");
  }
  try {
    std::size_t parsed = 0U;
    const unsigned long long value = std::stoull(text, &parsed, 10);
    if (parsed != text.size() || value > maximum) {
      return ValidationResult::failure(name + " is outside its version range");
    }
    *output = static_cast<std::uint64_t>(value);
  } catch (const std::exception&) {
    return ValidationResult::failure(name + " is outside its version range");
  }
  return ValidationResult::success();
}

ValidationResult parseBoolean(const YAML::Node& node, const std::string& name,
                              bool* output) {
  if (output == nullptr || !node.IsScalar()) {
    return ValidationResult::failure(name + " must be true or false");
  }
  const std::string text = node.Scalar();
  if (text.size() > kMaximumBooleanScalarBytes) {
    return ValidationResult::failure(name +
                                     " boolean scalar exceeds maximum byte count");
  }
  if (text == "true") {
    *output = true;
    return ValidationResult::success();
  }
  if (text == "false") {
    *output = false;
    return ValidationResult::success();
  }
  return ValidationResult::failure(name + " must be true or false");
}

ValidationResult parseFiniteDecimal(const YAML::Node& node,
                                    const std::string& name, double* output) {
  if (output == nullptr || !node.IsScalar()) {
    return ValidationResult::failure(name + " must be a finite number");
  }
  const std::string text = node.Scalar();
  if (text.size() > kMaximumNumericScalarBytes) {
    return ValidationResult::failure(name +
                                     " numeric scalar exceeds maximum byte count");
  }
  static const std::regex decimal_pattern(
      "^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$");
  if (!std::regex_match(text, decimal_pattern)) {
    return ValidationResult::failure(name + " has malformed number syntax");
  }
  try {
    std::size_t parsed = 0U;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value)) {
      return ValidationResult::failure(name + " must be a finite number");
    }
    *output = value;
  } catch (const std::exception&) {
    return ValidationResult::failure(name + " must be a finite number");
  }
  return ValidationResult::success();
}

RouteLoadResult parseRoute(const YAML::Node& root,
                           const VehicleProfile& profile,
                           const std::string& expected_frame,
                           std::int64_t load_stamp_ns) {
  if (load_stamp_ns <= 0) {
    return failure("route load time must be positive");
  }

  const std::set<std::string> top_keys{
      "schema_version", "frame_id", "route_id",
      "plan_version",   "loop",     "waypoints"};
  const ValidationResult top_result =
      validateMapKeys(root, top_keys, top_keys, "waypoint document");
  if (!top_result.ok) {
    return failure(top_result.reason);
  }

  RoutePlan route;
  route.stamp_ns = load_stamp_ns;

  std::uint64_t schema_version = 0U;
  ValidationResult result = parseUnsigned(
      root["schema_version"], "schema_version",
      static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()),
      &schema_version);
  if (!result.ok) {
    return failure(result.reason);
  }
  route.schema_version = static_cast<std::uint32_t>(schema_version);
  if (route.schema_version != 1U) {
    return failure("unsupported waypoint schema version");
  }

  result = parseNonEmptyString(root["frame_id"], "frame_id",
                               kMaximumFrameIdBytes, &route.frame_id);
  if (!result.ok) {
    return failure(result.reason);
  }
  result = parseNonEmptyString(root["route_id"], "route_id",
                               kMaximumRouteIdBytes, &route.route_id);
  if (!result.ok) {
    return failure(result.reason);
  }
  result = parseUnsigned(root["plan_version"], "plan_version",
                         std::numeric_limits<std::uint64_t>::max(),
                         &route.plan_version);
  if (!result.ok) {
    return failure(result.reason);
  }
  if (route.plan_version == 0U) {
    return failure("plan version must be positive");
  }
  result = parseBoolean(root["loop"], "loop", &route.loop);
  if (!result.ok) {
    return failure(result.reason);
  }

  const YAML::Node waypoint_nodes = root["waypoints"];
  if (!waypoint_nodes.IsSequence() || waypoint_nodes.size() < 2U) {
    return failure("waypoint list must contain at least two points");
  }
  if (waypoint_nodes.size() > kMaximumWaypointCount) {
    return failure("waypoint list exceeds maximum point count");
  }
  const std::set<std::string> waypoint_keys{"x_m", "y_m", "yaw_rad",
                                             "speed_mps"};
  route.waypoints.reserve(waypoint_nodes.size());
  for (std::size_t index = 0U; index < waypoint_nodes.size(); ++index) {
    const YAML::Node waypoint_node = waypoint_nodes[index];
    const ValidationResult keys = validateMapKeys(
        waypoint_node, waypoint_keys, waypoint_keys,
        "waypoint " + std::to_string(index));
    if (!keys.ok) {
      return failure(keys.reason);
    }
    RouteWaypoint waypoint;
    result = parseFiniteDecimal(waypoint_node["x_m"], "waypoint x_m",
                                &waypoint.x_m);
    if (!result.ok) {
      return failure(result.reason);
    }
    result = parseFiniteDecimal(waypoint_node["y_m"], "waypoint y_m",
                                &waypoint.y_m);
    if (!result.ok) {
      return failure(result.reason);
    }
    result = parseFiniteDecimal(waypoint_node["yaw_rad"], "waypoint yaw_rad",
                                &waypoint.yaw_rad);
    if (!result.ok) {
      return failure(result.reason);
    }
    waypoint.yaw_rad = normalizeAngle(waypoint.yaw_rad);
    result = parseFiniteDecimal(waypoint_node["speed_mps"],
                                "waypoint speed_mps", &waypoint.speed_mps);
    if (!result.ok) {
      return failure(result.reason);
    }
    for (const RouteWaypoint& previous : route.waypoints) {
      if (distance2d(previous.x_m, previous.y_m, waypoint.x_m, waypoint.y_m) <=
          kDuplicatePositionToleranceM) {
        return failure("duplicate waypoint position");
      }
    }
    route.waypoints.push_back(waypoint);
  }

  const ValidationResult contract =
      validateRoutePlan(route, profile, expected_frame);
  if (!contract.ok) {
    return failure(contract.reason);
  }
  RouteLoadResult loaded;
  loaded.route = std::move(route);
  loaded.validation = ValidationResult::success();
  return loaded;
}

}  // namespace

WaypointYamlLoader::WaypointYamlLoader(VehicleProfile profile,
                                       std::string expected_frame)
    : profile_(std::move(profile)), expected_frame_(std::move(expected_frame)) {}

RouteLoadResult WaypointYamlLoader::loadFile(
    const std::string& path, std::int64_t load_stamp_ns) const {
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return failure("waypoint file could not be read");
  }

  std::string yaml_text;
  yaml_text.reserve(8192U);
  std::array<char, 8192U> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count <= 0) {
      continue;
    }
    const std::size_t byte_count = static_cast<std::size_t>(count);
    if (byte_count > kMaximumWaypointYamlBytes - yaml_text.size()) {
      return failure("waypoint YAML document exceeds maximum byte count");
    }
    yaml_text.append(buffer.data(), byte_count);
  }
  if (input.bad() || (!input.eof() && input.fail())) {
    return failure("waypoint file could not be read completely");
  }
  return loadString(yaml_text, load_stamp_ns);
}

RouteLoadResult WaypointYamlLoader::loadString(
    const std::string& yaml_text, std::int64_t load_stamp_ns) const {
  if (yaml_text.size() > kMaximumWaypointYamlBytes) {
    return failure("waypoint YAML document exceeds maximum byte count");
  }
  try {
    return parseRoute(YAML::Load(yaml_text), profile_, expected_frame_,
                      load_stamp_ns);
  } catch (const YAML::Exception& error) {
    return failure("waypoint YAML parse error: " + std::string(error.what()));
  }
}

}  // namespace planning
}  // namespace auto_rover
