#pragma once

#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"

namespace auto_rover {
namespace planning {

struct RouteLoadResult {
  RoutePlan route;
  ValidationResult validation;
};

class WaypointYamlLoader {
 public:
  WaypointYamlLoader(VehicleProfile profile, std::string expected_frame);

  RouteLoadResult loadFile(const std::string& path,
                           std::int64_t load_stamp_ns) const;
  RouteLoadResult loadString(const std::string& yaml_text,
                             std::int64_t load_stamp_ns) const;

 private:
  VehicleProfile profile_;
  std::string expected_frame_;
};

}  // namespace planning
}  // namespace auto_rover
