#pragma once

#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"
#include "auto_rover_planning/waypoint_loader.hpp"

namespace auto_rover {
namespace planning {

class RouteRepository {
 public:
  RouteRepository(VehicleProfile profile, std::string expected_frame);

  ValidationResult reloadFromFile(const std::string& path,
                                  std::int64_t load_stamp_ns);
  ValidationResult reloadFromString(const std::string& yaml_text,
                                    std::int64_t load_stamp_ns);

  bool hasActiveRoute() const;
  const RoutePlan& activeRoute() const;
  const ValidationResult& lastResult() const;

 private:
  ValidationResult apply(RouteLoadResult candidate);
  ValidationResult invalidate(const std::string& reason);

  WaypointYamlLoader loader_;
  RoutePlan active_route_;
  ValidationResult last_result_;
  bool active_{false};
  bool has_history_{false};
  std::string route_id_history_;
  std::uint64_t plan_version_history_{0U};
};

}  // namespace planning
}  // namespace auto_rover
