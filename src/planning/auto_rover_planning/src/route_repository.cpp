#include "auto_rover_planning/route_repository.hpp"

#include <utility>

namespace auto_rover {
namespace planning {

RouteRepository::RouteRepository(VehicleProfile profile,
                                 std::string expected_frame)
    : loader_(std::move(profile), std::move(expected_frame)),
      last_result_(ValidationResult::failure("no route has been loaded")) {}

ValidationResult RouteRepository::reloadFromFile(
    const std::string& path, std::int64_t load_stamp_ns) {
  return apply(loader_.loadFile(path, load_stamp_ns));
}

ValidationResult RouteRepository::reloadFromString(
    const std::string& yaml_text, std::int64_t load_stamp_ns) {
  return apply(loader_.loadString(yaml_text, load_stamp_ns));
}

bool RouteRepository::hasActiveRoute() const { return active_; }

const RoutePlan& RouteRepository::activeRoute() const { return active_route_; }

const ValidationResult& RouteRepository::lastResult() const {
  return last_result_;
}

ValidationResult RouteRepository::apply(RouteLoadResult candidate) {
  if (!candidate.validation.ok) {
    return invalidate(candidate.validation.reason);
  }
  if (has_history_) {
    if (candidate.route.route_id != route_id_history_) {
      return invalidate("replacement route_id must match route history");
    }
    if (candidate.route.plan_version <= plan_version_history_) {
      return invalidate("replacement plan version must strictly increase");
    }
  }

  active_route_ = std::move(candidate.route);
  active_ = true;
  has_history_ = true;
  route_id_history_ = active_route_.route_id;
  plan_version_history_ = active_route_.plan_version;
  last_result_ = ValidationResult::success();
  return last_result_;
}

ValidationResult RouteRepository::invalidate(const std::string& reason) {
  active_route_ = RoutePlan{};
  active_ = false;
  last_result_ = ValidationResult::failure(reason);
  return last_result_;
}

}  // namespace planning
}  // namespace auto_rover
