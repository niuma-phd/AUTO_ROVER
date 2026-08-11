#pragma once

#include <cstdint>

#include "auto_rover_core/types.hpp"

namespace auto_rover {
namespace planning {

struct TrajectoryGeneratorConfig {
  double sampling_resolution_m{0.0};
  std::int64_t valid_for_ns{0};
};

struct TrajectoryGenerationResult {
  Trajectory trajectory;
  ValidationResult validation;
};

class HermiteTrajectoryGenerator {
 public:
  explicit HermiteTrajectoryGenerator(TrajectoryGeneratorConfig config);

  TrajectoryGenerationResult generate(const RoutePlan& route,
                                      const VehicleProfile& profile,
                                      std::int64_t generation_stamp_ns) const;

 private:
  TrajectoryGeneratorConfig config_;
};

}  // namespace planning
}  // namespace auto_rover
