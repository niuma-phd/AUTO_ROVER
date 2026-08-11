#pragma once

#include <cstddef>

namespace auto_rover {
namespace planning {

// These limits bound every caller-controlled planning allocation or iteration
// before a route can become executable. Identity lengths are UTF-8 byte counts,
// matching the ROS string and execution-boundary representation.
constexpr std::size_t kMaximumWaypointYamlBytes = 1024U * 1024U;
constexpr std::size_t kMaximumYamlKeyBytes = 64U;
constexpr std::size_t kMaximumNumericScalarBytes = 64U;
constexpr std::size_t kMaximumBooleanScalarBytes = 5U;
constexpr std::size_t kMaximumRouteIdBytes = 210U;
constexpr std::size_t kMaximumFrameIdBytes = 256U;
constexpr std::size_t kMaximumProfileIdBytes = 256U;
constexpr std::size_t kMaximumWaypointCount = 2048U;
constexpr std::size_t kMaximumTrajectoryIdBytes = 256U;
constexpr std::size_t kMaximumTrajectoryPointCount = 4096U;

// A generated identity is route_id + two separators + the longest uint64
// decimal plan version + "fnv1a64-" + sixteen hexadecimal hash characters.
constexpr std::size_t kMaximumGeneratedTrajectoryIdOverheadBytes = 46U;
static_assert(kMaximumRouteIdBytes +
                      kMaximumGeneratedTrajectoryIdOverheadBytes <=
                  kMaximumTrajectoryIdBytes,
              "route identity budget must fit the generated trajectory ID");

}  // namespace planning
}  // namespace auto_rover
