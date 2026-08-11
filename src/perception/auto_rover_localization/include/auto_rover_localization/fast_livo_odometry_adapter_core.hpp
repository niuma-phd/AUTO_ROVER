#pragma once

#include <cstdint>
#include <string>

#include "auto_rover_core/types.hpp"

namespace auto_rover {
namespace localization {

struct SourceOdometry {
  std::int64_t provider_stamp_ns{0};
  std::string frame_id;
  std::string child_frame_id;
  Pose3 pose;
};

struct FastLivoOdometryAdapterConfig {
  std::string expected_frame_id;
  std::string expected_child_frame_id;
  std::string source_revision;
  std::string process_generation_id;
  TimeSource timestamp_semantics{TimeSource::kUnknown};
  bool extrinsic_known{false};
  Pose3 source_T_rear;
  std::int64_t freshness_limit_ns{0};
};

struct FastLivoOdometryAdapterResult {
  EgoState ego_state;
  std::string reason;
};

class FastLivoOdometryAdapterCore {
 public:
  explicit FastLivoOdometryAdapterCore(
      const FastLivoOdometryAdapterConfig& config);

  FastLivoOdometryAdapterResult adapt(
      const SourceOdometry& source, std::int64_t receipt_monotonic_ns,
      std::int64_t now_monotonic_ns);

 private:
  FastLivoOdometryAdapterResult invalidResult(
      const SourceOdometry& source, const std::string& reason) const;

  FastLivoOdometryAdapterConfig config_;
  ValidationResult config_validation_;
  std::int64_t last_accepted_provider_stamp_ns_{0};
  std::uint64_t last_state_id_{0U};
};

}  // namespace localization
}  // namespace auto_rover
