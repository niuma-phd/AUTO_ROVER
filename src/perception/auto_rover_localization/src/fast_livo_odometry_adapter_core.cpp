#include "auto_rover_localization/fast_livo_odometry_adapter_core.hpp"

#include <cmath>
#include <limits>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_core/validation.hpp"

namespace auto_rover {
namespace localization {
namespace {

constexpr char kAuditedFrameId[] = "camera_init";
constexpr char kAuditedChildFrameId[] = "aft_mapped";
constexpr char kAuditedSourceRevision[] =
    "3df020182aee52d81fd1a6b543bfb611c46d11bc";
constexpr char kRearAxleReferenceFrame[] = "rear_axle_center";
constexpr char kSourceIdPrefix[] = "fast_livo2_ros1_main@";
constexpr double kQuaternionNormSquaredTolerance = 1e-6;

bool quaternionIsNormalized(const Quaternion& quaternion) {
  if (!isFinite(quaternion)) {
    return false;
  }
  const double norm_squared =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return isFinite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= kQuaternionNormSquaredTolerance;
}

ValidationResult validateConfig(
    const FastLivoOdometryAdapterConfig& config) {
  if (config.expected_frame_id != kAuditedFrameId) {
    return ValidationResult::failure(
        "expected frame_id must explicitly be camera_init");
  }
  if (config.expected_child_frame_id != kAuditedChildFrameId) {
    return ValidationResult::failure(
        "expected child_frame_id must explicitly be aft_mapped");
  }
  if (config.source_revision != kAuditedSourceRevision) {
    return ValidationResult::failure(
        "source revision does not match the audited ROS1 revision");
  }
  if (config.process_generation_id.empty()) {
    return ValidationResult::failure(
        "localization process generation identity is missing");
  }
  if (config.timestamp_semantics != TimeSource::kPublishTime) {
    return ValidationResult::failure(
        "FAST-LIVO2 timestamp semantics must be PUBLISH_TIME");
  }
  if (config.freshness_limit_ns <= 0) {
    return ValidationResult::failure(
        "receipt freshness limit must be positive");
  }
  if (!config.extrinsic_known) {
    return ValidationResult::failure(
        "source-to-rear extrinsic is not known");
  }
  if (!isFinite(config.source_T_rear.position) ||
      !isFinite(config.source_T_rear.orientation)) {
    return ValidationResult::failure(
        "source-to-rear extrinsic contains a non-finite value");
  }
  if (!quaternionIsNormalized(config.source_T_rear.orientation)) {
    return ValidationResult::failure(
        "source-to-rear extrinsic quaternion must be normalized");
  }
  return ValidationResult::success();
}

std::string sourceId(const FastLivoOdometryAdapterConfig& config) {
  return std::string(kSourceIdPrefix) + config.source_revision +
         "#process=" + config.process_generation_id;
}

}  // namespace

FastLivoOdometryAdapterCore::FastLivoOdometryAdapterCore(
    const FastLivoOdometryAdapterConfig& config)
    : config_(config), config_validation_(validateConfig(config_)) {}

FastLivoOdometryAdapterResult FastLivoOdometryAdapterCore::invalidResult(
    const SourceOdometry& source, const std::string& reason) const {
  FastLivoOdometryAdapterResult result;
  result.ego_state.stamp_ns = source.provider_stamp_ns;
  result.ego_state.frame_id = config_.expected_frame_id;
  result.ego_state.time_source = config_.timestamp_semantics;
  result.ego_state.reference_frame = kRearAxleReferenceFrame;
  result.ego_state.valid_mask = 0U;
  result.ego_state.source_id = sourceId(config_);
  result.ego_state.valid = false;
  result.reason = reason;
  return result;
}

FastLivoOdometryAdapterResult FastLivoOdometryAdapterCore::adapt(
    const SourceOdometry& source, std::int64_t receipt_monotonic_ns,
    std::int64_t now_monotonic_ns) {
  if (!config_validation_.ok) {
    return invalidResult(source, config_validation_.reason);
  }
  if (source.provider_stamp_ns <= 0) {
    return invalidResult(source, "provider stamp must be positive");
  }
  if (source.frame_id != config_.expected_frame_id) {
    return invalidResult(source, "source frame_id does not match camera_init");
  }
  if (source.child_frame_id != config_.expected_child_frame_id) {
    return invalidResult(source,
                         "source child_frame_id does not match aft_mapped");
  }
  if (!isFinite(source.pose.position)) {
    return invalidResult(source, "source pose position is non-finite");
  }
  if (!isFinite(source.pose.orientation)) {
    return invalidResult(source, "source pose quaternion is non-finite");
  }
  Pose3 world_T_source = source.pose;
  if (!normalizeQuaternion(source.pose.orientation,
                           &world_T_source.orientation)) {
    return invalidResult(
        source, "source pose quaternion must have a finite non-zero norm");
  }
  if (source.provider_stamp_ns <= last_accepted_provider_stamp_ns_) {
    return invalidResult(source,
                         "provider stamp must be strictly increasing");
  }
  if (receipt_monotonic_ns <= 0) {
    return invalidResult(source,
                         "receipt monotonic time must be positive");
  }
  if (now_monotonic_ns < receipt_monotonic_ns) {
    return invalidResult(source,
                         "monotonic clock regressed behind receipt time");
  }
  if (!isFresh(receipt_monotonic_ns, now_monotonic_ns,
               config_.freshness_limit_ns)) {
    return invalidResult(source, "source receipt is stale");
  }
  if (last_state_id_ == std::numeric_limits<std::uint64_t>::max()) {
    return invalidResult(source, "semantic state id space is exhausted");
  }

  const Pose3 world_T_rear =
      composePoses(world_T_source, config_.source_T_rear);
  if (!isFinite(world_T_rear) ||
      !quaternionIsNormalized(world_T_rear.orientation)) {
    return invalidResult(source, "composed rear-axle pose is invalid");
  }

  FastLivoOdometryAdapterResult result;
  result.ego_state.stamp_ns = source.provider_stamp_ns;
  result.ego_state.frame_id = config_.expected_frame_id;
  result.ego_state.state_id = last_state_id_ + 1U;
  result.ego_state.time_source = TimeSource::kPublishTime;
  result.ego_state.reference_frame = kRearAxleReferenceFrame;
  result.ego_state.pose = world_T_rear;
  result.ego_state.valid_mask = 0U;
  result.ego_state.source_id = sourceId(config_);
  result.ego_state.valid = true;

  last_accepted_provider_stamp_ns_ = source.provider_stamp_ns;
  last_state_id_ = result.ego_state.state_id;
  return result;
}

}  // namespace localization
}  // namespace auto_rover
