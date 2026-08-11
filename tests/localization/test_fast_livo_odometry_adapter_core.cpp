#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include "auto_rover_core/geometry.hpp"
#include "auto_rover_localization/fast_livo_odometry_adapter_core.hpp"

namespace {

constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr double kPi = 3.141592653589793238462643383279502884;

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) +
             " expected=" + std::to_string(expected));
}

void expectInvalid(
    const auto_rover::localization::FastLivoOdometryAdapterResult& result,
    const std::string& reason_fragment, const std::string& message) {
  expect(!result.ego_state.valid, message + " returns invalid EgoState");
  expect(!result.reason.empty(), message + " returns a reason");
  expect(result.reason.find(reason_fragment) != std::string::npos,
         message + " reason contains " + reason_fragment +
             ", actual=" + result.reason);
  expect(result.ego_state.valid_mask == 0U,
         message + " does not claim twist or covariance");
}

auto_rover::localization::FastLivoOdometryAdapterConfig validConfig() {
  auto_rover::localization::FastLivoOdometryAdapterConfig config;
  config.expected_frame_id = "camera_init";
  config.expected_child_frame_id = "aft_mapped";
  config.source_revision =
      "3df020182aee52d81fd1a6b543bfb611c46d11bc";
  config.process_generation_id = "localization-test-generation-1";
  config.timestamp_semantics = auto_rover::TimeSource::kPublishTime;
  config.extrinsic_known = true;
  config.source_T_rear.position.x = 1.0;
  config.source_T_rear.orientation =
      auto_rover::quaternionFromYaw(-kPi / 2.0);
  config.freshness_limit_ns = 100000000LL;
  return config;
}

auto_rover::localization::SourceOdometry validSource(std::int64_t stamp_ns) {
  auto_rover::localization::SourceOdometry source;
  source.provider_stamp_ns = stamp_ns;
  source.frame_id = "camera_init";
  source.child_frame_id = "aft_mapped";
  source.pose.position.x = 10.0;
  source.pose.position.y = 20.0;
  source.pose.orientation = auto_rover::quaternionFromYaw(kPi / 2.0);
  return source;
}

void testNonIdentityExtrinsicComposition() {
  auto_rover::localization::FastLivoOdometryAdapterCore adapter(validConfig());
  const auto result =
      adapter.adapt(validSource(5 * kSecondNs), 8 * kSecondNs,
                    8 * kSecondNs + 50000000LL);

  expect(result.ego_state.valid, "valid source produces a valid EgoState");
  expect(result.reason.empty(), "valid conversion has no failure reason");
  expect(result.ego_state.stamp_ns == 5 * kSecondNs,
         "provider publish stamp is preserved");
  expect(result.ego_state.time_source == auto_rover::TimeSource::kPublishTime,
         "audited timestamp is declared as publish time");
  expect(result.ego_state.frame_id == "camera_init",
         "world frame is preserved");
  expect(result.ego_state.reference_frame == "rear_axle_center",
         "output reference is the rear axle centre");
  expect(result.ego_state.state_id == 1U,
         "first accepted source receives state id one");
  expect(result.ego_state.valid_mask == 0U,
         "FAST-LIVO2 twist and covariance remain unavailable");
  expect(result.ego_state.source_id ==
             "fast_livo2_ros1_main@"
             "3df020182aee52d81fd1a6b543bfb611c46d11bc"
             "#process=localization-test-generation-1",
         "source identity pins revision and process generation");
  expectNear(result.ego_state.pose.position.x, 10.0, 1e-12,
             "source translation is rotated before composition");
  expectNear(result.ego_state.pose.position.y, 21.0, 1e-12,
             "world_T_rear equals world_T_source times source_T_rear");
  expectNear(auto_rover::yawFromQuaternion(
                 result.ego_state.pose.orientation),
             0.0, 1e-12,
             "non-identity source and extrinsic orientations compose");

  const auto second =
      adapter.adapt(validSource(6 * kSecondNs), 9 * kSecondNs,
                    9 * kSecondNs);
  expect(second.ego_state.valid && second.ego_state.state_id == 2U,
         "accepted provider stamps produce increasing semantic ids");
}

void testConfigurationFailsClosed() {
  auto config = validConfig();
  config.extrinsic_known = false;
  config.source_T_rear = auto_rover::Pose3{};
  config.source_T_rear.orientation.w = 1.0;
  auto_rover::localization::FastLivoOdometryAdapterCore unknown(config);
  expectInvalid(unknown.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
                "extrinsic", "unknown extrinsic");

  config = validConfig();
  config.source_revision = "3df0201";
  auto_rover::localization::FastLivoOdometryAdapterCore wrong_revision(config);
  expectInvalid(
      wrong_revision.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
      "revision", "unapproved source revision");

  config = validConfig();
  config.process_generation_id.clear();
  auto_rover::localization::FastLivoOdometryAdapterCore missing_generation(
      config);
  expectInvalid(
      missing_generation.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
      "generation", "missing process generation identity");

  config = validConfig();
  config.timestamp_semantics = auto_rover::TimeSource::kSampleTime;
  auto_rover::localization::FastLivoOdometryAdapterCore wrong_time(config);
  expectInvalid(wrong_time.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
                "PUBLISH_TIME", "stronger timestamp semantics");

  config = validConfig();
  config.expected_frame_id = "map";
  auto_rover::localization::FastLivoOdometryAdapterCore wrong_expected_frame(
      config);
  expectInvalid(
      wrong_expected_frame.adapt(validSource(kSecondNs), kSecondNs,
                                 kSecondNs),
      "expected frame", "non-audited configured world frame");

  config = validConfig();
  config.expected_child_frame_id.clear();
  auto_rover::localization::FastLivoOdometryAdapterCore implicit_child(config);
  expectInvalid(
      implicit_child.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
      "expected child", "implicit child-frame fallback");

  config = validConfig();
  config.source_T_rear.position.z =
      std::numeric_limits<double>::quiet_NaN();
  auto_rover::localization::FastLivoOdometryAdapterCore invalid_extrinsic(
      config);
  expectInvalid(
      invalid_extrinsic.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
      "extrinsic", "non-finite extrinsic");

  config = validConfig();
  config.source_T_rear.orientation.z *= 2.0;
  config.source_T_rear.orientation.w *= 2.0;
  auto_rover::localization::FastLivoOdometryAdapterCore
      non_normalized_extrinsic(config);
  expectInvalid(non_normalized_extrinsic.adapt(
                    validSource(kSecondNs), kSecondNs, kSecondNs),
                "normalized", "non-normalized configured extrinsic");

  config = validConfig();
  config.freshness_limit_ns = 0;
  auto_rover::localization::FastLivoOdometryAdapterCore missing_freshness(
      config);
  expectInvalid(
      missing_freshness.adapt(validSource(kSecondNs), kSecondNs, kSecondNs),
      "freshness", "missing receipt freshness policy");
}

void testFrameAndChildValidation() {
  auto_rover::localization::FastLivoOdometryAdapterCore adapter(validConfig());
  auto source = validSource(kSecondNs);
  source.frame_id = "map";
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "frame_id",
                "wrong provider world frame");

  source = validSource(kSecondNs);
  source.child_frame_id = "base_link";
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "child_frame_id",
                "wrong provider child frame");

  source = validSource(kSecondNs);
  expect(adapter.adapt(source, kSecondNs, kSecondNs).ego_state.valid,
         "frame failures do not poison the first matching sample");
}

void testProviderStampOrdering() {
  auto_rover::localization::FastLivoOdometryAdapterCore adapter(validConfig());
  auto source = validSource(0);
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "positive",
                "zero provider stamp");

  source = validSource(10 * kSecondNs);
  expect(adapter.adapt(source, kSecondNs, kSecondNs).ego_state.valid,
         "first positive provider stamp is accepted");

  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "increasing",
                "duplicate provider stamp");

  source = validSource(9 * kSecondNs);
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "increasing",
                "regressing provider stamp");

  source = validSource(11 * kSecondNs);
  const auto recovered = adapter.adapt(source, kSecondNs, kSecondNs);
  expect(recovered.ego_state.valid && recovered.ego_state.state_id == 2U,
         "strictly newer source recovers without incrementing on rejects");
}

void testRestartUsesANewSourceIdentity() {
  auto first_config = validConfig();
  auto second_config = validConfig();
  second_config.process_generation_id = "localization-test-generation-2";
  auto_rover::localization::FastLivoOdometryAdapterCore first(first_config);
  auto_rover::localization::FastLivoOdometryAdapterCore second(second_config);
  const auto first_state =
      first.adapt(validSource(10 * kSecondNs), kSecondNs, kSecondNs).ego_state;
  const auto second_state =
      second.adapt(validSource(1 * kSecondNs), kSecondNs, kSecondNs).ego_state;
  expect(first_state.valid && second_state.valid &&
             first_state.state_id == 1U && second_state.state_id == 1U,
         "each process generation owns its own state-id sequence");
  expect(first_state.source_id != second_state.source_id,
         "localization restart exposes a new producer identity");
}

void testReceiptFreshness() {
  auto_rover::localization::FastLivoOdometryAdapterCore adapter(validConfig());
  const auto source = validSource(kSecondNs);
  expect(adapter
             .adapt(source, 2 * kSecondNs,
                    2 * kSecondNs + 100000000LL)
             .ego_state.valid,
         "receipt at the inclusive freshness boundary is accepted");

  auto_rover::localization::FastLivoOdometryAdapterCore stale_adapter(
      validConfig());
  expectInvalid(stale_adapter.adapt(source, 2 * kSecondNs,
                                    2 * kSecondNs + 100000001LL),
                "stale", "receipt beyond the freshness limit");

  auto_rover::localization::FastLivoOdometryAdapterCore regressed_clock(
      validConfig());
  expectInvalid(regressed_clock.adapt(source, 2 * kSecondNs,
                                      2 * kSecondNs - 1LL),
                "monotonic", "monotonic clock regression");

  auto_rover::localization::FastLivoOdometryAdapterCore missing_receipt(
      validConfig());
  expectInvalid(missing_receipt.adapt(source, 0, 2 * kSecondNs), "receipt",
                "missing monotonic receipt time");
}

void testNonFiniteAndQuaternionValidation() {
  auto_rover::localization::FastLivoOdometryAdapterCore adapter(validConfig());
  auto source = validSource(kSecondNs);
  source.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "non-finite",
                "NaN provider position");

  source = validSource(kSecondNs);
  source.pose.orientation = auto_rover::Quaternion{};
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "quaternion",
                "zero provider quaternion");

  source = validSource(kSecondNs);
  source.pose.orientation.w =
      std::numeric_limits<double>::infinity();
  expectInvalid(adapter.adapt(source, kSecondNs, kSecondNs), "quaternion",
                "infinite provider quaternion");

  source = validSource(kSecondNs);
  source.pose.orientation.z *= 2.0;
  source.pose.orientation.w *= 2.0;
  const auto normalized = adapter.adapt(source, kSecondNs, kSecondNs);
  expect(normalized.ego_state.valid,
         "finite non-zero provider quaternion is normalized before use");
  expectNear(auto_rover::yawFromQuaternion(
                 normalized.ego_state.pose.orientation),
             0.0, 1e-12,
             "normalized provider quaternion composes with the extrinsic");
  expect(normalized.ego_state.state_id == 1U,
         "invalid numeric input does not consume semantic state ids");
}

}  // namespace

int main() {
  testNonIdentityExtrinsicComposition();
  testConfigurationFailsClosed();
  testFrameAndChildValidation();
  testProviderStampOrdering();
  testRestartUsesANewSourceIdentity();
  testReceiptFreshness();
  testNonFiniteAndQuaternionValidation();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "FAST-LIVO2 localization adapter tests passed\n";
  return EXIT_SUCCESS;
}
