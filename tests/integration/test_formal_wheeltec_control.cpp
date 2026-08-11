#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "auto_rover_control/pure_pursuit.hpp"
#include "auto_rover_vcu_wheeltec_serial/codec.hpp"
#include "auto_rover_vcu_wheeltec_serial/transport.hpp"
#include "auto_rover_vcu_wheeltec_serial/vehicle_backend.hpp"
#include "auto_rover_vehicle/vehicle_execution_core.hpp"

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

constexpr std::int64_t kMillisecondNs = 1000000LL;
constexpr std::int64_t kCyclePeriodNs = 50000000LL;

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

std::uint8_t xorBytes(const std::uint8_t* bytes, std::size_t count) {
  std::uint8_t value = 0U;
  for (std::size_t index = 0U; index < count; ++index) {
    value = static_cast<std::uint8_t>(value ^ bytes[index]);
  }
  return value;
}

void putSignedBigEndian(std::int16_t value, std::uint8_t* high,
                        std::uint8_t* low) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  *high = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
  *low = static_cast<std::uint8_t>(bits & 0xFFU);
}

wheeltec::FeedbackFrame feedbackFrame() {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = 0U;
  putSignedBigEndian(0, &frame[2U], &frame[3U]);
  putSignedBigEndian(0, &frame[6U], &frame[7U]);
  putSignedBigEndian(24000, &frame[20U], &frame[21U]);
  frame[22U] = xorBytes(frame.data(), 22U);
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

bool isExactZeroCommand(const std::vector<std::uint8_t>& frame) {
  return frame.size() == wheeltec::kCommandFrameSize &&
         frame[0U] == wheeltec::kFrameHeader && frame[1U] == 0U &&
         frame[2U] == 0U && frame[3U] == 0U && frame[4U] == 0U &&
         frame[5U] == 0U && frame[6U] == 0U && frame[7U] == 0U &&
         frame[8U] == 0U && frame[10U] == wheeltec::kFrameTail;
}

bool isNonzeroCommand(const std::vector<std::uint8_t>& frame) {
  return frame.size() == wheeltec::kCommandFrameSize &&
         (frame[3U] != 0U || frame[4U] != 0U || frame[7U] != 0U ||
          frame[8U] != 0U);
}

class ScriptedTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedTransport(std::int64_t* monotonic_clock)
      : monotonic_clock_(monotonic_clock) {}

  bool isConnected() const override { return true; }
  std::uint64_t connectionGeneration() const override { return 1U; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t) override {
    writes.emplace_back(data, data + size);
    advanceClock();
    return {wheeltec::TransportStatus::kOk, size, 0, false};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t) override {
    advanceClock();
    if (reads_.empty()) {
      return {wheeltec::TransportStatus::kDeadlineExceeded, 0U, 0, false};
    }
    const std::vector<std::uint8_t> bytes = reads_.front();
    reads_.pop_front();
    const std::size_t transferred = std::min(capacity, bytes.size());
    if (transferred > 0U && data != nullptr) {
      std::memcpy(data, bytes.data(), transferred);
    }
    return {wheeltec::TransportStatus::kOk, transferred, 0, false};
  }

  void enqueueFeedback() {
    const auto frame = feedbackFrame();
    reads_.emplace_back(frame.begin(), frame.end());
  }

  std::vector<std::vector<std::uint8_t>> writes;

 private:
  void advanceClock() {
    if (monotonic_clock_ != nullptr) {
      *monotonic_clock_ += kMillisecondNs;
    }
  }

  std::int64_t* monotonic_clock_{nullptr};
  std::deque<std::vector<std::uint8_t>> reads_;
};

auto_rover::VehicleProfile vehicleProfile() {
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

auto_rover_control::PurePursuitConfig trackerConfig() {
  auto_rover_control::PurePursuitConfig config;
  config.world_frame = "camera_init";
  config.control_frame = "rear_axle_center";
  config.producer_generation_id = "formal-wheeltec-control-integration";
  config.lookahead_min_m = 0.40;
  config.lookahead_max_m = 0.80;
  config.lookahead_speed_gain_s = 0.50;
  config.goal_position_tolerance_m = 0.08;
  config.standstill_speed_threshold_mps = 0.01;
  config.localization_freshness_ns = 2000000000LL;
  config.trajectory_freshness_ns = 2000000000LL;
  config.chassis_freshness_ns = 2000000000LL;
  config.safety_freshness_ns = 2000000000LL;
  config.motion_valid_for_ns = 2000000000LL;
  return config;
}

auto_rover_vehicle::VehicleExecutionCommonConfig executionConfig() {
  auto_rover_vehicle::VehicleExecutionCommonConfig config;
  config.vehicle_profile = vehicleProfile();
  config.safety_supervisor.actuation_enabled = true;
  config.safety_supervisor.reset_service_enabled = true;
  config.safety_supervisor.fresh_recovery_count = 1U;
  config.safety_supervisor.state_valid_for_ns = 2000000000LL;
  config.authorized_reset_operator_id = "formal-integration-reset";
  config.command_guard.world_frame = "camera_init";
  config.command_guard.localization_freshness_ns = 2000000000LL;
  config.command_guard.trajectory_freshness_ns = 2000000000LL;
  config.command_guard.motion_freshness_ns = 2000000000LL;
  config.command_guard.chassis_freshness_ns = 2000000000LL;
  config.command_guard.safety_freshness_ns = 2000000000LL;
  config.command_guard.fresh_recovery_count = 1U;
  config.vehicle_manager.command_valid_for_ns = 100000000LL;
  return config;
}

wheeltec::RuntimeConfig runtimeConfig() {
  wheeltec::RuntimeConfig config;
  config.adapter.codec_limits.max_forward_speed_mps = 0.50;
  config.adapter.max_command_age_ns = 100000000LL;
  config.adapter.fresh_commands_required = 3U;
  config.adapter.zero_retry_interval_ns = 20000000LL;
  config.adapter.max_zero_write_attempts = 3U;
  config.adapter.write_timeout_ns = 10000000LL;
  config.read_timeout_ns = 1000000LL;
  config.maximum_feedback_age_ns = 150000000LL;
  config.drain_read_timeout_ns = 1000000LL;
  config.maximum_drain_reads = 64U;
  config.control_allowed_receipts_required = 5U;
  config.control_allowed_minimum_span_ns = 200000000LL;
  config.maximum_normal_write_gap_ns = 100000000LL;
  config.maximum_step_duration_ns = 50000000LL;
  config.runtime_enabled = true;
  config.unverified_protocol_acknowledged = true;
  config.physical_device_opt_in = true;
  config.actuation_opt_in = true;
  config.readiness_gate_passed = true;
  config.external_or_durable_estop_strategy_approved = true;
  return config;
}

auto_rover::EgoState egoState(std::int64_t stamp_ns) {
  auto_rover::EgoState value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "camera_init";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kPublishTime;
  value.reference_frame = "rear_axle_center";
  value.pose.orientation.w = 1.0;
  value.source_id = "formal-integration-localization";
  value.valid = true;
  return value;
}

auto_rover::Trajectory trajectory(std::int64_t stamp_ns) {
  auto_rover::Trajectory value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "camera_init";
  value.trajectory_id = "formal-route:1:1";
  value.route_id = "formal-route";
  value.plan_version = 1U;
  value.vehicle_profile_id = "nuc_senior_akm_v1";
  value.valid_for_ns = 2000000000LL;
  value.completion_behavior = auto_rover::CompletionBehavior::kStopAndHold;
  value.valid = true;
  auto_rover::TrajectoryPoint first;
  first.target_speed_mps = 0.50;
  first.direction = auto_rover::Direction::kForward;
  auto_rover::TrajectoryPoint second = first;
  second.x_m = 2.0;
  second.arc_length_m = 2.0;
  value.points = {first, second};
  return value;
}

struct GraphStep {
  auto_rover_vehicle::VehicleExecutionCycleResult execution;
  auto_rover_control::TrackingResult tracking;
};

class FormalGraphRig {
 public:
  FormalGraphRig()
      : transport_(&monotonic_ns_),
        profile_(vehicleProfile()),
        tracker_(trackerConfig(), profile_),
        ego_(egoState(ros_ns_)),
        trajectory_(trajectory(ros_ns_)) {
    wheeltec::WheeltecVehicleBackendConfig backend_config;
    backend_config.runtime = runtimeConfig();
    backend_config.chassis_frame_id = "rear_axle_center";
    backend_config.source_id =
        "wheeltec_serial_unverified#process=formal-control-integration";
    wheeltec::WheeltecVehicleBackendOperations operations;
    operations.runtime.monotonic_now_ns = [this]() { return monotonic_ns_; };
    operations.runtime.wait_until_monotonic_ns =
        [this](std::int64_t deadline_ns) {
          if (deadline_ns <= 0 || deadline_ns < monotonic_ns_) {
            return false;
          }
          monotonic_ns_ = deadline_ns;
          return true;
        };
    operations.ros_now_ns = [this]() { return ros_ns_; };
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(
        new wheeltec::WheeltecVehicleBackend(backend_config, &transport_,
                                             operations));
    core_.reset(new auto_rover_vehicle::VehicleExecutionCore(
        executionConfig(), std::move(backend)));
  }

  void initialize() {
    expect(core_->initialize(monotonic_ns_, ros_ns_),
           "formal graph initializes through the Wheeltec backend");
    expect(transport_.writes.size() == 1U &&
               isExactZeroCommand(transport_.writes.front()),
           "formal graph initialization emits only the startup exact zero");
    monotonic_ns_ += kMillisecondNs;
    ros_ns_ += kMillisecondNs;
    ego_receipt_ns_ = monotonic_ns_;
    trajectory_receipt_ns_ = monotonic_ns_;
    expect(core_->updateEgoState(ego_, ego_receipt_ns_).ok,
           "formal graph accepts localization");
    expect(core_->updateTrajectory(trajectory_, trajectory_receipt_ns_).ok,
           "formal graph accepts trajectory");

    monotonic_ns_ += kCyclePeriodNs;
    ros_ns_ += kCyclePeriodNs;
    const auto drained = core_->cycle(monotonic_ns_, ros_ns_);
    expect(!drained.chassis_available && drained.stop_delivery.stop_attempted &&
               drained.stop_delivery.delivered,
           "formal graph drains opening backlog while remaining stopped");
  }

  GraphStep cycleAndTrack() {
    transport_.enqueueFeedback();
    monotonic_ns_ += kCyclePeriodNs;
    ros_ns_ += kCyclePeriodNs;
    GraphStep output;
    output.execution = core_->cycle(monotonic_ns_, ros_ns_);
    expect(output.execution.chassis_available,
           "formal graph publishes complete Wheeltec feedback");
    if (!output.execution.chassis_available) {
      return output;
    }
    latest_chassis_ = output.execution.chassis;
    latest_chassis_receipt_ns_ = monotonic_ns_;
    auto_rover_control::TrackingInput input;
    input.trajectory = {trajectory_, trajectory_receipt_ns_};
    input.ego = {ego_, ego_receipt_ns_};
    input.chassis = {latest_chassis_, latest_chassis_receipt_ns_};
    input.safety = {output.execution.safety, monotonic_ns_};
    input.now_monotonic_ns = monotonic_ns_;
    input.now_ros_ns = ros_ns_;
    output.tracking = tracker_.update(input);
    core_->updateMotionReference(output.tracking.reference, monotonic_ns_);
    return output;
  }

  auto_rover_control::TrackingResult trackSafety(
      const auto_rover::SafetyState& safety) {
    auto_rover_control::TrackingInput input;
    input.trajectory = {trajectory_, trajectory_receipt_ns_};
    input.ego = {ego_, ego_receipt_ns_};
    input.chassis = {latest_chassis_, latest_chassis_receipt_ns_};
    input.safety = {safety, monotonic_ns_};
    input.now_monotonic_ns = monotonic_ns_;
    input.now_ros_ns = ros_ns_;
    const auto output = tracker_.update(input);
    core_->updateMotionReference(output.reference, monotonic_ns_);
    return output;
  }

  auto_rover_vehicle::VehicleExecutionServiceResult arm() {
    monotonic_ns_ += kMillisecondNs;
    ros_ns_ += kMillisecondNs;
    const auto current = core_->currentSafetyState(ros_ns_);
    return core_->requestArm("formal-integration-operator", current.state_id,
                             true, monotonic_ns_, ros_ns_);
  }

  std::int64_t monotonicNow() const { return monotonic_ns_; }
  std::int64_t rosNow() const { return ros_ns_; }
  const std::vector<std::vector<std::uint8_t>>& writes() const {
    return transport_.writes;
  }

 private:
  std::int64_t monotonic_ns_{1000000000LL};
  std::int64_t ros_ns_{2000000000LL};
  ScriptedTransport transport_;
  auto_rover::VehicleProfile profile_;
  auto_rover_control::PurePursuit tracker_;
  std::unique_ptr<auto_rover_vehicle::VehicleExecutionCore> core_;
  auto_rover::EgoState ego_;
  auto_rover::Trajectory trajectory_;
  auto_rover::ChassisState latest_chassis_;
  std::int64_t ego_receipt_ns_{0};
  std::int64_t trajectory_receipt_ns_{0};
  std::int64_t latest_chassis_receipt_ns_{0};
};

void testFormalWheeltecGraphCanArmWithoutFabricatedEnableFeedback() {
  FormalGraphRig rig;
  rig.initialize();

  GraphStep recovered;
  bool reached_disarmed = false;
  for (std::size_t index = 0U; index < 8U; ++index) {
    recovered = rig.cycleAndTrack();
    expect(recovered.tracking.reference.valid,
           "inhibited formal graph supplies a valid zero reference");
    expectNear(recovered.tracking.reference.target_speed_mps, 0.0, 1e-12,
               "inhibited formal graph cannot accumulate a speed ramp");
    expect((recovered.execution.chassis.valid_mask &
            auto_rover::ChassisState::kControlEnabledValid) == 0U,
           "formal Wheeltec feedback never fabricates VCU control enable");
    if (recovered.execution.health_clear &&
        recovered.execution.safety.mode ==
            auto_rover::SafetyMode::kDisarmed) {
      reached_disarmed = true;
      break;
    }
  }
  expect(reached_disarmed,
         "zero references close the startup feedback loop to disarmed");

  const auto armed = rig.arm();
  expect(armed.success &&
             armed.state.mode == auto_rover::SafetyMode::kArmed,
         "the healthy formal graph accepts explicit software arm without a fabricated VCU enable bit");
  const auto armed_zero = rig.trackSafety(armed.state);
  expect(armed_zero.reference.valid,
         "armed software safety produces a valid formal reference");
  expectNear(armed_zero.reference.target_speed_mps, 0.0, 1e-12,
             "the first armed formal reference establishes a zero origin");

  GraphStep ramp = rig.cycleAndTrack();
  expect(ramp.tracking.reference.valid &&
             ramp.tracking.reference.target_speed_mps > 0.0,
         "the second armed formal controller update starts motion reference ramping");
  expect(ramp.tracking.reference.target_speed_mps <= 0.011,
         "the formal controller ramp stays within 0.20 m/s2 at 50 ms");

  bool wrote_nonzero = false;
  for (std::size_t index = 0U; index < 5U && !wrote_nonzero; ++index) {
    ramp = rig.cycleAndTrack();
    wrote_nonzero = !rig.writes().empty() &&
                    isNonzeroCommand(rig.writes().back());
  }
  expect(wrote_nonzero,
         "nonzero formal wire output appears only after controller and backend recovery gates");
}

}  // namespace

int main() {
  testFormalWheeltecGraphCanArmWithoutFabricatedEnableFeedback();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "formal Wheeltec control integration tests passed\n";
  return EXIT_SUCCESS;
}
