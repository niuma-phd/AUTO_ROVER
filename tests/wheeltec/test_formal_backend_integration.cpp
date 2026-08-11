#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

bool contains(const std::string& value, const std::string& fragment) {
  return value.find(fragment) != std::string::npos;
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

wheeltec::FeedbackFrame feedbackFrame(std::uint8_t flag_stop,
                                      std::int16_t forward = 0,
                                      std::int16_t yaw = 0) {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = flag_stop;
  putSignedBigEndian(forward, &frame[2U], &frame[3U]);
  putSignedBigEndian(yaw, &frame[6U], &frame[7U]);
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

struct ReadOutcome {
  wheeltec::TransportStatus status{
      wheeltec::TransportStatus::kDeadlineExceeded};
  std::vector<std::uint8_t> bytes;
  std::int64_t elapsed_ns{kMillisecondNs};
};

struct WriteOutcome {
  wheeltec::TransportStatus status{wheeltec::TransportStatus::kOk};
  std::size_t transferred{wheeltec::kCommandFrameSize};
  bool delivery_unconfirmed{false};
  std::int64_t elapsed_ns{kMillisecondNs};
  bool override_completion_clock{false};
  std::int64_t completion_clock_ns{0};
};

enum class IoKind : std::uint8_t { kWrite = 0, kRead };

class ScriptedTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedTransport(std::int64_t* monotonic_clock)
      : monotonic_clock_(monotonic_clock) {}

  bool isConnected() const override { return connected_; }

  std::uint64_t connectionGeneration() const override {
    return generation_;
  }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t) override {
    io_order.push_back(IoKind::kWrite);
    writes.emplace_back(data, data + size);
    WriteOutcome outcome;
    if (!write_outcomes.empty()) {
      outcome = write_outcomes.front();
      write_outcomes.pop_front();
    } else {
      outcome.transferred = size;
    }
    reported_write_transfers.push_back(outcome.transferred);
    if (outcome.override_completion_clock && monotonic_clock_ != nullptr) {
      *monotonic_clock_ = outcome.completion_clock_ns;
    } else {
      advanceClock(outcome.elapsed_ns);
    }
    if (outcome.status == wheeltec::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    return {outcome.status, outcome.transferred, 0,
            outcome.delivery_unconfirmed};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t) override {
    io_order.push_back(IoKind::kRead);
    ReadOutcome outcome;
    if (!read_outcomes.empty()) {
      outcome = read_outcomes.front();
      read_outcomes.pop_front();
    }
    advanceClock(outcome.elapsed_ns);
    read_completion_times_ns.push_back(
        monotonic_clock_ == nullptr ? 0 : *monotonic_clock_);
    if (outcome.status == wheeltec::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    const std::size_t transferred =
        std::min(capacity, outcome.bytes.size());
    if (transferred > 0U && data != nullptr) {
      std::memcpy(data, outcome.bytes.data(), transferred);
    }
    return {outcome.status, transferred, 0,
            outcome.status == wheeltec::TransportStatus::kDisconnected};
  }

  void enqueueFeedback(std::uint8_t flag_stop) {
    const auto frame = feedbackFrame(flag_stop);
    read_outcomes.push_back(
        {wheeltec::TransportStatus::kOk,
         std::vector<std::uint8_t>(frame.begin(), frame.end()),
         kMillisecondNs});
  }

  void enqueueIdle() {
    read_outcomes.push_back(
        {wheeltec::TransportStatus::kDeadlineExceeded, {}, kMillisecondNs});
  }

  void enqueueReadDisconnect() {
    read_outcomes.push_back(
        {wheeltec::TransportStatus::kDisconnected, {}, kMillisecondNs});
  }

  void enqueuePartialWrite(std::size_t transferred) {
    write_outcomes.push_back(
        {wheeltec::TransportStatus::kDeadlineExceeded, transferred, true,
         kMillisecondNs});
  }

  void enqueuePartialWriteWithCompletionClock(
      std::size_t transferred, std::int64_t completion_clock_ns) {
    write_outcomes.push_back(
        {wheeltec::TransportStatus::kDeadlineExceeded, transferred, true,
         0, true, completion_clock_ns});
  }

  std::vector<IoKind> io_order;
  std::vector<std::vector<std::uint8_t>> writes;
  std::vector<std::size_t> reported_write_transfers;
  std::vector<std::int64_t> read_completion_times_ns;

 private:
  void advanceClock(std::int64_t elapsed_ns) {
    if (monotonic_clock_ != nullptr) {
      *monotonic_clock_ += elapsed_ns;
    }
  }

  std::int64_t* monotonic_clock_{nullptr};
  bool connected_{true};
  std::uint64_t generation_{1U};
  std::deque<ReadOutcome> read_outcomes;
  std::deque<WriteOutcome> write_outcomes;
};

auto_rover::VehicleProfile vehicleProfile() {
  auto_rover::VehicleProfile profile;
  profile.schema_version = 1U;
  profile.profile_id = "nuc_senior_akm_v1";
  profile.kinematic_model =
      auto_rover::KinematicModel::kAckermannBicycle;
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

auto_rover_vehicle::VehicleExecutionCommonConfig commonConfig() {
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
  first.direction = auto_rover::Direction::kForward;
  auto_rover::TrajectoryPoint second = first;
  second.x_m = 1.0;
  second.arc_length_m = 1.0;
  value.points = {first, second};
  return value;
}

auto_rover::MotionReference motionReference(std::uint64_t command_id,
                                             std::int64_t stamp_ns,
                                             double speed_mps) {
  auto_rover::MotionReference value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "rear_axle_center";
  value.command_id = command_id;
  value.producer_generation_id = "formal-integration-tracker";
  value.trajectory_id = "formal-route:1:1";
  value.direction = auto_rover::Direction::kForward;
  value.target_speed_mps = speed_mps;
  value.target_curvature_inv_m = 0.25;
  value.valid_for_ns = 2000000000LL;
  value.valid = true;
  return value;
}

class FormalBackendHarness {
 public:
  FormalBackendHarness()
      : transport(&monotonic_clock_ns) {
    wheeltec::WheeltecVehicleBackendConfig backend_config;
    backend_config.runtime = runtimeConfig();
    backend_config.chassis_frame_id = "rear_axle_center";
    backend_config.source_id =
        "wheeltec_serial_unverified#process=formal-integration";
    wheeltec::WheeltecVehicleBackendOperations operations;
    operations.runtime.monotonic_now_ns = [this]() {
      return monotonic_clock_ns;
    };
    operations.runtime.wait_until_monotonic_ns =
        [this](std::int64_t deadline_ns) {
          if (deadline_ns <= 0 || deadline_ns < monotonic_clock_ns) {
            return false;
          }
          monotonic_clock_ns = deadline_ns;
          return true;
        };
    operations.ros_now_ns = [this]() { return ros_clock_ns; };
    auto* concrete = new wheeltec::WheeltecVehicleBackend(
        backend_config, &transport, operations);
    backend = concrete;
    std::unique_ptr<auto_rover_vehicle::VehicleBackend> owned(concrete);
    core.reset(new auto_rover_vehicle::VehicleExecutionCore(
        commonConfig(), std::move(owned)));
  }

  bool initialize() {
    const std::int64_t entry_monotonic_ns = monotonic_clock_ns;
    const bool initialized =
        core->initialize(entry_monotonic_ns, ros_clock_ns);
    expect(initialized, "formal Wheeltec backend initializes through the core");
    expect(transport.io_order.size() == 1U &&
               transport.io_order.front() == IoKind::kWrite &&
               transport.writes.size() == 1U &&
               isExactZeroCommand(transport.writes.front()),
           "startup exact-zero is the first and only initial transport I/O");
    return initialized;
  }

  void installRequiredInputs() {
    monotonic_clock_ns += kMillisecondNs;
    ros_clock_ns += kMillisecondNs;
    const auto ego = egoState(ros_clock_ns);
    const auto path = trajectory(ros_clock_ns);
    const auto motion = motionReference(1U, ros_clock_ns, 0.0);
    expect(core->updateEgoState(ego, monotonic_clock_ns).ok,
           "formal fixture accepts localization input");
    expect(core->updateTrajectory(path, monotonic_clock_ns).ok,
           "formal fixture accepts trajectory input");
    core->updateMotionReference(motion, monotonic_clock_ns);
  }

  auto_rover_vehicle::VehicleExecutionCycleResult drainBacklog() {
    transport.enqueueIdle();
    advanceCycleTime();
    return core->cycle(monotonic_clock_ns, ros_clock_ns);
  }

  auto_rover_vehicle::VehicleExecutionCycleResult cycleFeedback(
      std::uint8_t flag_stop) {
    transport.enqueueFeedback(flag_stop);
    advanceCycleTime();
    return core->cycle(monotonic_clock_ns, ros_clock_ns);
  }

  auto_rover_vehicle::VehicleExecutionCycleResult cycleDisconnect() {
    transport.enqueueReadDisconnect();
    advanceCycleTime();
    return core->cycle(monotonic_clock_ns, ros_clock_ns);
  }

  void updateMotion(std::uint64_t command_id, double speed_mps) {
    monotonic_clock_ns += kMillisecondNs;
    ros_clock_ns += kMillisecondNs;
    core->updateMotionReference(
        motionReference(command_id, ros_clock_ns, speed_mps),
        monotonic_clock_ns);
  }

  auto_rover_vehicle::VehicleExecutionServiceResult arm() {
    monotonic_clock_ns += kMillisecondNs;
    ros_clock_ns += kMillisecondNs;
    const auto state = core->currentSafetyState(ros_clock_ns);
    return core->requestArm("formal-integration-operator", state.state_id,
                            true, monotonic_clock_ns, ros_clock_ns);
  }

  void advanceCycleTime() {
    monotonic_clock_ns += kCyclePeriodNs;
    ros_clock_ns += kCyclePeriodNs;
  }

  std::int64_t monotonic_clock_ns{1000000000LL};
  std::int64_t ros_clock_ns{2000000000LL};
  ScriptedTransport transport;
  wheeltec::WheeltecVehicleBackend* backend{nullptr};
  std::unique_ptr<auto_rover_vehicle::VehicleExecutionCore> core;
};

auto_rover_vehicle::VehicleExecutionCycleResult recoverBackend(
    FormalBackendHarness* harness) {
  const auto drained = harness->drainBacklog();
  expect(!drained.health_clear &&
             drained.stop_delivery.stop_attempted &&
             drained.stop_delivery.delivered &&
             !drained.stop_delivery.delivery_unconfirmed,
         "the disarmed core uses a confirmed nonterminal stop while draining");
  harness->installRequiredInputs();

  auto_rover_vehicle::VehicleExecutionCycleResult recovered;
  const std::size_t reads_before_recovery =
      harness->transport.read_completion_times_ns.size();
  std::uint64_t first_state_id = 0U;
  for (std::size_t index = 0U; index < 5U; ++index) {
    recovered = harness->cycleFeedback(0U);
    if (index == 0U) {
      first_state_id = recovered.chassis_available
                           ? recovered.chassis.state_id
                           : 0U;
    }
    expect(!harness->core->executionAuthorized(),
           "feedback recovery never creates execution authorization");
    expect(recovered.stop_delivery.stop_attempted &&
               recovered.stop_delivery.delivered &&
               !recovered.stop_delivery.delivery_unconfirmed,
           "each disarmed cycle confirms its nonterminal zero delivery");
    expect(harness->backend->health().connected,
           "repeated disarmed revocation does not terminate the serial session");
    expect(!contains(harness->backend->health().diagnostic, "clock"),
           "poll I/O followed by same-cycle revoke/deliver does not roll back the runtime clock");
  }
  const auto& read_times = harness->transport.read_completion_times_ns;
  expect(read_times.size() == reads_before_recovery + 5U &&
             read_times.back() - read_times[reads_before_recovery] >=
                 200000000LL &&
             first_state_id > 0U && recovered.chassis_available &&
             recovered.chassis.state_id == first_state_id + 4U,
         "five distinct feedback receipts span at least 0.20 seconds");
  expect(recovered.health_clear &&
             recovered.safety.mode == auto_rover::SafetyMode::kDisarmed &&
             harness->backend->health().actuation_permitted,
         "FlagStop=0 recovery reaches clear backend health while the core remains disarmed");
  return recovered;
}

auto_rover_vehicle::VehicleExecutionCycleResult armAndReachMotion(
    FormalBackendHarness* harness) {
  const auto armed = harness->arm();
  expect(armed.success && harness->core->executionAuthorized() &&
             harness->backend->health().authorization_active,
         "an explicit healthy arm creates a new backend authorization epoch");

  auto_rover_vehicle::VehicleExecutionCycleResult moving;
  for (std::uint64_t command_id = 2U; command_id <= 4U; ++command_id) {
    harness->updateMotion(command_id, 0.25);
    moving = harness->cycleFeedback(0U);
    expect(!moving.delivery.delivery_unconfirmed,
           "fresh command recovery never reports an ambiguous delivery");
  }
  expect(moving.command.motion_enabled &&
             moving.command.signed_speed_mps > 0.0 &&
             moving.delivery.delivered && moving.delivery.motion_accepted &&
             !harness->transport.writes.empty() &&
             isNonzeroCommand(harness->transport.writes.back()),
         "the third fresh guarded command crosses both recovery barriers and writes nonzero motion");
  return moving;
}

void prepareMovingHarness(FormalBackendHarness* harness) {
  expect(harness->initialize(), "moving harness initializes");
  recoverBackend(harness);
  armAndReachMotion(harness);
}

void testFormalLifecycleFlagStopAndNoAutomaticRecovery() {
  FormalBackendHarness harness;
  prepareMovingHarness(&harness);

  const std::size_t writes_before_inhibit = harness.transport.writes.size();
  const auto inhibited = harness.cycleFeedback(1U);
  expect(!inhibited.health_clear && !harness.core->executionAuthorized() &&
             !harness.backend->health().authorization_active &&
             inhibited.stop_delivery.stop_attempted &&
             inhibited.stop_delivery.delivered,
         "FlagStop=1 immediately revokes authorization and confirms a zero stop");
  expect(harness.transport.writes.size() > writes_before_inhibit,
         "FlagStop=1 produces at least one same-cycle stop write");
  for (std::size_t index = writes_before_inhibit;
       index < harness.transport.writes.size(); ++index) {
    expect(isExactZeroCommand(harness.transport.writes[index]),
           "no nonzero frame follows a FlagStop=1 receipt");
  }

  for (std::size_t index = 0U; index < 5U; ++index) {
    const auto recovering = harness.cycleFeedback(0U);
    expect(!harness.core->executionAuthorized() &&
               !harness.backend->health().authorization_active &&
               recovering.safety.mode != auto_rover::SafetyMode::kArmed,
           "allowed feedback can recover health but never restores the retired arm epoch");
  }
  expect(harness.backend->health().actuation_permitted &&
             !harness.core->executionAuthorized(),
         "local feedback recovery remains disarmed until another explicit arm");
}

void testDisconnectFailsClosed() {
  FormalBackendHarness harness;
  prepareMovingHarness(&harness);
  const std::size_t writes_before_disconnect = harness.transport.writes.size();
  const auto disconnected = harness.cycleDisconnect();
  expect(!disconnected.health_clear &&
             disconnected.health_reason ==
                 auto_rover::StopReason::kBackendDisconnected &&
             !harness.core->executionAuthorized() &&
             !harness.backend->health().connected &&
             disconnected.stop_delivery.delivery_unconfirmed,
         "a read disconnect is terminal for the generation and fails closed");
  expect(harness.transport.writes.size() == writes_before_disconnect,
         "a disconnected transport cannot fabricate a successful appended zero");
}

void testPartialWritePoisonsGenerationWithoutAppendingZero() {
  FormalBackendHarness harness;
  prepareMovingHarness(&harness);
  harness.transport.enqueueFeedback(0U);
  harness.transport.enqueuePartialWrite(3U);
  harness.advanceCycleTime();
  const std::size_t writes_before_partial = harness.transport.writes.size();
  const auto partial = harness.core->cycle(harness.monotonic_clock_ns,
                                           harness.ros_clock_ns);
  expect(!partial.health_clear && !harness.core->executionAuthorized() &&
             harness.backend->health().delivery_unconfirmed,
         "a partial motion write poisons the generation and revokes execution");
  expect(harness.transport.writes.size() == writes_before_partial + 1U &&
             harness.transport.reported_write_transfers.back() == 3U,
         "no zero frame is appended after an unknown partial frame prefix");
  expect(!partial.stop_delivery.delivered &&
             partial.stop_delivery.delivery_unconfirmed,
         "the core exposes that a physical stop could not be confirmed after the partial write");
}

void testPartialWritePoisonPrecedesCompletionClockValidation() {
  FormalBackendHarness harness;
  prepareMovingHarness(&harness);
  harness.transport.enqueueFeedback(0U);
  harness.advanceCycleTime();
  const std::int64_t cycle_entry_ns = harness.monotonic_clock_ns;
  harness.transport.enqueuePartialWriteWithCompletionClock(3U,
                                                            cycle_entry_ns);
  const std::size_t writes_before_partial = harness.transport.writes.size();
  const auto partial = harness.core->cycle(harness.monotonic_clock_ns,
                                           harness.ros_clock_ns);
  expect(!partial.health_clear && !harness.core->executionAuthorized() &&
             harness.backend->health().delivery_unconfirmed &&
             harness.transport.writes.size() == writes_before_partial + 1U,
         "partial delivery remains poisoned when its completion clock rolls back");

  // A trustworthy local clock may return later, but it cannot make a second
  // frame safe to append after an unknown prefix on this generation.
  harness.monotonic_clock_ns = cycle_entry_ns + 2 * kCyclePeriodNs;
  const std::size_t writes_before_shutdown = harness.transport.writes.size();
  const auto shutdown = harness.core->shutdown(harness.monotonic_clock_ns);
  expect(shutdown.delivery_unconfirmed && !shutdown.delivered &&
             harness.transport.writes.size() == writes_before_shutdown,
         "clock recovery and core shutdown never append zero after a poisoned partial prefix");
}

}  // namespace

int main() {
  testFormalLifecycleFlagStopAndNoAutomaticRecovery();
  testDisconnectFailsClosed();
  testPartialWritePoisonsGenerationWithoutAppendingZero();
  testPartialWritePoisonPrecedesCompletionClockValidation();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "formal Wheeltec backend integration tests passed\n";
  return EXIT_SUCCESS;
}
