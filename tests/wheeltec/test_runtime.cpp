#include "auto_rover_vcu_wheeltec_serial/runtime.hpp"
#include "auto_rover_vcu_wheeltec_serial/vehicle_backend.hpp"
#include "auto_rover_vehicle/vehicle_execution_core.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <string>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace wheeltec = auto_rover::wheeltec_serial;

namespace {

int g_failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", message.c_str());
  }
}

void expectNear(double actual, double expected, double tolerance,
                const std::string& message) {
  expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
         message);
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

wheeltec::FeedbackFrame makeFeedback(std::int16_t forward,
                                     std::int16_t yaw,
                                     std::uint8_t flag_stop = 0U,
                                     bool checksum_valid = true) {
  wheeltec::FeedbackFrame frame{};
  frame[0U] = wheeltec::kFrameHeader;
  frame[1U] = flag_stop;
  putSignedBigEndian(forward, &frame[2U], &frame[3U]);
  putSignedBigEndian(0, &frame[4U], &frame[5U]);
  putSignedBigEndian(yaw, &frame[6U], &frame[7U]);
  putSignedBigEndian(24000, &frame[20U], &frame[21U]);
  frame[22U] = xorBytes(frame.data(), 22U);
  if (!checksum_valid) {
    frame[22U] ^= 0x80U;
  }
  frame[23U] = wheeltec::kFrameTail;
  return frame;
}

bool isExactZero(const std::vector<std::uint8_t>& bytes) {
  return bytes.size() == wheeltec::kCommandFrameSize &&
         bytes[0U] == wheeltec::kFrameHeader && bytes[1U] == 0U &&
         bytes[2U] == 0U && bytes[3U] == 0U && bytes[4U] == 0U &&
         bytes[5U] == 0U && bytes[6U] == 0U && bytes[7U] == 0U &&
         bytes[8U] == 0U && bytes[10U] == wheeltec::kFrameTail;
}

enum class IoKind : std::uint8_t { kWrite = 0, kRead };

struct ScriptedRead {
  wheeltec::TransportStatus status{wheeltec::TransportStatus::kDeadlineExceeded};
  std::vector<std::uint8_t> bytes;
  std::int64_t elapsed_ns{1};
};

struct ScriptedWrite {
  wheeltec::TransportStatus status{wheeltec::TransportStatus::kOk};
  std::size_t transferred{wheeltec::kCommandFrameSize};
  bool delivery_unconfirmed{false};
  std::int64_t elapsed_ns{1};
};

class ScriptedTransport final : public wheeltec::ByteTransport {
 public:
  explicit ScriptedTransport(std::int64_t* clock, std::uint64_t generation = 1U)
      : clock_(clock), generation_(generation) {}

  bool isConnected() const override { return connected_; }
  std::uint64_t connectionGeneration() const override { return generation_; }

  wheeltec::IoResult writeAll(const std::uint8_t* data, std::size_t size,
                              std::int64_t deadline_ns) override {
    io_order.push_back(IoKind::kWrite);
    write_deadlines.push_back(deadline_ns);
    if (throw_next_write) {
      throw_next_write = false;
      throw std::runtime_error("injected unknown write exception");
    }
    writes.emplace_back(data, data + size);
    ScriptedWrite outcome;
    if (!write_script.empty()) {
      outcome = write_script.front();
      write_script.pop_front();
    } else {
      outcome.transferred = size;
    }
    if (clock_ != nullptr) {
      *clock_ += outcome.elapsed_ns;
    }
    if (outcome.status == wheeltec::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    return {outcome.status, outcome.transferred, 0,
            outcome.delivery_unconfirmed};
  }

  wheeltec::IoResult readSome(std::uint8_t* data, std::size_t capacity,
                              std::int64_t deadline_ns) override {
    io_order.push_back(IoKind::kRead);
    read_deadlines.push_back(deadline_ns);
    ScriptedRead outcome;
    if (!read_script.empty()) {
      outcome = read_script.front();
      read_script.pop_front();
    }
    if (clock_ != nullptr) {
      *clock_ += outcome.elapsed_ns;
    }
    if (outcome.status == wheeltec::TransportStatus::kDisconnected) {
      connected_ = false;
    }
    const std::size_t transferred =
        std::min(capacity, outcome.bytes.size());
    if (transferred > 0U && data != nullptr) {
      std::memcpy(data, outcome.bytes.data(), transferred);
    }
    return {outcome.status, transferred,
            outcome.status == wheeltec::TransportStatus::kDeadlineExceeded
                ? ETIMEDOUT
                : 0,
            outcome.status == wheeltec::TransportStatus::kDisconnected};
  }

  void enqueueFrame(const wheeltec::FeedbackFrame& frame,
                    std::int64_t elapsed_ns = 10) {
    read_script.push_back({wheeltec::TransportStatus::kOk,
                           std::vector<std::uint8_t>(frame.begin(), frame.end()),
                           elapsed_ns});
  }

  void enqueueBytes(const std::vector<std::uint8_t>& bytes,
                    std::int64_t elapsed_ns = 10) {
    read_script.push_back(
        {wheeltec::TransportStatus::kOk, bytes, elapsed_ns});
  }

  void enqueueIdle(std::int64_t elapsed_ns = 1) {
    read_script.push_back(
        {wheeltec::TransportStatus::kDeadlineExceeded, {}, elapsed_ns});
  }

  void enqueueDisconnect() {
    read_script.push_back(
        {wheeltec::TransportStatus::kDisconnected, {}, 1});
  }

  std::int64_t* clock_{nullptr};
  bool connected_{true};
  std::uint64_t generation_{1U};
  bool throw_next_write{false};
  std::deque<ScriptedRead> read_script;
  std::deque<ScriptedWrite> write_script;
  std::vector<IoKind> io_order;
  std::vector<std::vector<std::uint8_t>> writes;
  std::vector<std::int64_t> write_deadlines;
  std::vector<std::int64_t> read_deadlines;
};

wheeltec::RuntimeConfig enabledConfig() {
  wheeltec::RuntimeConfig config;
  config.adapter.codec_limits.max_forward_speed_mps = 0.50;
  config.adapter.max_command_age_ns = 100;
  config.adapter.fresh_commands_required = 3U;
  config.adapter.zero_retry_interval_ns = 5;
  config.adapter.max_zero_write_attempts = 3U;
  config.adapter.write_timeout_ns = 5;
  config.read_timeout_ns = 1;
  config.maximum_feedback_age_ns = 150;
  config.drain_read_timeout_ns = 1;
  config.maximum_drain_reads = 64U;
  config.control_allowed_receipts_required = 3U;
  config.control_allowed_minimum_span_ns = 20;
  config.maximum_normal_write_gap_ns = 100;
  config.maximum_step_duration_ns = 50;
  config.runtime_enabled = true;
  config.unverified_protocol_acknowledged = true;
  config.physical_device_opt_in = true;
  config.actuation_opt_in = true;
  config.readiness_gate_passed = true;
  config.external_or_durable_estop_strategy_approved = true;
  return config;
}

wheeltec::RuntimeOperations clockOperations(std::int64_t* clock) {
  wheeltec::RuntimeOperations operations;
  operations.monotonic_now_ns = [clock]() { return *clock; };
  operations.wait_until_monotonic_ns = [clock](std::int64_t deadline_ns) {
    if (deadline_ns <= 0 || deadline_ns < *clock) {
      return false;
    }
    *clock = deadline_ns;
    return true;
  };
  return operations;
}

void finishStartupAndDrain(wheeltec::WheeltecSerialRuntime* runtime,
                           ScriptedTransport* transport,
                           std::int64_t* now) {
  expect(runtime->attachTransport(transport, *now),
         "enabled runtime attaches a connected new generation");
  wheeltec::RuntimeStepResult step = runtime->step(*now);
  expect(step.phase == wheeltec::RuntimePhase::kDrainBacklog &&
             step.cycle.action == wheeltec::AdapterCycleAction::kZeroWritten,
         "startup exact-zero succeeds before entering drain");
  expect(transport->io_order.size() == 1U &&
             transport->io_order.front() == IoKind::kWrite &&
             isExactZero(transport->writes.front()),
         "the first transport I/O is a complete exact-zero write");
  transport->enqueueIdle();
  step = runtime->step(*now);
  expect(step.phase == wheeltec::RuntimePhase::kAwaitingFeedback,
         "an empty bounded drain isolates the opening backlog");
}

void recoverAllowedFeedback(wheeltec::WheeltecSerialRuntime* runtime,
                            ScriptedTransport* transport,
                            std::int64_t* now) {
  for (std::int16_t index = 0; index < 3; ++index) {
    *now += 10;
    transport->enqueueFrame(makeFeedback(
        static_cast<std::int16_t>(100 + index),
        static_cast<std::int16_t>(10 + index)), 10);
    const wheeltec::RuntimeStepResult step = runtime->step(*now);
    expect(step.feedback_available,
           "each complete allowed feedback read produces one receipt sample");
  }
  expect(runtime->health(*now).actuation_permitted &&
             runtime->phase() == wheeltec::RuntimePhase::kReady,
         "multiple distinct allowed receipts spanning the configured interval complete recovery");
}

auto_rover::VehicleExecutionCommand command(std::uint64_t sequence,
                                             std::int64_t now,
                                             double speed = 0.25) {
  auto_rover::VehicleExecutionCommand value;
  value.sequence_id = sequence;
  value.created_monotonic_ns = now;
  value.deadline_monotonic_ns = now + 100;
  value.signed_speed_mps = speed;
  value.curvature_inv_m = 0.5;
  value.motion_enabled = true;
  value.hold = false;
  value.stop_reason = auto_rover::StopReason::kNone;
  return value;
}

auto_rover::VehicleProfile executionProfile() {
  auto_rover::VehicleProfile value;
  value.schema_version = 1U;
  value.profile_id = "nuc_senior_akm_v1";
  value.kinematic_model = auto_rover::KinematicModel::kAckermannBicycle;
  value.direction_capability =
      auto_rover::DirectionCapability::kSignedSpeedDirection;
  value.reference_frame = "rear_axle_center";
  value.wheelbase_m = 0.3187;
  value.max_forward_speed_mps = 0.50;
  value.max_longitudinal_accel_mps2 = 0.20;
  value.min_turning_radius_m = 0.95;
  value.reverse_supported = false;
  return value;
}

auto_rover_vehicle::VehicleExecutionCommonConfig executionConfig() {
  auto_rover_vehicle::VehicleExecutionCommonConfig value;
  value.vehicle_profile = executionProfile();
  value.safety_supervisor.actuation_enabled = true;
  value.safety_supervisor.reset_service_enabled = true;
  value.safety_supervisor.fresh_recovery_count = 1U;
  value.safety_supervisor.state_valid_for_ns = 1000000000LL;
  value.authorized_reset_operator_id = "reset-operator";
  value.command_guard.world_frame = "camera_init";
  value.command_guard.localization_freshness_ns = 500000000LL;
  value.command_guard.trajectory_freshness_ns = 500000000LL;
  value.command_guard.motion_freshness_ns = 500000000LL;
  value.command_guard.chassis_freshness_ns = 500000000LL;
  value.command_guard.safety_freshness_ns = 500000000LL;
  value.command_guard.fresh_recovery_count = 2U;
  value.vehicle_manager.command_valid_for_ns = 100000000LL;
  return value;
}

auto_rover::EgoState executionEgo(std::int64_t stamp_ns) {
  auto_rover::EgoState value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "camera_init";
  value.state_id = 1U;
  value.time_source = auto_rover::TimeSource::kPublishTime;
  value.reference_frame = "rear_axle_center";
  value.pose.orientation.w = 1.0;
  value.source_id = "wheeltec-runtime-integration-localization-generation-1";
  value.valid = true;
  return value;
}

auto_rover::Trajectory executionTrajectory(std::int64_t stamp_ns) {
  auto_rover::Trajectory value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "camera_init";
  value.trajectory_id = "wheeltec-runtime-route:1:1";
  value.route_id = "wheeltec-runtime-route";
  value.plan_version = 1U;
  value.vehicle_profile_id = "nuc_senior_akm_v1";
  value.valid_for_ns = 1000000000LL;
  value.completion_behavior = auto_rover::CompletionBehavior::kStopAndHold;
  value.valid = true;
  auto_rover::TrajectoryPoint first;
  first.direction = auto_rover::Direction::kForward;
  first.target_speed_mps = 0.25;
  auto_rover::TrajectoryPoint second = first;
  second.x_m = 1.0;
  second.arc_length_m = 1.0;
  value.points = {first, second};
  return value;
}

auto_rover::MotionReference executionMotion(std::uint64_t command_id,
                                             std::int64_t stamp_ns,
                                             double speed_mps) {
  auto_rover::MotionReference value;
  value.stamp_ns = stamp_ns;
  value.frame_id = "rear_axle_center";
  value.command_id = command_id;
  value.producer_generation_id =
      "wheeltec-runtime-integration-tracker-generation-1";
  value.trajectory_id = "wheeltec-runtime-route:1:1";
  value.direction = auto_rover::Direction::kForward;
  value.target_speed_mps = speed_mps;
  value.target_curvature_inv_m = 0.0;
  value.valid_for_ns = 1000000000LL;
  value.valid = true;
  return value;
}

void testDisabledRuntimeTouchesNoTransport() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::RuntimeConfig config = enabledConfig();
  config.runtime_enabled = false;
  wheeltec::WheeltecSerialRuntime runtime(config, clockOperations(&now));
  expect(runtime.configurationValid(),
         "a structurally valid disabled runtime is a valid inhibited configuration");
  expect(runtime.attachTransport(&transport, now),
         "disabled runtime initialization does not require or mount its supplied transport");
  const wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kDisabled &&
             transport.io_order.empty(),
         "disabled runtime performs zero transport I/O");
  expect(!runtime.setAuthorization(true, 1U, now),
         "disabled runtime rejects authorization");
  const wheeltec::RuntimeDelivery delivery =
      runtime.deliver(command(1U, now), now);
  expect(!delivery.delivered && transport.io_order.empty(),
         "disabled runtime rejects motion without touching transport");
  const wheeltec::RuntimeDelivery stopped = runtime.revokeAndStop(now);
  expect(stopped.stop_attempted && stopped.delivered &&
             !stopped.delivery_unconfirmed &&
             !stopped.controller_ack_available &&
             transport.io_order.empty(),
         "disabled runtime confirms its local inhibit boundary without transport I/O or a controller ACK claim");
}

void testStartupDrainLatestReceiptAndRecovery() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now), "runtime attaches");

  const wheeltec::FeedbackFrame backlog = makeFeedback(900, 90);
  transport.enqueueFrame(backlog);
  wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kDrainBacklog &&
             !step.feedback_available && transport.io_order.size() == 1U,
         "startup step writes only zero and cannot read queued feedback");
  step = runtime.step(now);
  expect(!step.feedback_available &&
             runtime.health(now).last_feedback_receipt_monotonic_ns == 0,
         "drained backlog never refreshes feedback freshness");
  transport.enqueueIdle();
  step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kAwaitingFeedback,
         "bounded drain ends on its first empty read");

  const wheeltec::FeedbackFrame first = makeFeedback(-123, -25);
  const wheeltec::FeedbackFrame latest = makeFeedback(456, 75);
  std::vector<std::uint8_t> adjacent(first.begin(), first.end());
  adjacent.insert(adjacent.end(), latest.begin(), latest.end());
  now += 10;
  transport.enqueueBytes(adjacent, 10);
  step = runtime.step(now);
  expect(step.feedback_available && step.valid_frames_in_read == 2U &&
             step.feedback_receipts_in_read == 1U,
         "multiple frames from one read form exactly one latest-value receipt");
  expectNear(step.feedback.signed_speed_mps, 0.456, 1e-12,
             "latest signed feedback speed wins within one read");
  expectNear(step.feedback.wheel_derived_yaw_rate_radps, 0.075, 1e-12,
             "latest wheel-derived yaw wins within one read");
  expectNear(step.feedback.supply_voltage_v, 24.0, 1e-12,
             "feedback voltage is normalized");
  expect(!step.feedback.gear_available &&
             !step.feedback.control_state_available &&
             !step.feedback.fault_state_available &&
             !step.feedback.controller_ack_available &&
             !step.feedback.source_time_available,
         "unavailable VCU semantics remain explicitly unavailable");
  expect(!runtime.health(now).actuation_permitted,
         "one FlagStop=0 read cannot complete feedback recovery");

  for (int index = 0; index < 2; ++index) {
    now += 10;
    transport.enqueueFrame(makeFeedback(500 + index, 80 + index), 10);
    step = runtime.step(now);
  }
  expect(runtime.phase() == wheeltec::RuntimePhase::kReady &&
             runtime.health(now).actuation_permitted,
         "three distinct receipts over the minimum span permit actuation locally");
}

void testBadPartialFlagStopAndStaleDoNotLeakMotion() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  const std::int64_t last_good =
      runtime.health(now).last_feedback_receipt_monotonic_ns;

  now += 10;
  const wheeltec::FeedbackFrame partial = makeFeedback(200, 20);
  transport.enqueueBytes(std::vector<std::uint8_t>(partial.begin(),
                                                   partial.begin() + 7),
                         10);
  wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(!step.feedback_available &&
             runtime.health(now).last_feedback_receipt_monotonic_ns ==
                 last_good,
         "a partial frame does not refresh feedback freshness");

  now += 10;
  transport.enqueueFrame(makeFeedback(200, 20, 0U, false), 10);
  step = runtime.step(now);
  expect(!step.feedback_available &&
             runtime.health(now).last_feedback_receipt_monotonic_ns ==
                 last_good,
         "a bad checksum does not refresh feedback freshness");

  now += 10;
  const wheeltec::FeedbackFrame inhibited = makeFeedback(0, 0, 1U);
  const wheeltec::FeedbackFrame allowed = makeFeedback(300, 30, 0U);
  std::vector<std::uint8_t> mixed(inhibited.begin(), inhibited.end());
  mixed.insert(mixed.end(), allowed.begin(), allowed.end());
  transport.enqueueBytes(mixed, 10);
  step = runtime.step(now);
  expect(step.feedback_available && step.inhibit_observed_in_read &&
             !runtime.health(now).actuation_permitted &&
             !runtime.health(now).authorization_active,
         "any FlagStop=1 in a read inhibits the whole receipt even when its latest frame is zero");

  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "fresh feedback recovery permits a new authorization epoch");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    now += 1;
    const std::size_t writes_before_command = transport.writes.size();
    const wheeltec::RuntimeDelivery delivered =
        runtime.deliver(command(sequence, now), now);
    if (sequence < 3U) {
      expect(delivered.delivered && !delivered.motion_accepted &&
                 transport.writes.size() == writes_before_command + 1U &&
                 isExactZero(transport.writes.back()),
             "each recovery-pending command refreshes exact zero without writing motion early");
    } else {
      expect(delivered.delivered && delivered.motion_accepted,
             "third fresh command crosses the adapter recovery gate");
    }
  }

  const std::size_t writes_before_stale = transport.writes.size();
  now = runtime.health(now).last_feedback_receipt_monotonic_ns + 151;
  transport.enqueueIdle();
  step = runtime.step(now);
  expect(step.feedback_watchdog_expired &&
             !runtime.health(now).authorization_active &&
             !runtime.health(now).actuation_permitted &&
             transport.writes.size() == writes_before_stale + 1U &&
             isExactZero(transport.writes.back()),
         "feedback stale is checked before TX and emits zero instead of stale motion");
}

void testInvalidCompositeFlagStopInhibitsWholeRead() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "invalid FlagStop fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }

  const wheeltec::FeedbackFrame invalid = makeFeedback(100, 10, 2U);
  const wheeltec::FeedbackFrame allowed = makeFeedback(200, 20, 0U);
  std::vector<std::uint8_t> mixed(invalid.begin(), invalid.end());
  mixed.insert(mixed.end(), allowed.begin(), allowed.end());
  transport.enqueueBytes(mixed, 1);
  const std::size_t writes_before = transport.writes.size();
  const wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.invalid_composite_stop_observed_in_read &&
             step.inhibit_observed_in_read && step.feedback_available &&
             step.valid_frames_in_read == 1U &&
             !runtime.health(now).actuation_permitted &&
             !runtime.health(now).authorization_active &&
             transport.writes.size() == writes_before + 1U &&
             isExactZero(transport.writes.back()),
         "an unsupported composite FlagStop inhibits the entire read and emits zero even beside a valid allowed frame");
}

void testStartupPartialWriteBlocksRead() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 4U, true, 5});
  transport.enqueueFrame(makeFeedback(100, 0));
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now), "partial-write runtime attaches");
  const wheeltec::RuntimeStepResult step = runtime.step(now);
  const std::size_t writes_after_partial = transport.writes.size();
  const wheeltec::RuntimeDelivery revoke = runtime.revokeAndStop(now);
  const wheeltec::RuntimeDelivery shutdown = runtime.shutdown(now);
  expect(step.phase == wheeltec::RuntimePhase::kTerminalFault &&
             step.delivery_unconfirmed && transport.io_order.size() == 1U &&
             transport.io_order.front() == IoKind::kWrite &&
             revoke.delivery_unconfirmed && shutdown.delivery_unconfirmed &&
             transport.writes.size() == writes_after_partial,
         "a partial startup zero poisons the generation, so no read, revoke, or shutdown frame follows its unknown prefix");
}

void testPartialOverrunNeverAppendsZero() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 4U, true, 51});
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now),
         "partial-overrun runtime attaches");
  const wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kTerminalFault &&
             step.step_overrun && transport.writes.size() == 1U &&
             transport.io_order.size() == 1U,
         "a partial write that also overruns is terminal and never appends a zero frame to the unknown prefix");
}

void testPartialWritePoisonsBeforeCompletionClockValidation() {
  std::int64_t clock = 100;
  const std::int64_t caller_time = clock;
  ScriptedTransport transport(&clock);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 4U, true, -1});
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&clock));
  expect(runtime.attachTransport(&transport, caller_time),
         "partial plus invalid-completion fixture attaches");
  const wheeltec::RuntimeStepResult partial = runtime.step(caller_time);
  const std::size_t writes_after_partial = transport.writes.size();
  const wheeltec::RuntimeDelivery stopped = runtime.revokeAndStop(caller_time);
  const wheeltec::RuntimeDelivery shutdown = runtime.shutdown(caller_time);
  expect(partial.phase == wheeltec::RuntimePhase::kTerminalFault &&
             partial.delivery_unconfirmed && stopped.delivery_unconfirmed &&
             shutdown.delivery_unconfirmed &&
             transport.writes.size() == writes_after_partial,
         "partial bytes poison the generation before completion-clock validation, so later stop paths never append a frame");
}

void testThrowingWritePoisonsGeneration() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  transport.throw_next_write = true;
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now),
         "throwing-write fixture attaches");
  const wheeltec::RuntimeStepResult thrown = runtime.step(now);
  const std::size_t io_after_throw = transport.io_order.size();
  const wheeltec::RuntimeDelivery revoke = runtime.revokeAndStop(now);
  const wheeltec::RuntimeDelivery shutdown = runtime.shutdown(now);
  expect(thrown.phase == wheeltec::RuntimePhase::kTerminalFault &&
             thrown.delivery_unconfirmed && revoke.delivery_unconfirmed &&
             shutdown.delivery_unconfirmed &&
             transport.io_order.size() == io_after_throw,
         "a throwing transport is treated as unknown write progress and poisons the generation against every later zero append");
}

void testStartupZeroByteFailureRetriesWithoutRead() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1});
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize, false, 1});
  transport.enqueueFrame(makeFeedback(100, 0));
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now),
         "zero-byte startup retry runtime attaches");
  wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kStartupZero &&
             transport.io_order.size() == 1U &&
             transport.io_order.front() == IoKind::kWrite,
         "an explicitly zero-byte startup failure stays in zero-only retry state");
  now = 105;
  step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kDrainBacklog &&
             transport.io_order.size() == 2U &&
             transport.io_order[1U] == IoKind::kWrite &&
             isExactZero(transport.writes[0U]) &&
             isExactZero(transport.writes[1U]),
         "a later full startup zero succeeds and every pre-drain I/O was an exact-zero write");
}

void testStartupZeroByteFailureCanRecoverThroughOrdinaryStop() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1});
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  expect(runtime.attachTransport(&transport, now),
         "ordinary-stop startup recovery runtime attaches");
  const wheeltec::RuntimeStepResult failed = runtime.step(now);
  expect(failed.phase == wheeltec::RuntimePhase::kStartupZero &&
             failed.delivery_unconfirmed,
         "known zero-byte startup failure records temporary uncertainty");
  const wheeltec::RuntimeDelivery stopped = runtime.revokeAndStop(now);
  expect(stopped.stop_attempted && stopped.delivered &&
             runtime.phase() == wheeltec::RuntimePhase::kDrainBacklog &&
             !runtime.health(now).delivery_unconfirmed &&
             transport.io_order.size() == 2U &&
             transport.io_order[0U] == IoKind::kWrite &&
             transport.io_order[1U] == IoKind::kWrite,
         "a later full zero from the ordinary core stop path completes startup and clears zero-only uncertainty without reading");
}

void testRejectedCommandStopsInSameDelivery() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "rejection fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  expect(!transport.writes.empty() && !isExactZero(transport.writes.back()),
         "rejection fixture first completes a nonzero host write");
  const std::size_t writes_before_rejection = transport.writes.size();
  ++now;
  const wheeltec::RuntimeDelivery rejected =
      runtime.deliver(command(4U, now, 0.6), now);
  expect(rejected.submission.status ==
                 wheeltec::SubmissionStatus::kCodecRejected &&
             rejected.stop_attempted && rejected.delivered &&
             !rejected.motion_accepted &&
             transport.writes.size() == writes_before_rejection + 1U &&
             isExactZero(transport.writes.back()) &&
             !runtime.health(now).authorization_active,
         "a rejected command revokes and writes exact zero in the same deliver call");
}

void testAuthorizationRevocationStopsInSameCall() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "authorization-revoke fixture arms");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  const std::size_t before_disable = transport.writes.size();
  ++now;
  expect(runtime.setAuthorization(false, 0U, now) &&
             transport.writes.size() == before_disable + 1U &&
             isExactZero(transport.writes.back()) &&
             !runtime.health(now).authorization_active &&
             runtime.phase() == wheeltec::RuntimePhase::kReady,
         "explicit authorization disable writes bounded exact zero in the same call while preserving fresh readiness");

  expect(runtime.setAuthorization(true, 2U, now),
         "fixture can establish a newer authorization after explicit stop");
  for (std::uint64_t sequence = 4U; sequence <= 6U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  const std::size_t before_stale_id = transport.writes.size();
  ++now;
  expect(!runtime.setAuthorization(true, 2U, now) &&
             transport.writes.size() == before_stale_id + 1U &&
             isExactZero(transport.writes.back()) &&
             !runtime.health(now).authorization_active,
         "a stale authorization identity also revokes and writes exact zero in the same call");
}

void testReadCannotSplitNormalWriteGap() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now), "gap fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  const std::int64_t last_write =
      runtime.health(now).last_successful_write_completion_ns;
  const std::size_t before = transport.writes.size();
  now = last_write + 99;
  transport.enqueueFrame(makeFeedback(250, 25), 2);
  const wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.feedback_watchdog_expired &&
             transport.writes.size() == before + 1U &&
             isExactZero(transport.writes.back()) &&
             !runtime.health(now).authorization_active,
         "a read crossing the remaining 100 ms TX budget revokes before write and emits zero");
}

void testMotionWriteOverrunGetsImmediateZero() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "overrun fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize, false, 51});
  ++now;
  const std::size_t before = transport.writes.size();
  const wheeltec::RuntimeDelivery overrun =
      runtime.deliver(command(3U, now), now);
  expect(overrun.command_watchdog_expired && overrun.stop_attempted &&
             !overrun.motion_accepted && overrun.delivery_unconfirmed &&
             transport.writes.size() == before + 2U &&
             !isExactZero(transport.writes[before]) &&
             isExactZero(transport.writes.back()) &&
             runtime.phase() == wheeltec::RuntimePhase::kTerminalFault,
         "an overlong motion write is followed immediately by zero before terminal fault");
}

void testStopRetryAndNonterminalRevoke() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1});
  transport.write_script.push_back(
      {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize, false, 1});
  const std::size_t before = transport.writes.size();
  const wheeltec::RuntimeDelivery stopped = runtime.revokeAndStop(now);
  expect(stopped.stop_attempted && stopped.delivered &&
             transport.writes.size() == before + 2U &&
             isExactZero(transport.writes[before]) &&
             isExactZero(transport.writes.back()) &&
             runtime.phase() == wheeltec::RuntimePhase::kReady &&
             runtime.health(now).actuation_permitted &&
             !runtime.health(now).authorization_active,
         "ordinary revoke retries a known zero-byte failure and preserves recovered feedback readiness");

  expect(runtime.setAuthorization(true, 1U, now),
         "ordinary revoke does not prevent a later fresh authorization");
}

void testStopPartialAndCompletionClockRollbackFailClosed() {
  std::int64_t partial_now = 100;
  ScriptedTransport partial_transport(&partial_now);
  wheeltec::WheeltecSerialRuntime partial_runtime(
      enabledConfig(), clockOperations(&partial_now));
  finishStartupAndDrain(&partial_runtime, &partial_transport, &partial_now);
  recoverAllowedFeedback(&partial_runtime, &partial_transport, &partial_now);
  partial_transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 4U, true, 1});
  const std::size_t partial_before = partial_transport.writes.size();
  const wheeltec::RuntimeDelivery partial_stop =
      partial_runtime.revokeAndStop(partial_now);
  expect(partial_stop.stop_attempted && !partial_stop.delivered &&
             partial_stop.delivery_unconfirmed &&
             partial_transport.writes.size() == partial_before + 1U &&
             partial_runtime.phase() ==
                 wheeltec::RuntimePhase::kTerminalFault,
         "a partial stop write is terminal and never appends a retry frame");
  const std::size_t writes_after_partial = partial_transport.writes.size();
  const wheeltec::RuntimeDelivery repeated_stop =
      partial_runtime.revokeAndStop(partial_now);
  const wheeltec::RuntimeDelivery partial_shutdown =
      partial_runtime.shutdown(partial_now);
  expect(repeated_stop.delivery_unconfirmed &&
             partial_shutdown.delivery_unconfirmed &&
             partial_transport.writes.size() == writes_after_partial,
         "a poisoned stop stream never appends another frame until a new connection generation");

  std::int64_t motion_clock = 100;
  ScriptedTransport motion_transport(&motion_clock);
  wheeltec::WheeltecSerialRuntime motion_runtime(
      enabledConfig(), clockOperations(&motion_clock));
  finishStartupAndDrain(&motion_runtime, &motion_transport, &motion_clock);
  recoverAllowedFeedback(&motion_runtime, &motion_transport, &motion_clock);
  expect(motion_runtime.setAuthorization(true, 1U, motion_clock),
         "completion-clock motion fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
    ++motion_clock;
    motion_runtime.deliver(command(sequence, motion_clock), motion_clock);
  }
  motion_transport.write_script.push_back(
      {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize, false, -1});
  ++motion_clock;
  const wheeltec::RuntimeDelivery regressed_motion =
      motion_runtime.deliver(command(3U, motion_clock), motion_clock);
  expect(!regressed_motion.delivered && !regressed_motion.motion_accepted &&
             regressed_motion.delivery_unconfirmed &&
             motion_runtime.phase() ==
                 wheeltec::RuntimePhase::kTerminalFault,
         "a regressed post-motion completion clock cannot report delivery or remain ready");

  std::int64_t shutdown_clock = 100;
  ScriptedTransport shutdown_transport(&shutdown_clock);
  wheeltec::WheeltecSerialRuntime shutdown_runtime(
      enabledConfig(), clockOperations(&shutdown_clock));
  finishStartupAndDrain(&shutdown_runtime, &shutdown_transport,
                        &shutdown_clock);
  recoverAllowedFeedback(&shutdown_runtime, &shutdown_transport,
                         &shutdown_clock);
  shutdown_transport.write_script.push_back(
      {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize, false, -1});
  const wheeltec::RuntimeDelivery regressed_shutdown =
      shutdown_runtime.shutdown(shutdown_clock);
  expect(!regressed_shutdown.delivered &&
             regressed_shutdown.delivery_unconfirmed &&
             shutdown_runtime.phase() ==
                 wheeltec::RuntimePhase::kTerminalFault,
         "a regressed shutdown-zero completion clock is terminal and unconfirmed");
}

void testWriteCompletionDeadlines() {
  {
    std::int64_t now = 100;
    ScriptedTransport transport(&now);
    transport.write_script.push_back(
        {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize,
         false, 6});
    wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                             clockOperations(&now));
    expect(runtime.attachTransport(&transport, now),
           "late startup-zero fixture attaches");
    const wheeltec::RuntimeStepResult late = runtime.step(now);
    expect(late.phase == wheeltec::RuntimePhase::kTerminalFault &&
               late.delivery_unconfirmed && transport.writes.size() == 1U &&
               isExactZero(transport.writes.front()),
           "a complete startup zero after its write deadline is known zero but not a successful bounded startup");
  }

  {
    std::int64_t now = 100;
    ScriptedTransport transport(&now);
    wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                             clockOperations(&now));
    finishStartupAndDrain(&runtime, &transport, &now);
    recoverAllowedFeedback(&runtime, &transport, &now);
    expect(runtime.setAuthorization(true, 1U, now),
           "write-timeout fixture authorizes");
    for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
      ++now;
      runtime.deliver(command(sequence, now), now);
    }
    transport.write_script.push_back(
        {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize,
         false, 6});
    ++now;
    const std::size_t before = transport.writes.size();
    const wheeltec::RuntimeDelivery late =
        runtime.deliver(command(3U, now), now);
    expect(late.command_watchdog_expired && late.stop_attempted &&
               late.delivery_unconfirmed && !late.motion_accepted &&
               transport.writes.size() == before + 2U &&
               !isExactZero(transport.writes[before]) &&
               isExactZero(transport.writes.back()) &&
               runtime.phase() == wheeltec::RuntimePhase::kTerminalFault,
           "a full motion write that completes after the adapter timeout gets same-call exact zero and a terminal timing fault");
  }

  {
    std::int64_t now = 100;
    ScriptedTransport transport(&now);
    wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                             clockOperations(&now));
    finishStartupAndDrain(&runtime, &transport, &now);
    recoverAllowedFeedback(&runtime, &transport, &now);
    expect(runtime.setAuthorization(true, 1U, now),
           "command-deadline fixture authorizes");
    for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
      ++now;
      runtime.deliver(command(sequence, now), now);
    }
    ++now;
    auto_rover::VehicleExecutionCommand early = command(3U, now);
    early.deadline_monotonic_ns = now + 2;
    transport.write_script.push_back(
        {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize,
         false, 3});
    const std::size_t before = transport.writes.size();
    const wheeltec::RuntimeDelivery late = runtime.deliver(early, now);
    expect(late.command_watchdog_expired && late.delivery_unconfirmed &&
               !late.motion_accepted && late.stop_attempted &&
               transport.write_deadlines[before] ==
                   early.deadline_monotonic_ns &&
               transport.writes.size() == before + 2U &&
               isExactZero(transport.writes.back()),
           "the command deadline wins over the adapter timeout and a late complete motion frame is stopped in the same call");
  }

  {
    std::int64_t now = 100;
    ScriptedTransport transport(&now);
    wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                             clockOperations(&now));
    finishStartupAndDrain(&runtime, &transport, &now);
    recoverAllowedFeedback(&runtime, &transport, &now);
    transport.write_script.push_back(
        {wheeltec::TransportStatus::kOk, wheeltec::kCommandFrameSize,
         false, 6});
    const std::size_t before = transport.writes.size();
    const wheeltec::RuntimeDelivery late_stop = runtime.revokeAndStop(now);
    expect(late_stop.stop_attempted && late_stop.delivered &&
               late_stop.delivery_unconfirmed &&
               transport.writes.size() == before + 1U &&
               isExactZero(transport.writes.back()) &&
               runtime.phase() == wheeltec::RuntimePhase::kTerminalFault,
           "a full exact zero after the stop deadline remains identifiable as zero but fails the bounded stop contract");
  }
}

void testPollFailuresStopInSameStep() {
  std::int64_t write_now = 100;
  ScriptedTransport write_transport(&write_now);
  wheeltec::WheeltecSerialRuntime write_runtime(
      enabledConfig(), clockOperations(&write_now));
  finishStartupAndDrain(&write_runtime, &write_transport, &write_now);
  recoverAllowedFeedback(&write_runtime, &write_transport, &write_now);
  expect(write_runtime.setAuthorization(true, 1U, write_now),
         "poll write-failure fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++write_now;
    write_runtime.deliver(command(sequence, write_now), write_now);
  }
  write_transport.enqueueFrame(makeFeedback(250, 25), 1);
  write_transport.write_script.push_back(
      {wheeltec::TransportStatus::kDeadlineExceeded, 0U, false, 1});
  const std::size_t write_before = write_transport.writes.size();
  const wheeltec::RuntimeStepResult failed_rewrite =
      write_runtime.step(write_now);
  expect(failed_rewrite.delivery_unconfirmed &&
             write_transport.writes.size() == write_before + 2U &&
             !isExactZero(write_transport.writes[write_before]) &&
             isExactZero(write_transport.writes.back()) &&
             write_runtime.phase() ==
                 wheeltec::RuntimePhase::kTerminalFault,
         "a zero-byte normal rewrite failure is followed by bounded exact zero in the same poll step");

  std::int64_t read_now = 100;
  ScriptedTransport read_transport(&read_now);
  wheeltec::WheeltecSerialRuntime read_runtime(
      enabledConfig(), clockOperations(&read_now));
  finishStartupAndDrain(&read_runtime, &read_transport, &read_now);
  recoverAllowedFeedback(&read_runtime, &read_transport, &read_now);
  expect(read_runtime.setAuthorization(true, 1U, read_now),
         "poll read-failure fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++read_now;
    read_runtime.deliver(command(sequence, read_now), read_now);
  }
  read_transport.read_script.push_back(
      {wheeltec::TransportStatus::kIoError, {}, 1});
  const std::size_t read_before = read_transport.writes.size();
  const wheeltec::RuntimeStepResult failed_read = read_runtime.step(read_now);
  expect(failed_read.delivery_unconfirmed &&
             read_transport.writes.size() == read_before + 1U &&
             isExactZero(read_transport.writes.back()) &&
             read_runtime.phase() == wheeltec::RuntimePhase::kTerminalFault,
         "a connected read I/O failure emits bounded exact zero before terminal fault");

  std::int64_t overrun_now = 100;
  ScriptedTransport overrun_transport(&overrun_now);
  wheeltec::WheeltecSerialRuntime overrun_runtime(
      enabledConfig(), clockOperations(&overrun_now));
  finishStartupAndDrain(&overrun_runtime, &overrun_transport, &overrun_now);
  recoverAllowedFeedback(&overrun_runtime, &overrun_transport, &overrun_now);
  expect(overrun_runtime.setAuthorization(true, 1U, overrun_now),
         "poll overrun fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++overrun_now;
    overrun_runtime.deliver(command(sequence, overrun_now), overrun_now);
  }
  overrun_transport.enqueueFrame(makeFeedback(250, 25), 51);
  const std::size_t overrun_before = overrun_transport.writes.size();
  const wheeltec::RuntimeStepResult overrun =
      overrun_runtime.step(overrun_now);
  expect(overrun.step_overrun && overrun.delivery_unconfirmed &&
             overrun_transport.writes.size() == overrun_before + 2U &&
             !isExactZero(overrun_transport.writes[overrun_before]) &&
             isExactZero(overrun_transport.writes.back()) &&
             overrun_runtime.phase() ==
                 wheeltec::RuntimePhase::kTerminalFault,
         "a poll step overrun performs bounded exact zero after its motion rewrite before terminal fault");
}

void testReconnectRequiresNewSessionEvidence() {
  std::int64_t now = 100;
  ScriptedTransport first(&now, 1U);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &first, &now);
  recoverAllowedFeedback(&runtime, &first, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "first connection is authorized");
  for (std::uint64_t sequence = 10U; sequence <= 12U; ++sequence) {
    now += 1;
    runtime.deliver(command(sequence, now), now);
  }
  first.enqueueDisconnect();
  now += 1;
  const wheeltec::RuntimeStepResult disconnected = runtime.step(now);
  expect(disconnected.phase == wheeltec::RuntimePhase::kTerminalDisconnected &&
             !runtime.health(now).authorization_active,
         "disconnect clears authorization, parser, command and readiness state");

  ScriptedTransport stale_generation(&now, 1U);
  expect(!runtime.attachTransport(&stale_generation, now),
         "a reused connection generation cannot remount");
  ScriptedTransport second(&now, 2U);
  expect(runtime.attachTransport(&second, now),
         "a strictly newer generation can remount");
  expect(runtime.phase() == wheeltec::RuntimePhase::kStartupZero,
         "new generation begins in startup-zero state");
  expect(!runtime.setAuthorization(true, 2U, now) &&
             runtime.phase() == wheeltec::RuntimePhase::kDrainBacklog &&
             second.writes.size() == 1U &&
             isExactZero(second.writes.front()),
         "premature reconnect authorization is denied while its same-call stop still satisfies startup-zero-first");
  expect(runtime.deliver(command(12U, now), now).submission.status ==
             wheeltec::SubmissionStatus::kSequenceInvalid,
         "sequence high-water survives transport remount");
  second.enqueueIdle();
  runtime.step(now);
  recoverAllowedFeedback(&runtime, &second, &now);
  expect(!runtime.setAuthorization(true, 1U, now) &&
             runtime.setAuthorization(true, 2U, now),
         "reconnect requires a new authorization identity");
}

void testActiveSessionCannotBeSilentlyRemounted() {
  std::int64_t now = 100;
  ScriptedTransport first(&now, 1U);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &first, &now);
  recoverAllowedFeedback(&runtime, &first, &now);
  expect(runtime.setAuthorization(true, 1U, now),
         "active-remount fixture authorizes");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    ++now;
    runtime.deliver(command(sequence, now), now);
  }
  ScriptedTransport second(&now, 2U);
  const std::size_t old_writes_before = first.writes.size();
  expect(!runtime.attachTransport(&second, now) &&
             first.writes.size() == old_writes_before + 1U &&
             isExactZero(first.writes.back()) && second.io_order.empty() &&
             runtime.phase() == wheeltec::RuntimePhase::kTerminalFault,
         "an active generation cannot be replaced silently; the old link gets bounded zero and the candidate link remains untouched");
  expect(runtime.attachTransport(&second, now) && second.io_order.empty(),
         "after terminalizing the old session, an explicit newer generation can be mounted without implicit I/O");
  const wheeltec::RuntimeStepResult startup = runtime.step(now);
  expect(startup.phase == wheeltec::RuntimePhase::kDrainBacklog &&
             second.writes.size() == 1U &&
             isExactZero(second.writes.front()),
         "the newer generation still performs exact zero as its first transport I/O");
}

void testClockRollbackAndShutdownZero() {
  std::int64_t now = 100;
  ScriptedTransport transport(&now);
  wheeltec::WheeltecSerialRuntime runtime(enabledConfig(),
                                           clockOperations(&now));
  finishStartupAndDrain(&runtime, &transport, &now);
  recoverAllowedFeedback(&runtime, &transport, &now);
  now += 10;
  const std::int64_t accepted_caller_time = now;
  transport.enqueueIdle();
  runtime.step(accepted_caller_time);
  const std::size_t writes_before_rollback = transport.writes.size();
  const wheeltec::RuntimeStepResult rollback =
      runtime.step(accepted_caller_time - 1);
  expect(rollback.phase == wheeltec::RuntimePhase::kTerminalFault &&
             !runtime.health(now).authorization_active &&
             transport.writes.size() == writes_before_rollback,
         "monotonic rollback writes nothing and enters a terminal fail-closed state");

  std::int64_t shutdown_now = 100;
  ScriptedTransport shutdown_transport(&shutdown_now);
  wheeltec::WheeltecSerialRuntime shutdown_runtime(
      enabledConfig(), clockOperations(&shutdown_now));
  finishStartupAndDrain(&shutdown_runtime, &shutdown_transport, &shutdown_now);
  recoverAllowedFeedback(&shutdown_runtime, &shutdown_transport, &shutdown_now);
  const wheeltec::RuntimeDelivery stopped = shutdown_runtime.shutdown(shutdown_now);
  expect(stopped.stop_attempted && stopped.delivered &&
             !stopped.controller_ack_available &&
             !stopped.controller_acknowledged &&
             isExactZero(shutdown_transport.writes.back()) &&
             shutdown_runtime.phase() == wheeltec::RuntimePhase::kShutdown,
         "shutdown completes a bounded exact-zero host write without inventing an ACK");
}

struct PtyPair {
  int master{-1};
  int slave{-1};
  ~PtyPair() {
    if (master >= 0) {
      ::close(master);
    }
    if (slave >= 0) {
      ::close(slave);
    }
  }
};

bool openPtyPair(PtyPair* pair) {
  if (pair == nullptr) {
    return false;
  }
  pair->master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (pair->master < 0 || ::grantpt(pair->master) != 0 ||
      ::unlockpt(pair->master) != 0) {
    return false;
  }
  char* const slave_name = ::ptsname(pair->master);
  if (slave_name == nullptr) {
    return false;
  }
  pair->slave =
      ::open(slave_name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (pair->slave < 0) {
    return false;
  }
  return wheeltec::configureSerial115200EightNOne(pair->slave);
}

void testPtyFragmentationNoiseAndHup() {
  PtyPair pair;
  expect(openPtyPair(&pair), "PTY opens for runtime fragmentation/HUP test");
  if (pair.master < 0 || pair.slave < 0) {
    return;
  }
  wheeltec::RuntimeConfig config = enabledConfig();
  config.adapter.max_command_age_ns = 100000000;
  config.adapter.zero_retry_interval_ns = 1000000;
  config.adapter.write_timeout_ns = 10000000;
  config.read_timeout_ns = 1000000;
  config.maximum_feedback_age_ns = 150000000;
  config.drain_read_timeout_ns = 1000000;
  config.control_allowed_minimum_span_ns = 1000000;
  config.maximum_normal_write_gap_ns = 100000000;
  config.maximum_step_duration_ns = 50000000;
  wheeltec::PosixFdTransport transport(pair.slave, false, 7U, true);
  wheeltec::WheeltecSerialRuntime runtime(config);
  std::int64_t now = wheeltec::monotonicNowNs();
  expect(runtime.attachTransport(&transport, now), "PTY runtime attaches");
  wheeltec::RuntimeStepResult step = runtime.step(now);
  expect(step.phase == wheeltec::RuntimePhase::kDrainBacklog,
         "PTY startup zero succeeds");
  std::uint8_t zero[wheeltec::kCommandFrameSize]{};
  expect(::read(pair.master, zero, sizeof(zero)) ==
                 static_cast<ssize_t>(sizeof(zero)),
         "PTY peer sees complete startup zero");
  step = runtime.step(wheeltec::monotonicNowNs());
  expect(step.phase == wheeltec::RuntimePhase::kAwaitingFeedback,
         "PTY drain completes on timeout");

  const wheeltec::FeedbackFrame frame = makeFeedback(-321, 42);
  const std::uint8_t noise[3U] = {0x00U, 0x55U, 0x7DU};
  expect(::write(pair.master, noise, sizeof(noise)) ==
                 static_cast<ssize_t>(sizeof(noise)) &&
             ::write(pair.master, frame.data(), 7U) == 7,
         "PTY peer writes noise plus a feedback fragment");
  step = runtime.step(wheeltec::monotonicNowNs());
  expect(!step.feedback_available,
         "noise and a partial PTY frame do not refresh feedback");
  expect(::write(pair.master, frame.data() + 7U, frame.size() - 7U) ==
             static_cast<ssize_t>(frame.size() - 7U),
         "PTY peer writes the remainder of the fragmented frame");
  step = runtime.step(wheeltec::monotonicNowNs());
  expect(step.feedback_available,
         "runtime parser reconstructs the complete PTY feedback frame");
  expectNear(step.feedback.signed_speed_mps, -0.321, 1e-12,
             "PTY feedback preserves signed speed");

  ::close(pair.master);
  pair.master = -1;
  step = runtime.step(wheeltec::monotonicNowNs());
  expect(step.phase == wheeltec::RuntimePhase::kTerminalDisconnected,
         "PTY HUP becomes a terminal disconnected session");
}

void testVehicleBackendNormalizationAndDisabledInit() {
  std::int64_t mono = 100;
  std::int64_t ros = 1000;
  ScriptedTransport disabled_transport(&mono);
  wheeltec::WheeltecVehicleBackendConfig disabled_config;
  disabled_config.runtime = enabledConfig();
  disabled_config.runtime.runtime_enabled = false;
  wheeltec::WheeltecVehicleBackendOperations operations;
  operations.runtime = clockOperations(&mono);
  operations.ros_now_ns = [&ros]() { return ros; };
  wheeltec::WheeltecVehicleBackend disabled(
      disabled_config, &disabled_transport, operations);
  expect(disabled.initialize(mono, ros) && disabled_transport.io_order.empty(),
         "disabled formal backend initializes without device I/O");
  const auto_rover_vehicle::BackendFeedback disabled_poll =
      disabled.poll(mono, ros);
  expect(disabled_poll.operation_completed_monotonic_ns == mono &&
             disabled_poll.operation_completed_ros_ns == ros &&
             disabled_transport.io_order.empty(),
         "disabled backend returns valid no-op completion clocks without device I/O");
  const auto_rover_vehicle::BackendHealth disabled_health = disabled.health();
  expect(disabled_health.configuration_valid && !disabled_health.connected &&
             disabled_health.reason == auto_rover::StopReason::kActuationDisabled,
         "disabled backend reports a valid intentionally inhibited state");
  const auto_rover_vehicle::BackendDelivery disabled_stop =
      disabled.revokeAndStop(mono);
  expect(disabled_stop.stop_attempted && disabled_stop.delivered &&
             !disabled_stop.delivery_unconfirmed &&
             disabled_transport.io_order.empty(),
         "disabled formal backend satisfies the local stop boundary without false unconfirmed alarms or device I/O");

  ScriptedTransport transport(&mono, 2U);
  wheeltec::WheeltecVehicleBackendConfig config;
  config.runtime = enabledConfig();
  wheeltec::WheeltecVehicleBackend backend(config, &transport, operations);
  expect(backend.initialize(mono, ros),
         "enabled formal backend performs startup zero initialization");
  transport.enqueueIdle();
  backend.poll(mono, ros);
  for (int index = 0; index < 3; ++index) {
    mono += 10;
    ros += 10;
    transport.enqueueFrame(makeFeedback(-250 + index, 50 + index), 10);
    const auto_rover_vehicle::BackendFeedback feedback =
        backend.poll(mono, ros);
    expect(feedback.available,
           "formal backend publishes each latest complete feedback receipt");
    expect(feedback.operation_completed_monotonic_ns >=
                   feedback.receipt_monotonic_ns &&
               feedback.operation_completed_ros_ns == ros &&
               feedback.state.stamp_ns <=
                   feedback.operation_completed_ros_ns,
           "formal feedback exposes both post-poll completion clocks and never stamps a sample after completion");
    expect((feedback.state.valid_mask &
            auto_rover::ChassisState::kMeasuredSpeedValid) != 0U &&
               (feedback.state.valid_mask &
                auto_rover::ChassisState::kYawRateValid) != 0U &&
               (feedback.state.valid_mask &
                auto_rover::ChassisState::kVoltageValid) != 0U &&
               (feedback.state.valid_mask &
                (auto_rover::ChassisState::kGearValid |
                 auto_rover::ChassisState::kControlEnabledValid |
                 auto_rover::ChassisState::kFaultValid)) == 0U,
           "backend validity mask exposes only measured speed, wheel yaw and voltage");
    expect(feedback.state.gear_state == auto_rover::GearState::kUnknown &&
               feedback.state.fault_state == auto_rover::FaultState::kUnknown &&
               !feedback.state.control_enabled,
           "backend never fabricates unavailable gear, fault or control state");
  }
  expect(backend.health().actuation_permitted,
         "formal backend exposes recovered local actuation permission separately from authorization");
  const auto_rover_vehicle::BackendDelivery ordinary_stop =
      backend.revokeAndStop(mono);
  expect(ordinary_stop.stop_attempted && ordinary_stop.delivered &&
             backend.health().actuation_permitted &&
             !backend.health().authorization_active &&
             backend.setAuthorization(true, 1U, mono),
         "formal backend ordinary revoke is nonterminal and preserves recovered feedback readiness");

  const std::size_t before_invalid_ros = transport.writes.size();
  const std::int64_t invalid_ros_caller_mono = mono;
  const std::int64_t valid_ros_caller = ros;
  ros = 0;
  transport.enqueueFrame(makeFeedback(0, 0), 1);
  const auto_rover_vehicle::BackendFeedback invalid_ros =
      backend.poll(mono, valid_ros_caller);
  expect(invalid_ros.operation_completed_ros_ns == 0 &&
             !invalid_ros.health.authorization_active &&
             invalid_ros.health.reason == auto_rover::StopReason::kInvalidInput &&
             transport.writes.size() >= before_invalid_ros + 1U &&
             isExactZero(transport.writes.back()),
         "an invalid post-poll ROS completion clock fails closed and performs bounded zero in the same backend call");
  const auto_rover_vehicle::BackendDelivery core_followup_stop =
      backend.revokeAndStop(invalid_ros_caller_mono);
  expect(core_followup_stop.stop_attempted &&
             core_followup_stop.delivered &&
             !core_followup_stop.delivery_unconfirmed &&
             isExactZero(transport.writes.back()),
         "the fail-closed ROS-clock stop preserves the caller timestamp so the core can repeat its same-cycle stop without a false rollback");
}

void testVehicleBackendStagesTransportWithoutIoBeforeInitialization() {
  std::int64_t mono = 100;
  std::int64_t ros = 1000;
  ScriptedTransport transport(&mono, 31U);
  wheeltec::WheeltecVehicleBackendConfig config;
  config.runtime = enabledConfig();
  wheeltec::WheeltecVehicleBackendOperations operations;
  operations.runtime = clockOperations(&mono);
  operations.ros_now_ns = [&ros]() { return ros; };
  wheeltec::WheeltecVehicleBackend backend(config, nullptr, operations);

  expect(backend.setTransportForInitialization(&transport) &&
             transport.io_order.empty(),
         "formal backend stages an already-open transport without protocol I/O");
  expect(backend.initialize(mono, ros) &&
             transport.io_order.size() == 1U &&
             transport.io_order.front() == IoKind::kWrite &&
             isExactZero(transport.writes.front()),
         "the staged backend initialization makes exact zero its first protocol I/O");
  expect(!backend.setTransportForInitialization(&transport),
         "an initialized backend cannot replace its staged transport");
}

void testExecutionCoreWithFormalWheeltecBackend() {
  std::int64_t mono = 1000000000LL;
  std::int64_t ros = 2000000000LL;
  ScriptedTransport transport(&mono, 11U);
  wheeltec::RuntimeConfig runtime_config = enabledConfig();
  runtime_config.adapter.max_command_age_ns = 100000000LL;
  runtime_config.adapter.zero_retry_interval_ns = 1000000LL;
  runtime_config.adapter.write_timeout_ns = 10000000LL;
  runtime_config.read_timeout_ns = 1000000LL;
  runtime_config.maximum_feedback_age_ns = 150000000LL;
  runtime_config.drain_read_timeout_ns = 1000000LL;
  runtime_config.control_allowed_receipts_required = 5U;
  runtime_config.control_allowed_minimum_span_ns = 200000000LL;
  runtime_config.maximum_normal_write_gap_ns = 100000000LL;
  runtime_config.maximum_step_duration_ns = 50000000LL;

  wheeltec::WheeltecVehicleBackendConfig backend_config;
  backend_config.runtime = runtime_config;
  wheeltec::WheeltecVehicleBackendOperations operations;
  operations.runtime = clockOperations(&mono);
  operations.ros_now_ns = [&ros]() { return ros; };
  std::unique_ptr<auto_rover_vehicle::VehicleBackend> backend(
      new wheeltec::WheeltecVehicleBackend(
          backend_config, &transport, operations));
  auto_rover_vehicle::VehicleExecutionCore core(
      executionConfig(), std::move(backend));
  expect(core.initialize(mono, ros) && transport.writes.size() == 1U &&
             isExactZero(transport.writes.front()),
         "formal core integration initializes only after the backend startup exact zero");

  expect(core.updateEgoState(executionEgo(ros), mono).ok,
         "formal core integration accepts localization input");
  expect(core.updateTrajectory(executionTrajectory(ros), mono).ok,
         "formal core integration accepts trajectory input");
  core.updateMotionReference(executionMotion(1U, ros, 0.25), mono);

  transport.enqueueIdle(1);
  core.cycle(mono, ros);
  auto_rover_vehicle::VehicleExecutionCycleResult recovered;
  std::uint64_t motion_id = 1U;
  for (int receipt = 0; receipt < 5; ++receipt) {
    mono += 50000000LL;
    ros += 50000000LL;
    ++motion_id;
    core.updateMotionReference(
        executionMotion(motion_id, ros, 0.25), mono);
    transport.enqueueFrame(makeFeedback(0, 0), 1000000LL);
    recovered = core.cycle(mono, ros);
  }
  expect(recovered.health_clear &&
             recovered.safety.mode == auto_rover::SafetyMode::kDisarmed &&
             !core.executionAuthorized(),
         "five distinct fresh FlagStop=0 receipts recover through repeated ordinary disarmed revoke-and-stop cycles without terminalizing the backend");

  const auto_rover::SafetyState ready = core.currentSafetyState(ros);
  const auto_rover_vehicle::VehicleExecutionServiceResult armed =
      core.requestArm("operator", ready.state_id, true, mono, ros);
  expect(armed.success && core.executionAuthorized(),
         "the recovered formal backend accepts a new explicit arm epoch");

  bool motion_written = false;
  for (int cycle = 0; cycle < 4; ++cycle) {
    mono += 50000000LL;
    ros += 50000000LL;
    ++motion_id;
    core.updateMotionReference(
        executionMotion(motion_id, ros, 0.25), mono);
    transport.enqueueFrame(makeFeedback(0, 0), 1000000LL);
    const auto_rover_vehicle::VehicleExecutionCycleResult result =
        core.cycle(mono, ros);
    motion_written = motion_written || result.delivery.motion_accepted;
  }
  bool nonzero_wire_frame_seen = false;
  for (const std::vector<std::uint8_t>& write : transport.writes) {
    nonzero_wire_frame_seen = nonzero_wire_frame_seen || !isExactZero(write);
  }
  expect(motion_written && nonzero_wire_frame_seen &&
             core.executionAuthorized(),
         "after arm and command recovery, the real core/backend objects complete a nonzero motion host write without false clock rollback");
}

}  // namespace

int main() {
  testDisabledRuntimeTouchesNoTransport();
  testStartupDrainLatestReceiptAndRecovery();
  testBadPartialFlagStopAndStaleDoNotLeakMotion();
  testInvalidCompositeFlagStopInhibitsWholeRead();
  testStartupPartialWriteBlocksRead();
  testPartialOverrunNeverAppendsZero();
  testPartialWritePoisonsBeforeCompletionClockValidation();
  testThrowingWritePoisonsGeneration();
  testStartupZeroByteFailureRetriesWithoutRead();
  testStartupZeroByteFailureCanRecoverThroughOrdinaryStop();
  testRejectedCommandStopsInSameDelivery();
  testAuthorizationRevocationStopsInSameCall();
  testReadCannotSplitNormalWriteGap();
  testMotionWriteOverrunGetsImmediateZero();
  testStopRetryAndNonterminalRevoke();
  testStopPartialAndCompletionClockRollbackFailClosed();
  testWriteCompletionDeadlines();
  testPollFailuresStopInSameStep();
  testReconnectRequiresNewSessionEvidence();
  testActiveSessionCannotBeSilentlyRemounted();
  testClockRollbackAndShutdownZero();
  testPtyFragmentationNoiseAndHup();
  testVehicleBackendNormalizationAndDisabledInit();
  testVehicleBackendStagesTransportWithoutIoBeforeInitialization();
  testExecutionCoreWithFormalWheeltecBackend();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d wheeltec runtime test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("wheeltec runtime tests passed\n");
  return 0;
}
