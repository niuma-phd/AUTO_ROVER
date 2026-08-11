#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "auto_rover_safety/command_guard.hpp"
#include "auto_rover_vehicle/fake_vcu.hpp"
#include "auto_rover_vehicle/vehicle_backend.hpp"
#include "auto_rover_vehicle/vehicle_motion_manager.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

auto_rover::VehicleProfile profile() {
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

auto_rover_safety::GuardResult guarded(double speed, std::uint64_t command_id,
                                        bool allowed = true) {
  auto_rover_safety::GuardResult result;
  result.reference.stamp_ns = 1000000000LL;
  result.reference.frame_id = "rear_axle_center";
  result.reference.command_id = command_id;
  result.reference.producer_generation_id =
      "vehicle-test-tracker-generation-1";
  result.reference.trajectory_id = "route:1:1";
  result.reference.direction = auto_rover::Direction::kForward;
  result.reference.target_speed_mps = speed;
  result.reference.valid_for_ns = 100000000LL;
  result.reference.valid = true;
  result.execution_allowed = allowed;
  result.reason = allowed ? auto_rover::StopReason::kNone
                          : auto_rover::StopReason::kDisarmed;
  return result;
}

void testManagerReconnectAndAcceleration() {
  auto_rover_vehicle::VehicleMotionManagerConfig config;
  config.command_valid_for_ns = 100000000LL;
  auto_rover_vehicle::VehicleMotionManager manager(config, profile());

  auto command = manager.makeCommand(guarded(0.0, 1U), 1000000000LL);
  expect(!command.motion_enabled,
         "vehicle manager starts inhibited before backend connection and arm");
  expect(command.signed_speed_mps == 0.0, "startup inhibit is explicit zero");

  manager.setBackendConnected(true, 1010000000LL);
  command = manager.makeCommand(guarded(0.0, 2U), 1020000000LL);
  expect(!command.motion_enabled, "connection alone cannot restore motion");
  expect(manager.acknowledgeOperatorArm(9U, 1030000000LL),
         "explicit operator arm clears reconnect inhibit");
  command = manager.makeCommand(guarded(0.0, 3U), 1040000000LL);
  expect(command.motion_enabled && command.hold,
         "armed zero command requests enabled stop and hold");

  command = manager.makeCommand(guarded(0.02, 4U), 1140000000LL);
  expect(command.motion_enabled, "0.02 m/s after 0.1 s respects acceleration");
  expect(std::abs(command.signed_speed_mps - 0.02) < 1e-12,
         "manager preserves a rate-compliant command");
  command = manager.makeCommand(guarded(0.50, 5U), 1240000000LL);
  expect(command.motion_enabled,
         "manager safely rate-limits an acceleration jump instead of deadlocking");
  expect(std::abs(command.signed_speed_mps - 0.04) < 1e-12,
         "manager advances by no more than 0.20 m/s2");
  expect(command.stop_reason == auto_rover::StopReason::kNone,
         "a safely limited valid request remains executable");

  manager.setBackendConnected(false, 1250000000LL);
  command = manager.makeCommand(guarded(0.0, 6U), 1260000000LL);
  expect(command.stop_reason == auto_rover::StopReason::kBackendDisconnected,
         "disconnect produces delivery-boundary inhibit");
  manager.setBackendConnected(true, 1270000000LL);
  command = manager.makeCommand(guarded(0.0, 7U), 1280000000LL);
  expect(!command.motion_enabled,
         "reconnect discards old authorization and requires re-arm");
}

auto_rover_vehicle::FakeVcuConfig fakeConfig() {
  auto_rover_vehicle::FakeVcuConfig config;
  config.process_generation_id = "vehicle-test-fake-vcu-generation-1";
  config.command_watchdog_ns = 150000000LL;
  config.feedback_period_ns = 50000000LL;
  config.fresh_recovery_count = 3U;
  config.max_accel_mps2 = 0.20;
  return config;
}

void testFakeVcuProcessGenerationIdentity() {
  auto missing_generation = fakeConfig();
  missing_generation.process_generation_id.clear();
  expect(!auto_rover_vehicle::validateFakeVcuConfig(
              missing_generation, profile()).ok,
         "fake VCU configuration without a process generation fails closed");

  auto first_config = fakeConfig();
  auto second_config = fakeConfig();
  second_config.process_generation_id =
      "vehicle-test-fake-vcu-generation-2";
  auto_rover_vehicle::FakeVcu first(first_config, profile());
  auto_rover_vehicle::FakeVcu second(second_config, profile());
  first.setConnected(true, 1000000000LL);
  second.setConnected(true, 1000000000LL);
  const auto first_feedback = first.step(1050000000LL, 2050000000LL);
  const auto second_feedback = second.step(1050000000LL, 2050000000LL);
  expect(first_feedback.available && second_feedback.available &&
             first_feedback.state.state_id == 1U &&
             second_feedback.state.state_id == 1U,
         "independent fake VCU processes may each begin at state identity one");
  expect(first_feedback.state.source_id ==
             "deterministic_fake_vcu_v1#process=" +
                 first_config.process_generation_id,
         "fake feedback combines stable backend and process identities");
  expect(first_feedback.state.source_id != second_feedback.state.source_id,
         "a fake VCU restart changes the ChassisState source identity");
}

auto_rover::VehicleExecutionCommand execution(std::uint64_t sequence,
                                              double speed,
                                              std::int64_t now,
                                              double curvature = 0.0) {
  auto_rover::VehicleExecutionCommand command;
  command.sequence_id = sequence;
  command.created_monotonic_ns = now;
  command.deadline_monotonic_ns = now + 100000000LL;
  command.signed_speed_mps = speed;
  command.curvature_inv_m = curvature;
  command.motion_enabled = true;
  command.hold = speed == 0.0;
  command.stop_reason = auto_rover::StopReason::kNone;
  return command;
}

void testFakeVcuSubstantiatedYawFeedback() {
  auto_rover_vehicle::FakeVcu fake(fakeConfig(), profile());
  fake.setConnected(true, 1000000000LL);
  fake.setControlEnabled(true, 1010000000LL);

  auto feedback = fake.step(1060000000LL, 1060000000LL);
  expect(feedback.available, "stationary fake VCU publishes feedback");
  expect((feedback.state.valid_mask &
          auto_rover::ChassisState::kYawRateValid) != 0U,
         "fake feedback substantiates modelled yaw rate");
  expect(feedback.state.measured_speed_mps == 0.0 &&
             feedback.state.yaw_rate_radps == 0.0,
         "zero fake speed naturally produces zero yaw rate");

  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    const std::int64_t now = 1060000000LL +
                             static_cast<std::int64_t>(sequence) * 10000000LL;
    fake.receive(execution(sequence, 0.02, now, 0.50), now);
  }
  feedback = fake.step(1140000000LL, 1140000000LL);
  expect(feedback.available && feedback.state.yaw_rate_radps > 0.0,
         "accepted curved motion produces modelled yaw feedback");
  expect(std::abs(feedback.state.yaw_rate_radps -
                  feedback.state.measured_speed_mps * 0.50) < 1e-12,
         "fake yaw feedback equals measured speed times target curvature");

  feedback = fake.step(1200000000LL, 1200000000LL);
  expect(feedback.watchdog_expired && feedback.available,
         "command deadline expires before watchdog feedback assertion");
  expect(feedback.state.yaw_rate_radps == 0.0,
         "watchdog clears target curvature before yaw feedback");
}

void testFakeVcuAuthorizationPublishesFreshControlState() {
  auto_rover_vehicle::FakeVcu fake(fakeConfig(), profile());
  expect(fake.initialize(1000000000LL, 2000000000LL),
         "fake VCU initializes for authorization feedback test");
  const auto disarmed = fake.step(1050000000LL, 2050000000LL);
  expect(disarmed.available && !disarmed.state.control_enabled,
         "fake VCU first reports its substantiated disarmed state");
  expect(fake.setAuthorization(true, 1U, 1060000000LL),
         "fake VCU accepts a fresh authorization generation");
  const auto armed = fake.step(1061000000LL, 2061000000LL);
  expect(armed.available && armed.state.control_enabled,
         "authorization forces a fresh enabled-state sample before the ordinary feedback period");

  fake.setFaulted(true, 1062000000LL);
  const auto faulted = fake.step(1111000000LL, 2111000000LL);
  expect(faulted.available && !faulted.state.control_enabled &&
             faulted.state.fault_state == auto_rover::FaultState::kFault,
         "fault feedback cannot reuse the forced authorized state");
  fake.setFaulted(false, 1112000000LL);
  expect(fake.setAuthorization(true, 2U, 1113000000LL),
         "fake VCU accepts a strictly newer re-authorization generation");
  const auto rearmed = fake.step(1114000000LL, 2114000000LL);
  expect(rearmed.available && rearmed.state.control_enabled,
         "fault recovery re-arm also forces fresh control state within one period");
  for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
    const std::int64_t now = 1115000000LL +
                             static_cast<std::int64_t>(sequence - 1U) *
                                 10000000LL;
    fake.receive(execution(sequence, 0.02, now), now);
  }
  const auto moving = fake.step(1165000000LL, 2165000000LL);
  expect(moving.available && moving.state.control_enabled &&
             moving.state.measured_speed_mps > 0.0,
         "re-arm remains authorized through fresh-command recovery and motion");
  expect(fake.setAuthorization(false, 0U, 1166000000LL),
         "explicit de-authorization is accepted");
  const auto early_disarmed = fake.step(1167000000LL, 2167000000LL);
  expect(!early_disarmed.available &&
             !fake.health().authorization_active,
         "de-authorization does not fabricate an early allowed feedback sample");
}

void testFakeVcuFaultsWatchdogAndReconnect() {
  auto_rover_vehicle::FakeVcu fake(fakeConfig(), profile());
  fake.setConnected(true, 1000000000LL);
  fake.setControlEnabled(true, 1010000000LL);

  for (std::uint64_t sequence = 1U; sequence <= 2U; ++sequence) {
    const std::int64_t now = 1010000000LL +
                             static_cast<std::int64_t>(sequence) * 10000000LL;
    const auto result = fake.receive(execution(sequence, 0.02, now), now);
    expect(result.delivered, "fresh fake command is delivered");
    expect(!result.motion_accepted,
           "fake VCU independently requires consecutive fresh recovery");
  }
  const auto accepted =
      fake.receive(execution(3U, 0.02, 1040000000LL), 1040000000LL);
  expect(accepted.motion_accepted,
         "third fresh command enables fake VCU motion target");
  auto feedback = fake.step(1090000000LL, 1090000000LL);
  expect(feedback.available, "connected fake VCU publishes feedback");
  expect((feedback.state.valid_mask &
          auto_rover::ChassisState::kMeasuredSpeedValid) != 0U,
         "fake feedback substantiates measured speed");
  expect(feedback.state.measured_speed_mps > 0.0 &&
             feedback.state.measured_speed_mps < 0.02,
         "fake feedback follows acceleration-limited motion");

  feedback = fake.step(1210000000LL, 1210000000LL);
  expect(feedback.watchdog_expired,
         "fake adapter output watchdog expires independently");
  expect(!feedback.state.control_enabled,
         "watchdog clears fake control enable and requires operator re-arm");

  fake.setFaulted(true, 1220000000LL);
  feedback = fake.step(1270000000LL, 1270000000LL);
  expect(feedback.available, "faulted fake VCU still reports state");
  expect(feedback.state.fault_state == auto_rover::FaultState::kFault,
         "fault injection is visible in normalized feedback");
  fake.setDropFeedback(true);
  feedback = fake.step(1320000000LL, 1320000000LL);
  expect(!feedback.available, "feedback drop injection suppresses publication");

  fake.setConnected(false, 1330000000LL);
  const auto disconnected =
      fake.receive(execution(4U, 0.0, 1340000000LL), 1340000000LL);
  expect(!disconnected.delivered && disconnected.delivery_unconfirmed,
         "disconnected write reports delivery unconfirmed");
  fake.setConnected(true, 1350000000LL);
  const auto after_reconnect =
      fake.receive(execution(5U, 0.02, 1360000000LL), 1360000000LL);
  expect(!after_reconnect.motion_accepted,
         "reconnect cannot replay the old enabled state");
}

void testFakeVcuImplementsProtocolIndependentBackendContract() {
  auto_rover_vehicle::FakeVcu fake(fakeConfig(), profile());
  auto_rover_vehicle::VehicleBackend* backend = &fake;
  expect(backend->validate(profile()).ok,
         "fake VCU validates through the protocol-independent backend contract");
  expect(backend->initialize(1000000000LL, 2000000000LL),
         "fake backend initializes without a protocol-specific core path");
  auto health = backend->health();
  expect(health.configuration_valid && health.connected &&
             health.actuation_permitted && health.feedback_fresh &&
             !health.authorization_active &&
             health.connection_generation != 0U,
         "fake backend health separates connection, permission, freshness, and authorization");
  expect(backend->setAuthorization(true, 1U, 1010000000LL),
         "fake backend accepts an explicit authorization epoch");
  health = backend->health();
  expect(health.authorization_active,
         "fake backend exposes its local authorization state");
  const auto feedback = backend->poll(1060000000LL, 2060000000LL);
  expect(feedback.available &&
             feedback.receipt_monotonic_ns == 1060000000LL &&
             feedback.health.authorization_active,
         "fake backend feedback carries its exact local receipt and health snapshot");
  const auto stopped = backend->revokeAndStop(1070000000LL);
  expect(stopped.stop_attempted && stopped.delivered &&
             !stopped.delivery_unconfirmed &&
             !backend->health().authorization_active,
         "fake backend revocation executes the immediate-stop contract");
}

}  // namespace

int main() {
  testManagerReconnectAndAcceleration();
  testFakeVcuProcessGenerationIdentity();
  testFakeVcuSubstantiatedYawFeedback();
  testFakeVcuAuthorizationPublishesFreshControlState();
  testFakeVcuFaultsWatchdogAndReconnect();
  testFakeVcuImplementsProtocolIndependentBackendContract();
  if (failures != 0) {
    std::cerr << failures << " assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "vehicle tests passed\n";
  return EXIT_SUCCESS;
}
