#include "ordered_cycle_publication_mailbox.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace mailbox = auto_rover::wheeltec_serial::node_internal;

namespace {

int failures = 0;

void expect(bool condition, const char* message) {
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

auto_rover::SafetyState safetyState(std::uint64_t state_id,
                                    auto_rover::SafetyMode mode,
                                    std::uint64_t latch_generation,
                                    std::int64_t stamp_ns) {
  auto_rover::SafetyState state;
  state.stamp_ns = stamp_ns;
  state.state_id = state_id;
  state.mode = mode;
  state.latch_generation = latch_generation;
  state.valid_for_ns = 100000000LL;
  state.valid = true;
  return state;
}

auto_rover_vehicle::VehicleExecutionCycleResult cycleResult(
    const auto_rover::SafetyState& safety) {
  auto_rover_vehicle::VehicleExecutionCycleResult result;
  result.safety = safety;
  return result;
}

mailbox::OrderedCycleStoreResult store(
    mailbox::OrderedCyclePublicationMailbox* publication_mailbox,
    const auto_rover::SafetyState& safety) {
  return publication_mailbox->tryStoreCycleResult(
      cycleResult(safety),
      std::chrono::steady_clock::now() + std::chrono::seconds(1));
}

void testArmTransitionCannotReplayPendingDisarmedCycle() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto disarmed = safetyState(
      10U, auto_rover::SafetyMode::kDisarmed, 0U, 1000LL);
  const auto armed = safetyState(
      11U, auto_rover::SafetyMode::kArmed, 0U, 1100LL);

  expect(store(&publication_mailbox, disarmed) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a pre-arm disarmed cycle enters the publication mailbox");
  expect(publication_mailbox.prepareDirectSafetyPublication(armed),
         "the arm transition reserves the next direct publication");

  auto_rover_vehicle::VehicleExecutionCycleResult output;
  expect(!publication_mailbox.takeCycleResult(&output),
         "the pending pre-arm disarmed cycle is discarded before ARMED is published");
  expect(store(&publication_mailbox, disarmed) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "a delayed pre-arm worker result cannot re-enter after ARMED");
  expect(!publication_mailbox.takeCycleResult(&output),
         "a dropped pre-arm result is never visible to the ROS publisher");

  auto armed_cycle = armed;
  armed_cycle.stamp_ns = 1200LL;
  expect(store(&publication_mailbox, armed_cycle) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a post-arm cycle with the current generation remains publishable");
  expect(publication_mailbox.takeCycleResult(&output) &&
             output.safety.state_id == armed.state_id &&
             output.safety.mode == auto_rover::SafetyMode::kArmed,
         "the first cycle after direct ARMED publication cannot roll back "
         "its generation");
  expect(publication_mailbox.claimCycleSafetyPublication(output.safety),
         "the current post-arm cycle wins its final publication claim");
}

void testEmergencyStopCannotBeFollowedByPendingOrDelayedArmedCycle() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto armed = safetyState(
      20U, auto_rover::SafetyMode::kArmed, 2U, 2000LL);
  const auto estop = safetyState(
      21U, auto_rover::SafetyMode::kEmergencyStopLatched, 3U, 2100LL);

  expect(store(&publication_mailbox, armed) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a pre-E-stop armed cycle enters the publication mailbox");
  expect(publication_mailbox.prepareDirectSafetyPublication(estop),
         "the E-stop transition reserves the latched publication");

  auto_rover_vehicle::VehicleExecutionCycleResult output;
  expect(!publication_mailbox.takeCycleResult(&output),
         "the pending pre-E-stop ARMED cycle is discarded before ESTOP_LATCHED");
  expect(store(&publication_mailbox, armed) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "a delayed ARMED worker result cannot follow the E-stop generation");
  expect(!publication_mailbox.takeCycleResult(&output),
         "the stale ARMED result is never published after E-stop");

  auto estop_cycle = estop;
  estop_cycle.stamp_ns = 2200LL;
  expect(store(&publication_mailbox, estop_cycle) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a current latched E-stop cycle remains publishable");
  expect(publication_mailbox.takeCycleResult(&output) &&
             output.safety.state_id == estop.state_id &&
             output.safety.latch_generation == estop.latch_generation &&
             output.safety.mode ==
                 auto_rover::SafetyMode::kEmergencyStopLatched,
         "only the current latched state follows direct E-stop publication");
  expect(publication_mailbox.claimCycleSafetyPublication(output.safety),
         "the current latched cycle wins its final publication claim");
}

void testResetAndDisarmTransitionsClearEarlierGenerations() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto estop = safetyState(
      30U, auto_rover::SafetyMode::kEmergencyStopLatched, 4U, 3000LL);
  const auto reset_disarmed = safetyState(
      31U, auto_rover::SafetyMode::kDisarmed, 4U, 3100LL);
  const auto rearmed = safetyState(
      32U, auto_rover::SafetyMode::kArmed, 4U, 3200LL);
  const auto operator_disarmed = safetyState(
      33U, auto_rover::SafetyMode::kDisarmed, 4U, 3300LL);

  expect(store(&publication_mailbox, estop) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a pre-reset E-stop cycle enters the mailbox");
  expect(publication_mailbox.prepareDirectSafetyPublication(reset_disarmed),
         "reset-to-disarmed reserves a newer safety generation");
  expect(store(&publication_mailbox, estop) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "a delayed pre-reset E-stop cycle cannot replay after reset");

  expect(publication_mailbox.prepareDirectSafetyPublication(rearmed),
         "a later arm transition advances the publication high-water mark");
  expect(store(&publication_mailbox, reset_disarmed) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "a delayed reset generation cannot replay after re-arm");

  expect(publication_mailbox.prepareDirectSafetyPublication(operator_disarmed),
         "operator disarm reserves its newer safety generation");
  expect(store(&publication_mailbox, rearmed) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "a delayed ARMED cycle cannot follow direct operator disarm");
}

void testDuplicateReceiptAndLateWorkerResultAreNotPublished() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto disarmed = safetyState(
      40U, auto_rover::SafetyMode::kDisarmed, 5U, 4000LL);

  expect(publication_mailbox.prepareDirectSafetyPublication(disarmed),
         "the first direct safety state establishes the high-water mark");
  expect(!publication_mailbox.prepareDirectSafetyPublication(disarmed),
         "an identical direct state is suppressed instead of becoming a "
         "replay with a new receipt");
  expect(store(&publication_mailbox, disarmed) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "an identical delayed cycle is suppressed as a replay");

  const auto late = publication_mailbox.tryStoreCycleResult(
      cycleResult(safetyState(
          41U, auto_rover::SafetyMode::kArmed, 5U, 4100LL)),
      std::chrono::steady_clock::now() - std::chrono::nanoseconds(1));
  expect(late == mailbox::OrderedCycleStoreResult::kLate,
         "a cycle completing after the mailbox deadline remains terminal");

  const auto current = safetyState(
      41U, auto_rover::SafetyMode::kArmed, 5U, 4100LL);
  expect(publication_mailbox.prepareDirectSafetyPublication(current),
         "a late unpublished worker result does not advance the publication "
         "high-water mark");
}

void testTakenCycleIsReclaimedAfterNewerDirectPublication() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto armed = safetyState(
      50U, auto_rover::SafetyMode::kArmed, 5U, 5000LL);
  const auto estop = safetyState(
      51U, auto_rover::SafetyMode::kEmergencyStopLatched, 6U, 5100LL);

  expect(store(&publication_mailbox, armed) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a pre-E-stop ARMED cycle is available to the publication callback");
  auto_rover_vehicle::VehicleExecutionCycleResult taken;
  expect(publication_mailbox.takeCycleResult(&taken),
         "the publication callback can take the old cycle before E-stop");

  expect(publication_mailbox.prepareDirectSafetyPublication(estop),
         "E-stop reserves a newer generation after the old cycle was taken");
  expect(publication_mailbox.claimPreparedDirectSafetyPublication(estop),
         "E-stop wins the serialized direct publication claim");
  expect(!publication_mailbox.claimCycleSafetyPublication(taken.safety),
         "the taken ARMED cycle is rejected when it later reaches the "
         "serialized publisher");
}

void testInvalidAndInconsistentOrderingTuplesFailClosed() {
  mailbox::OrderedCyclePublicationMailbox publication_mailbox;
  const auto valid = safetyState(
      60U, auto_rover::SafetyMode::kDisarmed, 7U, 6000LL);
  auto invalid_state_id = valid;
  invalid_state_id.state_id = 0U;
  auto invalid_stamp = valid;
  invalid_stamp.stamp_ns = 0;

  expect(store(&publication_mailbox, invalid_state_id) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "state_id zero is never accepted into an empty mailbox");
  expect(!publication_mailbox.prepareDirectSafetyPublication(invalid_stamp),
         "a non-positive source stamp cannot reserve a direct publication");

  expect(publication_mailbox.prepareDirectSafetyPublication(valid),
         "a valid state establishes a direct reservation");
  expect(publication_mailbox.claimPreparedDirectSafetyPublication(valid),
         "the valid reserved state can be published once");
  expect(!publication_mailbox.claimPreparedDirectSafetyPublication(valid),
         "a direct reservation cannot be claimed twice");

  auto changed_mode = valid;
  changed_mode.mode = auto_rover::SafetyMode::kArmed;
  changed_mode.stamp_ns = 6100LL;
  expect(store(&publication_mailbox, changed_mode) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "mode cannot change while state and latch identities are reused");

  auto changed_reasons = valid;
  changed_reasons.reasons.push_back(auto_rover::StopReason::kInvalidInput);
  changed_reasons.stamp_ns = 6200LL;
  expect(!publication_mailbox.prepareDirectSafetyPublication(changed_reasons),
         "reasons cannot change while state and latch identities are reused");

  mailbox::OrderedCyclePublicationMailbox clearing_mailbox;
  expect(store(&clearing_mailbox, valid) ==
             mailbox::OrderedCycleStoreResult::kStored,
         "a valid cycle can wait before an invalid current result arrives");
  expect(store(&clearing_mailbox, invalid_state_id) ==
             mailbox::OrderedCycleStoreResult::kStaleDropped,
         "an invalid current cycle is rejected after a valid predecessor");
  auto_rover_vehicle::VehicleExecutionCycleResult output;
  expect(!clearing_mailbox.takeCycleResult(&output),
         "rejecting the current cycle clears an older pending safety result");
}

}  // namespace

int main() {
  testArmTransitionCannotReplayPendingDisarmedCycle();
  testEmergencyStopCannotBeFollowedByPendingOrDelayedArmedCycle();
  testResetAndDisarmTransitionsClearEarlierGenerations();
  testDuplicateReceiptAndLateWorkerResultAreNotPublished();
  testTakenCycleIsReclaimedAfterNewerDirectPublication();
  testInvalidAndInconsistentOrderingTuplesFailClosed();
  if (failures != 0) {
    std::fprintf(stderr, "%d ordered-publication assertion(s) failed\n",
                 failures);
    return EXIT_FAILURE;
  }
  std::puts("ordered SafetyState publication mailbox tests passed");
  return EXIT_SUCCESS;
}
