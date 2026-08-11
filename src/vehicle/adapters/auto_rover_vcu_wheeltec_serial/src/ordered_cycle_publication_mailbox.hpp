#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

#include "auto_rover_vehicle/vehicle_execution_core.hpp"

namespace auto_rover {
namespace wheeltec_serial {
namespace node_internal {

enum class OrderedCycleStoreResult : std::uint8_t {
  kStored = 0,
  kContended,
  kLate,
  kStaleDropped,
};

// The physical execution worker remains the only producer of cycle results,
// while ROS callbacks may publish synchronous safety transitions.  This
// mailbox gives both paths one shared SafetyState ordering high-water mark.
// The worker only ever takes the mutex with try_to_lock; a ROS callback calls
// prepareDirectSafetyPublication() while it still owns the execution-core
// mutex, so the worker cannot be waiting to enter this mailbox at that point.
class OrderedCyclePublicationMailbox final {
 public:
  OrderedCycleStoreResult tryStoreCycleResult(
      auto_rover_vehicle::VehicleExecutionCycleResult result,
      std::chrono::steady_clock::time_point latest_completion) {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return OrderedCycleStoreResult::kContended;
    }
    if (std::chrono::steady_clock::now() >= latest_completion) {
      cycle_result_pending_ = false;
      return OrderedCycleStoreResult::kLate;
    }
    if (!strictlyAdvances(result.safety, reserved_high_water_) ||
        (cycle_result_pending_ &&
         !strictlyAdvances(result.safety,
                           pending_cycle_result_.safety))) {
      // A rejected current cycle must not leave an older result waiting for
      // publication.  Suppression is fail closed; the next orderable cycle
      // may establish a fresh claim.
      cycle_result_pending_ = false;
      return OrderedCycleStoreResult::kStaleDropped;
    }

    pending_cycle_result_ = std::move(result);
    cycle_result_pending_ = true;
    if (std::chrono::steady_clock::now() >= latest_completion) {
      // Do not reserve an ordering generation for a result that will never be
      // published.  A synchronous safety transition may still claim it.
      cycle_result_pending_ = false;
      return OrderedCycleStoreResult::kLate;
    }
    return OrderedCycleStoreResult::kStored;
  }

  // Call this while the same execution-core transition is still serialized by
  // the node's core mutex.  It removes any result computed before that
  // transition and reserves the transition's identity before the direct ROS
  // publish occurs.  A delayed worker result below the reservation is dropped.
  // False means the candidate is an exact replay, rollback, or inconsistent
  // ordering tuple and therefore must not receive a new ROS receipt.
  bool prepareDirectSafetyPublication(
      const auto_rover::SafetyState& safety) {
    std::lock_guard<std::mutex> lock(mutex_);
    cycle_result_pending_ = false;
    if (!strictlyAdvances(safety, reserved_high_water_)) {
      return false;
    }
    reserved_high_water_ = highWaterOf(safety);
    return true;
  }

  // Call only while the node owns its actual SafetyState publication mutex.
  // The candidate must still be the newest reservation and may be claimed at
  // most once.  A later direct transition or cycle makes this return false.
  bool claimPreparedDirectSafetyPublication(
      const auto_rover::SafetyState& safety) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sameOrderingPoint(safety, reserved_high_water_) ||
        !strictlyAdvances(safety, published_high_water_)) {
      return false;
    }
    published_high_water_ = highWaterOf(safety);
    return true;
  }

  // Call only while the node owns its actual SafetyState publication mutex.
  // This second claim closes the interval between taking a cycle from the
  // mailbox and publishing it: a direct transition reserved in that interval
  // makes the old cycle fail here.
  bool claimCycleSafetyPublication(const auto_rover::SafetyState& safety) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!strictlyAdvances(safety, reserved_high_water_) ||
        !strictlyAdvances(safety, published_high_water_)) {
      return false;
    }
    reserved_high_water_ = highWaterOf(safety);
    published_high_water_ = reserved_high_water_;
    return true;
  }

  bool takeCycleResult(
      auto_rover_vehicle::VehicleExecutionCycleResult* result) {
    if (result == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cycle_result_pending_) {
      return false;
    }
    *result = std::move(pending_cycle_result_);
    cycle_result_pending_ = false;
    return true;
  }

 private:
  struct SafetyHighWater {
    bool initialized{false};
    std::uint64_t state_id{0U};
    std::uint64_t latch_generation{0U};
    std::int64_t stamp_ns{0};
    auto_rover::SafetyMode mode{auto_rover::SafetyMode::kBootInhibited};
    std::vector<auto_rover::StopReason> reasons;
  };

  static bool orderable(const auto_rover::SafetyState& safety) {
    return safety.state_id > 0U && safety.stamp_ns > 0;
  }

  static SafetyHighWater highWaterOf(
      const auto_rover::SafetyState& safety) {
    SafetyHighWater high_water;
    high_water.initialized = orderable(safety);
    high_water.state_id = safety.state_id;
    high_water.latch_generation = safety.latch_generation;
    high_water.stamp_ns = safety.stamp_ns;
    high_water.mode = safety.mode;
    high_water.reasons = safety.reasons;
    return high_water;
  }

  static bool sameOrderingPoint(const auto_rover::SafetyState& safety,
                                const SafetyHighWater& high_water) {
    return orderable(safety) && high_water.initialized &&
        safety.state_id == high_water.state_id &&
        safety.latch_generation == high_water.latch_generation &&
        safety.stamp_ns == high_water.stamp_ns &&
        safety.mode == high_water.mode &&
        safety.reasons == high_water.reasons;
  }

  static bool strictlyAdvances(const auto_rover::SafetyState& safety,
                               const SafetyHighWater& high_water) {
    if (!orderable(safety)) {
      return false;
    }
    if (!high_water.initialized) {
      return true;
    }
    if (safety.state_id < high_water.state_id ||
        safety.latch_generation < high_water.latch_generation ||
        safety.stamp_ns <= high_water.stamp_ns) {
      return false;
    }
    if (safety.state_id == high_water.state_id &&
        (safety.latch_generation != high_water.latch_generation ||
         safety.mode != high_water.mode ||
         safety.reasons != high_water.reasons)) {
      // Pure Pursuit treats semantic mutation under a reused state identity
      // as a permanently compromised source.  Never emit such a tuple.
      return false;
    }
    return true;
  }

  // Worker-side pending comparison reads the already-owned state directly;
  // unlike materializing a SafetyHighWater, it cannot allocate while copying
  // the reasons vector inside the watchdog-critical core boundary.
  static bool strictlyAdvances(
      const auto_rover::SafetyState& safety,
      const auto_rover::SafetyState& high_water) {
    if (!orderable(safety) || !orderable(high_water) ||
        safety.state_id < high_water.state_id ||
        safety.latch_generation < high_water.latch_generation ||
        safety.stamp_ns <= high_water.stamp_ns) {
      return false;
    }
    if (safety.state_id == high_water.state_id &&
        (safety.latch_generation != high_water.latch_generation ||
         safety.mode != high_water.mode ||
         safety.reasons != high_water.reasons)) {
      return false;
    }
    return true;
  }

  std::mutex mutex_;
  auto_rover_vehicle::VehicleExecutionCycleResult pending_cycle_result_;
  bool cycle_result_pending_{false};
  SafetyHighWater reserved_high_water_;
  SafetyHighWater published_high_water_;
};

}  // namespace node_internal
}  // namespace wheeltec_serial
}  // namespace auto_rover
