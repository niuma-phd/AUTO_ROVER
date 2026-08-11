# ADR 0004: Pure Pursuit software-safety authorization feedback

- Status: Accepted
- Date: 2026-08-11
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: [Repository-owner acceptance in issue #13](https://github.com/niuma-phd/AUTO_ROVER/issues/13)

## Context

Phase 1 keeps protocol-specific feedback out of trajectory tracking.  The
normalized `ChassisState.control_enabled` field means only a VCU-confirmed
autonomous-control enable, and its validity bit must remain absent when the VCU
protocol cannot substantiate that fact.  In particular, the reviewed Wheeltec
feedback exposes only a composite current-cycle `FlagStop`; neither value of
that flag is an autonomous-enable acknowledgement.

The original Pure Pursuit core nevertheless required
`CONTROL_ENABLED_VALID` on every chassis sample before producing a valid
`MotionReference`.  The formal Wheeltec backend correctly omitted the bit, so
the controller always emitted an invalid reference.  Vehicle execution requires
a fresh valid reference before accepting an arm request.  The real formal graph
was therefore deterministically unable to arm even though its adapter preserved
the intended feedback contract.  Existing adapter integration tests injected a
`MotionReference` directly and did not exercise this feedback edge.

## Decision

Pure Pursuit takes the existing `SafetyState` as a fourth required input beside
`Trajectory`, `EgoState`, and `ChassisState`.  The ROS wrapper records a local
monotonic receipt time on a queue-depth-one `/safety_state` subscription.  The
core independently requires a valid, receiver-fresh, producer-valid, ordered
safety sample.  A missing, stale, invalid, replayed, or rolled-back safety sample
produces an invalid zero reference and resets the speed ramp.  Because
`SafetyState` has no producer-generation field, an ordering compromise remains
fail-closed until the controller process restarts.

Once all required controller inputs pass their existing checks, every valid
non-armed mode (`BOOT_INHIBITED`, `DISARMED`, `RECOVERY_INHIBITED`,
`FAULT_INHIBITED`, and `ESTOP_LATCHED`) produces a valid exact-zero hold
reference and resets the ramp.  The first subsequent `ARMED` update also emits
zero to establish a new ramp origin.  Later motion remains bounded by the
existing Phase-1 `0.20 m/s^2` acceleration limit, `0.50 m/s` forward ceiling,
`0.95 m` minimum radius, and no-reverse profile.

`SafetyState` is the vehicle-execution process's software authorization state.
It is not VCU enable feedback, command acknowledgement, physical emergency-stop
status, or proof of actuator readiness.  The controller applies the optional
VCU enable capability as follows:

- when the active chassis source has never supplied
  `CONTROL_ENABLED_VALID`, an `ARMED` software safety state may establish the
  zero-origin reference ramp;
- when the bit is present, nonzero tracking additionally requires
  `control_enabled=true`; a substantiated false value produces a valid zero
  hold; and
- after one active chassis source has supplied the bit, its later disappearance
  is an invalid capability regression.  A newly accepted chassis process source
  establishes its own capability history and always restarts the ramp at zero.

The final authority remains in `VehicleExecutionCore`, the command guard,
`VehicleMotionManager`, and the selected backend.  They independently check
freshness, backend health, current inhibit state, explicit operator arm,
authorization generation, command recovery, limits, and watchdogs.  A valid
controller reference alone never activates the Wheeltec transport.  This
decision does not change or remove the physical-actuation compile gate.

The formal Wheeltec ROS wrapper has an asynchronous execution worker but must
publish one ordered `SafetyState` stream.  Worker-cycle results and synchronous
arm, disarm, emergency-stop, and reset transitions therefore share one
publication high-water mark.  While a synchronous transition still holds the
execution-core mutex, it discards any pending pre-transition cycle and reserves
the transition's `state_id`, latch generation, source stamp, mode, and reasons
before the ROS publish.  Direct and cycle safety output then uses one actual
publication mutex.  A cycle taken from the mailbox must claim again inside that
mutex against the latest reservation, so a transition that arrives between
mailbox take and ROS publish suppresses the old result.  Unorderable tuples and
semantic mutation under a reused state identity are rejected.  In particular,
a pending or already-taken `ARMED` cycle can never follow a directly published
emergency-stop state, and an earlier inhibited generation cannot be replayed
after arm or reset.  Direct callbacks release the execution-core mutex before
entering the publication mutex.  The worker never enters the publication mutex
and retains its nonblocking `try_to_lock` mailbox boundary; contention and
deadline overrun keep their terminal stop semantics.

## Alternatives considered

- Mapping `FlagStop=0` or host authorization into
  `ChassisState.control_enabled` was rejected because it fabricates VCU feedback
  and weakens the normalized contract.
- Removing the controller's enable check without adding software authorization
  feedback was rejected because a disarmed controller could accumulate a
  nonzero ramp behind the final execution gate.
- Treating missing `CONTROL_ENABLED_VALID` as false was rejected because it
  preserves the permanent arm deadlock for protocols where the capability is
  documented as unavailable.
- Adding a new ROS message field was rejected because the existing
  `SafetyState` already expresses the software authorization state and changing
  a released message layout would require a V2 contract.

## Consequences

The formal Wheeltec graph can generate the valid zero reference required to
recover and arm without claiming unavailable VCU evidence.  Software disarm,
fault, recovery, boot inhibit, and emergency stop become explicit ramp-reset
inputs to the tracker.  A VCU that does provide enable feedback retains the
stricter requirement, including fail-closed capability disappearance.

This introduces a deliberate feedback edge from vehicle execution to control.
Startup remains safe: the execution core publishes an inhibited safety state,
Pure Pursuit returns zero, and only an explicit healthy arm may transition the
software state.  The edge is not a runtime task manager and does not give the
control package a VCU-protocol dependency.

## Compatibility and migration

No ROS message or service field, MD5 contract, domain type, frame, unit, or sign
convention changes.  Deployments must provide the existing `/safety_state` topic
to `pure_pursuit_node` and add the required positive
`safety_freshness_ns` parameter.  The known-map bringup already co-locates the
vehicle execution publisher and controller topic, so no protocol-specific
remapping is added.

Fake-VCU behavior remains compatible because its substantiated
`CONTROL_ENABLED_VALID` bit is still enforced.  The Wheeltec backend remains
`UNVERIFIED`, default-disabled, and compile-time blocked from physical
actuation.

## Verification

Deterministic controller tests cover missing, stale, invalid, expired, replayed,
and rolled-back safety state; all inhibited modes; zero-origin restart after
arm; unavailable VCU enable capability; substantiated false and true enable;
same-source capability disappearance; chassis source generation change; and the
Phase-1 acceleration bound.  ROS wrapper contract tests require the fourth
queue-depth-one subscription, local monotonic receipt, conversion, required
freshness parameter, and unconditional output publication.

A ROS-free ordered-mailbox regression deterministically interleaves a pending
cycle with direct arm, disarm, emergency-stop, and reset transitions.  It must
prove that the pending predecessor is removed, a delayed predecessor is
dropped by the shared high-water mark, an old cycle already taken by the
publisher loses its final claim after a newer direct publication, invalid and
inconsistent ordering tuples fail closed, duplicate receipts are suppressed,
an unpublished late result does not advance the high-water mark, and a current
post-transition cycle remains publishable.  Static wrapper checks additionally
require every direct transition to establish the ordering barrier while the
execution-core mutex is held, release that mutex before the common publication
mutex, and claim the state immediately before its ROS publication.

The pure-core full-loop integration supplies the supervisor's actual
`SafetyState` to Pure Pursuit and retains default-stop, explicit-arm, recovery,
fault, emergency-stop, and re-arm behavior.  A formal Wheeltec graph integration
must additionally prove that feedback with no `CONTROL_ENABLED_VALID` recovers
to disarmed, accepts an otherwise healthy explicit arm, emits zero on the first
armed controller update, and only then ramps under all execution and backend
guards.  All repository, Noetic, replay, integration, and adapter tests remain
required before review.
