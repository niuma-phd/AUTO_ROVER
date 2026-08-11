# VCU Adapter Readiness Gate

A VCU adapter is a protocol and vehicle-integration boundary, not a trajectory
controller. No adapter may be described as ready, enabled by default, or used
for a real-vehicle phase-1 acceptance run until every applicable item below has
reviewed evidence. Unknown or unsupported behavior fails closed.

Record the evidence against a named, versioned vehicle profile, VCU hardware
and firmware revision, protocol revision, adapter commit, and test fixture.
Values such as rates, timeouts, limits, and standstill thresholds come from that
evidence; this checklist does not guess them.

## Current Wheeltec evidence status (2026-08-11)

The
[expanded raw-profile matrix](../evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md)
now preserves `52/52` requested raised-wheel profiles: twelve straight
60-second holds from `0.50` through `6.0 m/s`, and forty outer-wheel-tier
left/right holds through `5.0 m/s` at R=`2.0 m` and R=`0.95 m`. Every capture
completed its host-side profile and final exact-zero host write, retained
`FlagStop=0`, and had zero parser-integrity counts. The mean yaw signs,
left/right mirror residuals, and derived inner/outer rear-wheel ordering support
the candidate protocol convention. Earlier breaker-open manual mapping
independently supports the rear-encoder channel and sign reconstruction.

This evidence advances mechanism characterization but checks no readiness box
by itself. The executable was an uninstalled, ROS-free raised-bench instrument;
feedback magnitude, tracking, asymmetry, and post-zero tail were observations,
not acceptance gates. A complete serial write is not a VCU ACK, and the
feedback supplies neither command echo nor source time. The exact running
firmware/Flash parameters, controller-side command-loss safe state, active
watchdog, restart/reconnect behavior, braking/hold semantics, release-grade
independent emergency-stop chain, ground-loaded steering/radius behavior, and
deployment acceptance remain unresolved. A standalone formal ROS backend
candidate is now available behind an explicit `UNVERIFIED` launch, but it is
not part of the default fake Phase-1 graph. Real-device, application-actuation,
physical-opt-in, transport-actuation-opt-in, readiness, and approved
external-or-durable E-stop-strategy gates, plus the explicit unverified-protocol
acknowledgement, all default false, and its device path defaults empty. The
candidate therefore remains disabled by default; its existence closes no box
below. In addition, a compile-time release gate rejects every physical
actuation open before device I/O and cannot be overridden by YAML, launch
arguments, or caller opt-ins. The generic codec limit is a required finite
parameter in `0 < v < 6.0 m/s`; the selected Phase-1 profile and launch remain
`0.50 m/s`.

The hard release freeze is required because the reviewed MCU parser retains a
function-static partial 11-byte command across a Linux close/reopen. It has no
inter-byte timeout or host-generation signal, so a complete startup-zero host
write after an unclean exit is not proof that the MCU parsed zero. Removing the
freeze requires a new ADR, read-back identity for the installed firmware, an
actuator-power-isolated parser resynchronization and unclean-restart test, and
the remaining stop/ownership/watchdog evidence below. Receive-only capture
stays byte-level read-only and sends no startup frame.

The formal wrapper runs its serial execution cycle from an independent
monotonic worker rather than the ROS callback queue. ROS publication uses a
single-slot nonblocking mailbox; a missed worker deadline, mailbox contention,
callback/backend exception, or oversized input performs the poison-aware stop
path and terminates the physical node. Trajectories are bounded at 4096 points,
identifiers at 256 bytes, and retired source/generation histories at 1024
entries. A targeted 4096-point conversion/validation/copy benchmark stayed
below 0.354 ms on this NUC, but full worker-cycle timing under scheduler and
callback contention remains an unchecked readiness item rather than a release
claim.

The firmware `FlagStop` bit is consumed only as a current-cycle backend inhibit.
It is not normalized into `ChassisState.control_enabled` or `fault_state`, and
`FlagStop=0` is not an ACK, autonomous-enable confirmation, or proof of a
fault-free controller. Likewise, a complete host write is not a VCU ACK. The
candidate reports acknowledgement unavailable and retains
`delivery_unconfirmed` whenever a stop write cannot be confirmed at the host
transport boundary.

The matrix also provides raised-wheel diagnostic candidates without closing a
readiness item: feedback was approximately `20 Hz`, the largest receipt gap was
`61.257 ms`, and the maximum five-frame pre-motion derived-wheel envelope was
`0.003703 m/s`. The experimental `150 ms` freshness deadline and inclusive
`0.005 m/s` per-wheel standstill envelope are therefore evidence-supported for
this fixture. Receipt-associated onset and post-zero values remain unsuitable
as deadband, controller-delay, braking, or hold guarantees because there is no
VCU source timestamp or ACK.

## Protocol and transport

- [ ] Identify the physical and logical transport, bus settings, addressing,
  session/enable sequence, message direction, protocol revision, and authority
  for the protocol documentation.
- [ ] Record command and feedback encodings, byte order, scaling, offsets,
  units, signs, ranges, reserved values, checksums or CRCs, rolling counters,
  acknowledgement behavior, and malformed-frame handling.
- [ ] Measure required, minimum, and maximum command rates and feedback rates,
  including startup, reconnect, jitter, and burst behavior.
- [ ] Record whether the VCU or firmware provides a watchdog. If present,
  characterize and test its timeout and safe-state output. If absent, record
  that absence, verify the adapter's independent receiver/output watchdog and
  local deadline behavior without relying on ROS time alone, and obtain an
  explicit residual-risk disposition.

## Direction, standstill, and steering

- [ ] Select exactly one supported direction capability in `VehicleProfile`:
  signed-speed direction with no independent gear operation, or confirmed-gear
  direction with a separate request and valid matching feedback.
- [ ] For confirmed-gear direction, document request, acknowledgement, actual
  state, rejection, neutral/park behavior, timeouts, and recovery. Demonstrate
  that stale or missing confirmation prevents motion.
- [ ] Identify the measured-speed source used to confirm standstill, its sign,
  resolution, freshness, validity, and failure behavior. Verify that direction
  change and hold release remain inhibited until the configured standstill and
  capability conditions are met.
- [ ] Define steering input semantics precisely: steering-wheel, road-wheel,
  virtual front-tire angle, curvature, yaw rate, wheel speed, or another audited
  quantity. Record its reference point, frame, units, signs, ratio/conversion,
  limits, saturation, zeroing, calibration, and behavior at zero speed.

## Feedback and control-loop ownership

- [ ] Inventory every available chassis feedback value and its update rate,
  timestamp source, accuracy/resolution, validity indicator, fault behavior,
  and conversion to the normalized `ChassisState` semantics. Mark unavailable
  normalized values invalid instead of fabricating them.
- [ ] Audit existing VCU inner loops, including encoder speed, steering, torque,
  braking, or actuator loops. Preserve a functioning vehicle-level loop when
  the VCU accepts corresponding vehicle-level targets.
- [ ] Identify autonomous-control enable, ready, manual override, disengage,
  preemption, communications timeout, degraded state, fault, and emergency-stop
  indications. Define which states inhibit commands, latch a stop, or require an
  authorized recovery.
- [ ] Document start-up, shutdown, reconnection, sequence reset, partial
  feedback, and VCU-reboot behavior. Demonstrate that no stale command is
  replayed after any transition.
- [ ] Prove how the installed MCU command parser returns to a known frame
  boundary after every partial or unknown host write and across host process
  restart. Perform that proof with actuator power independently isolated and
  define a durable unclean-session disposition; a new host connection
  generation alone is not a parser reset.

## Braking, holding, and independent safety

- [ ] Document how ordinary deceleration, service braking, standstill holding,
  parking behavior, release, and loss of power or communications are requested
  and reported. Define the configured safe state for each failure.
- [ ] Verify command limiting, saturation, invalid-value rejection, expiry, and
  unsupported-command behavior at both the vehicle manager and adapter boundary.
- [ ] Document the physical hardware emergency-stop chain, its effect on the
  VCU and actuators, and the feedback software can observe. Confirm that the ROS
  software path neither replaces nor weakens that independent chain.
- [ ] Verify the independent latched software emergency-stop path, authorized
  reset conditions, and behavior when acknowledgement or feedback is absent.

## Replay and review evidence

- [ ] Capture sanitized, redistributable protocol fixtures for normal operation,
  invalid frames, stale feedback, timeout, enable/override/fault transitions,
  stop, hold, and every supported direction change. Record their provenance and
  license or access restrictions.
- [ ] Add deterministic decode, encode, conversion, watchdog, state-machine,
  saturation, and fail-closed tests using the recorded fixtures or an audited
  simulator.
- [ ] Demonstrate that protocol-specific types and rules remain inside the
  adapter and that no tracking algorithm or vendor field leaks into planning,
  control, or `MotionReference`.
- [ ] Link the reviewed vehicle profile, protocol evidence, threat assessment,
  test results, residual risks, responsible owner, and the versioned phase-1
  acceptance profile.

Readiness requires all applicable boxes and reproducible evidence. The
repository owner, or a vehicle-integration maintainer explicitly delegated by
the owner in the tracking issue, is the readiness decision authority. That
authority and the designated safety reviewer record explicit approval in the
adapter pull request or tracking issue; the evidence record links both
approvals. In a single-maintainer phase they may be the same named person, but
the readiness and safety dispositions remain separately recorded.

An item may be marked not applicable only with a written reason and evidence
that the capability cannot affect command execution or safety. A conditional
item is instead satisfied by evidence of presence or absence, the required
compensating controls, and an approved residual-risk disposition. A fake VCU can
validate the architecture, but it cannot establish that a real VCU adapter is
ready.
