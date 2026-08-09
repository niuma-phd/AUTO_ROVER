# VCU Adapter Readiness Gate

A VCU adapter is a protocol and vehicle-integration boundary, not a trajectory
controller. No adapter may be described as ready, enabled by default, or used
for a real-vehicle phase-1 acceptance run until every applicable item below has
reviewed evidence. Unknown or unsupported behavior fails closed.

Record the evidence against a named, versioned vehicle profile, VCU hardware
and firmware revision, protocol revision, adapter commit, and test fixture.
Values such as rates, timeouts, limits, and standstill thresholds come from that
evidence; this checklist does not guess them.

## Protocol and transport

- [ ] Identify the physical and logical transport, bus settings, addressing,
  session/enable sequence, message direction, protocol revision, and authority
  for the protocol documentation.
- [ ] Record command and feedback encodings, byte order, scaling, offsets,
  units, signs, ranges, reserved values, checksums or CRCs, rolling counters,
  acknowledgement behavior, and malformed-frame handling.
- [ ] Measure required, minimum, and maximum command rates and feedback rates,
  including startup, reconnect, jitter, and burst behavior.
- [ ] Document the VCU or firmware watchdog, its timeout and safe-state output,
  then verify the adapter's independent receiver/output watchdog and local
  deadline behavior without relying on ROS time alone.

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

Readiness requires all applicable boxes, reproducible evidence, and explicit
review approval. An item may be marked not applicable only with a written reason
and evidence that the capability cannot affect command execution or safety. A
fake VCU can validate the architecture, but it cannot establish that a real VCU
adapter is ready.
