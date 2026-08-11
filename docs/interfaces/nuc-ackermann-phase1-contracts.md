# NUC Ackermann Phase-1 Contract Semantics

This document defines the first concrete v1 semantics governed by
[ADR 0001](../adr/0001-nuc-ackermann-phase1-contracts-and-safety.md). It is a
compatibility review for the initial ROS 1 Noetic messages and the corresponding
middleware-independent values. It does not establish real-vehicle readiness.

## Shared conventions

- Geometry uses SI units and right-handed coordinates. The world/route frame is
  `camera_init` for one FAST-LIVO2 session. The vehicle control point is the rear
  axle centre with x forward, y left, and z up.
- Yaw is radians about +z and increases counter-clockwise. Longitudinal speed is
  positive forward. Curvature is in 1/m and positive for a left turn.
- Required floating-point values must be finite. Quaternions must have a finite,
  non-zero norm and are normalized before use. Angles are normalized to
  `[-pi, pi]` by conversion and algorithm boundaries.
- A ROS `Header.stamp` uses the clock stated below. `Header.seq` has no semantic
  meaning. Route, trajectory, state, command, request, and latch identities have
  explicit fields.
- Every subscriber records a local monotonic receipt time outside the ROS
  message. Receiver age is calculated only from that monotonic time. Source
  timestamp ordering is checked separately in its declared clock domain.
- A declared `valid_for` is a producer horizon, not a substitute for the
  receiver timeout. A value is usable only while both limits pass.
- Queue depth on motion and execution command paths is one. Consumers implement
  latest-value semantics and never replay queued commands.
- Missing, stale, non-finite, unsupported, out-of-order, frame-mismatched, or
  internally inconsistent required input fails closed and selects an explicit
  stop/inhibit result with a diagnostic reason.

## `EgoState`

`EgoState` describes the rear-axle-centre pose in the route/world frame after the
explicit fixed extrinsic is applied.

| Item | v1 meaning |
|---|---|
| `header.stamp` | Provider-supplied ROS timestamp with its meaning declared by `time_source`. |
| `header.frame_id` | World frame containing the pose; phase-1 configuration requires `camera_init`. |
| `state_id` | Adapter-assigned strictly increasing semantic sample identity within one adapter process generation. |
| `time_source` | `SAMPLE_TIME`, `PUBLISH_TIME`, or `RECEIPT_TIME`; it never implies a stronger meaning than the provider supports. |
| `reference_frame` | Rear-axle control-frame name, configured as `rear_axle_center`. |
| `pose` | World-to-rear-axle pose; metres and quaternion. Covariance validity is a separate bit. |
| `twist` | Rear-axle twist in the reference frame when substantiated; SI units. |
| `valid_mask` | Bits declare pose covariance and twist/twist covariance availability. Pose itself is required. |
| `source_id` | Stable provider/configuration identity combined with a non-empty adapter process generation; it is not a filesystem or credential. |
| `valid` | Whole-state validity at production; receiver freshness is still required. |

The FAST-LIVO2 adapter computes
`world_T_rear = world_T_source * source_T_rear`. It accepts only the configured
`nav_msgs/Odometry` input and expected `camera_init`/`aft_mapped` identities. It
does not query or publish TF. The audited publisher stamps the message with
`ros::Time::now()` at publication, so the converted state uses `PUBLISH_TIME`;
sensor sample time is unavailable. Local monotonic receipt age protects against
a stopped or queued publisher but cannot prove sensor latency. That limitation
remains an explicit real-vehicle readiness item. An unknown extrinsic, invalid
quaternion, frame mismatch, duplicate/regressing provider timestamp, or stale
receipt produces `valid=false`; the adapter never substitutes identity.
Cross-session reuse is unsupported without a separately approved relocalization
mechanism.

The localization node creates its process component from independent wall time,
steady time, and PID before constructing the ROS-free adapter; it is not a YAML
value. Consumers order required ego samples by `(source_id, state_id)`, source
stamp, and local monotonic receipt. A later receipt carrying the same semantic
sample is a replay and compromises that source generation; periodic evaluation
of the exact same cached `Received` snapshot remains usable only until its
original receipt freshness expires. A new, previously unseen `source_id` with a
later receipt establishes an adapter restart and may begin `state_id` at one;
replay from a retired generation fails closed.

## `RoutePlan`

`RoutePlan` is a versioned, off-board/static description in the same frame as
`EgoState`.

| Item | v1 meaning |
|---|---|
| `header.stamp` | File load or producer creation time in ROS time; not used as a monotonic watchdog. |
| `header.frame_id` | Route geometry frame; must equal the configured ego frame. |
| `schema_version` | File and transport schema, exactly 1. |
| `route_id` | Non-empty stable route identity. |
| `plan_version` | Strictly increasing replacement version for a `route_id`. |
| `loop` | Whether completion connects to the first point; phase-1 commissioning configuration is false. |
| `waypoints` | Ordered finite x/y/yaw and non-negative forward target-speed magnitudes. |

The fixed YAML loader rejects an empty list, unknown/duplicate keys, malformed
numbers, non-finite values, duplicate adjacent positions, wrong frame, empty
identity, non-increasing replacement version, a negative speed, speed above the
profile, reverse intent, and geometry whose sampled curvature exceeds the
profile. Reload is transactional: failure invalidates active route execution and
does not keep executing the old plan.

## `Trajectory`

`Trajectory` is a finite executable sample sequence generated from one accepted
route and vehicle-profile revision.

| Item | v1 meaning |
|---|---|
| `header.stamp` | Producer validation/publication time in ROS time; refresh may republish the same immutable generation identity. |
| `header.frame_id` | Same world/route frame as its source route. |
| `trajectory_id` | Explicit immutable generation identity; periodic publication does not change it. |
| `route_id`, `plan_version` | Exact source route identity and version. |
| `vehicle_profile_id` | Exact profile identity used for feasibility. |
| `valid_for` | Producer-declared ROS duration; receiver monotonic timeout is independent. |
| `points` | Ordered pose, accumulated arc length, curvature, forward speed magnitude, and direction. |
| `completion_behavior` | `STOP_AND_HOLD` for this phase-1 profile. |
| `valid` | Generation validation result. Invalid trajectories contain no executable points. |

Arc length is metres and strictly increases after the first point. Direction is
`FORWARD`; reverse values are rejected by the selected profile. Curvature is at
the rear axle and must satisfy `abs(curvature) <= 1 / 0.95`. Point target speeds
must be within `[0, 0.50]`. The final approach uses the controller's configured
stopping envelope and completion threshold; disappearance of the trajectory is
never a stop command.

The route planner republishes the currently validated immutable trajectory with
a refreshed `header.stamp` and unchanged `trajectory_id`. This provides an
explicit producer validity horizon without making the tracker reset progress on
every publication. The ID is a deterministic, non-cryptographic fingerprint of
the normalized route content and version, selected vehicle profile, and
trajectory-generator configuration. The same immutable input therefore retains
its identity across a planner-process restart. Changed content under the same
`route_id` and `plan_version` produces a different ID and is rejected by
consumers because a replacement must increase `plan_version`; the fingerprint is
sequencing identity, not an integrity or authenticity proof. A failed reload
publishes an invalid empty trajectory instead of refreshing the previous
generation.

Planning may reject or construct a larger offline candidate, but the
safety-critical vehicle-execution consumer accepts no more than 4,096 points.
At the selected 0.05 m sample spacing this is approximately 204.8 m of sampled
path. Every frame, route, trajectory, profile, source, producer, request, and
operator identity crossing the execution boundary is limited to 256 bytes;
the per-process retired source/trajectory/route ordering histories are capped
at 1,024 entries each. The ROS wrapper checks these sizes in constant time
before conversion and before taking the execution mutex, and the ROS-free core
rechecks them. Over-limit input is rejected without truncation and revokes
execution. The formal physical wrapper treats it as terminal and exits after
the bounded stop path; a direct core integration remains disarmed and requires
its normal fresh-input recovery plus a new explicit arm.

## `MotionReference`

`MotionReference` is the protocol-independent tracker output in the rear-axle
control frame.

| Item | v1 meaning |
|---|---|
| `header.stamp` | Production time in ROS time. |
| `header.frame_id` | `rear_axle_center`. |
| `command_id` | Strictly increasing tracker command identity within one `producer_generation_id`; a new tracker process starts its own sequence at one. |
| `producer_generation_id` | Non-empty opaque identity generated once by the tracker node for that running process; it remains stable for the process lifetime and changes on restart. |
| `trajectory_id` | Trajectory being tracked. |
| `direction` | `FORWARD` only for this profile. |
| `target_speed_mps` | Non-negative rear-axle speed magnitude, at most 0.50 m/s. |
| `target_curvature_inv_m` | Rear-axle curvature, positive left, magnitude at most `1 / 0.95`. |
| `valid_for` | Producer-declared lifetime. |
| `route_complete` | Tracker has entered terminal stop-and-hold. |
| `valid` | Required inputs and output passed controller validation at production. |

The tracker applies the configured 0.20 m/s^2 acceleration/deceleration ceiling
using injected monotonic elapsed time. Therefore the first non-zero ramp command
is below the 0.50 m/s commissioning target. At completion it continues producing
fresh, valid, explicit zero-speed references while the route remains active.

The command guard orders `MotionReference` by the pair
`(producer_generation_id, command_id)` and by local monotonic receipt time. A
strictly later receipt from a previously unseen producer generation establishes
a tracker restart, retires the prior generation, and restarts the configured
consecutive-fresh recovery run; the first observed command in the new generation
need not exceed the old generation's high-water mark. A generation change in the
same receipt snapshot, a non-increasing command within the active generation, or
any later replay from a retired generation fails closed. A producer generation
is sequencing identity, not operator authorization or an emergency-stop reset.

## `ChassisState`

`ChassisState` contains only normalized feedback substantiated by the selected
backend.

| Item | v1 meaning |
|---|---|
| `header.stamp` | Adapter receipt time in ROS time when the VCU has no source timestamp. |
| `header.frame_id` | Configured vehicle control frame where kinematic values apply. |
| `state_id` | Adapter-assigned strictly increasing receipt identity within one backend process generation; a restarted process may begin again at one only under a new `source_id`. |
| `time_source` | `SOURCE_TIME` or `RECEIPT_TIME`; Wheeltec static evidence supports only receipt time. |
| `measured_speed_mps` | Signed longitudinal rear-axle speed when its validity bit is set. |
| `yaw_rate_radps` | Body yaw rate when its validity bit is set. |
| `steering_tire_angle_rad` | Virtual front-tire angle when substantiated; otherwise invalid. |
| `gear_state` | Normalized gear only when substantiated; otherwise `UNKNOWN` with no validity bit. |
| `control_enabled` | VCU-confirmed autonomous enable only; command publication does not imply it. |
| `fault_state` | Normalized evidence-backed fault state; otherwise `UNKNOWN`. |
| `supply_voltage_v` | Diagnostic voltage when substantiated. |
| `valid_mask` | One documented bit for every optional value. |
| `valid` | Frame integrity and required selected-profile feedback passed at receipt. |
| `source_id` | Stable backend/protocol-evidence identity combined with a non-empty process generation. The fake backend generates its process component from independent wall time, steady time, and PID at node startup; it is not deployment YAML. |

`measured_speed_mps` is feedback, so every finite signed value is preserved and
may pass validation when its validity bit is set. In particular, the adapter and
core do not clamp negative samples or invent a standstill/direction threshold.
The phase-1 forward-only restriction applies to requested motion and execution
commands; it does not redefine the sign or validity of measured feedback.

Only a complete feedback frame with correct framing and checksum refreshes local
receipt freshness. Bad, partial, stale, or disconnected input cannot refresh it.
Unavailable gear, steering, enable, acknowledgement, or source time is invalid;
the adapter does not fabricate it.

For the current Wheeltec candidate, byte 1 `FlagStop` is a non-latched,
current-cycle composite inhibit reported by the firmware. `FlagStop=1` makes
backend actuation not permitted and forces revoke/stop handling. `FlagStop=0`
does not substantiate autonomous-control enable, a normalized fault state,
command acceptance, command echo, or acknowledgement. Consequently neither
value sets the `CONTROL_ENABLED_VALID` or `FAULT_VALID` bits. Backend health and
optional chassis feedback remain separate contracts.

One bounded serial read can contain several complete feedback frames. The
backend publishes at most the newest normalized sample for that read with one
local monotonic receipt time; it does not invent ordering timestamps for frames
that the VCU did not timestamp. Any valid frame carrying an inhibit in the same
read dominates the batch, revokes authorization, and resets the consecutive-
allowed-feedback recovery run even when the newest frame says allowed. Opening
backlog is discarded after the startup-zero write and cannot contribute to
freshness or recovery.

Consumers order `ChassisState` by `(source_id, state_id)`, source stamp, and
local monotonic receipt time. A strictly later sample from a previously unseen
process generation establishes a backend restart and may reset `state_id`.
The prior generation is retired; a later replay from it, a rollback within the
active generation, or an empty `source_id` fails closed. The fake backend also
refuses startup when it cannot establish its non-empty process generation.

## `VehicleProfile`

`VehicleProfile` is schema-versioned deployment data, not a continuously
published ROS message. Version 1 selects `ackermann_bicycle`,
`SIGNED_SPEED_DIRECTION`, `rear_axle_center`, wheelbase 0.3187 m, maximum
forward speed 0.50 m/s, maximum
longitudinal acceleration 0.20 m/s^2, minimum operational radius 0.95 m, and no
reverse. Catalogue/CAD geometry and the 0.80 m catalogue radius remain
`UNVERIFIED`; absent measured values remain null rather than being guessed.

Profile loading is atomic. A missing required value, unknown schema/model/frame,
non-finite or non-positive geometry/limit, inconsistent curvature/radius,
missing/mismatched direction capability, or unsupported capability invalidates
the entire profile and inhibits motion. This profile still rejects negative
speed because `reverse_supported=false`; signed-speed capability identifies the
VCU direction contract and does not itself authorize reverse.

The selected deployment passes the profile's `0.50 m/s` ceiling explicitly to
the serial codec. The reusable codec accepts only a finite caller-supplied
maximum strictly within `0 < v < 6.0 m/s`; its zero default, an omitted value,
a non-finite value, or a value at or above `6.0 m/s` cannot authorize nonzero
motion. Exact zero remains available for the protocol shutdown path. The
exclusive configuration bound is not an application speed authorization and
does not supersede this profile's `0.50 m/s` ceiling.

## `SafetyState` and `EmergencyStop`

`SafetyState` reports one of `BOOT_INHIBITED`, `DISARMED`,
`RECOVERY_INHIBITED`, `ARMED`, `FAULT_INHIBITED`, or `ESTOP_LATCHED`, together
with a latch generation, producer ROS timestamp, declared lifetime, validity,
and explicit reason codes. Its ROS timestamp is diagnostic; the guard records a
monotonic receipt time.

An `EmergencyStop` assertion has a non-empty request and source identity and a
source timestamp. Every valid assertion received by the execution process
latches immediately and increments the latch generation even when the vehicle
is already stopped. The bounded event topic remains available for independent
publishers, while the `AssertEmergencyStop` service is the acknowledged operator
boundary: a caller must receive `success=true` and the resulting latch
generation before claiming that software latched the request. Loss, queueing, or
staleness of either request path cannot clear an existing latch. Reset is a
different service boundary, disabled by default. It requires an enabled
authorization boundary, non-empty operator identity, exact current latch
generation, cleared trigger conditions, fresh required inputs, and a disarmed
vehicle. A rejected reset has no state effect. Successful reset transitions to
`DISARMED`, never directly to motion.

The software path cannot claim or replace the effect of the physical hardware
emergency stop.

The current phase-1 latch is process-memory state. Within a running execution
process no timeout, false topic event, clock fault, repeated boot call, ordinary
fault recovery, or rejected reset clears it. An execution-process restart
revokes all motion authorization and cannot resume motion automatically, but it
does not preserve proof of a pre-restart assertion. Consequently the fake-only
software result does not satisfy a physical deployment's durable-latch gate;
that release requires an approved persistent or external latch strategy and the
independent hardware emergency-stop chain.

## `VehicleExecutionCommand`

This internal vehicle-layer value contains a sequence ID, creation and deadline
in the adapter's monotonic clock, signed longitudinal speed, rear-axle curvature,
motion-enable, hold request, and stop reason. It is produced only after guard and
vehicle-manager validation. It is not a public ROS command and is not received
directly from a tracker.

The VCU backend independently rejects non-finite values, an expired deadline,
non-increasing sequence, negative speed for this profile, lateral velocity,
zero-speed yaw rate, any speed or curvature beyond the profile, and a missing
or invalid explicitly supplied codec limit. Output
watchdog expiry selects an explicit zero/hold frame when the link remains
writable. Before that boundary, the vehicle motion manager independently clamps
every guarded speed request to the 0.20 m/s^2 acceleration/deceleration envelope
using its own monotonic command history; it does not weaken the limit to escape
recovery. A failed or disconnected write reports `delivery_unconfirmed`.

`VehicleExecutionCommand` crosses a protocol-independent, in-process
`VehicleBackend` interface. Backend health separately states configuration
validity, connection, current actuation permission, feedback freshness,
authorization state, connection generation, and delivery uncertainty. A
backend restart or reconnect changes its connection generation, discards prior
authorization and command history, and requires a new explicit arm after the
configured consecutive-fresh recovery run.

Disabled Wheeltec initialization is a valid inhibited state and performs no
device open or transport I/O. The current revision compiles its physical
actuation release gate off, before any open; no parameter may override it. The
prepared future activation path makes an exact-zero host write its first serial
I/O, forbids feedback reads before that completes, drains opening backlog, and
requires fresh post-drain control-allowed receipts before arm. This host-side
sequence is not yet a physical release guarantee: the reviewed MCU parser can
retain a partial command across host reconnect, so removing the compile-time
freeze requires installed-firmware identity and actuator-power-isolated parser
resynchronization evidence through a new ADR. A terminal disconnect never
silently reopens or resumes the prior session; a future remount would use a new
connection generation and still require recovery plus a new arm.

Backend delivery distinguishes a complete local transport delivery from a VCU
acknowledgement. For the current Wheeltec protocol, a complete 11-byte host
write may report local delivery, but `controller_ack_available=false`; it never
sets `controller_acknowledged=true`. A write failure or broken link sets
`delivery_unconfirmed`, including when the backend attempted an exact-zero stop.
The caller must not claim physical stopping from a host write result.

This value is not published as a public ROS command. The selected fake or
Wheeltec backend is injected into the same vehicle-execution wrapper, and the
Wheeltec serial mapping occurs there without a `/cmd_vel` bridge. The formal
Wheeltec launch remains `UNVERIFIED` and default disabled: its real-device,
application-actuation, physical-opt-in, transport-actuation-opt-in, readiness,
approved external-or-durable E-stop-strategy, and explicit
unverified-protocol-acknowledgement gates all default false, and its device path
defaults empty. Local edits to these gates do not constitute the evidence or
approvals required by the readiness checklist, and the compile-time physical
actuation freeze prevents them from opening a command channel in this revision.

## Compatibility review

The v1 messages expose semantic IDs instead of relying on `Header.seq`, retain
provider/vendor data behind adapters, and represent optional feedback with
validity masks. Cores use the same meanings without ROS types. No pre-existing
released AUTO_ROVER schema is changed.

After release, field-layout or semantic breaks require V2 types and explicit
converters. A new optional `ChassisState` meaning requires a reserved validity
bit, evidence from a concrete adapter, conversion tests, and review of every
consumer. Safety weakening, clock-domain changes, reference-point changes, or
new direction capability require a new ADR and safety/compatibility review.
