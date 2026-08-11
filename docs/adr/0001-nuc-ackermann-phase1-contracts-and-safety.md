# ADR 0001: NUC Ackermann phase-1 contracts and safety boundary

- Status: Proposed
- Date: 2026-08-10
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: Pending review of the implementing pull request

## Context

The repository baseline names the phase-1 contracts and guarded command path,
but intentionally does not freeze ROS message fields or vehicle-specific safety
semantics. The first implementation must connect an external FAST-LIVO2 pose to
an Ackermann vehicle without leaking ROS or a vendor protocol into algorithm
cores. The available Wheeltec sources are static implementation evidence only;
the physical protocol, feedback timing, steering semantics, and hardware safe
state have not been verified on the target VCU.

The audited ROS 1 FAST-LIVO2 publisher emits `nav_msgs/Odometry` on
`/aft_mapped_to_init` with world frame `camera_init` and child `aft_mapped`, but
writes `ros::Time::now()` at publication. The stamp is not a sensor sample
timestamp. Pose composition in the implementation is consistent with an IMU
reference point, not a rear-axle reference, although conflicting source comments
prevent treating that inference as a measured extrinsic.

The selected NUC vehicle uses a rear-axle-centre control point. Its sensor-to-
control-point extrinsic has not yet been measured. Phase 1 is forward-only. The
commissioning target and initial speed ceiling are 0.50 m/s, but that value is
neither a measured deadband nor the first non-zero ramp command. The only
approved operational minimum turning radius is 0.95 m; a 0.80 m catalogue value
has an unknown reference definition and remains unverified.

## Decision

The v1 semantic contracts are defined in
[NUC Ackermann phase-1 contracts](../interfaces/nuc-ackermann-phase1-contracts.md).
Middleware-independent values live in `auto_rover_core`. ROS 1 Noetic messages
live in `auto_rover_interfaces`, and every mapping is explicit in
`auto_rover_ros1_conversions`. Algorithm cores receive values, configuration,
and injected monotonic time; they contain no ROS API.

Localization numerically applies the configured rigid transform in this order:

```text
world_T_rear_axle = world_T_fast_livo_reference * fast_livo_reference_T_rear_axle
```

The adapter neither queries nor broadcasts TF. The transform must be explicitly
marked known and contain a finite translation and normalized quaternion. An
unknown, invalid, mismatched, out-of-order, or stale source produces an invalid
`EgoState` and inhibits motion. An identity transform is not an implicit
fallback.

`EgoState` identifies whether its ROS stamp is a sample, publish, or receipt
time. The audited FAST-LIVO2 path uses `PUBLISH_TIME`; sensor sample time remains
unavailable. Receiver freshness always uses local monotonic receipt time. The
unmeasured sensor-to-publish latency is a commissioning/readiness limitation and
cannot be hidden by the recent publish stamp.

The selected profile is an Ackermann bicycle model referenced at the rear axle,
with x forward, y left, z up, speed in m/s, curvature in 1/m, and positive
curvature turning left. Reverse is unsupported and every negative motion request
fails closed. The initial forward-speed ceiling is 0.50 m/s, the longitudinal
acceleration ceiling is 0.20 m/s^2, and the runtime curvature magnitude may not
exceed `1 / 0.95 m`. Planning, control-output validation, the command guard, and
the adapter boundary all enforce the approved radius independently where the
quantity is available.

`MotionReference` remains protocol-independent. The guarded vehicle layer emits
a middleware-independent `VehicleExecutionCommand`. Only the selected adapter
may map it to an external command. The provisional Wheeltec mapping is
`linear.x = signed_speed`, `linear.y = 0`, and
`angular.z = signed_speed * curvature`; zero speed forces zero yaw rate. This
mapping and the serial protocol remain `UNVERIFIED` and disabled for physical
devices by default.

The vehicle-execution core owns a protocol-independent `VehicleBackend`
boundary and receives the selected backend in process. The fake and Wheeltec
implementations use that same boundary; selecting the Wheeltec implementation
does not add a ROS command hop. `VehicleExecutionCommand` remains an internal
value, and neither `/cmd_vel` nor a public execution-command topic is inserted
between the guard and the selected backend. The fake and physical candidates
have separate launch files so choosing the physical protocol is an explicit
deployment action rather than a runtime task-mode switch.

The formal execution wrapper is driven by an independent monotonic worker,
not by the ROS callback queue. ROS publication is decoupled through a
latest-result mailbox, and a missed worker deadline, callback exception, or
mailbox failure first invokes the poison-aware bounded stop path and then
terminates the node. To bound work that can contend with that worker, the
execution consumer accepts at most 4,096 trajectory points, at most 256 bytes
per frame/identity/operator string, and at most 1,024 retired identities in
each ordering history. The ROS wrapper checks array and string lengths before
conversion or taking the execution-core mutex; the ROS-free core enforces the
same limits. At the selected 0.05 m planner sampling resolution, 4,096 points
cover about 204.8 m of sampled path. Exceeding any execution budget revokes
authorization and is not silently truncated; the formal physical wrapper then
selects its terminal stop-and-exit path.

For the candidate Wheeltec feedback, firmware `FlagStop` is represented only as
a current backend actuation-inhibit fact. `FlagStop=1` revokes backend
authorization and selects the stop path. `FlagStop=0` is not proof of an
autonomous-control enable, a fault-free state, command acceptance, or command
acknowledgement. It therefore does not populate `ChassisState.control_enabled`
or `ChassisState.fault_state`; those optional fields remain unavailable. A
complete host write similarly proves only completion of the host transport
boundary. Backend delivery reports controller acknowledgement as unavailable
unless a future evidence-backed protocol revision supplies it.

The standalone Wheeltec launch and configuration are an `UNVERIFIED`,
default-disabled integration candidate. `real_device_enabled`, application
`actuation_enabled`, `physical_device_opt_in`, transport `actuation_opt_in`,
`readiness_gate_passed`, and
`external_or_durable_estop_strategy_approved`, plus the explicit
`unverified_protocol_acknowledged` acknowledgement, all default false, and the
device path defaults empty. The implementation must fail closed unless every
required gate and pinned physical identity is explicitly supplied. These
booleans record deployment selections; setting them does not itself create
readiness evidence or approve a physical run.

This revision additionally compiles the shared physical-actuation release gate
as false. It is not a parameter and therefore rejects an actuation open before
device I/O even if all deployment booleans are locally changed. Static firmware
review found that the MCU command callback keeps a partial 11-byte receive count
across a host process close/reopen, with no inter-byte timeout. Removing the
compile-time freeze is a later safety decision requiring a new ADR, installed
firmware identity, actuator-power-isolated parser resynchronization and
unclean-session evidence, plus the remaining watchdog/ownership/stop gates.

With the default gates, backend initialization succeeds as an inhibited
software state and performs no device open or transport I/O. The future
identity-pinned activation path is prepared and tested with injected/PTTY
transports: zero-only parser-resynchronization padding followed by an exact-zero
frame is its first serial I/O, then bounded backlog drain and a consecutive
post-drain `FlagStop=0` recovery run precede a new arm.
The compiled release freeze prevents that path from acquiring a physical
actuation fd in this revision. A batch containing any `FlagStop=1` is inhibited
even if its newest frame says allowed. Terminal disconnect does not auto-reopen
or restore authorization.

The reusable serial codec does not embed the Phase-1 speed ceiling. Its caller
must explicitly provide a finite `max_forward_speed_mps` strictly greater than
zero and strictly less than `6.0 m/s`; the default is zero so omission, zero,
non-finite values, and values at or above `6.0 m/s` fail closed for nonzero
motion. Exact zero remains encodable for the independent shutdown path when
motion configuration is invalid. The selected Phase-1 configuration supplies
`0.50 m/s`, and the route, tracker, guard, vehicle manager, and adapter still
enforce that application profile independently. The exclusive `6.0 m/s`
configuration-domain bound is neither a verified vehicle maximum nor an
authorization to widen the Phase-1 profile.

Software starts disarmed and actuation-disabled. The command guard checks each
required input's validity, receiver-monotonic age, declared validity horizon,
semantic identity/order, direction, capability, speed, curvature, and safety
state. The vehicle motion manager independently applies the 0.20 m/s^2
acceleration/deceleration envelope to every guarded reference. The VCU backend
independently checks each execution command's receiver-monotonic age and local
deadline and maintains an output watchdog. Ordinary timeout recovery requires a
configured consecutive run of genuinely new fresh commands. Startup,
disconnect, reconnect, backend restart, and shutdown clear that run, inhibit
output, discard the last command, and require a new explicit arm.

Invalid input and ordinary communication failure continuously select an explicit
zero-speed safe command while a writable link remains. A disconnected link is
reported as `delivery_unconfirmed`; software does not claim the physical vehicle
stopped. Software emergency stop is a separate request path. It latches
independently, never auto-recovers, and can be cleared only after trigger
conditions clear and an enabled authorization boundary approves a generation-
matched reset. The physical emergency-stop chain remains independent.

The fixed route edit entry is
`src/apps/known_map_navigation/auto_rover_known_map_bringup/config/waypoints.yaml`.
Reload is explicit and transactional. A failed reload inhibits route execution
and does not continue the previously active route.

## Alternatives considered

- Treating the FAST-LIVO2 pose as the rear axle was rejected because it silently
  changes the control reference point and makes curvature tracking inconsistent.
- Runtime TF lookup or a fabricated TF tree was rejected for this application;
  an explicit versioned transform is sufficient and exposes missing calibration.
- A universal `/cmd_vel` command was rejected because its steering semantics are
  VCU-specific and it would bypass the guarded vehicle contract.
- Raising small positive commands to 0.50 m/s was rejected because it breaks the
  acceleration and stop contracts and mistakes a commissioning target for a
  measured deadband.
- Auto-clearing timeout or emergency-stop state after one fresh message was
  rejected because queued, stale, or transient input could restart motion.
- Copying the vendor package was rejected because the project license gate is
  open and the existing implementation contains unchecked and platform-dependent
  protocol behaviour.

## Consequences

The software loop can be completed against deterministic fake and PTY backends
before hardware access. Real motion remains impossible from the default launch,
and the FAST-LIVO2 adapter remains invalid until the fixed extrinsic is supplied.
The initial controller may command values below 0.50 m/s during acceleration and
deceleration. Bench evidence is required before deadband compensation, physical
serial enablement, timeout acceptance values, or a real-vehicle readiness claim.

The phase-1 software latch is held in the vehicle-execution process. Restarting
that process revokes execution and requires a new explicit arm, so it cannot
resume motion automatically, but it does not provide durable evidence of a
pre-restart software E-stop assertion. A physical-backend release therefore
requires an approved durable or external latch strategy, in addition to the
independent hardware emergency-stop chain; the fake-only result is not evidence
that this deployment item is closed.

Adding a default-disabled physical-backend executable and launch does not close
that release gate. The corresponding parameters remain false and the
non-configurable physical-actuation release constant remains disabled; a local
parameter change is not a substitute for the reviewed strategy and linked
acceptance evidence.

The ROS v1 messages become compatibility-sensitive once released. Optional VCU
feedback is represented by validity bits rather than invented values. Extra
state and configuration are required to preserve monotonic receiver time because
ROS timestamps alone cannot implement either watchdog safely.

## Compatibility and migration

This is the first implementation, so no released AUTO_ROVER ROS message is
migrated. Provider-specific FAST-LIVO2 and Wheeltec messages remain behind
adapters. After release, a semantic or field-layout break requires a V2 type,
explicit converter, compatibility tests, and migration documentation. Additive
optional feedback requires a new documented validity bit and consumer review.

The vehicle profile schema is version 1. A schema break requires a new version
and migration. Measured geometry, deadband, timeout, feedback, and protocol
evidence are added as a new profile revision; historic acceptance criteria are
never edited to fit results.

## Verification

Verification consists of deterministic tests for non-finite values, transforms,
frame and ordering checks, route version/reload validation, trajectory curvature,
Pure Pursuit direction and terminal stopping, acceleration limiting, independent
freshness failures, recovery runs, latched emergency stop and authorized reset,
disconnect/reconnect inhibit, zero-command delivery reporting, fake-VCU closed
loop, codec bounds and resynchronization, partial writes, and PTY disconnects.

The repository checks, pure-core build and tests, Noetic catkin build and tests,
replay/integration scenarios, and default-launch inspection form the software
evidence. Physical readiness remains governed by the
[VCU adapter readiness gate](../vehicles/vcu-adapter-readiness.md) and the
[phase-1 acceptance evidence policy](../testing/phase-1-acceptance.md).
