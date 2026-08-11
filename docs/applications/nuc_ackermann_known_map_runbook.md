# NUC Ackermann Known-Map Runbook

This runbook covers the ROS 1 Noetic phase-1 composition only. It does not
authorize physical actuation and does not establish Wheeltec VCU readiness.
The default execution launch below selects the deterministic fake VCU. A
separate formal Wheeltec execution node and
`vehicle_execution_wheeltec_unverified.launch` are checked in, but all physical
and actuation gates default to disabled; that launch is not authorized by this
runbook and does not establish hardware readiness.

## Build and software verification

Use the pinned Focal/Noetic toolchain. The repository helper deliberately
removes user-local CMake from `PATH`, checks every installed package revision,
builds all catkin packages in isolation, runs package and repository tests, and
runs the pure-core integration scenario under ASan and UBSan:

```sh
tools/ci/run_noetic_phase1_checks.sh
```

The editable route is fixed at
`src/apps/known_map_navigation/auto_rover_known_map_bringup/config/waypoints.yaml`.
Keep `frame_id: camera_init`, `loop: false`, finite geometry, and speeds within
the selected vehicle profile. A replacement keeps the same `route_id` and must
strictly increase `plan_version`. A failed reload invalidates execution instead
of retaining the previous route.

## Independent process composition

The graph is intentionally split into three application processes. Start the
external pinned ROS 1 FAST-LIVO2 provider separately, then use:

```sh
roslaunch auto_rover_known_map_bringup localization.launch
roslaunch auto_rover_known_map_bringup planning_control.launch
roslaunch auto_rover_known_map_bringup vehicle_execution.launch
```

Localization, Pure Pursuit, and fake vehicle execution create opaque process
generation identities at startup; they are not copied from YAML. A consumer can
therefore accept a fresh process generation whose semantic counter restarts at
one while rejecting replay from a retired generation. Repeated publication of
an old `state_id`/source stamp under a new receipt is not a freshness refresh and
fails closed. The planner instead derives a deterministic trajectory identity
from immutable route/profile/generator content, so restarting it with identical
input does not reset controller progress.

The default real localization configuration is deliberately incomplete:
`extrinsic/known` is false and the live freshness limit is unset. The
localization process therefore fails closed until a measured, reviewed
source/IMU-to-rear-axle transform and live timing limit replace those values.
It performs the explicit multiplication
`world_T_rear = world_T_source * source_T_rear`; no TF tree is queried or
broadcast.

`vehicle_execution.launch` defaults to `actuation_enabled:=false`. Missing or
false actuation configuration cannot be armed. Even when changed to true, this
phase-1 launch controls only the fake VCU and never opens a serial device.

To reload an edited route after increasing `plan_version`:

```sh
rosservice call /reload_route '{}'
```

## Deterministic fake graph

The combined launch uses a non-identity synthetic source-to-rear transform and
the fake VCU:

```sh
roslaunch auto_rover_known_map_bringup known_map_fake.launch
```

With its default `actuation_enabled:=false`, the graph publishes feedback and
safety state but remains at explicit zero/hold. The automated rostest exercises
this default and a separate test-only explicitly enabled run.

For an attended fake-only manual run, opt in at launch, wait for
`/safety_state` to reach `MODE_DISARMED`, note its current `state_id`, and then
arm with a non-empty software-test operator identity:

```sh
roslaunch auto_rover_known_map_bringup known_map_fake.launch actuation_enabled:=true
rostopic echo -n 1 /safety_state
rosservice call /arm_vehicle "{operator_id: 'software-test', safety_generation: 7, arm: true}"
```

Replace `7` with the observed current state ID. The service rejects a stale
generation. To disarm, call the same service with `arm: false`; disarm does not
clear a latched emergency stop.

The fake-only fault services are `/fake_connected`, `/fake_faulted`, and
`/fake_drop_feedback`, each using `std_srvs/SetBool`. Any injected fault or
disconnect revokes authorization. Clearing it begins consecutive-fresh
recovery and returns only to `DISARMED`; a new explicit arm is required.

Software emergency stop accepts independent events on the `/emergency_stop`
topic, but an operator should use the acknowledged `/assert_emergency_stop`
service and require a successful response with the new latch generation. The
separate `/reset_emergency_stop` service is disabled by default. A false topic
message cannot clear a latch. When reset is explicitly enabled for a test it
requires the configured operator identity, exact latch generation, acknowledged
cleared conditions, and the configured run of fresh inputs. Successful reset
returns to `DISARMED`, never directly to motion.

## Operational invariants

- `camera_init` is the session-scoped route/world frame and
  `rear_axle_center` is the vehicle control point.
- The commissioning target and software ceiling are 0.50 m/s. The first actual
  nonzero command is acceleration-limited and lower than that target.
- The initial acceleration/deceleration ceiling is 0.20 m/s² and the only
  approved operational minimum turning radius is 0.95 m.
- Reverse is unsupported. Unknown direction, required feedback, freshness,
  frame, capability, or configuration fails closed.
- The guard and backend use independent monotonic watchdogs. ROS timestamps
  supply producer validity and diagnostics, not receiver age.
- Route completion continues publishing a fresh explicit zero/hold reference;
  topic disappearance is never interpreted as a stop command.
- A software stop cannot replace the independent physical emergency-stop chain.

Physical work starts only through the separate
[commissioning checklist](../vehicles/nuc_ackermann_commissioning.md).
