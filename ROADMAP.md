# AUTO_ROVER Roadmap

This roadmap delivers a single, measurable known-map navigation loop before
adding applications or extracting repositories. Phase 1 targets Ubuntu 20.04
and ROS 1 Noetic only.

## 1. Foundation

Establish the monorepo documentation, decision records, ownership rules, pinned
Noetic toolchain, and baseline CI. Keep the repository free of empty ROS and
ROS 2 packages. No public source license is selected yet.

## 2. Contracts and core

Define and test middleware-independent value types and validation rules for
`EgoState`, `RoutePlan`, `Trajectory`, `MotionReference`, `ChassisState`,
`VehicleProfile`, safety state, and emergency stop. Add ROS 1 interfaces and
explicit conversions around the ROS-independent core.

## 3. Fakes, route, and tracker

Build fake localization and VCU integrations plus recorded replay fixtures.
Load an off-board `RoutePlan`, create a feasible `Trajectory`, and track it
from `EgoState` and `ChassisState` to a `MotionReference`. Test stale data,
timeouts, stop behavior, and direction transitions before vehicle integration.

## 4. VCU audit and adapter

Audit the first VCU protocol, feedback, limits, watchdog, and control-loop
ownership. Create the vehicle profile and adapter only after that evidence is
available. The adapter translates checked `VehicleExecutionCommand` to the VCU
and normalizes only substantiated feedback to `ChassisState`.

## 5. Known-map integration

Integrate one selected Ackermann test vehicle: route loading, localization,
trajectory generation, tracking, guarded execution, VCU feedback, and safety
inhibition. Preserve a VCU encoder-level inner loop when one exists.

## 6. Quantitative vehicle acceptance

Run a versioned vehicle/scenario acceptance profile with numeric run count,
route-completion rate, lateral and heading error, command frequency and age,
stale-input response, stopping time and distance, direction-change threshold,
and hold-release conditions. Completion requires real-vehicle evidence, not
unmeasured claims of being fresh, repeatable, or safe.

## Later capabilities — separately designed

Online environment modelling and local avoidance, autonomous exploration,
field coverage, and native ROS 2 each require their own approved design and do
not expand phase 1. They may reuse the established trajectory and vehicle
boundaries but do not justify a runtime task manager in the current system.

## Future repository split — proposals only

AUTO_ROVER remains the integration and acceptance monorepo unless a later ADR
approves extraction. Proposed future names are `auto-rover-interfaces`,
`auto-rover-perception`, `auto-rover-planning`, `auto-rover-control`,
`auto-rover-vehicle`, and optionally `auto-rover-tools` and `auto-rover-safety`.
They are neither created nor committed plans.

An extraction proposal must show a stable interface across at least two system
releases, independent build and test, mostly independent changes, and an
independent consumer, maintainer, or release cadence. A placeholder world model,
a one-VCU integration, or frequently coupled planning/control APIs do not meet
that bar.
