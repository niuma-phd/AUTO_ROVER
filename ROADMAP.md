# AUTO_ROVER Roadmap

This roadmap delivers a single, measurable known-map navigation loop before
adding applications or extracting repositories. Phase 1 targets Ubuntu 20.04
and ROS 1 Noetic only.

## 1. Foundation

Establish the monorepo documentation, decision records, ownership rules, pinned
Noetic toolchain, and baseline CI. Keep the repository free of empty ROS and
ROS 2 packages. Apache-2.0 is the selected project license under
[ADR 0005](docs/adr/0005-apache-2.0-license-and-reusable-module-publication.md);
external material remains subject to its own reviewed license.

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

## Reusable-module publication

[ADR 0005](docs/adr/0005-apache-2.0-license-and-reusable-module-publication.md)
authorizes curated, public source repositories for reviewed reusable ROS
packages. AUTO_ROVER remains the canonical integration and vehicle-acceptance
source; an initial package repository is a commit-pinned publication, not an
independent compatibility or safety release.

Every export must include package-specific source, tests, documentation, and
Apache-2.0 licensing, and must exclude vehicle deployment, field evidence,
vendor firmware, copied third-party source, generated artifacts, and unreviewed
material. Independent release authority or divergent development still requires
a later ADR with stable interfaces, independent build and test, and an explicit
maintainer and compatibility policy.
