# AUTO_ROVER

AUTO_ROVER is a documentation-first monorepo for a modular autonomous ground
vehicle stack. Its algorithmic architecture is **perception, planning, and
control**; vehicle integration and safety provide the guarded path to the
vehicle.

Phase 1 is deliberately narrow: known-map navigation on Ubuntu 20.04 and ROS 1
Noetic. An off-board tool supplies a `RoutePlan`; the on-board planner turns it into a
`Trajectory`, tracks it using `EgoState` and `ChassisState` feedback, and sends
only guarded commands to the VCU. The core domain and algorithm libraries are
ROS-independent, with explicit ROS 1 conversion and wrapper layers. A native
ROS 2 implementation is a later migration, not a parallel phase-1 stack.

## Read first

- [Architecture](ARCHITECTURE.md) — current system boundaries and safety path.
- [Roadmap](ROADMAP.md) — delivery order and explicitly deferred work.
- [Approved architecture baseline](docs/superpowers/specs/2026-08-10-auto-rover-architecture-design.md) — detailed decisions and rationale.

## Phase-1 boundary

Phase 1 does not include a task manager, world model, online obstacle detection
or local avoidance, autonomous exploration, field-coverage planning on the
vehicle, empty ROS packages, or ROS 2 packages. It also does not assume a
universal VCU protocol or replace a VCU's existing encoder-level control loop.

No public source license has been selected. Do not copy, modify, or redistribute
third-party source into this repository until the licensing decision is recorded.

## Support warning

Ubuntu 20.04 and ROS 1 Noetic are outside normal upstream support. Phase-1
deployments must pin their toolchain, base image, dependencies, and upstream
source revisions, and must account for the resulting security and maintenance
risks. This repository is not a safety certification or a substitute for the
vehicle's independent hardware safety systems.
