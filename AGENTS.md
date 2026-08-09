# AUTO_ROVER Repository Policy

This policy applies to the entire repository. A deeper `AGENTS.md` may add
stricter rules for its subtree, but it may never relax these rules.

## Phase-1 boundary

Phase 1 targets Ubuntu 20.04 and ROS 1 Noetic only. It proves known-map
navigation: an off-board `RoutePlan` becomes an on-board `Trajectory`, which is
tracked from `EgoState` and `ChassisState` feedback through guarded vehicle
execution. Keep known-map navigation, autonomous exploration, and field
coverage as separate application compositions with their own bringup and
deployment configuration. Do not add a runtime task manager to switch among
them.

Do not implement a phase-1 world model, online obstacle detection, local
avoidance, online exploration, or on-vehicle coverage planning. Do not create
empty ROS packages, speculative VCU adapters, or ROS 2 shells. A ROS 2 port is
a later native implementation, not a parallel phase-1 stack.

## Boundaries and dependencies

Algorithm cores and domain types are free of ROS APIs: no ROS headers, nodes,
publishers, TF listeners, parameter lookup, or ROS logging. Use thin ROS 1
wrappers and explicit conversions between `auto_rover_interfaces` messages and
middleware-independent core types. ROS concerns belong at the graph edge.

Dependency direction is enforced:

- Planning must not depend on a localization implementation or VCU protocol.
- Control must not depend on a VCU protocol.
- A VCU adapter must not contain trajectory tracking.
- Modules must not depend on application bringup or deployment packages.

`/cmd_vel` is only an explicit adapter mapping for a selected VCU contract; it
is never the universal internal command. Every public contract documents frame,
units, timestamp, sign conventions, validity, timeout, and failure semantics.
After a released ROS 1 message, any semantic or field-layout breaking change
requires a V2 type plus converter; do not silently alter its MD5 contract.

Preserve a VCU's existing encoder-level inner loop. The tracker emits
`MotionReference`, the guard and vehicle motion manager produce checked
`VehicleExecutionCommand`, and the adapter alone handles protocol conversion.

## Safety

Required input, capability, freshness, direction, and gear failures fail
closed. Software emergency stop is an independent, latched path: it never
auto-recovers and requires an authorized reset after conditions clear. The
physical hardware E-stop remains an independent safety chain; software must not
replace or weaken it.

Never weaken watchdogs, freshness checks, limits, or safety interlocks merely
to make a test pass.

## Change and release rules

Use `feat/*`, `fix/*`, `exp/*`, or `agent/*` branches. Merge through a reviewed
pull request with relevant test evidence; squash merge is the default. Changes
to public contracts, safety semantics, module boundaries, dependency direction,
or supported platform assumptions require an ADR and compatibility review.

Add or update tests before behavior changes. Before every pull request, run:

```text
python -m unittest discover -s tests/repository -p "test_*.py" -v
python tools/ci/validate_repository.py --root .
python -m compileall -q tools/ci tests/repository
git diff --check
```

Run all relevant package, replay, integration, and vehicle tests in addition to
these repository checks.

Pin the Noetic toolchain, base image, apt dependencies, and external source
revisions. Record external dependencies as reviewed tags or SHAs with license
review. Releases pin component and third-party revisions and preserve their
acceptance evidence.
