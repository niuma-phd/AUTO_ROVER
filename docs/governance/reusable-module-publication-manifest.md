# Reusable module publication manifest

- Status: approved export plan; no module repository has been created
- Governing decision: [ADR 0005](../adr/0005-apache-2.0-license-and-reusable-module-publication.md)
- Approval evidence: repository-owner decision in issue #12, including its
  historical-author identity confirmation
- Canonical source: `niuma-phd/AUTO_ROVER`
- Initial publication mode: commit-pinned incubation source only
- Tags and releases: prohibited for the initial publication

## Purpose and authority boundary

This manifest defines the bounded source export for the first seven reusable
ROS 1 packages. It authorizes preparation and review of those exports; it does
not create a GitHub repository, transfer release authority, or turn an exported
copy into the canonical source.

Every initial repository must identify one reviewed AUTO_ROVER commit with its
full 40-character Git SHA. The package source and package-specific tests in the
export must be byte-for-byte traceable to that commit. A README, dependency
pin, CI workflow, and package-specific documentation may be added in a clearly
marked export-bootstrap commit, but they must not alter the package behavior or
claim an independent release. Modified files must be identified as required by
Apache-2.0.

Until a later accepted ADR delegates authority, changes are made and reviewed
in AUTO_ROVER first and then republished. The extracted repositories do not
accept divergent package behavior, independent semantic versions, ROS release
tags, binary releases, or vehicle-safety claims. Their `package.xml` version is
source metadata, not evidence of an independently supported release.

The repositories named below are targets only. They do not exist merely
because their names appear in this document, and this manifest must not be used
to infer that a push occurred.

## Common export rules

### Required source record

Before each push, record all of the following in the target repository's
`docs/source-provenance.md` and in the reviewing AUTO_ROVER change:

- the full AUTO_ROVER source commit and branch or reviewed pull request;
- the exact package and test paths selected by this manifest;
- a deterministic digest inventory of every exported package and test file;
- the extraction tool and version, the complete path allowlist, and any path
  renames;
- the filtered source tip, the exact relative-path list of every file the
  bootstrap commit adds or modifies, and the commands that reproduce the
  complete publication tree listing;
- the exact commit pins used for AUTO_ROVER package dependencies;
- the package-specific build and test results; and
- the reviewer and publication operator. The target repository records the
  publication commit and tree symbolically as `git rev-parse HEAD` and `git
  rev-parse HEAD^{tree}`, and records `git ls-tree -r --full-tree HEAD` as the
  command for the complete tree listing. It does not embed its own resolved
  commit SHA, tree SHA, or a digest of `docs/source-provenance.md`; each would
  change the object it names. The resolved commit, resolved tree, complete
  listing, UTC push time, and post-push comparison are written back to the
  canonical AUTO_ROVER publication record.

Export from a fresh read-only mirror of the reviewed commit with a path
allowlist. Do not copy from a dirty working tree, use a denylist as the primary
selection control, or publish unreachable objects, alternate refs, local
branches, stashes, generated build trees, or Git LFS objects outside the
allowlist. Preserving source authorship, dates, and commit messages is the
default; any author-identity rewrite requires a separate recorded review.

The initial export retains the existing monorepo-relative package and test
layout. This preserves the tested `CMakeLists.txt` relative paths and avoids an
export-only source patch. A later layout change must first be made and tested in
AUTO_ROVER or be explicitly marked and reviewed as a modification.

### Files added to every incubation repository

These files are created for the exported repository; they are not an implicit
license to copy unrelated monorepo content:

- `LICENSE`, byte-identical to the AUTO_ROVER Apache License 2.0 text;
- a package-specific `README.md` that names the canonical repository, exact
  source commit and path, dependencies, build instructions, maturity, and the
  absence of tags, releases, warranty, certification, and vehicle acceptance;
- `docs/source-provenance.md` containing the required source record;
- `docs/contracts.md` containing only the package contracts identified below;
- a package-scoped dependency lock containing only the exact compiler, Noetic,
  system-library, and incubation-repository pins used by that package;
- a minimal CI workflow using the pinned Ubuntu 20.04 and ROS 1 Noetic inputs;
  and
- a `NOTICE` file only when a reviewed attribution obligation applies. No
  third-party notice or copyright holder is invented.

Every exported `package.xml` must declare `Apache-2.0`. Dependencies keep their
own licenses. No dependency source is vendored merely to make the exported
repository self-contained.

The assembled tree must verify the root `LICENSE` against the reviewed
AUTO_ROVER SHA-256
`c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4` and
parse the target `package.xml` to require exactly the `Apache-2.0`
declaration. The pre-push publication-record check also requires
the extraction tool and version, complete allowlist, filtered source tip,
source/test digest inventory, bootstrap file list, dependency pins, test and
scan results, operator/reviewer, and a valid UTC verification time. The same
checks run in CI after publication; a green CI result is not a substitute for
the pre-push record.

### Common include allowlist

Each repository's allowlist is exactly the union of:

1. the root `LICENSE`;
2. the package source path listed in its section;
3. the individual package-specific test files listed in its section; and
4. the export-bootstrap files listed above.

An allowlist entry for a directory includes normal tracked files below it but
never symlink targets outside that directory, ignored files, untracked files,
submodules, build products, or alternate Git objects. Shared tests may enter an
export only after they are split in canonical AUTO_ROVER into a
package-specific tracked file; a line-level or ad-hoc copy from a shared test is
not an approved extraction method.

### Common exclusions

The following remain excluded from all seven initial repositories even if they
are Apache-2.0 project-authored material:

- `docs/evidence/**`, raw recordings, bags, maps, captures, analysis output,
  host-, user-, workspace-, or device-specific absolute paths, device
  identities, operator attestations, and field or raised-wheel results;
- `docs/applications/**`, `docs/vehicles/**`, hardware-acceptance results, and
  vehicle commissioning or deployment runbooks;
- `src/apps/**`, `deploy/**`, udev rules, real vehicle profiles, localization
  extrinsics, serial identities, operator identifiers, route files, and launch
  compositions;
- `src/vehicle/adapters/**`, `tools/vehicle/**`, Wheeltec protocol tools,
  firmware-derived diagnostics, and their tests;
- `src/perception/auto_rover_localization/**` and the external FAST-LIVO2
  checkout for this first publication set;
- `tests/integration/**`, `tests/scenarios/**`, `tests/wheeltec/**`, and
  repository-wide tests that require packages outside the target repository;
- vendor firmware, archives, HEX files, copied third-party source, external
  checkouts, or material with unresolved authorship or license metadata;
- binaries, generated ROS message code, compiler output, caches, virtual
  environments, coverage output, sanitizer output, and CI artifacts;
- credentials, tokens, private keys, secrets, host inventory, and Git metadata
  or refs outside the reviewed filtered history; and
- the monorepo README, roadmap, generic CI, deployment dependency lock, and
  governance documents except for narrowly rewritten package-specific material
  required above.

After filtering, scan every reachable object and commit message for excluded
paths, device identifiers, local paths, secrets, binary objects, and unreviewed
third-party material. An allowlist match is necessary but not sufficient for a
push.

Before the first source push, create an active repository ruleset that rejects
tag creation. The source CI intentionally runs only for `main` pushes and pull
requests, so an in-workflow `GITHUB_REF` comparison cannot enforce the no-tag
policy. After every publication push, query GitHub for repository visibility,
rulesets, tags, and releases, and record that the repository is public, the tag
creation rule is active, and no tag or release exists.

## Dependency and publication topology

The package dependency graph has no cycle. The exact internal edges are:

| Consumer | Required AUTO_ROVER packages |
|---|---|
| `auto_rover_core` | None |
| `auto_rover_interfaces` | None |
| `auto_rover_ros1_conversions` | core, interfaces |
| `auto_rover_planning` | core, interfaces, ROS 1 conversions |
| `auto_rover_control` | core, interfaces, ROS 1 conversions |
| `auto_rover_safety` | core |
| `auto_rover_vehicle` | core, interfaces, ROS 1 conversions, safety |

`auto_rover_core` and `auto_rover_interfaces` may be published first in either
order. `auto_rover_ros1_conversions` follows both. Planning and control follow
core, interfaces, and conversions; safety follows core; vehicle follows core,
interfaces, conversions, and safety. This is publication ordering, not release
ordering, and no publication receives a tag.

Each dependent repository must carry a small `.repos` or equivalent CI source
manifest that pins incubation dependencies by full Git commit. Floating
branches are not build evidence. The integration and whole-vehicle tests remain
in canonical AUTO_ROVER and must pass at the source commit before any package
export is approved.

## 1. `auto_rover_core`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-core` (planned, not created) |
| ROS package | `auto_rover_core` |
| Source path | `src/core/auto_rover_core/**` |
| Test allowlist | `tests/core/test_core.cpp` |
| AUTO_ROVER dependencies | None |
| External build dependencies | C++14 toolchain; catkin for the current package build |
| Maturity | Incubation; ROS-API-free domain implementation verified inside AUTO_ROVER, not an independently released general robotics core |

The include allowlist is `LICENSE`, the source path, and the one test file
above. Required documentation describes the middleware-independent domain
types, frames, units, time semantics, validity horizons, monotonic receipt
times, ordering identities, and validation failure behavior.

The README must disclose that the current validation API contains the selected
NUC Phase-1 policy: rear-axle control point, forward-only operation, no reverse,
`0.50 m/s`, `0.20 m/s^2`, and minimum radius `0.95 m`. Publication does not make
that policy a generic Ackermann standard. The repository must not advertise a
ROS-independent build until its package has actually passed a no-catkin
configure, build, install, and test; the present source is ROS-API-free but its
CMake install layout is catkin-oriented.

Build and test acceptance requires an isolated Noetic package build with tests
enabled, execution of `auto_rover_core_tests`, a warning-clean C++14 build, an
ASan/UBSan run of the same unit boundary, package install validation, and
comparison of the exported package/test digest inventory with the recorded
AUTO_ROVER commit.

Package-specific exclusions include every ROS message, conversion, algorithm,
vehicle backend, configuration, route, and cross-package integration test.

## 2. `auto_rover_interfaces`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-interfaces` (planned, not created) |
| ROS package | `auto_rover_interfaces` |
| Source path | `src/interfaces/auto_rover_interfaces/**` |
| Test allowlist | `tests/interfaces/test_ros_contract_files.py` |
| AUTO_ROVER dependencies | None |
| External dependencies | catkin, `geometry_msgs`, `std_msgs`, `message_generation`, `message_runtime` |
| Maturity | Incubation v1 contract source; compatibility-sensitive once independently released, but no independent release exists now |

The include allowlist is `LICENSE`, the source path, and the one Python contract
test above. Generated C++, Python, Lisp, or JavaScript message artifacts are not
source and are excluded.

Required documentation lists all nine messages and four services and states
every field's frame, unit, sign, clock, validity, timeout, identity, and failure
semantics. It must explain that `VehicleExecutionCommand` is deliberately not a
ROS message and `/cmd_vel` is not the internal command contract. It must also
state the v1 rule: a semantic or field-layout break after release requires a V2
type and converter. Publication itself does not assert that a stable release
has occurred.

Build and test acceptance requires Noetic message generation, XML/package
validation, execution of `test_ros_contract_files.py`, verification that the
expected nine message and four service sources are the only exported public
contracts, and recording their generated ROS MD5 values as incubation evidence
without promising future compatibility.

Package-specific exclusions include `auto_rover_core`, conversion source,
generated message output, application configs, and tests that exercise C++
conversions rather than message source contracts.

## 3. `auto_rover_ros1_conversions`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-ros1-conversions` (planned, not created) |
| ROS package | `auto_rover_ros1_conversions` |
| Source path | `src/interfaces/auto_rover_ros1_conversions/**` |
| Test allowlist | `tests/interfaces/test_ros1_conversions.cpp` |
| AUTO_ROVER dependencies | Commit-pinned `auto_rover_core`, `auto_rover_interfaces` |
| External dependencies | catkin, roscpp, `geometry_msgs`, `std_msgs`, GoogleTest for tests |
| Maturity | Incubation; thin, explicit ROS 1 edge verified in AUTO_ROVER, with no independent release cadence |

The include allowlist is `LICENSE`, the source path, and the one C++ conversion
test above. Dependency packages are fetched by commit in CI and are not copied
into this repository.

Required documentation provides a complete core-to-ROS mapping table,
including enum values, duration conversion, timestamp treatment, quaternion
normalization, validity masks, unavailable optional fields, and conversion
failure behavior. It must not claim that conversion establishes freshness;
subscribers record local monotonic receipt time separately.

Build and test acceptance requires an isolated Noetic workspace containing the
two pinned dependency commits, generated interface headers, successful install
of the conversion library and headers, and execution of
`auto_rover_ros1_conversions_tests` with warning-clean C++14 compilation.

Package-specific exclusions include the message definitions themselves, core
implementation, ROS nodes, provider adapters, and integration tests. They are
dependencies or canonical evidence, not exported source for this repository.

## 4. `auto_rover_planning`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-planning` (planned, not created) |
| ROS package | `auto_rover_planning` |
| Source path | `src/planning/auto_rover_planning/**` |
| Test allowlist | `tests/planning/test_planning.cpp`; `tests/ros_wrappers/test_planning_wrapper_contract.py` |
| AUTO_ROVER dependencies | Commit-pinned core, interfaces, and ROS 1 conversions |
| External dependencies | catkin, roscpp, yaml-cpp, GoogleTest |
| Maturity | Incubation known-map planning only; strict static route loading and trajectory generation, not exploration or obstacle avoidance |

The include allowlist is `LICENSE`, the source path,
`tests/planning/test_planning.cpp`, and the canonical planning-only
`tests/ros_wrappers/test_planning_wrapper_contract.py`. The former combined
wrapper test has been deleted after canonical package-specific splitting, so
the export does not copy localization or control tests.

Required documentation defines the strict YAML schema, route identity and
version replacement rules, failed-reload invalidation, Hermite construction,
feasibility probes, sampling, deterministic trajectory identity, and
`STOP_AND_HOLD`. It records the proposed resource envelope from ADR 0006:
1,048,576 YAML bytes before parsing, 64-byte keys and numeric scalars, 210-byte
route IDs, 256-byte frame/profile/generated-trajectory IDs, 2,048 input
waypoints, and 4,096 total output points. A synthetic YAML snippet may be written
for documentation, but no vehicle route or known map is copied from bringup.

Build and test acceptance requires the pinned dependency workspace, yaml-cpp
revision check, execution of `auto_rover_planning_tests`, the package-specific
ROS wrapper contract test, malformed/non-finite/duplicate-key and resource-limit
coverage, install validation, and warning-clean C++14 compilation.

Package-specific exclusions include `waypoints.yaml`, maps, localization,
control, exploration, coverage planning, a runtime task manager, and the
cross-package full-loop test.

## 5. `auto_rover_control`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-control` (planned, not created) |
| ROS package | `auto_rover_control` |
| Source path | `src/control/auto_rover_control/**` |
| Test allowlist | `tests/control/test_pure_pursuit.cpp`; `tests/ros_wrappers/test_control_wrapper_contract.py` |
| AUTO_ROVER dependencies | Commit-pinned core, interfaces, and ROS 1 conversions |
| External dependencies | catkin and roscpp |
| Maturity | Incubation Pure Pursuit tracker; produces desired motion only and has no VCU protocol or final actuation authority |

The include allowlist is `LICENSE`, the source path,
`tests/control/test_pure_pursuit.cpp`, and the canonical control-only
`tests/ros_wrappers/test_control_wrapper_contract.py`. The former combined
wrapper test has been deleted after canonical package-specific splitting. The
formal Wheeltec graph integration remains canonical evidence and is not copied
into this repository.

Required documentation describes dynamic lookahead, curvature sign, goal
slowdown, terminal zero hold, receiver and producer freshness, identity/replay
handling, and the acceleration envelope. It must also document the intentional
`SafetyState` feedback edge: all valid non-armed modes hold zero, the first
armed update establishes a zero ramp origin, an unavailable optional
`CONTROL_ENABLED_VALID` capability is not fabricated, a substantiated false
holds zero, and same-source capability disappearance fails closed.

The README must state that a valid `MotionReference` is not actuation
authorization. Vehicle execution, the command guard, motion manager, selected
backend, hardware E-stop, and physical release gates remain independent.

Build and test acceptance requires the pinned dependency workspace, execution
of `auto_rover_control_core_tests`, the package-specific ROS wrapper contract
test, explicit safety freshness/order/replay and optional-control-enable cases,
ASan/UBSan coverage of the ROS-free core, install validation, and warning-clean
C++14 compilation. The source commit must also have passed the canonical formal
graph regression; that vendor-coupled test is referenced by commit but not
exported.

Package-specific exclusions include vehicle execution, safety implementation,
VCU adapters, `control.yaml`, route files, and all formal or physical Wheeltec
tests.

## 6. `auto_rover_safety`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-safety` (planned, not created) |
| ROS package | `auto_rover_safety` |
| Source path | `src/safety/auto_rover_safety/**` |
| Test allowlist | `tests/safety/test_safety.cpp` |
| AUTO_ROVER dependencies | Commit-pinned `auto_rover_core` |
| External dependencies | catkin; C++14 toolchain |
| Maturity | Incubation software supervisor and command guard; not a functional-safety product or substitute for an independent hardware chain |

The include allowlist is `LICENSE`, the source path, and the one safety test
above.

Required documentation contains the complete supervisor state machine,
recovery counts, explicit arm generation, fault revocation, latched software
emergency stop, authorized generation-matched reset, freshness and validity
checks, command-guard inputs, stop reasons, and restart limitations. It must say
that the latch is process memory in Phase 1, reset never transitions directly
to motion, and neither Apache-2.0 nor passing unit tests establishes safety
certification.

Build and test acceptance requires the pinned core commit, execution of
`auto_rover_safety_core_tests`, ASan/UBSan coverage, invalid time and
configuration cases, every state transition and latch/reset path, install
validation, and warning-clean C++14 compilation.

Package-specific exclusions include the vehicle execution node, FakeVcu,
backend protocol code, operator/deployment configuration, and physical E-stop
or braking claims.

## 7. `auto_rover_vehicle`

| Field | Decision |
|---|---|
| Target repository | `niuma-phd/auto-rover-vehicle` (planned, not created) |
| ROS package | `auto_rover_vehicle` |
| Source path | `src/vehicle/auto_rover_vehicle/**` |
| Test allowlist | `tests/vehicle/test_vehicle.cpp`; `tests/vehicle_ros/test_vehicle_execution_core.cpp`; `tests/vehicle_ros/test_vehicle_execution_node_contract.py` |
| AUTO_ROVER dependencies | Commit-pinned core, interfaces, ROS 1 conversions, and safety |
| External dependencies | catkin, roscpp, `std_srvs`; C++14 toolchain |
| Maturity | Incubation guarded execution and deterministic fake backend; no physical VCU adapter or real-vehicle acceptance is included |

The include allowlist is `LICENSE`, the source path, and the three individual
test files above. The whole `tests/vehicle_ros` directory is not implicitly
allowed if additional tests are later added; the file list must be reviewed
again.

Required documentation defines `VehicleBackend`, `BackendHealth`,
`BackendDelivery`, `BackendFeedback`, internal `VehicleExecutionCommand`,
`VehicleExecutionCore`, `VehicleMotionManager`, FakeVcu behavior, arm/disarm and
emergency-stop services, connection-generation recovery, watchdogs, explicit
zero delivery, and `delivery_unconfirmed`. It must state that the command is an
in-process value, not a ROS topic, and that the fake node is deterministic test
infrastructure rather than hardware acceptance.

Build and test acceptance requires the four pinned AUTO_ROVER dependency
commits, execution of `auto_rover_vehicle_tests`,
`auto_rover_vehicle_execution_core_tests`, and the node contract test,
disconnect/reconnect and watchdog coverage, sanitizer coverage of the core and
fake backend, isolated install validation, and warning-clean C++14 compilation.
The corresponding canonical source commit must also pass the whole-loop tests,
but those tests and their other packages are not exported.

Package-specific exclusions include the entire Wheeltec adapter, serial
transport and codec, physical activation, bench tools, real launch files,
device configuration, udev rules, hardware evidence, and any statement that a
host zero write proves a physical stop.

## Deferred and excluded packages

### `auto_rover_localization` — deferred

Source path `src/perception/auto_rover_localization/**` is not in the first
publication set. The adapter is project-authored, but its named FAST-LIVO2
provider currently has conflicting license metadata: the package manifest says
BSD while the source tree contains GPL-2.0-only text. Process separation over a
ROS message does not resolve redistribution provenance, and the external
checkout is never copied.

Reconsider publication only after the upstream license identity is resolved in
the dependency register, the exact allowed provider relationship is reviewed,
and a localization-specific manifest excludes provider source, recordings,
real extrinsics, maps, and vehicle evidence.

### `auto_rover_vcu_wheeltec_serial` — deferred

Source path `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/**` is not in
the first publication set. Project-originated implementation being
Apache-2.0 does not close the separate protocol provenance, vendor firmware,
installed-firmware identity, parser recovery, post-open USB ownership,
watchdog, independent stop, braking/holding, and physical safety gates. The
vendor archive remains all rights reserved and outside the repository.

Reconsider publication only through a dedicated reviewed manifest after the
license/protocol boundary and required safety evidence are closed. Raw profiles,
device identities, local captures, firmware, bench evidence, and compiled
binaries remain excluded in every case. Public source availability must never
be described as VCU readiness or permission to change the compile-time physical
actuation gate.

### `auto_rover_known_map_bringup` — deployment-specific exclusion

Source path
`src/apps/known_map_navigation/auto_rover_known_map_bringup/**` is an
application composition for the selected NUC Ackermann vehicle, not a reusable
module export. Its vehicle profile, route, localization assumptions, safety
parameters, launch selection, fake scenario, and unverified hardware launch
remain in canonical AUTO_ROVER. A future synthetic tutorial must be authored
and reviewed as a separate example; it is not extracted from the deployment
package by this manifest.

## Per-publication review checklist

No target repository may be pushed until every item below is recorded for that
specific package:

- [ ] The source is a reviewed, clean AUTO_ROVER commit, recorded as a full
  40-character SHA.
- [ ] Only the exact package and test allowlist in this manifest is reachable
  in filtered history.
- [ ] Package source and test digests match the recorded canonical commit.
- [ ] `LICENSE` is the unmodified Apache-2.0 text and `package.xml` declares
  `Apache-2.0`.
- [ ] Applicable notices and authorship are preserved; no third-party or vendor
  source is relicensed or bundled.
- [ ] README and provenance documents state incubation status, canonical
  ownership, source path and commit, dependencies, maturity, and no
  tag/release/safety claim.
- [ ] The structured publication record contains every required field, no
  unresolved sentinel, and no self-referential resolved commit/tree digest.
- [ ] Every AUTO_ROVER dependency is pinned by full target-repository commit,
  and every external dependency matches the reviewed register and Noetic lock.
- [ ] Package-specific build, test, sanitizer where applicable, install, and
  clean-clone checks pass in the pinned environment.
- [ ] The canonical source commit passes the repository and whole-stack checks
  required by `AGENTS.md`.
- [ ] All reachable objects pass secret, binary, local-path, device-identity,
  evidence, and provenance scanning.
- [ ] No Git tag, GitHub release, package index release, binary artifact, or
  claim of independent compatibility or vehicle readiness is created.
- [ ] The GitHub repository is public, its tag-creation blocking ruleset is
  active, and post-push API checks report zero tags and zero releases.
- [ ] The target commit and publication time are written back to the canonical
  review record.

Failure of any item stops that package's publication without relaxing the
others. A successful incubation export does not close a deferred package gate
or change AUTO_ROVER's role as the canonical integration and acceptance source.
