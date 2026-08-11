# Dependency Register

These records accompany the first phase-1 implementation. The source/build
dependencies explicitly marked Accepted below are approved for the bounded
Apache-2.0 incubation publications authorized by
[ADR 0005](../adr/0005-apache-2.0-license-and-reusable-module-publication.md).
That approval does not relicense an external dependency, authorize bundling its
source or binaries, establish deployment security, or establish vehicle
readiness. Entries that remain Proposed or unresolved continue to fail closed.

## Wheeltec C50C firmware evidence artifact

- Status: static evidence only; not a build or runtime dependency
- Name and purpose: vendor-supplied STM32 VCU source/project archive used to
  corroborate the candidate serial layout and interpret raised-bench feedback
- Source: operator-provided local archive
  `/home/nuc_x/AUTO_ROVER/底盘VCU固件/WHEELTEC_C50X_2026.05.29.zip`
- Archive SHA-256:
  `d5ab5e0179736732b0e871090b8f158543c7583ef3a608b6fa01e2f88a80536d`
- Candidate `Akm_Car.hex` SHA-256:
  `8f81854d15725869dd73ae098b3160192ab1c2240db6c8c934fb7ed7d6c03fda`
- License: unresolved; Wheeltec application headers state `All rights
  reserved` and the archive contains no covering top-level license
- Modifications and redistribution: none; the archive is not copied into this
  repository and no vendor implementation is incorporated
- Identity limit: source/project/HEX consistency was inspected offline, but no
  device readback or reproducible build binds the connected VCU to the supplied
  HEX; installed firmware remains `UNVERIFIED`
- Interface fit: protocol facts inform an independently implemented codec,
  parser, and bench tool; they do not enter planning or control
- Safety/security relevance: the reviewed source defaults to retaining the
  last command after serial loss, performs automatic startup wheel motion, and
  does not clear PWM in CPU fault handlers; ground operation remains blocked
- Owner: Wheeltec/vendor ownership unresolved
- Approval evidence: pending repository-owner, license, firmware-identity, and
  hardware-safety review

The full record is in the
[Wheeltec C50C firmware source audit](../evidence/2026-08-11-wheeltec-c50c-firmware-source-audit.md).

## FAST-LIVO2 ROS 1 provider

- Status: Proposed external deployment dependency; source is not redistributed
- Name and purpose: FAST-LIVO2 localization process supplying the provider pose
  consumed by `auto_rover_localization`
- Upstream and source URL: `https://github.com/niuma-phd/fastlivo2-suite`
- Revision: immutable commit
  `3df020182aee52d81fd1a6b543bfb611c46d11bc` on the ROS 1 `main` history
- License: unresolved conflict requiring license review; the FAST-LIVO2 package
  manifest declares BSD while `src/FAST-LIVO2/LICENSE` contains GPL-2.0-only
- Modifications: none in AUTO_ROVER; the provider runs as a separate external
  ROS process and only `nav_msgs/Odometry` crosses the adapter boundary
- Redistribution obligations: AUTO_ROVER redistributes neither its source nor
  its recordings. Any bundled deployment requires a separate resolution of the
  conflicting license metadata and all transitive components.
- Platform fit: the exact Noetic revision was built and exercised by an existing
  offline Ubuntu 20.04 bag regression; live vehicle rate and latency remain
  `UNVERIFIED`
- Interface fit: the adapter accepts only `/aft_mapped_to_init`,
  `nav_msgs/Odometry`, `camera_init`, and `aft_mapped`; provider details do not
  enter planning or control
- Maintenance evidence: the pinned fork commit is dated 2026-08-08; the mutable
  external checkout was already on a ROS 2 branch at audit time, so deployments
  must use a fixed Noetic worktree rather than the current branch
- Safety/security relevance: the pose is a required motion input; source
  timestamp is only publisher ROS time and an unknown rear-axle extrinsic fails
  closed
- Owner: niuma-phd
- Approval evidence: pending repository-owner and license review in the
  implementing pull request

The reproducible source audit is in
[FAST-LIVO2 output audit](../evidence/2026-08-10-fast-livo2-output-audit.md).

## yaml-cpp

- Status: Accepted system build/runtime dependency for the planning source
  incubation; no library or Debian packaging files are redistributed
- Name and purpose: yaml-cpp, used only by the strict versioned waypoint loader
- Upstream and source URL: `https://github.com/jbeder/yaml-cpp`
- Revision: upstream tag `yaml-cpp-0.6.2`, commit
  `562aefc114938e388457e6a531ed7b54d9dc1b62`; deployed Ubuntu package
  `libyaml-cpp-dev=0.6.2-4ubuntu1`
- License: X11/MIT for upstream library files; Ubuntu/Debian packaging metadata
  also records GPL-2.0-or-later for the packaging files themselves
- Modifications: none; linked as the unmodified Ubuntu system library
- Redistribution obligations: preserve the upstream copyright and permission
  notice when distributing the library; review any redistribution of Ubuntu
  packaging files separately
- Platform fit: Ubuntu 20.04 amd64 package; C++14 compile/link probe and strict
  route-loader tests pass with the installed 0.6.2 library
- Interface fit: parsing remains inside `auto_rover_planning`; parsed values are
  converted immediately to the middleware-independent `RoutePlan` and undergo
  independent contract validation
- Maintenance evidence: pinned distribution version is older but remains the
  target-platform package; upgrades require route corpus, duplicate-key,
  non-finite-value, and ABI review
- Safety/security relevance: parses operator-edited input that can command
  motion; a 1 MiB envelope is checked before the pinned parser sees either file
  or string input, and strict key/scalar/identity/waypoint budgets plus invalid
  geometry fail closed as proposed in
  [ADR 0006](../adr/0006-bounded-known-map-planning-resources.md)
- Owner: niuma-phd
- Approval evidence: MIT/X11 system-linking review recorded here and bounded
  publication approval in [issue #12](https://github.com/niuma-phd/AUTO_ROVER/issues/12)

## ROS 1 Noetic build and message runtime

- Status: Accepted pinned target-platform build/runtime dependency set for the
  seven incubation source repositories; no ROS package source or binary is
  copied into those repositories
- Name and purpose: catkin, roscpp, roslaunch/rostest, message generation/runtime,
  and the standard geometry, navigation, and header message packages used only
  by thin ROS 1 wrappers
- Upstream and source URL: ROS repositories referenced by the installed package
  manifests, including `https://github.com/ros/catkin`,
  `https://github.com/ros/ros_comm`, and the ROS common-message repositories
- Revision: exact Ubuntu/ROS apt binary revisions are recorded in
  `dependencies/noetic-focal-amd64.lock`; floating apt candidates are not used as
  evidence
- License: installed manifests declare BSD-family licenses; the lock and target
  image review must retain every package's copyright metadata and transitive
  license obligations
- Modifications: none; system packages are consumed through normal headers,
  libraries, message generation, and command-line tooling
- Redistribution obligations: deployments retain apt/package copyright records;
  any container distribution receives a complete image-level license and source
  obligation review
- Platform fit: Ubuntu 20.04.6 amd64, ROS Noetic, GCC 9.4, system CMake 3.16.3;
  isolated catkin builds and tests are part of the required evidence
- Interface fit: ROS APIs exist only in message/conversion and wrapper targets;
  algorithm cores build without them
- Maintenance evidence: Noetic and Ubuntu 20.04 are outside normal upstream
  support. The deployment must be isolated, threat-assessed, and rebuilt only
  after explicit pin/security review.
- Safety/security relevance: transports localization, route, control, safety,
  and vehicle feedback. Monotonic receiver watchdogs do not rely on ROS time.
- Owner: niuma-phd
- Approval evidence: installed-manifest and source-publication boundary review
  recorded here, with bounded publication approval in
  [issue #12](https://github.com/niuma-phd/AUTO_ROVER/issues/12). Ubuntu 20.04
  and Noetic end-of-life risk remains a deployment-security blocker, not a
  reason to replace the Phase-1 target silently.

## GoogleTest target-platform test library

- Status: Accepted test-only system dependency for incubation CI; not a runtime
  dependency and not redistributed by the source repositories
- Name and purpose: GoogleTest for deterministic package and conversion tests
- Upstream and source URL: `https://github.com/google/googletest`
- Revision: Ubuntu package `libgtest-dev=1.10.0-2`, corresponding to upstream
  release 1.10.0
- License: BSD-3-Clause
- Modifications: none; test binaries link the installed static test library
- Redistribution obligations: test-binary distribution preserves the license
  notice; the library is not part of the vehicle runtime deliverable
- Platform fit: Ubuntu 20.04 amd64 C++14 compile/link probe and catkin tests pass
- Interface fit: test targets only; no public or runtime contract is affected
- Maintenance evidence: target-distribution version is fixed; upgrades require
  rerunning every deterministic and sanitizer test
- Safety/security relevance: none at runtime, but it supplies evidence for
  safety paths
- Owner: niuma-phd
- Approval evidence: BSD-3-Clause test-linking review recorded here and bounded
  publication approval in [issue #12](https://github.com/niuma-phd/AUTO_ROVER/issues/12)

## Incubation publication CI inputs

- Status: Accepted CI-only inputs for the seven source repositories
- Name and purpose: GitHub-hosted Linux runner, `actions/checkout`, and an
  official ROS Noetic/Focal container used to reproduce the target toolchain
- Runner selector: `ubuntu-24.04`; this hosts the job only and is not treated as
  the Phase-1 target platform
- Checkout revision: `actions/checkout` commit
  `3d3c42e5aac5ba805825da76410c181273ba90b1`; floating action tags are not
  permitted
- Container: `ros:noetic-ros-core-focal` amd64 manifest digest
  `sha256:4c1435fd85be3edde3820f0d132ab30a6b030209dce4fa3ac428a2aa5a763caf`;
  tags without this digest are not equivalent evidence
- Container-installed build tool: the pinned digest contains
  `ros-noetic-catkin=0.8.12-1focal.20250426.001935`. Incubation repository
  locks retain and verify that exact container revision; this intentionally
  differs from the NUC target-host lock at catkin `0.8.11` and does not change
  the Phase-1 deployment toolchain.
- Container-installed ROS revisions used across the package-scoped locks are
  fixed as follows; an individual repository retains only the rows required by
  its dependency closure:

  | Package | Exact revision in the pinned container |
  |---|---|
  | `ros-noetic-geometry-msgs` | `1.13.2-1focal.20250426.011953` |
  | `ros-noetic-message-generation` | `0.4.1-1focal.20250426.010337` |
  | `ros-noetic-message-runtime` | `0.4.13-1focal.20250426.011132` |
  | `ros-noetic-roscpp` | `1.17.4-1focal.20250519.225343` |
  | `ros-noetic-rosmsg` | `1.17.4-1focal.20250519.234838` |
  | `ros-noetic-std-msgs` | `0.5.14-1focal.20250426.011621` |
  | `ros-noetic-std-srvs` | `1.11.4-1focal.20250426.011617` |

  These are CI-image facts, not replacements for the independently pinned NUC
  target-host revisions. Every public workflow first verifies package state
  and exact version, installs only genuinely missing packages, then repeats the
  complete exact-version check before building.
- Dependency import tool: Ubuntu/ROS package `python3-vcstool=0.3.0-1`, used
  only in dependent-module CI to materialize the full-commit pins in
  `auto-rover.repos`
- Direct CI utilities: `git=1:2.25.1-1ubuntu3.14`,
  `make=4.2.1-1.2`, and
  `ca-certificates=20240203~20.04.1`; package-scoped locks include only the
  utilities each workflow directly invokes
- License: the checkout action is MIT; the pulled Ubuntu/ROS image contains
  packages under their own recorded licenses. The incubation repositories pull
  the image for CI and do not publish, vendor, or relicense it.
- Modifications and redistribution: none; the workflow installs only the exact
  apt revisions in its package-scoped lock and does not upload a container,
  binary, generated message tree, or package artifact
- Platform fit: all build/test commands run inside Ubuntu 20.04 with ROS 1
  Noetic even though the disposable GitHub runner host is newer
- Security relevance: third-party CI is not trusted with vehicle credentials,
  device access, field evidence, or physical actuation. Workflow permissions
  are read-only, dependencies are full-commit/digest pinned, and no release or
  tag job exists.
- Owner: niuma-phd
- Approval evidence: dependency/digest review recorded here and bounded source
  publication approval in [issue #12](https://github.com/niuma-phd/AUTO_ROVER/issues/12)
