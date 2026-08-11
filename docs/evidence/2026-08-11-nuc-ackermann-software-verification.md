# NUC Ackermann Phase-1 Software Verification

- Verification date: 2026-08-11 (Asia/Shanghai)
- Branch: `feat/nuc_ackermann`
- Scope: software-only ROS 1 Noetic known-map loop, fake VCU, formal
  default-disabled Wheeltec backend, and PTY transport
- Physical controller access: none
- Result: PASS for the software-only scope described below
- Release status: not a hardware-readiness claim; ADR, dependency, license, and
  physical acceptance approvals remain open

This record applies to the working-tree contents presented for review. The
reviewed squash commit must rerun the same command and bind its output to that
commit before release.

## Verified path and boundary

The verified path is:

```text
synthetic FAST-LIVO2 nav_msgs/Odometry
  -> explicit source_T_rear conversion
  -> EgoState
  -> strict versioned RoutePlan YAML
  -> deterministic Trajectory
  -> Pure Pursuit MotionReference
  -> SafetySupervisor + CommandGuard
  -> VehicleMotionManager
  -> in-process VehicleBackend selection
       -> FakeVcu, or
       -> WheeltecSerialRuntime -> guarded serial transport
  -> normalized ChassisState feedback
```

The graph uses a non-identity fixed extrinsic in the fake scenario and never
queries or broadcasts TF. The default launch remains actuation-disabled. The
Wheeltec package now contains a formal ROS execution node, but all seven
physical/readiness gates default false and its device path defaults empty. Its
ROS-free codec, parser, transport, watchdog/runtime, receive-only capture,
raised-bench tools, backend integration, and pseudo-terminal harness were
compiled and tested. No command was sent to a physical VCU and no real
controller path was opened by this verification run.

Physical command-channel opening is additionally frozen by a compile-time
release gate. No YAML value, launch argument, or caller opt-in can enable it in
this revision. The formal backend remains fully exercised with injected/PTY
transports, while receive-only capture remains `O_RDONLY`. This freeze follows
the source finding that the installed MCU command parser may retain a partial
11-byte frame across a host close/reopen; a host-complete startup zero is not a
controller acknowledgement or proof of parser alignment.

## Pinned verification environment

| Item | Observed value |
|---|---|
| Host | Ubuntu 20.04.6 LTS, x86_64 |
| ROS | ROS 1 Noetic |
| Compiler | GCC/G++ 9.4.0, C++14 |
| CMake/CTest | Ubuntu system 3.16.3 |
| catkin | 0.8.11 |
| yaml-cpp | Ubuntu `0.6.2-4ubuntu1` |
| GoogleTest | Ubuntu `1.10.0-2` |
| NumPy used by rostest | Ubuntu `1:1.17.4-5ubuntu3.1` |

The exact binary revisions are in
[`dependencies/noetic-focal-amd64.lock`](../../dependencies/noetic-focal-amd64.lock).
The runner fixes `PATH=/opt/ros/noetic/bin:/usr/bin:/bin` and
`PYTHONNOUSERSITE=1`; this prevents the user-local CMake 4.x and user Python
packages from changing the Noetic build or test result.

## Reproducible command and results

The complete gate was run from the repository root:

```sh
tools/ci/run_noetic_phase1_checks.sh
```

| Gate | Result |
|---|---|
| Repository policy unit tests | 69/69 passed |
| ROS interface contract-file tests | 4/4 passed |
| ROS wrapper/dependency-boundary tests | 9/9 passed |
| Fake localization math scenarios | 3/3 passed |
| Repository validator, Python compileall, `git diff --check` | passed |
| Deterministic pure-core full loop with ASan+UBSan | 1/1 passed |
| Wheeltec codec/transport/activation/runtime/capture/bench/raw-profile PTY and injected tests with ASan+UBSan | 6/6 passed |
| Isolated Noetic install build | 10/10 packages passed |
| catkin-managed test summary | 30 tests, 0 errors, 0 failures, 0 skipped |
| Exhaustive per-package raw CTest | 19/19 passed |
| ROS fake closed-loop rostest | 1/1 passed, `RESULT: SUCCESS` |

The ROS test exercised both a default-disabled namespace and a separate
explicitly enabled fake-only namespace. It checked arm rejection while disabled,
the first non-zero command below 0.50 m/s, the 0.20 m/s² ramp, rear-axle motion,
fault-stop and fresh re-arm, acknowledged E-stop assertion, generation-preserving
latch, arm rejection while latched, and reset rejection while the reset boundary
is disabled. Expected warning logs were produced by deliberate stale-input and
fake-fault injection.

Additional regressions cover:

- unsupported schemas/enums, non-finite values, invalid masks, gear and reverse
  feedback, yaw normalization, and 180-degree direction mismatch;
- repeated semantic Ego/Chassis samples with a new receipt, source rollback,
  retired process-generation replay, and valid process restart recovery;
- deterministic trajectory identity across planner restart and changed-content
  rejection without a higher `plan_version`;
- tracker process restart from `command_id=1`, genuinely new consecutive-command
  recovery, and retired MotionReference generation replay;
- independent receiver and output watchdog expiry, reconnect revocation,
  authorization epochs, pre-authorization command rejection, and bounded zero
  retries; Wheeltec receipt/cycle monotonic rollback revokes authorization,
  writes no cached motion, and requires a new epoch plus fresh recovery run;
- E-stop latch preservation under invalid clocks/repeated boot calls, current
  health re-evaluation at reset time, exact-generation authorized reset, and no
  transition directly from reset to motion;
- wire quantization boundaries, post-quantization curvature, zero-speed/zero-yaw
  coupling, fragmented/noisy feedback resynchronization, short writes, PTY
  disconnect, and delivery-unconfirmed reporting;
- binary composite `FlagStop` decoding with fail-closed defaults and rejection
  of unsupported byte values; and
- exact integer normal-wire acceleration over successful-write completion to
  next-write-attempt time, including ramp-up, hold, ramp-down, final normal
  zero, failed/partial writes, timing jitter, and the isolated emergency-zero
  exception. An independent exhaustive harness passed 2,516,776 combinations
  over the complete 0..500 raw-speed domain and `5 ms * n +/- 1 ns` boundaries.
- formal-backend startup exact zero before reads, backlog isolation, distinct
  multi-receipt recovery, backend-local `FlagStop` inhibition, signed feedback,
  connection-generation rollback/replay, authorization high-water marks,
  partial-write generation poisoning, write-completion deadlines, shutdown,
  and full-write-without-ACK reporting;
- physical-worker independence from the ROS callback queue, nonblocking result
  mailbox delivery, deadline rechecks before and after core work, callback and
  backend exception rollback, terminal stop before transport destruction, and
  bounded ROS input/identity histories; and
- a public maximum of 4096 trajectory points, 256-byte identifiers, and 1024
  retired-identity entries. Oversize input revokes execution instead of being
  truncated or copied into the physical worker boundary.

## Targeted input-boundary timing sample

A strict GCC 9.4 `-O2 -Wall -Wextra -Wpedantic -Werror` host benchmark repeated
the 4096-point ROS `Trajectory` conversion, core validation, and stored-value
copy 2000 times. It observed a 0.287364 ms mean, 0.308393 ms p95,
0.324079 ms p99, and 0.353727 ms maximum on this NUC. This is useful evidence
that the selected point-count cap bounds one major callback cost, but it is not
a complete worst-case execution-worker budget. Full-cycle timing under callback
contention, scheduler jitter, and physical I/O remains a required physical
readiness measurement; the benchmark does not authorize the real backend.

## Proposed ROS 1 v1 MD5 snapshot

These hashes were generated from a clean isolated install of
`auto_rover_interfaces` using the pinned Noetic `rosmsg md5` and `rossrv md5`
tools. The schemas are proposed and unreleased until ADR 0001 is accepted; after
release a breaking change requires V2 plus an explicit converter.

| Interface | MD5 |
|---|---|
| `msg/ChassisState` | `b65269d1b618bda2a1b2403dc4e375fc` |
| `msg/EgoState` | `298c4589a920d08bb763fd9a72f346c2` |
| `msg/EmergencyStop` | `394726ed703436ea6807a8e1cbc7c4a9` |
| `msg/MotionReference` | `f27328d09aba70b872ff258ff6bfd553` |
| `msg/RoutePlan` | `67bbfb1a01da65bc955a6296a58c27b9` |
| `msg/RouteWaypoint` | `e8b3b8afc46bbd0d63ec5cf8b5a91685` |
| `msg/SafetyState` | `2fe899f1602fc0f1c5c4c550b314bfca` |
| `msg/Trajectory` | `9ca64d76a03bc5f649eed863faed77f4` |
| `msg/TrajectoryPoint` | `287c30617f650daac0ea69b345817b93` |
| `srv/ArmVehicle` | `07983e447813e3c199a207070ce589da` |
| `srv/AssertEmergencyStop` | `2740e92ac38a5bcb77cb1c18afc94e9f` |
| `srv/ReloadRoute` | `cf6ef2f9aeccdfd203075a8b26c28097` |
| `srv/ResetEmergencyStop` | `aee6f0d9982b396494d5bc6aae904f7c` |

## Explicitly unverified or incomplete

The following items are outside this PASS result and remain gates, not assumed
facts:

- The supplied firmware source and raised-bench evidence corroborate the
  candidate command/feedback layout, 115200 8N1 setting, unsolicited 20 Hz
  feedback, encoder scaling, and composite `FlagStop` interpretation for this
  attachment. Installed-firmware identity, physical signs under load, subtype
  and Flash parameters, controller acknowledgement, safe state, braking/hold,
  and reconnect remain `UNVERIFIED`. The candidate firmware defaults to keeping
  the last command after serial loss and performs startup wheel motion.
  Vendor sources also have no usable redistribution license.
- The MCU UART parser's function-static receive count has no inter-byte
  timeout and is not reset by a Linux process or connection generation. The
  release gate must remain compiled off until installed-firmware identity,
  actuator-power-isolated parser resynchronization, durable unclean-session
  handling, and restart tests are reviewed through a new ADR.
- The physical factory now enforces direct no-symlink opening, TTY and
  pre/post-open character-device identity, owner/group/mode, exclusive access,
  USB VID/PID/serial, and serial-setting readback. A formal in-process ROS
  execution backend now composes that transport behind the safety supervisor,
  command guard, and motion manager, with independent monotonic worker timing
  and no public execution-command bypass. It remains default-disabled and
  `UNVERIFIED`; its identity gates are not a deployment allowlist, and a
  disconnect is terminal rather than automatically reopened.
- Live FAST-LIVO2 source-to-publish latency, rate/jitter, one-session world-frame
  behavior, and the physical IMU/source-to-rear-axle extrinsic are unmeasured.
  The real localization configuration therefore remains fail-closed.
- The software E-stop latch is in vehicle-execution process memory. Restart
  revokes authorization and cannot resume motion automatically, but it does not
  preserve proof of a pre-restart assertion. A durable or external latch strategy
  is required before a physical-backend release.
- The 0.50 m/s value is only the commissioning target and initial ceiling. It is
  not a measured deadband or first non-zero command. The historical 0.35 m/s
  post-collision fallback remains risk evidence. The operational minimum radius
  is 0.95 m; the 0.80 m catalogue value remains definition-unknown.
- ADR 0001 is still `Proposed`; dependency/license/security approval and all
  hardware acceptance records remain pending.

Physical handoff is governed by the
[commissioning checklist](../vehicles/nuc_ackermann_commissioning.md), the
[Wheeltec static-source audit](2026-08-10-wheeltec-static-source-audit.md),
[C50C firmware source audit](2026-08-11-wheeltec-c50c-firmware-source-audit.md),
and the repository's
[phase-1 acceptance policy](../testing/phase-1-acceptance.md).
