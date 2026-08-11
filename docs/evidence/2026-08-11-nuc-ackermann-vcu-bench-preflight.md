# NUC Ackermann VCU Bench Preflight

- Preflight date: 2026-08-11 (Asia/Shanghai)
- Branch: `feat/nuc_ackermann`
- Requested scope: test all currently safe non-perception work with the driven
  wheels off the ground
- Explicit exclusions: FAST-LIVO2, LiDAR, camera, localization-node tests, and
  ground motion
- Physical serial access during this preflight: none
- Physical command transmission: none
- Outcome: non-perception software PASS; physical VCU work remains inhibited at
  the device and readiness gates below

This record separates completed software verification from observations about
the newly connected USB serial device. Device enumeration is not protocol,
firmware, actuation, or safe-state evidence.

## Connected-device observations

Read-only operating-system enumeration, without opening the TTY, reported:

| Item | Observed value |
|---|---|
| Application alias | `/dev/wheeltec_controller -> ttyACM0` |
| Stable USB alias | `/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00` |
| Direct character device | `/dev/ttyACM0` |
| Character-device identity | major `166`, minor `0` for this attachment |
| USB VID:PID | `1a86:55d4` |
| USB serial | `0002` |
| Manufacturer/product | `WCH.CN` / `USB Single Serial` |
| USB driver | `cdc_acm` |
| Current owner/group | `root:dialout` |
| Current mode | `0777` |
| Observed holder | none reported by `fuser` at preflight time |

The installed udev rule matches VID, PID, and serial, but sets `MODE:=0777`.
That world-writable policy does not meet the commissioning requirement for an
approved least-privilege device boundary. It must not be silently accepted or
worked around for a qualifying run.

The USB identity identifies the bridge only. The VCU hardware revision,
firmware revision, vehicle mode, and protocol revision remain unknown.

## Non-perception verification result

The following checks were run with the Focal/Noetic system toolchain. No
FAST-LIVO2, LiDAR, camera, localization process, or real VCU process was
started, exercised, or stopped by this task, and no physical serial path was
opened.

| Gate | Result |
|---|---|
| Pinned Focal/Noetic dependency versions | no mismatches |
| Repository policy tests | 50/50 passed |
| ROS interface-file tests | 4/4 passed |
| ROS wrapper static contracts | 9/9 passed |
| Repository validator, compileall, `git diff --check` | passed |
| Pure-core synthetic loop, ASan+UBSan | 1/1 passed |
| Wheeltec codec/parser/watchdog/PTY and receive capture, ASan+UBSan | 2/2 passed |
| Isolated Noetic install build | 10/10 packages passed |
| Selected non-perception package CTest | 10/10 passed |
| Post-change isolated core + Wheeltec install | 2/2 packages passed |
| Post-change Wheeltec package CTest | 2/2 passed |

The selected CTest set covered core, ROS conversions, planning, control,
safety, fake vehicle execution, and the Wheeltec PTY implementation. The
localization unit test, fake FAST-LIVO2 scenarios, and ROS fake closed-loop
rostest were deliberately omitted for this request. The pure-core loop used
only synthetic values and no perception process.

## Why physical execution remains inhibited

The checked-in Wheeltec package is still an `UNVERIFIED` protocol candidate.
Before this task it had no physical CLI or ROS node. This task added only the
receive-only `wheeltec_feedback_capture`; it did not add an actuation CLI or
connect the adapter to bringup. The phase-1 vehicle execution core still
selects only `FakeVcu`. More importantly, the candidate 24-byte feedback frame
contains no substantiated autonomous-control-enable, fault, gear, steering,
sequence, firmware identity, or source timestamp fields. The phase-1 execution
health contract requires valid control-enable and fault evidence. Those unknown
values must not be fabricated as `true` or `OK` merely to arm a vehicle.

The following independent gates also remain open:

- ADR 0001 is `Proposed`, and the hardware acceptance profile is still an
  unapproved template with no named safety disposition or immutable criteria.
- The physical emergency-stop procedure, exclusion zone, spotter, exact VCU
  hardware/firmware, and test window have not been recorded.
- The software E-stop latch is process-memory only and does not preserve proof
  of an assertion across vehicle-execution restart.
- Controller-side watchdog, feedback rate, acknowledgement semantics,
  reconnect safe state, braking/hold behavior, signs, scale, and physical
  protocol layout remain unverified.
- The physical transport has no approved reconnect manager capable of
  preserving replay and authorization history across reopen.

Consequently, wheels-off-ground status reduces mechanical exposure but does
not authorize bypassing required health, identity, E-stop, or protocol gates.

## Receive-only tool and fail-closed device preflight

The new ROS-independent capture path is implemented in
`auto_rover_vcu_wheeltec_serial`. Its physical factory now requires a direct
path, expected major/minor, owner/group, USB VID/PID/serial, `O_NOFOLLOW`, three
consistent path/fd identities, a character TTY, `TIOCEXCL`, one common USB
sysfs ancestor, and a post-configuration 115200 8N1 readback. Feedback-only
access uses `O_RDONLY`; the resulting transport returns `kDisabled` from every
`writeAll`. Read-write access has a separate third actuation opt-in and remains
unused.

The capture session records every raw read chunk before parsing, every valid
frame, local `CLOCK_MONOTONIC` receipts, and parser/rate statistics to a new
mode-0600 NDJSON file. Output creation is exclusive and no-follow, and success
requires `fsync` plus checked close. Injection and PTY tests confirmed that the
capture does not call `writeAll` and that the PTY peer receives no outbound
byte.

A system-call-traced preflight then supplied the observed direct-device
identity while the real device still had mode `0777`. The tool exited `4` with
`invalid_argument`/`EPERM`. The trace contained one `lstat("/dev/ttyACM0", ...
0777 ...)` and **zero** open syscalls for `/dev/ttyACM0`. Thus the current
permission policy was rejected before opening, reading, configuring, or writing
the physical TTY.

## Staged continuation

The first permissible physical software shape is now available as the
receive-only capture tool described above, but the installed `0777` udev policy
and the approval gates still prevent using it on the controller. It does not
create a serial adapter or a motion command. Its `O_RDONLY` path prevents this
tool from transmitting bytes, but the electrical line-state or reset effect of
opening/configuring this USB CDC ACM device is still unknown and therefore
remains physical commissioning work.

Only after receive evidence, exact-zero behavior, independent physical E-stop,
and the applicable readiness decisions are reviewed may a separate bench
command harness be considered. Its first nonzero wire command must be produced
by the 0.20 m/s^2 ramp and be below the 0.50 m/s commissioning target. The
phase-1 ceiling remains 0.50 m/s and the runtime minimum radius remains 0.95 m.
The user's statement that the chassis can reach 6 m/s is recorded only as an
unverified capability statement; it does not raise this commissioning limit.

This preflight is not phase-1 hardware acceptance and does not close any
physical item in the
[VCU readiness gate](../vehicles/vcu-adapter-readiness.md) or
[commissioning handoff](../vehicles/nuc_ackermann_commissioning.md).
