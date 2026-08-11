# Wheeltec USB-powered, traction-isolated passive follow-up

- Observation date: 2026-08-11 (Asia/Shanghai)
- AUTO_ROVER base commit: `dfb31716b230cc5777cbd6617426584b81072b99`
- Operator-reported physical state: vehicle traction power off; LiDAR off;
  VCU logic powered from the NUC USB connection
- Serial access: `O_RDONLY` feedback capture only
- Host transmission: none
- Physical-actuation release gate: compiled `false`
- Protocol and installed-firmware status: `UNVERIFIED`

This record covers only work that can be performed without traction power or
perception.  No LiDAR, FAST-LIVO2, localization, ROS vehicle execution, exact
zero, parser scrub, or motion command was started.  The operator's power-state
statement is recorded as an attestation; serial voltage and `FlagStop` are
corroborating observations, not an electrical proof of the external motor bus.

## Pre-open identity and ownership

Immediately before the capture, host enumeration reported:

| Item | Observed value |
|---|---|
| Direct device | `/dev/ttyACM0` |
| Application alias | `/dev/wheeltec_controller -> ttyACM0` |
| Character-device identity | major `166`, minor `0`, inode `613` |
| Owner/group/mode | `root:dialout`, `0660` |
| USB VID:PID | `1a86:55d4` |
| USB serial | `0002` |
| Product/driver | `WCH.CN USB Single Serial`, `cdc_acm` |
| Existing holder | none reported by `fuser` |
| ModemManager | inactive and masked |

The repository udev rule and installed rule remained byte-identical with
SHA-256
`26cdf98a81ec8dae4dab44c602af3a562d35a5c2a0ff2df0131886c1d02bcc1e`.
The USB identity names the WCH bridge only.  It does not name the STM32F407
application image, C50C hardware subtype, ADC-selected vehicle type, or
Flash-overridden parameters.

Two older vendor rules remain installed for alternative `ttyUSB*` and
`ttyCH343USB*` controller enumerations and specify mode `0777`.  Neither matches
the current `ttyACM0`/`cdc_acm` device; `udevadm test` selected the hardened
`wheeltec_controller3.rules` result above.  Removing or hardening the stale
root-owned rules requires administrator credentials that were not available in
this session.  They remain a deployment-hygiene action before treating device
ownership as closed.

## Capture implementation and raw evidence

The ROS-free `wheeltec_feedback_capture` executable was built directly from the
base commit with GCC C++14, `-O2 -Wall -Wextra -Wpedantic -Werror`.  Its
SHA-256 was
`f467edc560ce4595a57546c3827abcb02fafc1335f6790c4347a145ea6c2416a`.
The implementation opens the pinned direct character device `O_RDONLY`, never
constructs an actuation adapter, and its transport rejects all writes.

Relevant source identities at the base commit were:

| Source | SHA-256 |
|---|---|
| `transport.hpp` | `dd681b7c64edf54f1be66af10255e1c5b463327dfeb9ce77367794a798b36c72` |
| `feedback_capture.hpp` | `a31d4f76a74b82c6a72b8ba991662b5b32e111c0e2e5a8c26fe63ffa01cab301` |
| `transport.cpp` | `6e96e98df824281f95ea4e4c5eae9e893708013dcbe8ca3b4fb7e48a0aae1da9` |
| `feedback_capture.cpp` | `af58dae0c1fa184e173041e9d8330c8ac686ff307e125bf06d0591e684c2c12d` |
| `feedback_capture_main.cpp` | `69dac75a508b4aea0a671a2c41764bf87070683778d88e8e03ff29a01ba7f663` |
| `codec.cpp` | `2da72cc54c3ca06621e566ff17d9a0da58d67dcd593e991f268cc05d76eb473e` |
| `stream_parser.cpp` | `f6a36e5e0649793630d15c04f2d2322d7dfa669be09cc6b0c0083753617f8ba2` |

The new mode-`0600` raw NDJSON evidence is stored outside the repository:

```text
/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/power_isolated_passive_30s_20260811T223759.ndjson
```

Its SHA-256 is
`96df1739ea91ab2042c1d7bdeb33ef02c97d2fcf7b7d4f12a013e21052005e7e`.
The file contains the exact raw read chunks, accepted frames, local
`CLOCK_MONOTONIC` receipts, and terminal summary.

## Results

The 30-second capture completed and recorded 14,736 raw bytes and 612 accepted
24-byte frames.  Opening backlog produced as many as nine frames at one receipt.
After the opening burst and two sub-20-ms receipt intervals, 599 live intervals
were `49.716971--50.278975 ms`, mean `49.998498 ms`, consistent with the prior
approximately 20 Hz observation.

| Observation | Result |
|---|---|
| `FlagStop` | `612/612` were `1` (`control_inhibited`) |
| Candidate supply voltage | `4.455--4.456 V` |
| Forward feedback | `-0.001--+0.001 m/s` |
| Lateral feedback | exactly `0.000 m/s` |
| Yaw-rate feedback | `-0.011--+0.011 rad/s` |
| Parser counters | checksum `1`, framing `1`, discarded bytes `48`, trailing bytes `0` |

The low USB-powered voltage and continuous `FlagStop=1` are consistent with the
reported traction-power-off state and the earlier breaker-open captures.  The
speed and yaw envelopes are consistent with previously explained encoder
quantization at standstill.  Bad/opening bytes did not refresh a distinct VCU
source clock because the protocol has no such clock.

A final read-only host enumeration at `2026-08-11T22:58:34+08:00` still resolved
the alias to `/dev/ttyACM0`, observed character device `166:0`, inode `613`,
`root:dialout` mode `0660`, the same USB VID:PID/serial/driver, no holder reported
by `fuser`, and ModemManager inactive and masked.  This final check did not open
the character device and does not extend the firmware or electrical claims.

## Installed firmware and parser disposition

The supplied archive still hashes to
`d5ab5e0179736732b0e871090b8f158543c7583ef3a608b6fa01e2f88a80536d`;
its two candidate `Akm_Car.hex` files hash to
`8f81854d15725869dd73ae098b3160192ab1c2240db6c8c934fb7ed7d6c03fda`.
No ST-Link/SWD device or authenticated firmware-report channel was present.
The 24-byte feedback supplies no version, build hash, subtype, Flash parameters,
command echo, or acknowledgement.  Installed firmware identity therefore
remains open and cannot be closed from this USB serial observation.

No parser-resynchronization bytes were transmitted in this capture.  In
particular, it is not evidence for the ten-`0x00` padding plus exact-zero
sequence specified by
[ADR 0003](../adr/0003-wheeltec-power-isolated-parser-recovery.md).  That
sequence remains software/PTY-only: accepting the ADR does not authorize its
dedicated zero-only physical procedure.  Installed-firmware identity,
traction-energy isolation evidence, and explicit physical-stage authorization
remain prerequisites.  The general physical-actuation compile gate remains
false.

## Remaining boundary

This capture closes the currently available power-off passive observation only.
It does not close installed-firmware identity, external motor-bus isolation,
parser acceptance, unclean restart, command ownership, controller watchdog,
software or physical E-stop durability, braking, holding, ground operation, or
FAST-LIVO2 readiness.  Those items retain their existing fail-closed status.
