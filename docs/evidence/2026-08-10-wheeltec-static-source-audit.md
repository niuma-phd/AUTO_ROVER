# Wheeltec Serial Static-Source Audit

- Audit date: 2026-08-10
- Evidence type: read-only source, configuration, and historical bag metadata
- Physical VCU access: none; `/dev/wheeltec_controller` was absent
- Adapter readiness: `UNVERIFIED`

This audit records observed implementation facts so AUTO_ROVER can create an
original, isolated codec and transport harness. It does not establish the
physical protocol or authorize a real device backend.

## Source identity and license boundary

The two vendor directories listed in the application handoff each contained 519
regular files, no symbolic links, and produced `diff -qr` exit status zero. Their
sorted path/content manifest digest was identical:

```text
33ed872e338eb567d25ca00449e16101d93ac0c50b7c91604184bfa8858e75f9
```

The shared `wheeltec_robot.cpp` SHA-256 was
`2d3eef3ea06e909684e0749dabb02e9df48a328d0c9a8ed1f2f23918b0b0b410`.
The second directory contained no separate JP514 codec or calibration.

Neither vendor tree nor the vdemo package contains a usable license; each
package manifest uses an unresolved license placeholder. Historical bags also
lack redistribution provenance. No vendor code, comments, parser structure, or
bag bytes may be copied into AUTO_ROVER. Only independently implemented protocol
facts are used.

## Candidate command frame

All three source trees independently assemble the same 11-byte candidate frame:

```text
[0]     0x7B
[1..2]  zero; physical meaning unknown
[3..4]  intended signed linear.x * 1000, high byte first
[5..6]  intended signed linear.y * 1000, high byte first
[7..8]  intended signed angular.z * 1000, high byte first
[9]     XOR of bytes 0 through 8
[10]    0x7D
```

Vendor evidence is at `src/wheeltec_robot.cpp:48-86`; the modified copy is at
`src/wheeltec_robot.cpp:54-107` within its respective package. The old code uses
platform `short` and implicit floating conversion without finite/range checks.
AUTO_ROVER instead uses fixed-width integers, explicit toward-zero quantization,
and rejection before conversion. It forces lateral speed to zero and zero speed
to zero yaw rate.

Static host source and Ackermann planner configuration support a software
intention that `angular.z` is body yaw rate and the control board performs
steering conversion. The original firmware was not available during this
audit, so physical meaning, sign, saturation, byte order, and scale remained
`UNVERIFIED` at this evidence point. The later
[C50C firmware source audit](2026-08-11-wheeltec-c50c-firmware-source-audit.md)
corroborates the layout and defines `FlagStop`, while installed-firmware
identity and physical semantics still require vehicle evidence.

## Candidate feedback frame

The source parser treats a 24-byte frame as:

```text
[0]       0x7B
[1]       reserved / Flag_Stop; meaning was unknown in the host source
[2..3]    intended signed forward speed, high byte first, /1000
[4..5]    intended signed lateral speed, high byte first, /1000
[6..7]    intended signed yaw rate, high byte first, /1000
[8..13]   three intended signed accelerometer channels
[14..19]  three intended signed gyroscope channels
[20..21]  intended voltage in millivolts
[22]      XOR of bytes 0 through 21
[23]      0x7D
```

There is no visible VCU timestamp, sequence, gear, steering angle, autonomous
enable, fault, or emergency-stop acknowledgement. AUTO_ROVER marks those values
unavailable. Only a complete, correctly framed and checksummed frame refreshes
local monotonic receipt freshness.

The vendor parser discards a complete 24-byte window after failure rather than
rolling to an embedded header. The modified parser adds another preceding-tail
condition that can discard startup frames. The new parser is an original rolling
stream parser tested against noise, fragmentation, bad checksums, and embedded
headers.

## Transport and watchdog evidence

The sources configure `/dev/wheeltec_controller`, 115200 baud, and a two-second
library timeout, but rely on the ROS `serial` 1.2.1 default 8N1/no-flow settings.
Those defaults are not proof of the VCU's physical contract. Vendor code ignores
write length; vdemo accepts only an exact single-call 11-byte write and does not
complete a short suffix.

vdemo adds a useful monotonic command-silence watchdog, non-blocking receive
polling, and bounded exit zero attempts. It still has a command queue of 100,
one-message recovery, no reconnect/re-arm generation, and no physical delivery
acknowledgement. Its software stop boolean auto-clears rather than latching.
AUTO_ROVER implements independent command and adapter watchdogs, queue-one
latest-value semantics, consecutive-fresh recovery, reconnect inhibit, explicit
arm, full-write handling, and `delivery_unconfirmed` reporting.

Four historical vdemo bags reported `/odom` at approximately 20 Hz. They contain
no raw serial bytes, firmware/protocol revision, source sample time, or reusable
license metadata. The observation is not a feedback-rate contract.

## Vehicle limits and risk evidence

The active vdemo record identifies `senior_akm`, approximately 0.319 m wheelbase,
0.95 m planning radius, and a 0.35 m/s ground-test fallback after a P5 collision.
Other source values of 0.75 m and 0.80 m have conflicting or unknown radius
definitions. AUTO_ROVER uses the approved 0.3187 m initial wheelbase and 0.95 m
runtime minimum radius everywhere. The 0.80 m value is catalogue evidence only.

The old trees contain no measured startup/stop deadband or hysteresis record.
The 0.50 m/s value is therefore only the user-selected, ramp-limited bench target
and initial ceiling. It is not a measured minimum controllable speed, an
automatic small-command uplift, or the first non-zero command.

## Readiness outcome

The real transport is disabled by default and the entire Wheeltec adapter remains
`UNVERIFIED`. Codec, parser, write, watchdog, zero-retry, fake, and PTY evidence
can validate software behaviour only. Physical semantics, rates, safe state,
firmware watchdog, restart/reconnect behaviour, braking/holding, physical
emergency stop, feedback source/resolution, steering signs, and deadbands remain
subject to the repository's VCU readiness gate and user-controlled bench work.
