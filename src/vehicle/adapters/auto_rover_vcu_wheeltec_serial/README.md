# Wheeltec serial candidate adapter

The entire package is **UNVERIFIED**. It is an independently written,
ROS-independent codec, stream parser, transport primitive, adapter-level
watchdog, formal session runtime, and vehicle-backend boundary derived from
static source observations. The later review of the
operator-provided `WHEELTEC_C50X_2026.05.29` firmware tree strengthens the
candidate byte-layout interpretation, but no source-to-flashed-binary identity
has been established. This package is therefore not evidence that the attached
controller runs that source, uses the assumed vehicle mode, or applies the
assumed steering/yaw semantics.

The reviewed source identities, exact digests, line references, observed open
flags, and uncertainty boundary are recorded in the
[static-source audit](../../../../docs/evidence/2026-08-10-wheeltec-static-source-audit.md).

The physical backend is disabled by default, and the package starts no
actuator by default. Its formal ROS wrapper can start in
a valid inhibited state without opening a device; the checked-in configuration
sets `real_device_enabled`, `actuation_enabled`, protocol acknowledgement,
physical-device opt-in, actuation opt-in, readiness, and the independently
approved external/durable stop-strategy gate false, and leaves the device path
empty. The physical factory separately refuses a path unless its access gates
and pinned identity are complete. This revision also compiles
`kPhysicalActuationReleaseEnabled` as `false`; no parameter, launch argument,
or caller opt-in can open an actuation fd. Changing that constant requires a
new safety ADR plus installed-firmware, actuator-power-isolated parser
resynchronization, unclean-restart, ownership, and watchdog evidence.
Receive-only capture must explicitly select
`kFeedbackOnly`; the fd
is then opened `O_RDONLY` and its transport rejects every `writeAll`, including
an empty write. Read-write `kActuation` access remains release-frozen even when
the independent `actuation_opt_in` is supplied. No device name is selected by
default.

## Receive-only feedback capture

`wheeltec_feedback_capture` is the first physical commissioning boundary. It
is a ROS-independent, duration-bounded diagnostic that passes only
`ByteTransport::readSome` data into `FeedbackStreamParser`. It never constructs
`WheeltecSerialAdapter`, never calls `ByteTransport::writeAll`, and never
publishes a motion command. The feedback-only physical factory opens the
explicitly named character device `O_RDONLY`; its resulting transport rejects
any attempted `writeAll` with `TransportStatus::kDisabled`.

The tool is disabled unless every one of these values is supplied explicitly:

- `--feedback-only-opt-in` and `--unverified-protocol-acknowledged`;
- a direct `--device /dev/...` path (symlinks and path traversal are rejected);
- expected character-device major/minor, owner UID, and group GID;
- expected USB VID, PID, and nonempty serial identity;
- a bounded duration from 0.1 to 3600 seconds; and
- an absolute output path naming a file that does not already exist.

The output is newline-delimited JSON using schema
`auto_rover.wheeltec.feedback_capture.v1`. Every successful read produces a
`raw_chunk` record before parsing, including exact bytes and local
`CLOCK_MONOTONIC` receipt time. This preserves noise, fragmented frames, and
checksum-invalid candidates for later protocol review. Each complete valid
frame also produces a `frame` record with the exact 24 raw bytes, the same
receipt-time semantics, and all decoded candidate values. This includes byte 1
both as `composite_stop_flag_raw` and as the fail-closed derived booleans
`control_allowed`/`control_inhibited`; it also explicitly records that VCU ACK,
specific-fault, and command-echo fields are unavailable. The final `summary`
records raw-byte, valid-frame, checksum, framing, discard, trailing-buffer,
read-timeout, capture-average-rate, and inter-frame-rate statistics. The VCU
frame has no observed source timestamp; the receipt timestamp must not be
interpreted as controller time.

The output is created with `O_CREAT | O_EXCL | O_NOFOLLOW` and mode `0600`, so
an existing file or symlink is never overwritten. Completion includes `fsync`
and checked `close`; an output failure makes the command fail. A representative
invocation, using values independently recorded for the exact device, is:

```text
wheeltec_feedback_capture \
  --feedback-only-opt-in \
  --unverified-protocol-acknowledged \
  --device /dev/EXACT_DIRECT_CHARACTER_DEVICE \
  --expected-major MAJOR --expected-minor MINOR \
  --expected-owner-uid UID --expected-group-gid GID \
  --expected-usb-vid VID --expected-usb-pid PID \
  --expected-usb-serial SERIAL \
  --duration-s 30 \
  --output /absolute/path/to/new-feedback-capture.ndjson
```

This capture can provide evidence about inbound candidate framing and rate; it
cannot verify command bytes, steering sign, vehicle mode, actuation, or a
controller-side watchdog. `O_RDONLY` and the write-disabled transport prove
that this tool does not transmit command bytes; they do not prove that opening
or configuring a USB CDC ACM TTY has no controller-side line-state or reset
effect. Do not start perception for this test, and keep the independent
hardware emergency stop available even while the vehicle is raised. No capture
command is run automatically by build, test, install, or bringup.

## Raised-bench command characterization

`wheeltec_bench_characterize` is a separate ROS-free diagnostic. It is not a
VCU node and is never connected to the phase-1 execution graph. It has two
explicit modes: `exact-zero`, which can encode and write only the candidate
exact-zero frame, and `straight-ramp`, which can write only forward,
zero-curvature commands. Nothing selects a device or enables actuation by
default.

Before it opens a physical path, the CLI requires all three transport gates
(`--unverified-protocol-acknowledged`, `--physical-device-opt-in`, and
`--actuation-opt-in`), the exact operator token
`I_CONFIRM_RAISED_WHEELTEC_BENCH_ACTUATION`, and every device/USB identity
field required by the hardened `PhysicalAccessMode::kActuation` factory.
`exact-zero` additionally requires an operator-attested passive-capture digest
formatted as `passive-capture-sha256:` followed by exactly 64 lowercase hex
characters. `straight-ramp` also requires an independently attested successful
zero-run digest formatted as `exact-zero-sha256:` plus 64 lowercase hex
characters. The CLI validates the complete token shape before opening the
device. It does not open or parse the referenced evidence files, so the
operator must independently run `sha256sum` and check the evidence contents.

The command session drains pre-open serial backlog before accepting feedback.
Motion authorization needs five fresh standstill receipts spanning at least
0.20 seconds. The current abort thresholds are scoped only to this UNVERIFIED
bench characterization and come from the operator-attested passive capture:
absolute forward speed at standstill at most 0.005 m/s, absolute lateral speed
at most 0.001 m/s, and absolute yaw rate at most 0.023 rad/s. They are recorded
in every output file as `bench_evidence_only_UNVERIFIED`; they are not phase-1
deployment acceptance limits. The inclusive 0.005 m/s standstill threshold
rounds one observed type-9 encoder count of approximately 0.0037905 m/s upward;
it does not relax the original authorized-motion reverse-direction gate.
During ramp-up and target hold, measured forward
speed more than 0.001 m/s above the latest low-speed request also aborts, even
when it remains below the global 0.50 m/s ceiling. Ramp-down permits bounded
positive plant lag while retaining the hard 0.50 m/s, reverse, lateral/yaw,
freshness, final-zero, and post-zero standstill gates; it never raises a
command to catch up with feedback. A valid feedback frame whose composite stop
flag inhibits control also revokes motion authorization immediately.

Ordinary `straight-ramp` retains that strict yaw gate and a 5 second hold/15
second session. The optional
`--allow-raised-wheel-start-asymmetry` flag is valid only for an explicitly
raised `straight-ramp`; it is false by default and is recorded in both CLI and
session metadata. While motion is authorized, this mode interprets the
firmware-derived rear-encoder feedback using the UNVERIFIED 0.322 m rear track:
`v_left = v_forward - yaw_rate * 0.322 / 2` and
`v_right = v_forward + yaw_rate * 0.322 / 2`. Each derived rear-wheel speed
must remain in `[-0.005, 0.500] m/s`; during ramp-up and hold each must also be
at most the last successfully written normal wire request plus 0.020 m/s. The
0.005 m/s lower-bound tolerance remains exclusively a conservative rear-speed
reconstruction/quantization allowance; it is not reused as a tracking margin
or accepted reverse command. In the explicit raised mode only, the overall
forward feedback uses the same inclusive -0.005 m/s one-count lower bound as
the two derived wheels; ordinary authorized motion retains -0.001 m/s. The
separate 0.020 m/s raised-wheel tracking
margin rounds up the evidence sum `0.0122 + 0.0037905 + 0.001161 = 0.0171515`
m/s (observed tracking excess, one type-9 encoder count, and reconstruction
truncation). It remains UNVERIFIED and is not a field control limit. A failed
or partial write cannot advance its baseline. Ramp-down keeps the existing
positive plant-lag exception but retains both per-wheel absolute limits. The
ordinary mode's forward tracking margin remains 0.001 m/s. The original
forward absolute and lateral gates,
composite stop flag, freshness, 100 ms timing, exact 0.20 m/s2 wire envelope,
clock, disconnect, evidence, and emergency-zero paths remain active. Pre-arm
and post-zero observations still use the strict yaw gate. This explicit mode
permits at most a 60 second hold inside a hard 70 second session; it is not
valid for `exact-zero` and is not a deployment setting.

`--record-uncalibrated-motion-feedback` is a second, independent opt-in and is
valid only together with raised-wheel asymmetry in `straight-ramp`. During
authorized motion it records tracking, deadband, and left/right asymmetry
instead of gating them against the ordinary forward `+0.001 m/s` or raised
per-wheel `+0.020 m/s` request-relative limits. It still rejects overall or
derived reverse beyond `-0.005 m/s`, any derived wheel or overall forward above
the inclusive `1.000 m/s` raw-calibration emergency feedback ceiling, lateral
motion, inhibited/invalid feedback, stale feedback, timing, wire-slew, clock,
transport, or recording failures. This user-authorized raised-only feedback
ceiling is not an acceptance limit: target commands and wire encoding remain
hard-capped at `0.500 m/s`, and guarded raised/ordinary feedback remains capped
at `0.500 m/s`. Pre-arm and post-zero each
require five fresh receipts spanning at least 0.2 seconds for which both the
overall forward value and each derived rear-wheel value have magnitude at most
0.005 m/s; the strict 0.023 rad/s yaw gate also remains. Metadata labels
tracking/deadband/asymmetry as `recorded_not_gated`, states
`acceptance_evidence=false`, records the user-requested raised-only 60 second
hold, and reiterates that no VCU acknowledgement is available. This calibration
record cannot be used as acceptance evidence.

Normal TX is fixed at no more than 50 Hz. Every completely successful normal
host write, including normal zero, establishes the next wire-rate baseline at
its post-write monotonic time. Just before the next normal write, an independent
transport-side gate enforces the exact integer invariant
`abs(raw_i - raw_(i-1)) * 5,000,000 <= before_i_ns - after_(i-1)_ns`, where one
raw speed unit is 0.001 m/s. The generator is coupled to the same floor-quantized
budget. There is no extra 1 mm/s tolerance: both ramp-up and ramp-down remain at
or below 0.20 m/s² despite 17--25 ms timing jitter or a failed write. A command
gap over 100 ms aborts instead of catching up with a larger step. A direct
emergency exact-zero write is the sole deliberate wire-rate exception. The
first nonzero request is therefore strictly below 0.50 m/s. Reverse, lateral
command, nonzero yaw, reserved-byte changes, feedback loss/invalidity, excessive
feedback, serial disconnect, and monotonic-clock rollback all revoke
authorization and enter a bounded exact-zero host-write sequence. Ramp-down
reaches exact zero before a second five-receipt standstill observation is
required.

Every RX/TX attempt and result records raw bytes, `CLOCK_MONOTONIC` times,
absolute deadline, OS status, byte count, and `delivery_unconfirmed`. Session
records use a bounded 16 MiB buffer reserved before physical open and are
flushed to disk only after the final zero attempt and serial close, so a slow
evidence filesystem is not in the active command path. The NDJSON
metadata states `vcu_ack_available=false`: a completed host OS write is not a
VCU acknowledgement and does not prove physical effect. Evidence-write or
sticky-clock failure blocks further normal commands but cannot block the
separate, maximum-three-attempt emergency exact-zero write directly to the
already-open transport; this write happens before its best-effort evidence
record. SIGINT, SIGTERM, SIGHUP, SIGQUIT, and SIGTSTP use that same exit path.
SIGKILL and SIGSTOP cannot be caught. Disconnect and failed/partial exit writes
remain explicitly unconfirmed.

This characterization executable is not a deployment safety loop. A spotter,
raised and secured vehicle, clear wheel envelope, and immediately available
independent physical emergency stop remain mandatory throughout every run.
The reviewed source also contains a 100 Hz startup self-check that requests
0.2 m/s from approximately 10 to 12 seconds after boot. Whether the attached
binary retains that behavior is unknown, so merely powering or resetting the
VCU must be treated as a potential motion event independently of this tool.

Representative exact-zero invocation (all values must be independently
measured for the exact device and evidence file):

```text
wheeltec_bench_characterize \
  --unverified-protocol-acknowledged \
  --physical-device-opt-in --actuation-opt-in \
  --operator-confirmation I_CONFIRM_RAISED_WHEELTEC_BENCH_ACTUATION \
  --mode exact-zero \
  --passive-evidence-token passive-capture-sha256:<64-lowercase-hex> \
  --device /dev/EXACT_DIRECT_CHARACTER_DEVICE \
  --expected-major MAJOR --expected-minor MINOR \
  --expected-owner-uid UID --expected-group-gid GID \
  --expected-usb-vid VID --expected-usb-pid PID \
  --expected-usb-serial SERIAL \
  --duration-s 1 \
  --output /absolute/path/to/new-exact-zero.ndjson
```

`straight-ramp` is refused until the exact-zero output has been reviewed and
its SHA supplied. It then requires explicit `--target-speed-mps` (hard maximum
0.50), bounded `--hold-s` (maximum 5 by default, or 60 only with the explicit
raised-wheel asymmetry flag), and both evidence digests. No bench command is
run automatically by build, test, install, or bringup.

## Experimental raw raised-bench profile capture

[ADR 0002](../../../../docs/adr/0002-wheeltec-experimental-raw-profile-capture.md)
defines this real-actuation experimental boundary and its compatibility review.

`wheeltec_raw_profile_capture` is an independent, ROS-free experimental
executable for collecting uncalibrated VCU motion evidence. It does not use
`WheeltecSerialAdapter` or the Phase-1 command encoder, is not linked into the
vehicle graph, is excluded from the default build, and is not installed. Its
header is excluded from the package install. The selected Phase-1 application
configuration remains explicitly capped at 0.50 m/s; the separate experimental
encoder has its own 6.0 m/s center-command ceiling and is reachable only by
explicitly building and invoking this non-installed target. It does not consume
or bypass the configurable production codec.

Every invocation selects exactly one of 64 fixed profiles; the CLI has no free
numeric speed, yaw, curvature, radius, ramp, hold, or session input. The closed
catalog contains:

- twelve straight profiles, `straight-0.5` through `straight-6.0` in exact
  0.5 m/s steps;
- the original four center-command turn profiles, `left-0.5-r2.0`,
  `right-0.5-r2.0`, `left-0.5-r0.95`, and `right-0.5-r0.95`, whose names and
  center=0.5 m/s semantics are deliberately unchanged; and
- 48 outer-tier turn profiles named like `left-outer0.5-r2.0`: left/right,
  R=2.0/0.95 m, and outer-rear-wheel tiers from 0.5 through 6.0 m/s in exact
  0.5 m/s steps.

For an outer-tier turn, the tool chooses the greatest integer center wire `c`
in `[0, 6000]` for which `q=floor(1000*c/radius_mm)` and
`1000*c + 161*q <= 1000*tier_wire`. Yaw is `+q` for left and `-q` for right.
Thus the encoded outer rear-wheel command, derived with the UNVERIFIED 0.322 m
track, cannot exceed the selected tier; the inner command remains nonnegative,
and left/right profiles are integer mirror images. At the 6.0 m/s outer tier,
the selected `(center, |yaw|, inner, outer)` values are
`(5.553, 2.776, 5.106064, 5.999936)` for R=2.0 m and
`(5.130, 5.400, 4.260600, 5.999400)` for R=0.95 m. Metadata records the tier
reference and wire, actual center/yaw wire, derived left/right and inner/outer
commands, radius, track, and integer selection rule, so an outer tier cannot be
misread as a center target.

Each profile uses a strict 0.20 m/s2 **center-command** integer-wire ramp, a
fixed 60 second target hold, a bounded ramp to zero, and two seconds of
post-zero observation. Turn commands use the UNVERIFIED convention
`yaw_rate = forward_speed / radius`, positive for left and negative for right;
R=2.0 m supports screening before the 0.95 m Phase-1 minimum-radius profile.
Profiles are never chained automatically. Normal TX is at most 50 Hz. The
post-drain active profile session has a hard 130 second monotonic deadline;
startup zero and bounded backlog drain precede that deadline origin and emit no
nonzero command.

The original eight profiles were captured as separate supervised, raised-wheel
60-second runs before this catalog expansion. The later requested-scope matrix
completed all twelve straight profiles through 6.0 m/s and all forty
left/right outer-tier profiles through 5.0 m/s at both radii. The catalogued
5.5/6.0 m/s outer-tier turns were outside the final requested physical scope
and were not run. Raw paths, SHA-256 identities, host-write outcomes, and
offline hold/onset/tail observations are preserved in the
[expanded raised-bench matrix](../../../../docs/evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md).
Those runs remain `acceptance_evidence=false`. No run verifies a VCU ACK or
source-timed latency, connects this executable to ROS, raises the production
Phase-1 deployment's explicitly configured `0.50 m/s` ceiling, or authorizes
ground motion.

This is a measurement tool, not an acceptance guard. Every raw RX chunk is
recorded before parsing. Checksum/framing/discard events are recorded while
the parser resynchronizes and do not refresh the valid-frame watchdog. Each
valid feedback event preserves raw bytes, `FlagStop`, forward/lateral/yaw,
request-relative tracking error, and rear-wheel values derived using the
UNVERIFIED 0.322 m track. Feedback magnitude, tracking error, left/right
difference, apparent deadband, standstill noise, and post-zero tail do not stop
a run. Metadata labels these fields observation-only and
`acceptance_evidence=false`. The feedback format has no observed source
timestamp; all event times are host `CLOCK_MONOTONIC` receipt times.

After a successful physical open, the first transport transaction is ten
`0x00` parser-resynchronization bytes, followed by one pre-encoded exact-zero
command candidate, with no evidence allocation or intervening read. The whole
zero-only sequence must complete within the bounded retry/deadline policy
before any session I/O. Its v1 startup record retains the historic record type
but new captures separately report padding completion, exact-zero completion,
recovery restarts, and stream poison. The core then repeats normal exact zero
to establish its wire-slew baseline and drains bounded serial backlog. Motion
remains zero until at least five distinct, strictly increasing valid
`FlagStop=0` read receipts span at least 0.20 seconds. Multiple valid frames in
one read count as one receipt. Parser noise and malformed `FlagStop` values do
not count and do not refresh freshness.

Valid feedback remains mandatory within 150 ms. A valid `FlagStop=1` frame,
invalid/non-finite decoded semantics, serial disconnect, failed or partial
write, monotonic-clock rollback, more than 100 ms between successful normal
write completions, evidence-buffer append failure, session deadline, or caught
termination signal stops normal transmission. A full write that completes
after its 10 ms deadline, the 100 ms completion gap, or the hard session
deadline is recorded as having occurred but cannot advance the slew baseline;
it immediately enters the zero path. The independent exit path attempts exact
zero at most three times, 20 ms apart, with a 10 ms write deadline. All bounded
zero retries happen before their best-effort emergency records, so evidence
callbacks cannot separate or suppress the attempts. Disconnect or a
failed/partial zero remains explicitly
`delivery_unconfirmed`; a full host write is not a VCU ACK.

The CLI preflight record is written to an exclusively created mode-0600 NDJSON
file before physical open. After open, the open result and all active-session
records go only to a 32 MiB buffer fully reserved before open; there is no
synchronous filesystem I/O on the command or zero path. Capacity or in-memory
append failure is detected immediately and invokes bounded zero. Only after
the zero path and serial close does the CLI flush the buffer, append the
summary, fsync, and close. A filesystem failure at that point cannot be known
during motion. A crash, SIGKILL, or power loss may lose the entire buffered
motion session, and SIGKILL cannot invoke zero. Any file without a complete,
internally consistent summary is invalid evidence. The injected
one-frame-per-20-ms `straight-6.0` test completes
the full up/60-second hold/down/post-zero sequence in 122.181 seconds and emits
17,270,025 bytes of core session evidence. That leaves 16,284,407 bytes in the
32 MiB buffer; the regression additionally budgets 64 KiB for CLI-buffered
prefix records and requires at least 15 MiB remaining. Capacity exhaustion
continues to fail closed through the independent exact-zero path. Successful
normal TX and feedback records include the profile and phase (`baseline`,
`ramp_up`, `hold`, `ramp_down`, or `post_stop`).

The CLI requires the three physical-actuation gates plus the distinct
`--raw-raised-bench-opt-in`, the exact token
`I_CONFIRM_RAISED_WHEELTEC_RAW_PROFILE_ACTUATION`, both independently reviewed
passive and exact-zero evidence SHA tokens, one fixed profile, the complete
direct character-device and USB identity, and a new absolute output path. No
value is selected by default. A representative shape is:

```text
wheeltec_raw_profile_capture \
  --unverified-protocol-acknowledged \
  --physical-device-opt-in --actuation-opt-in \
  --raw-raised-bench-opt-in \
  --operator-confirmation I_CONFIRM_RAISED_WHEELTEC_RAW_PROFILE_ACTUATION \
  --profile straight-0.5 \
  --passive-evidence-token passive-capture-sha256:<64-lowercase-hex> \
  --exact-zero-evidence-token exact-zero-sha256:<64-lowercase-hex> \
  --device /dev/EXACT_DIRECT_CHARACTER_DEVICE \
  --expected-major MAJOR --expected-minor MINOR \
  --expected-owner-uid UID --expected-group-gid GID \
  --expected-usb-vid VID --expected-usb-pid PID \
  --expected-usb-serial SERIAL \
  --output /absolute/path/to/new-raw-profile.ndjson
```

Builds and tests use injected transports only and never open a physical device
automatically. Actual profile order remains an operator-reviewed screening
decision; the reported 6 m/s chassis maximum is not a software authorization.
Every physical run still requires the vehicle raised and mechanically secured,
clear wheel-to-ground clearance and wheel envelope, a dedicated spotter, and
an immediately operable independent main-power breaker. Software zero does not
replace that independent chain.

## Formal runtime and vehicle backend

`WheeltecSerialRuntime` owns one explicitly mounted connection generation. It
does not reopen a path. A remount requires a caller-supplied connected
transport with a strictly newer generation; parser state, authorization,
cached motion, feedback recovery, and startup evidence are cleared, while
sequence and authorization high-water marks survive. Before a physical
transport is mounted, every write-enabled caller performs the common ten-zero
padding plus exact-zero activation. The runtime's first own transport I/O is a
second bounded exact-zero full host write. Reads are forbidden until that
succeeds, and opening backlog is then drained separately. A partial or
otherwise unknown-prefix write poisons that generation: no later revoke,
shutdown, or retry appends another frame. Only a known zero-byte non-delivery
may use the configured bounded whole-frame retry policy.

That reset applies only to the host receive parser. The reviewed MCU command
callback retains a function-static partial 11-byte receive count across a
Linux close/reopen and has no inter-byte timeout. Consequently a new host
connection generation and a complete host-written startup zero do not prove
the controller parser is aligned or that zero was accepted. This is the reason
the physical-actuation release gate remains compiled off; the formal runtime
is exercised through injected/PTY transports only in this revision.

Normal worker steps read first with a maximum 1 ms timeout, stamp feedback only
after that read completes, and publish at most the latest complete frame from
one read. Multiple complete `FlagStop=0` receipts must be distinct and span the
configured recovery interval (five receipts over 200 ms in Phase 1). Any
`FlagStop=1`, or any checksum-valid frame whose stop flag is outside the binary
domain, inhibits the entire read batch. Bad and partial frames do not refresh
feedback. Freshness expires after 150 ms. Before every normal TX the runtime
checks the 100 ms gap from the last successful write completion; it also checks
the actual completion against the adapter/command write deadline and the 50 ms
worker bound. A completed motion write that crosses a trusted timing boundary
is followed by a bounded exact zero in the same call before terminal fault.

`WheeltecVehicleBackend` normalizes only signed measured speed, wheel-derived
yaw rate, voltage, and local monotonic/ROS receipt evidence. Gear, substantiated
control state, fault provenance, controller ACK, and VCU source time remain
unavailable. Full host write completion is therefore never reported as a VCU
ACK. Raw `FlagStop` stays a backend-local current-cycle inhibit; readiness also
requires fresh multi-receipt recovery. Ordinary `revokeAndStop` clears the
authorization epoch and performs bounded zero without destroying a healthy
recovered session, while explicit runtime `shutdown` is terminal.

The installed formal ROS wrapper drives this backend from an independent
steady-clock thread; ROS subscriptions, services, publication, and logging are
not part of the physical watchdog loop. Cycle results cross through a
single-slot, nonblocking mailbox. A missed deadline, mailbox contention,
callback/backend exception, invalid connection generation, or oversized input
runs the poison-aware bounded stop while the transport is still owned, closes
ROS entry, and terminates the node. The wrapper accepts at most 4096 trajectory
points, 256 bytes per semantic identifier, and 1024 retired identities per
tracked source class. These are execution-work bounds, not vehicle dynamics
limits.

Worker-cycle safety output and synchronous arm, disarm, emergency-stop, and
reset output share one publication ordering high-water mark. A direct
transition clears any pending pre-transition cycle while the execution core is
still locked, then reserves its state ID, latch generation, and source stamp.
Direct and cycle paths serialize the actual safety ROS publish, and a cycle
already taken from the mailbox must reclaim against that reservation inside the
publish mutex. A delayed lower-generation worker result is dropped, so an old
`ARMED` cycle cannot be published after emergency stop and a pre-arm/pre-reset
generation cannot roll back the controller's required safety feedback.
Unorderable tuples and semantic changes under a reused identity are suppressed.
Direct callbacks release the execution-core lock before entering the publish
mutex; the worker still uses only a nonblocking mailbox acquisition and never
enters that mutex, so this ordering barrier adds no blocking operation to its
physical watchdog path.

## Candidate command encoding

The fixed 11-byte candidate frame is:

| Bytes | Candidate meaning |
| --- | --- |
| 0 | header `0x7b` |
| 1-2 | reserved zero |
| 3-4 | signed forward speed times 1000, high byte first |
| 5-6 | signed lateral speed times 1000, high byte first |
| 7-8 | signed yaw rate times 1000, high byte first |
| 9 | XOR of bytes 0 through 8 |
| 10 | tail `0x7d` |

All wire values are fixed-width `int16_t`. Quantization uses truncation toward
zero. The adapter maps a checked `VehicleExecutionCommand` to
`forward_speed / 0 / forward_speed * curvature`. Zero forward speed always
produces an exact zero yaw-rate field. Non-finite values, reverse, nonzero
lateral motion, an inhibited nonzero command, wire overflow, speed above
the caller-supplied maximum, and absolute curvature above `1 / 0.95 m` are
rejected before any write.

`CodecLimits::max_forward_speed_mps` has no motion-authorizing default. The
caller must explicitly supply a finite value strictly greater than `0.0 m/s`
and strictly less than `6.0 m/s`. An omitted value retains the fail-closed zero
default; zero, non-finite values, `6.0 m/s`, and larger values cannot authorize
a nonzero frame. The protocol-level exact-zero safe-state frame remains
available when motion configuration is invalid. The exclusive `6.0 m/s`
configuration bound is not a verified vehicle capability, deployment setting,
or permission to raise an application profile.

The software-side contract uses positive speed for forward motion and positive
rear-axle curvature for a left turn. Whether the candidate positive yaw field
has that physical effect remains unverified.

The checked-in Phase-1 configuration explicitly supplies `0.50 m/s` as its
commissioning target and initial application ceiling. That value is not a
measured deadband and is not an instruction to make the first nonzero command
at that value. Historic demo configuration contained a 0.35 m/s command-risk
signal, so physical commissioning still requires the documented restrained
bench gate before any low-speed field work.

## Candidate feedback decoding

The rolling parser recognizes only complete 24-byte candidates:

| Bytes | Candidate meaning |
| --- | --- |
| 0 | header `0x7b` |
| 1 | binary composite `FlagStop`: `0` permits control this cycle, `1` inhibits it |
| 2-7 | signed forward, lateral, and yaw-rate values divided by 1000 |
| 8-13 | three signed acceleration values divided by 1671.84 |
| 14-19 | three signed gyro values multiplied by 0.00026644 |
| 20-21 | unsigned supply voltage divided by 1000 |
| 22 | XOR of bytes 0 through 21 |
| 23 | tail `0x7d` |

Firmware-source data flow shows byte 1 is the current-cycle aggregate result of
low-voltage, enable/emergency input, self-check, and software-stop conditions.
It is not a VCU acknowledgement, a specific fault code, or a command echo. The
decoder accepts only the binary domain 0/1; any other value is a framing error.
`FeedbackCandidate` defaults to inhibited/fail-closed and preserves both the raw
byte and the derived `control_allowed`/`control_inhibited` values. The formal
runtime consumes that value only as its local actuation inhibit. It does not
map it into a normalized `ChassisState` control-enabled or fault field and does
not claim fault provenance.

Input may arrive in fragments or with noise. On an invalid candidate the
parser discards one byte, searches again for the header, and retains at most a
partial frame. Only header, tail, XOR-valid, and binary-FlagStop-valid frames
are emitted. The frame contains no observed sequence or timestamp, so decoded
candidates explicitly report `source_time_available=false`; a wrapper must
assign and disclose receipt time rather than inventing source time.

Historical demo bags exposed `/odom` at approximately 20 Hz, but contained no
raw frames or source timestamps. That observation is not a feedback-rate or
freshness contract.

## Transport and watchdog behavior

`writeAll` is deadline bounded and handles interrupted calls, would-block,
short writes, and partial progress. A disconnect is reported with
`delivery_unconfirmed=true`. A successful local write confirms only that all
bytes reached the operating-system transport; it does not confirm controller
acceptance or physical motion.

After the applicable gates, the physical factory requires an explicit direct
`/dev/...` path, device major/minor, owner UID, group GID, USB VID/PID, and USB
serial. It rejects symlinks in every path component, non-character devices,
non-TTYs, changed pre-open/post-open identities, executable or world-accessible
device modes, and unexpected owner/group. The open uses `O_NOFOLLOW`,
`O_NOCTTY`, `O_NONBLOCK`, and `O_CLOEXEC`, plus `O_RDONLY` for feedback-only or
`O_RDWR` only for three-gate actuation access. During allocation-permitted
preparation it resolves `/sys/dev/char/<major>:<minor>/device` below
`/sys/devices` and requires all three USB attributes at one common ancestor.
After raw open and `fstat`, it repeats the component-wise no-symlink `/dev`
identity observation and requires before/fd/after equality before `flock`,
`TIOCEXCL`, or termios configuration. Any absent or mismatched observation
fails closed. USB attributes are not currently reread after raw open; the
three-way devtmpfs identity detects node replacement but is not treated as a
general proof of post-open USB ownership across every hotplug race. That
allocation-free sysfs-binding proof or equivalent reviewed evidence remains a
release blocker while the compile gate is false. Raw 115200 8N1 with no flow
control is applied only after file-identity verification, then read back and
checked field by field. These remain candidate host settings, not a verified
physical contract.

The reviewed firmware source does not provide a controller-side stop guarantee
that this host may rely on. Its initialized `SecurityLevel` is 1, for which the
100 Hz control task bypasses the command-loss counter and may retain the last
target after UART loss. At `SecurityLevel` 0 the source clears target velocity
only after roughly one second without a valid command, after which its internal
smoothing may still take time to reach zero. No received ACK, latched
communications fault, or positive command echo is present in the 24-byte frame.
The independent host watchdog and physical emergency stop therefore remain
mandatory; the feedback composite stop flag must not be promoted into any of
those missing guarantees.

The adapter watchdog is independent of any upstream motion watchdog. It checks
strictly increasing sequence IDs, creation time, absolute deadline, maximum
age, codec limits, connection generation, and dual authorization. Authorization
includes its local monotonic transition time; only commands created strictly
after that transition may enter recovery. A non-positive or regressing local
receipt/cycle time clears motion, recovery, and authorization before any write,
and requires a new authorization epoch; a later valid clock cannot resume the
old command. Loss of a valid command starts an explicit zero-frame episode.
Failed zero writes are retried at a bounded interval and stop at a configured
attempt limit. Recovery requires a configured number of consecutive fresh
commands. A disconnect clears the cached command; a new connection generation
also clears enable, arm, and authorization-time state, while retaining the
highest observed sequence to reject replay.

For an orderly process shutdown, the caller must clear both authorization
inputs and continue calling `cycle` until a zero write succeeds or the bounded
retry result is reported. Object destruction alone cannot assert that a zero
frame reached the controller; a disconnect or failed exit write remains
delivery-unconfirmed.

## Physical validation and license boundary

Physical vehicle mode, steering interpretation, controller configuration,
feedback behavior under every inhibit, reconnect behavior, command effect, and
the correspondence between reviewed source and the flashed binary remain
unknown until restrained bench evidence is recorded. Static source now supports
the candidate framing, checksum, scaling, and composite-stop interpretation;
that evidence is not a physical acceptance result or an acknowledgement. The
hardware emergency-stop chain must remain independent and available throughout
that work.

No manufacturer or historic demo source is copied into this package. The
reviewed external trees did not establish a reusable license grant, so they
serve only as static interoperability evidence and are not redistributed or
linked. The PTY and injected-I/O tests exercise this original implementation
without enabling both physical-device gates and without opening a physical
controller path.
