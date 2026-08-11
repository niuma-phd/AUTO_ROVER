# ADR 0002: Wheeltec experimental raw-profile capture boundary

- Status: Proposed
- Date: 2026-08-11
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: Pending review of the implementing pull request

## Context

The operator explicitly requested supervised, raised-wheel evidence collection
at fixed `0.50 m/s` steps through the reported `6.0 m/s` chassis maximum,
followed by fixed left and right turn profiles across the same speed tiers.
Each profile needs a 60 second target hold so wheel onset, unloaded tracking,
asymmetry, and stopping behaviour can be examined offline. This is
experimental VCU characterization, not Phase-1 navigation or vehicle
acceptance. The reported maximum and every profile above the existing evidence
range remain `UNVERIFIED` until separately captured and reviewed.

[ADR 0001](0001-nuc-ackermann-phase1-contracts-and-safety.md) keeps the selected
Phase-1 ROS production configuration's forward ceiling at `0.50 m/s`, the
longitudinal acceleration ceiling at `0.20 m/s^2`, and the minimum runtime turn
radius at `0.95 m`. The higher raised-wheel targets must not silently change
those decisions or make an experimental encoder reachable from the guarded
vehicle graph.

The candidate protocol remains `UNVERIFIED`. A complete host OS write is not a
VCU acknowledgement, the feedback frame has no source timestamp or command
echo, and the reviewed firmware may retain its last target after host
communication loss. The attached controller has no separately verified
mushroom E-stop. For the documented raised-wheel work only, the supervised,
latching main-power breaker is the independent motor-power interruption; USB
backfeed can leave the VCU logic powered after that breaker is opened. These
conditions do not authorize ground operation.

Evidence persistence and command-path timing conflict if ordinary filesystem
I/O is performed synchronously while a nonzero target may still be retained.
A regular-file write can block without a useful userspace deadline, preventing
the host watchdog, signal handling, and emergency-zero path from running.

## Decision

The requested matrix was subsequently completed and preserved as historical
raised-bench evidence. A later firmware review found that the MCU UART command
parser can retain a partial-frame count across host close/reopen. Accordingly,
the shared physical factory now compiles its actuation release gate as false;
this experimental executable cannot currently acquire a physical write fd even
when all of its opt-ins are supplied. Re-enabling it is outside this ADR and
requires the new safety decision and parser/installed-firmware evidence defined
by ADR 0001. The design below remains the record of the isolated capture
boundary and its injected/PTTY regression behavior.

`wheeltec_raw_profile_capture` is a ROS-free, raised-bench-only experimental
executable with an independently encoded, fixed command set. It is outside the
Phase-1 application and production adapter. Its library and executable are
excluded from the package's default build, install, and exports; its header is
not installed; and no launch, bringup, or runtime graph may reference it.

One process runs exactly one explicitly selected profile and then exits.
Profiles are never chained or automatically escalated:

- straight targets are the twelve fixed `0.50 m/s` tiers from `0.50` through
  `6.0 m/s`;
- the four original turn names retain their center-forward `0.50 m/s`
  semantics at signed radii `2.0 m` or `0.95 m` so recorded evidence remains
  reproducible;
- the expanded turn grid has twelve fixed outer-rear-wheel command tiers from
  `0.50` through `6.0 m/s`, for left and right turns at both `2.0 m` and
  `0.95 m`. For each cell, the center-forward wire value is the greatest
  integer value whose derived outer rear-wheel command does not exceed the
  selected tier and whose inner command is nonnegative. This avoids treating a
  `6.0 m/s` center command as safe when its outer wheel would exceed the
  reported maximum;
- turn yaw is encoded by the `UNVERIFIED` convention `w = v / R`, positive for
  left and negative for right;
- every profile has a fixed 60 second target hold, uses an exact integer-wire
  `0.20 m/s^2` ramp, transmits normal commands at no more than 50 Hz, and has a
  hard 130 second monotonic session deadline; and
- no straight or center-forward command may exceed `6.0 m/s`, and no expanded
  turn profile may command a derived outer rear wheel above its fixed tier.
  This experimental ceiling does not alter the
  selected ROS Phase-1 configuration's `0.50 m/s` ceiling.

Before physical open, the CLI requires all physical-actuation acknowledgements,
a distinct raised-bench opt-in, the exact operator confirmation token, the
independently reviewed passive-capture and exact-zero SHA-256 tokens, one fixed
profile, a complete direct character-device and USB identity, and a new output
path. Nothing is selected or enabled by default.

After physical open, the first transport transaction must be an exact-zero
command. A complete host write is required before any receive, drain, pre-arm,
or nonzero work; failure or disconnect ends the session with delivery explicitly
unconfirmed. The session then drains a bounded receive backlog. It keeps writing
zero during pre-arm and cannot emit a nonzero command until at least five
distinct, strictly increasing, valid `FlagStop=0` receipt events span at least
`0.20 s`. Multiple frames in one read are one receipt because no controller
source time is available.

The following are hard stop conditions: missing or stale valid feedback beyond
`150 ms`; a valid `FlagStop=1`; invalid or non-finite decoded semantics;
monotonic-clock regression; the 130 second session deadline; more than `100 ms`
between successful normal-write completions; failed, partial, late, or
delivery-unconfirmed normal writes; transport disconnect; evidence-buffer
failure; and a caught termination request. Such a condition prevents further
normal transmission and enters the bounded exact-zero path. The exit path makes
at most three exact-zero attempts, 20 ms apart, each with a 10 ms write
deadline. All attempts precede their best-effort evidence formatting so a
record failure cannot separate the retries. A disconnect or unsuccessful zero
remains delivery-unconfirmed. SIGKILL, process failure, host failure, and loss
of the link cannot be converted into a software stop guarantee.

Checksum, framing, discard, and resynchronization events are preserved as raw
evidence but do not refresh feedback freshness. Valid feedback magnitude,
tracking error, apparent deadband, left/right asymmetry, standstill noise, and
post-zero tail are observation-only; they are not online acceptance gates for
this measurement tool. `FlagStop`, finite/binary semantics, receipt freshness,
transport state, command limits, wire slew, and timing remain hard gates.

The active session uses one 32 MiB evidence buffer fully reserved before
physical open. The CLI writes only its preflight record to the exclusive
mode-`0600` file before open. Open-result and session records are appended to
the bounded memory buffer; the command and zero paths perform no synchronous
filesystem I/O. Capacity or append failure stops normal transmission and enters
the zero path. Only after the zero path and serial close are the buffered
records written, followed by the summary, `fsync`, and checked close.

This deliberately prioritizes a bounded command/zero path over streaming
durability. A crash, SIGKILL, power loss, or host failure can lose the entire
buffered motion session, not just its tail. An output without a complete,
internally consistent summary is invalid evidence. Moving to streaming evidence
requires a separately reviewed design that cannot block the safety path, such
as an independently bounded asynchronous writer; it is not implied by this
decision.

Every physical run still requires the raised and secured vehicle, clear wheel
envelope, continuous spotter supervision, and an immediately operable latching
main-power breaker. That conditional raised-bench interruption does not replace
the independent hardware E-stop required for ground or released operation.

## Alternatives considered

- Raising the Phase-1 profile or its configured production ceiling was rejected
  because the requested data collection is not a navigation requirement or
  acceptance result.
- Chaining profiles or automatically advancing after a successful run was
  rejected because each higher-energy profile requires separate evidence and
  operator review.
- Reusing the guarded Phase-1 production encoder was rejected because it would
  either violate that application's configured `0.50 m/s` contract or leak an
  experimental exception into the released path.
- Synchronous per-record filesystem writes during motion were rejected because
  they can block the only thread able to apply the 100 ms watchdog and send
  exact zero.
- Claiming crash-durable streaming from the in-memory buffer was rejected. The
  buffer's evidence-loss boundary is explicit and incomplete output is invalid.
- Treating tracking, asymmetry, deadband, or tail estimates as online limits was
  rejected because those are the unknowns the experiment is intended to
  measure. Promoting them requires reviewed evidence and a later decision.
- Relying on the candidate VCU communication-loss behaviour was rejected because
  the flashed binary identity and a controller-side stop guarantee are absent.

## Consequences

The experimental profiles can collect a consistent, offline-comparable record
without expanding the Phase-1 graph or production command ceiling. Fixed
profiles, explicit gates, startup zero, pre-arm zero, bounded timing, and final
zero reduce accidental or automatic actuation paths.

The tool is not a deployment safety loop and cannot establish that the vehicle
stopped physically. Host-write completion is not an ACK, serial disconnect is
unconfirmed, the VCU may retain a target, and an uncatchable process or host
failure can skip software zero. The supervised main breaker therefore remains
part of the raised-bench procedure.

The no-filesystem-I/O command path avoids an unbounded recording stall, but a
crash can erase the complete motion record. Evidence is useful only after the
buffer, summary, `fsync`, close, checksum, and offline consistency checks all
succeed. Buffer exhaustion causes a safe abort rather than a partial successful
profile.

Higher targets and every turn profile remain `UNVERIFIED` until their own
supervised evidence is captured and reviewed. The existing completed
`0.50 m/s` raised-wheel profile does not authorize another profile, ground
motion, or release.

The later requested-scope evidence run completed all twelve straight profiles
through `6.0 m/s` and all forty left/right outer-tier turns through `5.0 m/s`
at both radii. The catalogued `5.5/6.0 m/s` turns were explicitly removed from
the final requested physical scope and were not executed. This completion is
recorded in the
[expanded raw-profile matrix](../evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md);
it does not alter this experimental boundary or the production ceiling.

## Compatibility and migration

No ROS message, MD5 contract, public domain type, or Phase-1 launch changes.
Before first release, the production codec configuration contract is clarified:
its caller must explicitly provide a finite maximum strictly within
`0 < v < 6.0 m/s`, and its zero default cannot authorize nonzero motion. The
selected Phase-1 adapter configuration remains `0.50 m/s` with physical
actuation disabled by default. The experimental source remains available only
to an explicit source-tree build and does not consume the production codec.

Promoting any part of this tool into installed software, a bringup graph, a
field workflow, or the production adapter requires a new compatibility and
safety review. Measured deadband, delay, tracking, or steering results must be
introduced as reviewed vehicle-profile evidence rather than by editing historic
limits to fit an experiment.

## Verification

Software verification requires injected-transport tests for every fixed
profile, exact `w = v / R` signs, the `6.0 m/s` experimental bound, integer
outer-tier geometry and left/right mirror symmetry, the independently supplied
Phase-1 `0.50 m/s` bound, integer-wire ramp-up and ramp-down, one-profile selection,
startup zero as the first transport I/O, zero-only pre-arm, distinct receipt
counting, parser noise and resynchronization, freshness, `FlagStop`, partial
writes, disconnects, monotonic rollback, split read/write watchdog boundaries,
the session deadline (including a complete `6.0 m/s` ramp-up, 60 second hold,
ramp-down, and post-zero interval below 130 seconds), bounded zero retries,
caught termination, exception escape, and evidence-buffer exhaustion. Tests
and builds must use injected or pseudo-terminal transports and must not open a
physical VCU automatically.

Build/install review verifies that the experimental targets remain outside the
default package build and install, the header is excluded, and no launch or
bringup references the executable. A complete evidence file must pass schema,
raw-frame, summary-count, timing, and internal-consistency validation before it
is reviewed. Physical evidence is interpreted with the limitations in the
[main-breaker raised-bench follow-up](../evidence/2026-08-11-wheeltec-breaker-raised-bench-follow-up.md);
it cannot by itself accept the production adapter or ground operation.
