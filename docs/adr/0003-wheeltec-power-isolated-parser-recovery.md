# ADR 0003: Wheeltec power-isolated parser recovery and release sequencing

- Status: Accepted
- Date: 2026-08-11
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: [Repository-owner acceptance in issue #13](https://github.com/niuma-phd/AUTO_ROVER/issues/13)

## Context

The Phase-1 software loop and the default-disabled in-process Wheeltec backend
are complete.  Physical actuation remains compile-time frozen.  Static review
of the supplied candidate STM32F407 firmware found that its 11-byte UART
command parser retains a function-static byte count across a Linux process
close and reopen, has no inter-byte timeout, and resets the count only after a
complete 11-byte candidate or a change of MCU UART.  A host-complete startup
zero frame is therefore not, by itself, proof that the controller parsed zero.

On 2026-08-11 the operator explicitly reported that vehicle traction power and
the LiDAR were off while the VCU logic remained powered from the NUC USB
connection.  Read-only host enumeration observed `/dev/ttyACM0`, character
device `166:0`, owner/group `root:dialout`, mode `0660`, USB VID:PID
`1a86:55d4`, USB serial `0002`, and no holder.  This identifies the USB bridge,
not the VCU hardware revision, Ackermann subtype, Flash parameters, or
installed MCU image.

The supplied source archive and candidate HEX remain identified by their
reviewed SHA-256 digests, but the production protocol exposes no firmware
version, build hash, command echo, acknowledgement, source timestamp, or parser
state.  Reading the installed STM32 image therefore requires a separately
approved SWD/JTAG or vendor-supported read-back procedure; it cannot be inferred
from USB descriptors or the 24-byte feedback stream.

## Decision

The work is split into three non-interchangeable stages.

1. **Power-isolated observation.** With traction power explicitly reported off,
   the existing identity-pinned `O_RDONLY` feedback capture may be used only
   after a fresh no-holder and device-identity check.  It sends no bytes.  The
   capture records USB-powered voltage and `FlagStop` only as observations; it
   does not prove the external motor bus is de-energized, identify firmware, or
   acknowledge a command.
2. **Prepared zero-only recovery.** Software and PTY evidence may implement and
   verify the recovery sequence below while the physical-actuation compile gate
   remains false.  A physical zero-only probe requires this ADR to be accepted,
   an independently confirmed traction-energy isolation, an identity-pinned
   dedicated tool incapable of encoding nonzero motion, and a new immutable
   evidence file.  The future write paths in the formal ROS node, installed
   bench, and non-installed raw-profile tool all prepare the same recovery
   primitive before open and invoke it immediately after a successful open.
   The compile gate prevents those actuation opens in this revision; only
   injected and PTY transports exercise the sequence.
3. **Released actuation.** The compile-time physical-actuation gate remains
   false until installed-firmware identity, physical zero-only recovery,
   unclean-restart, exclusive ownership, controller-watchdog disposition,
   external/durable E-stop strategy, and ground braking/holding evidence have
   all been reviewed.  A later ADR must explicitly authorize changing the
   constant; this ADR does not do so.

For the candidate parser, the deterministic recovery prefix is ten `0x00`
bytes followed by one checksum-valid exact-zero 11-byte command frame.  The
proof obligation is exhaustive over all eleven possible initial parser counts:

- at count zero, each padding byte is ignored because it is not `0x7B`;
- at counts one through ten, a padding byte eventually becomes byte 10 of the
  candidate, so its `0x00` value fails the required `0x7D` tail and resets the
  count; and
- all remaining padding is ignored at count zero, after which the exact-zero
  frame begins at a known frame boundary.

The recovery prefix contains no frame header and cannot itself form a candidate
command in the reviewed parser.  A failed, partial, late, or exception-escaped
padding or exact-zero write does not authorize motion.  The implementation
restarts from the ten-byte padding prefix only within a bounded zero-only
recovery operation.  It never appends a nonzero frame.  A complete host write
continues to mean only host transport completion, not VCU acknowledgement.

Every future physical command-channel open must perform this recovery
unconditionally before reads, backlog handling, authorization, or normal
frames.  Unconditional recovery is the durable unclean-session disposition:
it does not trust a previous process to have recorded a clean exit, and a new
process always starts disarmed with a new authorization epoch.  The adapter
does not automatically reopen a disconnected device.  Any nonzero-session
partial write or abnormal exit remains terminal for that process; a later
process may start only through the same recovery boundary.

Installed-firmware identity is a separate prerequisite.  The accepted evidence
must bind an STM32 read-back or authenticated vendor report to the exact VCU,
hardware subtype, configuration/Flash parameters, acquisition method, tool
versions, raw artifact digest, operator, and time.  USB VID/PID/serial,
feedback quantization, or behavioral similarity to the supplied source are
corroboration only.

Character-device ownership and USB ownership are also separate observations.
The hardened open checks the direct `/dev` path before open, the opened fd, and
the path again after `fstat`, including filesystem device, inode, device
major/minor, owner/group, and mode.  USB VID/PID/serial are validated during
preparation, before raw open.  The three-way devtmpfs identity check detects a
replaced device node but is not, by itself, a general post-open proof that the
same sysfs USB object still owns that device number under every hotplug race.
Until an allocation-free post-open sysfs binding check or equivalent reviewed
hotplug/ownership evidence exists, this remains an explicit release blocker;
the compile-time gate is not weakened on the assumption that inode and device
number are sufficient.

No step in this decision changes ROS 1 Noetic, the Phase-1 `0.50 m/s` forward
ceiling, the `0.20 m/s^2` longitudinal envelope, the `0.95 m` minimum runtime
turn radius, or the prohibition on reverse.  The reusable codec still requires
a finite configuration strictly inside `0 < v < 6.0 m/s`.

## Alternatives considered

- Sending one or several complete zero frames without padding was rejected.
  Repetition preserves the same unknown 11-byte phase and need never align.
- Treating a Linux close/reopen or a new connection generation as an MCU parser
  reset was rejected because neither event is visible to the reviewed parser.
- Inferring firmware identity from the WCH USB bridge or feedback quantization
  was rejected because both can remain unchanged across different MCU images
  and Flash settings.
- Detecting an unclean previous process through a best-effort file only was
  rejected as the primary recovery control.  A crash can occur before the file
  is updated; unconditional recovery removes that dependency.
- Enabling the general actuation transport for this test was rejected.  The
  physical evidence tool must be structurally unable to encode nonzero motion.

## Consequences

The host can have a small, reviewable parser-boundary recovery primitive and
can exhaustively test it without widening the Phase-1 command path.  The same
primitive can later make a clean or unclean process start independent of the
unknown retained byte count for the installed parser revision.

The sequence does not identify firmware, prove the traction bus is isolated,
or provide an acknowledgement.  Until read-back identity exists, its parser
proof remains conditional on the reviewed candidate source.  Until the
remaining watchdog, ownership, independent-stop, and braking/holding evidence
is accepted, the real backend and ground operation remain `UNVERIFIED` and
compile-time disabled.

## Compatibility and migration

No public ROS message, service, MD5 contract, domain type, launch default, or
Phase-1 vehicle limit changes.  The recovery primitive stays inside
`auto_rover_vcu_wheeltec_serial`.  Existing fake and PTY integrations remain
available.  Evidence schemas must distinguish padding bytes, the exact-zero
frame, host completion, controller acknowledgement availability, and physical
power-isolation attestation.

Live documentation and tests migrate from "exact zero is the first protocol
I/O" to "zero-only resynchronization is the first protocol I/O; exact zero is
the first valid command candidate."  Historic ADRs and captured evidence are
not rewritten.  The raw-profile v1 prefix record remains readable for historic
captures; new records additionally distinguish padding completion, exact-zero
completion, recovery restarts, and stream-poison state.

## Verification

Before any physical zero-only probe:

- exhaustively model initial parser counts `0..10` and prove that ten zero
  padding bytes leave the next `0x7B` at count zero;
- test successful, zero-byte, partial, disconnected, deadline, clock-rollback,
  and exception outcomes with injected transports and PTYs;
- prove every transmitted byte before recovery completion is either `0x00` or
  part of the one exact-zero frame; enumerate every production physical-open
  caller and prove each write-enabled caller invokes the prepared recovery
  before any read, record construction, backlog handling, or session I/O; and
  prove no ROS launch references the diagnostic;
- rerun the complete pinned Noetic/repository verification gate; and
- review a physical procedure that records fresh device identity, exclusive
  ownership, operator power-isolation attestation, raw TX/RX bytes, monotonic
  times, result hashes, and explicit lack of VCU acknowledgement.

Physical evidence must include a baseline `O_RDONLY` capture, a clean zero-only
recovery, and a new-process recovery after each deliberately injected partial
exact-zero prefix length `1..10`.  Because the candidate feedback has no command
echo, those runs establish host transcript and stable inhibited telemetry only.
Final parser acceptance additionally depends on installed-firmware identity or
another independently observable, non-actuating controller acknowledgement.
