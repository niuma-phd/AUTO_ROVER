# NUC Ackermann Hardware Commissioning Handoff

The software-only phase is complete only when the pinned repository checks,
package tests, deterministic fake loop, failure injection, and PTY tests pass.
That evidence does not make the real Wheeltec path ready. The candidate codec,
transport, field meanings, serial timing, physical signs, controller watchdog,
and safe state remain `UNVERIFIED`, and the checked-in real-device gates remain
false.

Do not start physical work from a general navigation launch. Create an approved
acceptance profile from the
[pre-test template](../testing/nuc_ackermann_hardware_acceptance_profile.template.yaml),
name the decision authority and safety reviewer, and preserve results separately
using the
[results template](../testing/nuc_ackermann_hardware_results.template.yaml).

## Release and authorization gate

- [ ] ADR 0001 is reviewed and changed from `Proposed` to `Accepted` by the
  recorded decision authority.
- [ ] Dependency and license review approves every pinned binary and resolves
  the FAST-LIVO2 manifest/license conflict for the intended deployment.
- [ ] The exact vehicle, VCU hardware, firmware, FAST-LIVO2 revision, AUTO_ROVER
  commit, configuration blobs, operator, site, and test window are recorded.
- [ ] The independent physical emergency-stop chain is inspected, reachable by
  a second person, and verified under the vehicle's own safe procedure.
- [ ] A reviewed durable or external software-latch strategy preserves an
  asserted E-stop across vehicle-execution process restart; the current
  in-memory fake-only latch is not accepted as that evidence.
- [ ] The Wheeltec readiness gate has evidence for protocol ownership,
  transport settings, command/feedback encoding, safe state, inner-loop
  ownership, enable/fault behavior, watchdog, reconnect, and shutdown.
- [ ] The selected adapter is explicitly configured with the Phase-1
  `max_forward_speed_mps: 0.50`; omission, zero, non-finite values, and values
  at or above the generic codec's exclusive `6.0 m/s` configuration bound have
  been verified to inhibit nonzero output. The generic bound is not permission
  to raise the Phase-1 profile.
- [ ] Physical device selection has a deployment-approved allowlist and
  ownership policy. The bench implementation now has an identity-pinned udev
  rule, symlink-resistant open, pre/post-open TTY identity checks, exclusive
  ownership, USB VID/PID/serial verification, and serial-setting readback, but
  this one observed USB identity and the no-reopen bench session are not a
  deployment allowlist or reconnect design.
- [ ] The acceptance profile contains reviewed numeric criteria for every field
  required by `docs/testing/phase-1-acceptance.md`; no criterion is copied from
  a software fixture or filled after observing results.
- [ ] A reviewer explicitly authorizes the exact physical backend revision and
  both real-device opt-ins. Until then the serial path stays disabled and no ROS
  node is connected to it.

## Restrained bench sequence

Keep driven wheels clear of the ground, the work area exclusion zone clear, and
the physical emergency stop continuously available. A separately reviewed,
spotter-controlled traction-power breaker may be accepted for a specific
raised/fixed characterization session, but that exception does not satisfy the
release or low-speed ground gate.

1. Confirm the commanded forward axis, measured-speed sign, left-curvature
   sign, source/IMU axes, and the measured fixed transform to
   `rear_axle_center`. Any mismatch is a stop condition.
2. Capture raw, redistributable serial fixtures with firmware/hardware identity.
   Verify the candidate 11-byte command and 24-byte feedback layouts, byte order,
   scaling, checksum, malformed-frame resynchronization, and controller response.
3. Measure command and feedback rates, scheduling jitter, end-to-end latency,
   source timestamps, VCU watchdog behavior, loss-of-link state, reconnect
   behavior, and whether a local write has any acknowledgement semantics.
4. Verify zero/hold, bounded shutdown zero attempts, software guard timeout,
   independent adapter/output watchdog, fault inhibition, disconnect, reconnect,
   replay rejection, and mandatory re-authorization. A delivery-unconfirmed
   result is never recorded as a confirmed stop.
5. Apply an acceleration-limited short forward target whose commissioning goal
   is 0.50 m/s. Record the actual first nonzero wire command; it must be lower
   than 0.50 m/s under the 0.20 m/s² ramp. Do not add a hard minimum-speed jump.
6. Measure sustained wheel motion to determine start/stop deadband and
   hysteresis from actual command and valid measured speed. The 0.50 m/s target
   is not an assumed deadband. Preserve the historical 0.35 m/s post-collision
   fallback as a risk record, not as a newly approved target or pass criterion.
7. Test small left and right curvature under the approved
   `abs(curvature) <= 1 / 0.95 m` limit. The catalogue 0.80 m value has an
   unknown definition and cannot replace the 0.95 m operational radius.
8. Demonstrate ordinary deceleration, confirmed standstill, hold, power loss,
   software emergency-stop latch, authorized reset to disarmed, and the
   independent physical emergency-stop effect. Record time and distance using
   the pre-approved measurement methods.

Stop immediately on unexpected motion, wrong sign, missing/invalid feedback,
clock ambiguity, checksum disagreement, control-enable uncertainty, fault,
watchdog disagreement, ineffective hold, reconnect auto-motion, or an
unconfirmed safe state. Do not continue by widening a timeout, relaxing a
limit, disabling a latch, or reclassifying missing feedback as valid.

### 2026-08-11 restrained-bench outcome

The operator initially attested the independent emergency stop, raised/fixed
vehicle, wheel clearance, and spotter. The operator later clarified that no
hardware emergency-stop button is installed, so the independent-E-stop
attestation is retracted. Identity-gated receive-only captures established the
current candidate 24-byte stream at approximately 20 Hz. A two-second
exact-zero characterization completed 100 exact-zero host writes, zero
nonzero writes, and stationary feedback. These remain host/feedback
observations, not a compliant physical-safety acceptance or VCU
acknowledgements.

One reviewed straight-ramp attempt started at an actual first nonzero wire
speed of `0.012 m/s`. It was inhibited at a maximum command of `0.285 m/s`
when candidate feedback reached forward `0.007 m/s` and yaw `0.047 rad/s`,
above the provisional straight bench yaw gate. The independent final-zero host
write completed. A subsequent receive-only capture found an opening backlog
with delayed motion/yaw feedback followed by 60 live stationary samples at
approximately 20 Hz. The 0.50 m/s commissioning target was not reached and the
target hold and normal ramp-down were not exercised; the run was not retried.

Offline evidence analysis also found a software nonconformance: wire
quantization let normal positive increments reach `0.232924 m/s^2` despite the
nominal `0.20 m/s^2` request envelope. Later nonzero hardware runs were `NO-GO`
pending enforcement over consecutive completed wire writes without a fixed
quantization allowance. That software condition has now passed an independent
exhaustive property review, ASan/UBSan PTY tests, and the
full pinned Noetic gate; the failed run remains failed and any new physical run
still requires its own explicit, supervised authorization. See the
[raised-bench evidence](../evidence/2026-08-11-wheeltec-raised-bench-characterization.md).

That initial result did not check curvature, hardware watchdog, physical stop
effect, reconnect, enable/fault, gear, acknowledgement, ground-loaded
straightness, or the low-speed controlled-area items below.

After the no-button correction, a further 20-second `O_RDONLY` capture recorded
410 valid feedback frames, all with candidate `FlagStop=0`, and sent no
command. The operator then clarified that a latching main-power breaker removes
traction power while USB keeps the VCU logic alive. This did not retroactively
validate the earlier sessions, but it was conditionally accepted as the
spotter-controlled cutoff for a later raised/fixed follow-up.

In that follow-up, a breaker-open receive-only capture reported live
`4.459--4.460 V`, `FlagStop=1`; after reclose, live feedback reported
`23.332--23.335 V`, `FlagStop=0`. Opening backlogs had no source timestamps, so
the captures do not establish breaker latency or electrically prove the motor
bus. A post-reclose exact-zero run completed 61 full host writes and retained
stationary feedback. These are host-write and feedback observations, not VCU
acknowledgements.

Two corrected-slew nonzero runs then failed closed. The strict run aborted at a
maximum command of `0.312 m/s` on feedback `0.037 m/s` forward and
`0.211 rad/s` yaw. A requested 20-second-hold run, with a 30-second session
ceiling and explicit raised-wheel asymmetry handling, aborted at a maximum
command of only `0.080 m/s` on `-0.170 m/s` forward and `1.059 rad/s` yaw;
the derived rear-wheel values were approximately `-0.3405` and `0.0005 m/s`.
Both completed their independent final-zero host write. A later receive-only
capture contained an untimed opening motion backlog followed by 100 live
stationary samples. Neither run measured deadband, sustained onset, normal
ramp-down, stop hysteresis, or control delay. A final 60-second powered,
receive-only baseline accepted 1,213 frames; none exceeded absolute forward
speed `0.001 m/s` or absolute yaw `0.011 rad/s`. It therefore contradicts
treating the `-0.170/1.059` trigger as ordinary stationary noise, without
uniquely identifying the root cause. See the
[main-breaker follow-up](../evidence/2026-08-11-wheeltec-breaker-raised-bench-follow-up.md)
and the corrected
[stop-gate timeline](../evidence/2026-08-11-wheeltec-estop-gate-correction.md).

A later, explicitly authorized experimental raw-calibration profile completed a
`0 -> 0.50 m/s`, 60-second hold, and normal ramp-down under the same raised-
wheel boundary. The 1,200 hold samples had mean/median forward feedback
`0.4905125/0.490 m/s`; derived left and right means were
`0.490998/0.490027 m/s`. The first receipt-aligned per-wheel motion outside the
`0.005 m/s` standstill envelope occurred with the latest complete command at
`raw=288`, and both wheels exceeded the positive threshold at `raw=307`.
Without VCU source timestamps, those values are host-observed onset brackets,
not an accepted deadband or pure control delay.

The run's terminal `feedback_limit_violation` occurred only after the normal
zero write: a residual-motion frame with yaw `0.153 rad/s` immediately hit the
restored strict post-zero `0.023 rad/s` yaw gate before the standstill-wait path
could complete. The independent emergency-zero host write completed. A later
receive-only capture contained nine untimed backlog frames followed by 202 live
stationary frames over `9.981742785 s`. This establishes later stationary
feedback, not a VCU stop ACK or stop time. Full statistics and hashes are in the
[main-breaker follow-up](../evidence/2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).

The later dedicated raw-profile tool ultimately completed `52/52` requested,
separately invoked raised-wheel profiles. Twelve straight profiles covered
`0.50--6.0 m/s`; forty outer-rear-wheel-tier turns covered
`0.50--5.0 m/s`, left and right, at R=`2.0 m` and R=`0.95 m`. Every profile
held its target for approximately 60 seconds, ended with `status=completed`,
and completed its final exact-zero host write. All `93,599` normalized feedback
frames retained candidate `FlagStop=0`, and all aggregate parser-integrity
counts were zero.

The twelve straight hold means fit
`feedback = 1.000313388 * command - 0.010680115 m/s` with
R-squared `0.9999999375`. All forty turns had center-speed mean error from
`-0.010973` to `-0.008297 m/s` and yaw mean error from `-0.003028` to
`+0.004123 rad/s`. Across twenty left/right pairs, maximum absolute
center-speed mirror residual was `0.000629 m/s` and yaw antisymmetry residual
was `0.003553 rad/s`. The derived inner/outer rear-wheel ordering matched the
experimental sign convention through the executed `5.0 m/s` outer-wheel tier.

Those are unloaded mechanism observations, not acceptance results. The VCU
provided no ACK, command echo, source timestamp, or sequence number, and the
tool recorded feedback tracking, asymmetry, and post-zero motion rather than
using them as acceptance gates. Full raw and analysis paths, SHA-256 values, and
statistics and the machine-readable aggregate are in the
[expanded profile matrix](../evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md).
The experimental executable remains uninstalled and disconnected from ROS.
These runs do not change the Phase-1 ROS production configuration's `0.50 m/s`
target and ceiling, authorize ground motion, or close the real-backend gate.

The matrix supports a `20 Hz` expected feedback rate, `150 ms` experimental
freshness deadline, and inclusive `0.005 m/s` per-derived-wheel standstill
candidate for this raised fixture. Straight sustained-motion candidates were
associated with `0.228--0.338 m/s` commands, median `0.266 m/s`, but they do
not define deadband or pure delay because the input was ramping and the VCU has
no ACK or source timestamp. Similarly, post-zero receipt candidates do not
prove braking or hold. The catalogued `5.5/6.0 m/s` turn profiles were outside
the final requested scope and were not executed.

### Supplied C50C firmware audit and remaining raised-bench work

The operator subsequently supplied the candidate STM32 source archive. Static
inspection confirms the 11-byte command and 24-byte feedback layouts and
defines feedback byte 1 as a composite, non-latched `FlagStop`. A value of zero
means the firmware's current 100 Hz check permits actuation; one means at least
one of low voltage, PE4 enable, self-check, or software-stop currently inhibits
it. It is not an ACK or a specific fault code. Values outside zero and one must
fail closed.

The source also explains the stationary `±0.001 m/s` and `±0.011 rad/s`
samples as subtype-9 encoder quantization. The user's 6 m/s description and
this exact quantization are strong evidence for subtype 9, but the ADC-selected
type and Flash-overridden PI/LineDiff/servo parameters still require a running
report or display observation. The 6 m/s firmware capability does not change
the 0.50 m/s commissioning ceiling.

Three source findings prohibit ground operation in the current state:

- startup requests `0.2 m/s` for approximately 10--12 seconds as an internal
  motor self-test;
- `SecurityLevel=1` disables command-loss stopping by default, so a lost host
  can leave the last nonzero target active; and
- CPU fault handlers loop without clearing timer PWM and no active independent
  MCU watchdog was found.

The supplied HEX has not been read back from this VCU, and the reviewed source
does not prove an independent physical power cut. The operator's main-breaker
description and voltage/`FlagStop` correlation are useful raised-bench evidence,
but they are not an electrical review of the external motor bus. These facts
are documented in the
[C50C firmware source audit](../evidence/2026-08-11-wheeltec-c50c-firmware-source-audit.md).

The corrected wire-level `0.20 m/s²` implementation and its independent
PTY/property review have passed. Breaker open/reclose, post-reclose exact zero,
two earlier bounded aborts, manual direction mapping, and the requested `52/52`
profile matrix are complete. This completes the requested raised-wheel profile
collection only; it did not produce a source-timed or acknowledged stop and
did not exercise the Phase-1 ROS backend. Before any production promotion or
ground work:

1. resolve the powered drive/PI response or physical-drive asymmetry that
   produced the derived negative wheel value; breaker-open manual rotation has
   separately confirmed the candidate left/right encoder channels and signs;
2. record the displayed/reported Ackermann subtype, hardware revision, and
   installed firmware/Flash parameter identity;
3. establish loaded sustained-onset and stop-tail criteria using a measurement
   method that does not infer physical timing from source-timestamp-free
   feedback; the raised matrix only brackets straight onset candidates at
   `0.228--0.338 m/s`;
4. characterize and review command-loss, watchdog, reconnect, restart,
   shutdown, braking/hold, and safe-state behavior for the exact installed VCU;
5. retain front-wheel centring and both rear-wheel direction as immediate
   operator stop conditions; and
6. preserve the independent traction-energy cutoff and obtain the separate
   release-grade emergency-stop disposition required for ground work.

Do not label `0.50 m/s`, `0.312 m/s`, `0.080 m/s`, or an offline receipt-
associated onset candidate as a fixed deadband.

Do not intentionally test host-kill or cable-loss persistence with a nonzero
command: the candidate source already predicts last-command retention. Treat
the lifted test as mechanism characterization, not as real-backend or ground
acceptance.

## Low-speed controlled-area gate

Low-speed ground work is permitted only after every applicable bench item and
numeric gate passes and the recorded decision authority issues an explicit
go/no-go approval.

- [ ] Reconfirm exclusion zone, spotter, physical emergency stop, surface and
  environmental envelope, localization axes/extrinsic, route revision, and
  software/hardware revisions.
- [ ] Use a short forward-only route at a 0.50 m/s target and ceiling. Preserve
  the acceleration/deceleration ramp and 0.95 m operational radius.
- [ ] Record every required completion, tracking, command-age, stale-input,
  stop-time, stop-distance, hold-release, and recovery metric against the
  immutable pre-test criteria.
- [ ] Abort a run that leaves the approved envelope. Record it as aborted; do
  not edit thresholds to make it qualifying.
- [ ] Store raw data, calculations, actual operator and conditions, anomalies,
  reruns, and residual risks in a result set that references the exact profile
  commit and blob identity.

Real-vehicle readiness requires separate protocol, safety, and acceptance
approval. Passing the fake loop or PTY suite cannot check any box that depends
on physical behavior.
