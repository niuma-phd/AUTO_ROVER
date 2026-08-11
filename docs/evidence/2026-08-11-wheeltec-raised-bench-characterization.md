# Wheeltec Raised-Bench Command Characterization

- Date: 2026-08-11 (Asia/Shanghai)
- Branch: `feat/nuc_ackermann`
- Protocol and hardware capability status: `UNVERIFIED`
- Physical state initially attested by the operator: independent emergency
  stop, vehicle lifted and fixed, wheel clearance, and on-site spotter
- Later correction: no hardware emergency-stop button was installed; the
  independent E-stop attestation is retracted
- Later clarification: a latching main-power breaker cuts traction energy while
  USB keeps the VCU logic powered; it was conditionally accepted only for a
  subsequent supervised raised-bench follow-up
- Perception scope: excluded
- Ground motion: prohibited and not performed

This record continues the identity-gated
[passive feedback capture](2026-08-11-wheeltec-passive-feedback-capture.md).
It characterizes a candidate VCU protocol on a raised bench. A successful host
write is not a VCU acknowledgement, and this evidence does not authorize the
real phase-1 closed loop or ground operation.

## Post-run physical-safety correction and later clarification

Before a later motion run, the operator clarified that the vehicle has no
hardware emergency-stop button. The earlier independent-E-stop attestation was
therefore incorrect. The raw exact-zero and aborted-ramp observations below are
retained for protocol analysis, but the required physical-safety gate was not
actually evidenced during those command sessions.

The operator later clarified that a latching main-power breaker independently
removes battery traction power while the VCU remains USB-powered. That later
fact does not retroactively satisfy the earlier sessions' recorded prerequisite.
It conditionally enabled only a later raised/fixed, spotter-controlled follow-up.
None of these records authorizes ground operation. See the
[stop-gate correction](2026-08-11-wheeltec-estop-gate-correction.md) and
[main-breaker follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).

## Pre-command gates

Immediately before each command run, the direct TTY was `/dev/ttyACM0`, mode
`0660`, owner/group `root:dialout`, character-device major/minor `166:0`, USB
VID/PID `1a86:55d4`, serial `0002`, driver `cdc_acm`, with no holder reported
by `fuser` or `lsof`. No vendor, vdemo, or ROS serial node was used. The udev
daemon had successfully reloaded the identity-pinned least-privilege rule.

The command tool was an original ROS-independent implementation. It required
three physical opt-ins, an exact operator confirmation, complete device
identity, and operator-attested evidence SHA-256 values before physical open.
It enforced candidate-frame structure, a 0.50 m/s hard ceiling, no reverse,
zero lateral/yaw command for this test, feedback freshness and standstill gates,
bounded write deadlines, and an independent final exact-zero write path.
Signals, feedback/clock failures, and evidence-record failures revoke
authorization before the bounded final-zero attempts.

## Exact-zero result: retained host observation; physical gate nonconforming

The exact-zero run used the passive evidence SHA-256
`9e43dc4c87abbfdfddcdfe9668a6e76d7f7582ca0e9864d2c997f271b91fff7b`.
The executed sanitizer-instrumented command binary had SHA-256
`117116302a61a697e3b736021641653a4586e7c5261933d41657ecbf1d795e41`.

The two-second run completed with:

- 40 accepted feedback frames;
- 100 completed host writes, all exact zero;
- zero nonzero host writes;
- one successful independent final-zero host write included in that total;
- final authorization revoked; and
- no host-side delivery-unconfirmed result.

Every TX record containing an 11-byte frame had the single value:

```text
7B 00 00 00 00 00 00 00 00 7B 7D
```

The feedback was 39 samples at `0.000 m/s` and one at `0.001 m/s`, lateral
speed was always zero, yaw feedback was 39 samples at zero and one at
`-0.011 rad/s`, and byte 1 was always zero. The subsequently supplied firmware
source defines byte 1 as a composite `FlagStop`; zero is consistent with its
source-level control-allowed state. Because the connected VCU has not been
read back and no physical inhibit transition was captured, this is not yet an
installed-firmware identity or enable-chain verification. The run shows that
the candidate zero frame did not produce an observed motion-feedback response;
it does not prove disable, park, E-stop, braking, or acknowledgement semantics.

Raw evidence:

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/exact_zero_2s_01.ndjson` | `5d7ae4163fa8a3d33474dd33fa5bd6f73d887152652a9e0de7202654138f8b14` |

## Straight-ramp result: safely inhibited; physical gate nonconforming

An independent software review conditionally allowed one raised-bench,
straight `0 -> 0.50 -> 0 m/s` characterization with a one-second hold. The run
used the exact-zero SHA-256 above and a freshly built GCC C++14 `-O2 -Werror`
binary, SHA-256
`43b665e6bc5f7d1326236c911f4695aaebe836dd5642ad6327813c342e1936a3`.
The relevant `bench.cpp` snapshot SHA-256 was
`2ced2890807c1f2d1bdef6511bc96249efa9cb64daf83846fc131c576b490d4d`.

The run did not reach 0.50 m/s. It ended after approximately 1.65 seconds with
`feedback_limit_violation`:

- first completed nonzero host write: `0.012 m/s`;
- maximum commanded wire speed: `0.285 m/s`;
- 69 completed nonzero host writes and 10 completed zero host writes;
- all commands had reserved bytes, lateral speed, and yaw rate equal to zero;
- last accepted feedback before the violation remained near zero; and
- the rejected candidate feedback was forward `0.007 m/s`, lateral
  `0.000 m/s`, yaw `0.047 rad/s`, and byte 1 zero.

The yaw value exceeded the passive-evidence-only straight bench gate of
`0.023 rad/s`. The harness revoked authorization and completed the independent
exact-zero host write; that direct attempt began 17.654 microseconds after the
terminal RX call completed. The session reported no host-side delivery
uncertainty, but there was still no VCU acknowledgement. The target, hold, and
normal ramp-down phases were **not tested**. Per the one-run review condition,
the ramp was not automatically retried. This is both a straight/yaw bench
nonconformance and, separately, the wire-slew software nonconformance below.

The supplied subtype-9 source later explained the scale of this value. Its yaw
feedback is `(right_rear_speed - left_rear_speed) / 0.322`, derived from a
10 ms encoder window rather than from the IMU or steering angle. One count of
rear-wheel difference corresponds to approximately `0.011772 rad/s`; the
reported `0.047 rad/s` is therefore consistent with a four-count difference.
It remains valid evidence of unequal raised-wheel response, but it is not a
direct measurement of body yaw or front-wheel steering.

Raw evidence:

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/straight_ramp_0p50_hold1s_01.ndjson` | `892dd3348d5518ab197fb1cbd9d69e90fd1c20a55176eea2284b8585b53876d7` |

## Stop verification and delayed feedback

A three-second receive-only capture followed the failed ramp. Its opening
pre-open receive backlog contained ten ordered motion-transient frames,
starting at forward/yaw
`0.007/0.047`, peaking at `0.089 m/s` and `0.576 rad/s`, then decreasing to
`0.060/0.376`. These frames shared receive receipts and had no VCU source
timestamp, so their exact production times and stop latency cannot be inferred.
Their ordered shape is consistent with a cached delayed response and strong
non-straight wheel-derived motion while the raised wheels were unloaded, but
the evidence cannot bind them to a particular instant before or after the
emergency-zero write. The opening cross-frame backlog also caused 40 discarded
bytes and two framing failures; it is not classified as a clean live stream.

After the opening backlog, 60 live frames at 20.00084 Hz were all within the
observed stationary quantization set: forward `-0.001`, `0.000`, or
`0.001 m/s`; lateral zero; yaw `-0.011`, `0.000`, or `0.011 rad/s`; byte 1
zero. Thus the available live feedback confirms a stationary software
observation after the final zero, without claiming a VCU stop acknowledgement.

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/post_ramp_stop_3s_01.ndjson` | `1636cb58b23f5ff01d2b80ff20e6df03beeefbe1ff22a1bd85b6a667de838175` |

## Wire-slew evidence defect and disposition

Offline analysis found that the then-current test and implementation allowed a
fixed `0.001 m/s` quantization allowance on top of the nominal acceleration
envelope. Twenty-eight of 64 positive normal wire increments exceeded a strict
`0.20 m/s^2` ratio when measured using completed host-write timestamps; the
maximum was `0.232924 m/s^2` for a `0.015 -> 0.020 m/s` step over 21.466 ms.
The independent reviewer withdrew the prior unqualified wire-envelope finding.

This software issue did not defeat the feedback abort or final-zero path, but
it made later nonzero hardware runs `NO-GO` until a wire-level,
quantization-coupled constraint over consecutive completed normal writes was
implemented without a fixed 1 mm/s allowance. An emergency safety-zero remains
an explicit step exception.

That software correction is now complete. The bench transport independently
enforces
`abs(raw_i - raw_(i-1)) * 5,000,000 <= before_i_ns - after_(i-1)_ns`
for every normal zero, ramp-up, hold, and ramp-down write; only a complete,
host-confirmed write advances the baseline. A separate reviewer passed
2,516,776 property checks over all 0..500 raw-speed pairs and
`5 ms * n - 1 ns`, exact, and `+1 ns` boundaries. GCC 9.4 C++14 `-Werror`
ASan+UBSan tests passed 3/3 under the pinned Focal CMake 3.16 toolchain, and the
full Noetic workspace gate passed. This closed the identified software blocker
for a later explicitly authorized, jointly observed raised-bench run; it did
not reclassify the failed evidence above or verify physical behavior. That
later run was performed with the clarified main-breaker gate and separately
aborted on unacceptable wheel-derived feedback.

## Main-breaker follow-up

The later follow-up captured the breaker-open state at approximately `4.46 V`
with `FlagStop=1`, the reclosed state at approximately `23.33 V` with
`FlagStop=0`, and a successful two-second exact-zero host session. A strict
straight-ramp run aborted at a maximum command of `0.312 m/s`; a requested
20-second-hold run with raised-wheel asymmetry handling aborted at a maximum
command of only `0.080 m/s` when its derived per-rear-wheel direction gate
failed. Both completed their independent final-zero host write. A subsequent
receive-only capture contained an untimed opening backlog followed by 100 live
stationary samples. A separate 60-second powered, receive-only baseline then
kept all 1,213 frames within `|forward| <= 0.001 m/s` and
`|yaw| <= 0.011 rad/s`; the abort trigger is therefore not classified as
ordinary stationary noise.

The raw file hashes, exact rejected feedback, derived rear-wheel values, and
source-time limitations are recorded in the
[main-breaker raised-bench follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).
No deadband or control-delay value was established.

## Mechanism conclusion

The evidence supports the following narrow conclusions for this attachment:

- the current VCU actively emits the candidate 24-byte feedback at about 20 Hz;
- the candidate 11-byte zero and straight positive-speed frames were accepted
  by the host serial stack;
- nonzero command transmission produced delayed nonzero motion/yaw feedback;
- the raised-wheel straight response did not satisfy the provisional yaw gate;
- byte 1 remained zero, consistent with the supplied source's composite
  `FlagStop=0` control-allowed state, but not with a command ACK or a specific
  fault, gear, steering, or braking state;
- a final zero was followed by live stationary feedback; and
- feedback lacks source time, so opening backlogs cannot establish event
  latency or refresh a control watchdog.

The supplied source artifact defaults its controller-side command-loss stop to
disabled and contains an automatic `0.2 m/s` startup self-test around 10--12
seconds after boot. It also does not establish an independent PWM-clearing CPU
fault path. Whether that exact HEX is installed, the physical enable/E-stop
chain, gear, steering angle, command acknowledgement, exact command-to-motion
scale, ground-loaded straightness, and safe reconnect behavior remain
unverified. See the
[firmware source audit](2026-08-11-wheeltec-c50c-firmware-source-audit.md).
Real phase-1 actuation stays disabled.
