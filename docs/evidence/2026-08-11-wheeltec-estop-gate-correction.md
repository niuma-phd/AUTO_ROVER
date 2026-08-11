# Wheeltec Hardware Stop-Gate Correction and Breaker Clarification

- Date: 2026-08-11 (Asia/Shanghai)
- Branch: `feat/nuc_ackermann`
- Perception processes: not started or exercised
- Mushroom-style hardware E-stop button: not installed
- Latching main-power breaker: conditionally used only for the later raised-bench
  follow-up
- Ground operation: prohibited

## Corrected operator timeline

The operator first stated that an independent emergency stop, lifted/fixed
wheels, wheel clearance, and an on-site observer were confirmed. The operator
then clarified that the vehicle has **no installed hardware emergency-stop
button**. That clarification supersedes the earlier button/E-stop attestation.
Lifted wheels and a software stop do not by themselves provide an independent
traction-energy cutoff.

The operator later supplied a separate fact: the vehicle has a reachable,
latching main-power breaker which a spotter can immediately open and hold open.
The operator states that it removes vehicle-battery power from the motors even
though the VCU stays powered at about 5 V through the NUC USB connection. This
is not a mushroom E-stop and its external wiring was not electrically proven by
the serial capture. A safety review nevertheless accepted it as a temporary,
independent traction-energy cutoff only for a raised, fixed, clear-wheel test
with continuous on-site control of the breaker.

The later breaker clarification does not retroactively make the earlier
exact-zero and aborted straight-ramp sessions compliant with their originally
recorded E-stop prerequisite. Those records remain non-acceptance protocol
observations. The later, explicitly breaker-gated sessions are separately
recorded in the
[main-breaker follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).
Neither set authorizes ground operation or a real ROS backend.

This is a hard gate because the supplied candidate firmware source shows all
of the following:

- serial command-loss stopping initializes disabled and the last target may be
  retained;
- startup contains an internally initiated `0.2 m/s` wheel-motion self-test;
- CPU fault handlers do not clear timer PWM; and
- the reviewed source does not prove an independent hardware power/enable cut.

A NUC watchdog, software E-stop service, serial zero frame, process shutdown,
or lifted vehicle is not an equivalent independent chain. For release or
ground operation, the physical stop chain still requires its own inspection,
procedure, and acceptance; the conditional raised-bench use of the breaker
does not satisfy that gate.

## Receive-only capture completed after the correction

A 20-second capture had already been started before the operator correction
arrived. It used the repository's `wheeltec_feedback_capture` path, selected
feedback-only access, opened `/dev/ttyACM0` `O_RDONLY`, and had no command or
write path. It was safe to let the bounded capture finish.

Pre-open observations were:

- direct character device `/dev/ttyACM0`, major/minor `166:0`;
- mode/owner/group `0660`, `root:dialout`;
- USB VID/PID `1a86:55d4`, serial `0002`, driver `cdc_acm`; and
- no existing holder reported by `fuser`; `lsof` reported only its unrelated
  FUSE visibility warning.

The completed file is:

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/flagstop_toggle_20s_01.ndjson` | `b57e4eb711b146ff75785e141e94796f345f5a3e2df7edbbda80b7481a0f9450` |

It recorded 9,888 raw bytes, 410 valid primary feedback frames, one checksum
failure, zero framing failures, 48 discarded bytes, no trailing buffer, and one
terminal read timeout. All 410 valid frames carried
`composite_stop_flag_raw=0`, `control_allowed=true`, and
`control_inhibited=false`. No stop or breaker transition occurred during this
capture, so it does **not** validate a 0-to-1-to-0 response or the breaker's
electrical action. `FlagStop=0` remains only a candidate current-cycle allow
observation, not an ACK or proof of safety.

A later `O_RDONLY` breaker-open capture reported live `FlagStop=1` at
approximately `4.46 V`, while a later reclose capture reported live
`FlagStop=0` at approximately `23.33 V`. Those observations support state
correlation for this attachment but have no VCU source timestamps and do not
measure the motor bus. Their raw hashes and backlog limitations are in the
[main-breaker follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).

## Disposition

The earlier blanket statement that no further nonzero testing was possible is
superseded only in this narrow respect: the later clarified main breaker could
conditionally serve as the independent traction-energy cutoff for the jointly
observed raised-bench follow-up. That follow-up was performed and both nonzero
runs safely aborted on unacceptable wheel-derived feedback. It did not close
the VCU readiness gate.

Do not perform a nonzero cable-loss, host-kill, or reconnect test because the
candidate firmware source predicts last-target retention. A further
raised-bench run requires a new explicit procedure and authorization. Ground
motion, the real ROS VCU backend, and release remain blocked until an
independently reviewed physical stop chain and all other hardware acceptance
gates pass.
