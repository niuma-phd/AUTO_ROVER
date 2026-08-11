# Wheeltec expanded raised-bench raw-profile matrix

## Scope and disposition

On 2026-08-11 the operator authorized a raised, fixed, continuously supervised
VCU characterization. Perception and the ROS navigation graph were not run.
The final requested matrix was:

- twelve straight profiles from `0.50` through `6.0 m/s` in `0.50 m/s`
  increments; and
- forty outer-rear-wheel-tier turn profiles from `0.50` through `5.0 m/s`,
  left and right, at radii `2.0 m` and `0.95 m`.

Each profile was a separate process with a wire-level `0.20 m/s^2` center-speed
ramp, an approximately 60 second hold, a normal ramp to exact zero, and a final
bounded exact-zero attempt. The requested turn scope was deliberately stopped
at `5.0 m/s`; the catalogued `5.5` and `6.0 m/s` turn profiles were not run.

This is unloaded mechanism evidence. It does not authorize ground motion,
change the Phase-1 or production `0.50 m/s` ceiling, establish front-wheel
steering geometry, or make the real VCU backend ready. The VCU supplies no
command ACK, echo, source timestamp, or sequence number.

## Executable and source provenance

The physical profiles used this uninstalled, ROS-free executable:

| Item | Path | SHA-256 |
|---|---|---|
| executable | `/tmp/auto_rover_raw6_release.WX54xa/wheeltec_raw_profile_capture` | `cbeeddc50af1a63f07f78d7e68d869dff99e9615f952896131ca6d69bb5329df` |
| public experimental header | `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/include/auto_rover_vcu_wheeltec_serial/raw_profile.hpp` | `536f3ed2d72794ae95dbe1ae77831492eba61df7f08aedd59663e45fe7df0dfe` |
| session implementation | `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/src/raw_profile.cpp` | `82517575015802ea39bc357f447fdfa8cb1637dda390b510783be94c3ef3d798` |
| CLI implementation | `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/src/raw_profile_main.cpp` | `72b36689470e400179f6c0b9305114fb65dfc65ecdd8785d6f5ab10aee1b8432` |
| offline analyzer | `tools/vehicle/analyze_wheeltec_profile.py` | `3e6be70cbb7617a059a808061709d5bf2e5535fd3855b911d50f279f047ee0f4` |

All runs pinned character device `166:0`, owner/group `0:20`, USB VID/PID
`1a86:55d4`, and USB serial `0002`. They reused the reviewed passive-capture
and exact-zero evidence tokens recorded in the
[main-breaker follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).

## Aggregate evidence integrity

The machine-readable aggregate is:

| Artifact | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_requested52_aggregate_01.json` | `e0b86a86450f63a9f03bfab47196d604a160c7a3668d74abf7afec815688038c` |

Its complete per-run inventory manifest SHA-256 is
`6468c50de67e1b84fbf3426f7310a7cd86122cd3ff7114aa0b3d305c63e6d462`.
The aggregate embeds every raw and analysis filename, byte count, SHA-256,
profile contract, event-ledger check, terminal result, and hold statistic.

The requested-scope audit result was:

- `52/52` profiles valid, `0` missing, and `0` invalid;
- all `52` terminal states were `completed`;
- every profile completed its final exact-zero host write and reported
  `delivery_unconfirmed=false`;
- all `93,599` normalized feedback frames carried candidate `FlagStop=0`;
- aggregate checksum failures, framing failures, discarded bytes, and trailing
  buffered bytes were all zero;
- all `20/20` requested left/right mirror pairs were present;
- the eight `5.5/6.0 m/s` turn catalog slots were explicitly
  `out_of_requested_scope`, with no unexpected evidence file; and
- the runs recorded `354,771,219` raw-evidence bytes, `47,817,240` analysis
  bytes, `62,443` hold frames, and `140,403` successful normal host writes.

A successful host write means only that the host operating system reported a
complete 11-byte write. It is not a VCU acknowledgement or proof of physical
execution.

## Straight profile results

The table uses every normalized feedback frame associated with each 60-second
hold. `P05/P95` and minima/maxima are host-receipt populations, not accuracy
specifications.

| Command, m/s | Mean feedback, m/s | Mean error, m/s | P05/P95, m/s | Min/max, m/s | Mean supply, V |
|---:|---:|---:|---:|---:|---:|
| `0.5` | `0.489943` | `-0.010057` | `0.458/0.521` | `0.432/0.542` | `23.308` |
| `1.0` | `0.989237` | `-0.010763` | `0.949/1.023` | `0.877/1.053` | `23.300` |
| `1.5` | `1.490442` | `-0.009558` | `1.411/1.557` | `1.379/1.576` | `23.285` |
| `2.0` | `1.989551` | `-0.010449` | `1.899/2.058` | `1.874/2.082` | `23.270` |
| `2.5` | `2.489495` | `-0.010505` | `2.399/2.554` | `2.342/2.571` | `23.264` |
| `3.0` | `2.990044` | `-0.009956` | `2.922/3.036` | `2.877/3.057` | `23.248` |
| `3.5` | `3.490187` | `-0.009813` | `3.424/3.538` | `3.390/3.557` | `23.233` |
| `4.0` | `3.990741` | `-0.009259` | `3.938/4.053` | `3.892/4.080` | `23.220` |
| `4.5` | `4.491391` | `-0.008609` | `4.429/4.577` | `4.404/4.614` | `23.207` |
| `5.0` | `4.991213` | `-0.008787` | `4.925/5.077` | `4.885/5.126` | `23.196` |
| `5.5` | `5.490557` | `-0.009443` | `5.418/5.581` | `5.378/5.625` | `23.186` |
| `6.0` | `5.991258` | `-0.008743` | `5.924/6.074` | `5.886/6.108` | `23.181` |

Least-squares fitting of the twelve hold means gives:

```text
feedback_forward_mean_mps =
    1.000313387998 * command_forward_mps - 0.010680114866
R^2 = 0.999999937457
fit residual RMSE = 0.000431792 m/s
```

The fixed approximately `-0.010 m/s` mean offset is a repeatable raised-wheel
observation. It is not yet a ground-loaded calibration correction. Straight
single-frame yaw was much noisier than its near-zero hold mean: the largest
per-profile absolute hold extreme was `0.753 rad/s`, and the P05/P95 bounds
widened to roughly `-0.517/+0.529 rad/s` at some targets. A single-frame yaw or
derived-wheel tracking gate would therefore reject normal raised-wheel data.

## Turn profile results

The expanded turn tier is an outer-rear-wheel command ceiling, not a center
speed. Integer protocol values choose the largest center speed for which the
derived outer command stays below the tier and the inner command is
nonnegative. Both radii used the project convention `yaw_rate = center / R`,
positive left and negative right.

Across all forty turns:

- center-speed hold-mean error ranged from `-0.010973` to `-0.008297 m/s`;
- inner-wheel hold-mean error ranged from `-0.010899` to `-0.008433 m/s`;
- outer-wheel hold-mean error ranged from `-0.011636` to `-0.008162 m/s`;
- yaw hold-mean error ranged from `-0.003028` to `+0.004123 rad/s`;
- left/right mirror center-speed residual had maximum absolute value
  `0.000629 m/s` and RMS `0.000313 m/s`;
- left/right mirror yaw antisymmetry residual had maximum absolute value
  `0.003553 rad/s` and RMS `0.001942 rad/s`; and
- mirrored inner- and outer-wheel hold-mean residuals had maximum absolute
  values `0.001027` and `0.000902 m/s` respectively.

At the highest executed tier:

| Radius | Direction | Center/yaw command | Center/yaw hold mean | Outer-wheel hold mean |
|---:|---|---:|---:|---:|
| `2.0 m` | left | `4.627 m/s`, `+2.313 rad/s` | `4.618703 m/s`, `+2.313843 rad/s` | `4.991231 m/s` |
| `2.0 m` | right | `4.627 m/s`, `-2.313 rad/s` | `4.618074 m/s`, `-2.315863 rad/s` | `4.990928 m/s` |
| `0.95 m` | left | `4.275 m/s`, `+4.500 rad/s` | `4.265836 m/s`, `+4.502843 rad/s` | `4.990794 m/s` |
| `0.95 m` | right | `4.275 m/s`, `-4.500 rad/s` | `4.265898 m/s`, `-4.501532 rad/s` | `4.990644 m/s` |

This strongly supports the candidate host yaw sign, `0.322 m` rear-track
reconstruction, and mean left/right symmetry on the raised fixture. It does
not prove actual front-wheel angle, loaded curvature, tire slip, or a physical
`0.95 m` path radius.

## Rate, standstill, onset, and stop observations

Feedback receipt rate over the 52 runs was `19.9973--20.0059 Hz`. The largest
positive inter-receipt interval was `61.257 ms`; no interval exceeded the
analyzer's three-median gap criterion. This supports retaining the experimental
`150 ms` feedback-freshness deadline. It does not establish a controller-side
source-time deadline because the frame has no source timestamp.

The 52 five-frame pre-motion baselines contained 287 samples. Absolute P95/max
values were `0.001/0.003 m/s` forward, `0.011/0.023 rad/s` yaw, and
`0.002771/0.003703 m/s` for each derived rear wheel. Together with the
independent stationary capture and subtype-9 count estimate, this supports the
existing pre-motion raised-bench observation of forward/each derived wheel at
or below `0.005 m/s` and yaw at or below `0.023 rad/s`, using at least five
distinct receipts spanning at least `0.20 s`. It does not establish a loaded
hold or brake threshold.

For the twelve straight ramps, the analyzer's first five-frame sustained-motion
candidate was associated with command `0.228--0.338 m/s`, median `0.266 m/s`.
The host interval from first nonzero write to first candidate receipt was
`1.200--1.760 s`, median approximately `1.400 s`. Across all 52 profiles the
associated center command ranged `0.167--0.338 m/s`. These are useful bounds
for a later dedicated deadband test, but they are not a deadband or pure VCU
delay: the command was continuously ramping at `0.20 m/s^2`, feedback has no
source time, and the first derived-wheel values were sometimes asymmetric.
No automatic minimum-speed jump is justified.

Forty-nine profiles produced a five-frame post-zero return candidate. The
first-return interval was `0.020--1.460 s` with median `0.200 s`, and the
five-frame confirmation interval was `0.220--1.661 s` with median `0.400 s`.
The last sample above each run's exact observed baseline occurred as late as
`2.040 s`; three profiles did not confirm return inside their approximately
two-second observation. This exact-baseline heuristic is too sensitive to
encoder quantization to be a physical stop-time criterion. Future evidence
should report the strict pre-motion envelope separately from a post-motion
quiet indicator and retain at least a three-second post-zero observation. The
data does not prove braking, holding, or a VCU zero ACK.

A looser, non-blocking quiet indicator using absolute forward `0.01 m/s`, each
derived wheel `0.02 m/s`, yaw `0.06 rad/s`, and five distinct receipts found an
initial window in all 52 profiles. It still had later deassertions, and one run
ended before a final five-frame reconfirmation after its last excursion. Such
an indicator may be useful for plots and operator diagnostics only; it must
deassert on any violation and must not authorize an E-stop reset or claim a
physical stop.

## Evidence-derived configuration disposition

The following values now have repeatable raised-wheel evidence and may be used
as experimental diagnostic candidates while the backend remains disabled:

- expected feedback rate: `20 Hz`;
- host receipt freshness deadline: `150 ms`;
- per-derived-rear-wheel standstill envelope: inclusive `0.005 m/s`;
- standstill evidence: five distinct receipts spanning at least `0.20 s`;
- nominal unloaded hold-mean relationship: the fitted straight equation above;
  and
- nominal left/right mirror comparison: use hold/window statistics, not a
  single feedback frame.

The observed 60-second mean speed errors all fit inside `0.012 m/s`; a rounded
`0.02 m/s` mean-error band is a reasonable future raised-bench diagnostic, but
is not a real-time or ground safety limit. For offline mirror regression,
rounded candidates of `0.005 m/s` for inner/outer mean mismatch and
`0.01 rad/s` for yaw antisymmetry leave margin above every observation.
Individual straight samples ranged as far as `-0.158/+0.126 m/s` around the
target, and turn yaw sample error reached `1.041 rad/s` even though every
turn's 60-second mean yaw error stayed within `0.004124 rad/s`. A non-blocking
forward tracking warning at absolute error above `0.15 m/s` for five distinct
receipts would not have fired on this matrix; the only two excursions above
that value were isolated single frames. Tracking and yaw decisions must use a
specified observation window whose distribution is separately tested; this
matrix does not justify any new per-frame tracking hard stop.

The onset candidates justify a focused later sweep around `0.20--0.35 m/s` if
a precise loaded deadband is needed. They do not justify treating `0.266`,
`0.30`, or `0.50 m/s` as a hard deadband or automatically boosting small
commands. The stop candidates do not justify a braking-delay threshold.

The production and Phase-1 target/ceiling remains `0.50 m/s`, the operational
minimum turn radius remains `0.95 m`, and real actuation remains disabled by
default. Ground-loaded steering, braking, watchdog/loss-of-link behavior,
installed firmware identity and Flash parameters, control enable/fault
semantics, reconnect, and an independent release-grade emergency-stop chain
remain `UNVERIFIED`.
