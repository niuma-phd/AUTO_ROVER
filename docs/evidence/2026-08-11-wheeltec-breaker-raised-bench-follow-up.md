# Wheeltec Main-Breaker Raised-Bench Follow-up

- Date: 2026-08-11 (Asia/Shanghai)
- Branch: `feat/nuc_ackermann`
- Protocol and installed-firmware status: `UNVERIFIED`
- Scope: isolated serial/VCU characterization on a raised and fixed vehicle
- Perception scope: excluded; LiDAR, camera, FAST-LIVO2, and localization were
  not exercised
- Ground operation: prohibited and not performed

This record follows the earlier
[raised-bench characterization](2026-08-11-wheeltec-raised-bench-characterization.md).
The vehicle has no mushroom-style hardware emergency-stop button. The operator
subsequently clarified that a reachable, latching main-power breaker removes
battery power from the motors while the VCU logic remains powered from the NUC
USB connection. With the vehicle raised and fixed, wheel clearance maintained,
and a spotter continuously able to hold that breaker open, it was accepted only
as a temporary independent traction-energy cutoff for this supervised session.
It is not evidence of a release-grade E-stop, and it does not authorize ground
operation or the real ROS backend.

Every successful TX result below means only that the host operating system
reported a complete 11-byte serial write. The candidate protocol contains no
command sequence or acknowledgement, and `FlagStop` is a composite current-cycle
allow/inhibit bit rather than an ACK or a specific fault.

## Breaker-open and reclose observations

The breaker-open and reclose checks used feedback-only `O_RDONLY` access and
sent no commands.

| State | File | SHA-256 |
|---|---|---|
| breaker open | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/breaker_off_readonly_15s_01.ndjson` | `7e20a74403c8cd3e0b8bc64ead747f8de3dc42b78a855da3e4e61510accef764` |
| breaker reclosed | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/breaker_reclose_confirmed_15s_01.ndjson` | `eec55eebb760ade7f5a746775b4097ef51f06ddbabd6a888d17cda6711729564` |

The breaker-open capture accepted 313 frames. Its opening receive backlog
contained ten `23.331 V`, `FlagStop=0` frames. Excluding that opening
burst and the first millisecond, 300 live frames reported `4.459--4.460 V`,
`FlagStop=1`, forward speed within `-0.001--0.001 m/s`, lateral speed zero, and
yaw within `-0.011--0.011 rad/s`.

The reclose capture accepted 312 frames. Its opening backlog contained ten
`4.454 V`, `FlagStop=1` frames. Excluding the opening burst and first
millisecond, 300 live frames reported `23.332--23.335 V`, `FlagStop=0`, and the
same stationary quantization set. This correlates the breaker state with the
candidate voltage field and composite inhibit bit for this attachment. It does
not measure the external motor-bus voltage or prove the breaker's electrical
topology.

The approximately `4.46 V` breaker-open value and continuing 20 Hz telemetry
show that USB keeps the VCU logic alive and weakly backfeeds the monitored rail.
The VCU frames have no source timestamps. Opening backlogs therefore cannot be
used to calculate breaker response time, control latency, or stop latency.

## Exact-zero result after reclose

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/breaker_reclose_exact_zero_2s_02.ndjson` | `c9dcdee9d60fdeb7e7a6c93db3e26c4e72d9b406aca229306455413a9a70e48c` |

The two-second run completed 61 full host writes, all the exact candidate zero
frame, and accepted 40 feedback frames. Feedback stayed within
`-0.001--0.001 m/s` forward, zero lateral, and `-0.011--0.011 rad/s` yaw;
all frames carried `FlagStop=0` at `23.305 V`. The final independent zero write
also completed. This is a successful host/feedback observation, not a VCU ACK,
brake, park, hold, or safe-state proof.

## Two straight-ramp safety aborts

Both runs requested a `0.50 m/s` target under the strict integer
`0.20 m/s^2` wire-slew invariant. Neither reached the target or entered its
requested hold. Each revoked authorization on the first out-of-envelope
feedback and completed the independent final-zero host write.

| Run | Host command and trigger | File and SHA-256 |
|---|---|---|
| strict straight gate, requested 1 s hold | first nonzero `0.019 m/s`; maximum `0.312 m/s`; rejected feedback `0.037 m/s` forward and `0.211 rad/s` yaw | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/straight_ramp_0p50_raised_02.ndjson`; `56f1307927dc2f9815a8e044d96c5cb7d4f00a9135553f2cf4154268f1d182a0` |
| raised-wheel asymmetry opt-in, requested 20 s hold with 30 s session ceiling | first nonzero `0.019 m/s`; maximum `0.080 m/s`; rejected feedback `-0.170 m/s` forward and `1.059 rad/s` yaw | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/straight_ramp_0p50_hold20_raised_01.ndjson`; `714d8583e524f0ce1570d9a538ac0c5da492e1a4a05b2cbbd4315af77ef17f05` |

Using the firmware-evidence rear track of `0.322 m`, the first rejected sample
corresponds to derived rear-wheel speeds of approximately `0.0030` and
`0.0710 m/s`. It exceeded the strict `0.023 rad/s` straight/yaw evidence gate.
The long-hold opt-in deliberately replaced that strict in-motion yaw gate with
per-rear-wheel bounds; its rejected sample corresponds to approximately
`-0.3405` and `0.0005 m/s`. One derived rear wheel was therefore well below the
`-0.005 m/s` minimum, so the 30-second ceiling did not justify continuing the
run.

These runs alone did not distinguish encoder mapping, unloaded-wheel mechanics,
firmware subtype/parameters, or another physical mismatch. The later
breaker-open manual mapping below rules out a static left/right channel swap and
decoder sign inversion. The runs still show that a positive, zero-yaw host
command did not produce the bounded same-direction rear-wheel feedback required
by the test. No command deadband, sustained start time, normal ramp-down, stop
hysteresis, or command-to-motion delay was measured.

## Post-abort receive-only observation

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/post_long_abort_readonly_5s_01.ndjson` | `a5861d9a99bed4211f79f4c8d8017eefbc8ef0f3846d4886d348964af01a4636` |

The five-second `O_RDONLY` capture accepted 111 frames. Its opening
sub-millisecond backlog contained motion-transient frames, including forward as
low as `-0.147 m/s` and yaw as high as `0.918 rad/s`. Because those frames carry
only shared/local receipt times and no VCU source time, their production times
relative to the final-zero write and the stop tail cannot be established. After
the opening backlog, 100 live frames were within the stationary quantization set
(`-0.001--0.001 m/s` forward, zero lateral, and `-0.011--0.011 rad/s` yaw).
This confirms only a later stationary feedback observation, not an acknowledged
stop.

## Powered stationary-noise baseline

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/powered_stationary_noise_60s_01.ndjson` | `4a946059d7f73ed963bc1b5d8a6a5cb07085c7a852f78f3a53657375e1c1c076` |

A 60-second `O_RDONLY` capture established a longer powered-stationary
baseline without transmitting a command. It accepted 1,213 valid frames, with
zero checksum failures, one opening-resynchronization framing failure, and 16
discarded opening bytes. All frames carried `FlagStop=0`, lateral speed zero,
and `23.297--23.299 V`. The complete forward/yaw distribution was:

- 1,059 frames at `0.000 m/s`, `0.000 rad/s`;
- 78 frames at `0.001 m/s`, `0.011 rad/s`;
- 74 frames at `-0.001 m/s`, `-0.011 rad/s`; and
- two frames at `-0.001 m/s`, `0.011 rad/s`.

No frame exceeded absolute forward speed `0.001 m/s` or absolute yaw
`0.011 rad/s`. This directly contradicts classifying the long-ramp trigger
(`-0.170 m/s`, `1.059 rad/s`) as ordinary stationary random noise under the
observed powered state. By itself, it did not distinguish a mapping fault,
powered drive mechanics, firmware configuration, or another root cause; the
breaker-open test below separately addresses the static mapping question.

## Breaker-open manual rear-wheel mapping

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/manual_rear_wheel_direction_map_60s_01.ndjson` | `bcfe8a0a5c20a9d31760efab5a4ecd110fb65bfc54ce7d656dfd5afaec74f175` |

This 60-second `O_RDONLY` capture sent no command. The main breaker was open;
per the operator this removed battery traction power, while live `FlagStop=1`
corroborated the candidate control-inhibited state. The rear wheels could then
be turned manually without a powered motor response. It accepted 1,212 valid
frames. The opening backlog contained
ten `FlagStop=0`, `23.298 V` frames; beginning 0.223 ms after the first receipt,
the remaining 1,202 frames all carried `FlagStop=1` at `4.456--4.459 V`. The
summary also recorded one checksum failure, one framing failure, and 48 bytes
discarded during opening resynchronization.

Using the firmware-evidence reconstruction
`left = forward - yaw * 0.322 / 2` and
`right = forward + yaw * 0.322 / 2`, the four active clusters matched the
operator-attested order exactly:

| Manual action | Active duration | Median derived left | Median derived right |
|---|---:|---:|---:|
| left rear wheel forward | 5.0 s | `0.499965 m/s` | `-0.000398 m/s` |
| left rear wheel reverse | 8.0 s | `-0.644229 m/s` | `0.000369 m/s` |
| right rear wheel forward | 7.9 s | `-0.000431 m/s` | `0.500262 m/s` |
| right rear wheel reverse | 8.5 s | `0.000334 m/s` | `-0.772697 m/s` |

This supports the candidate left/right encoder-channel assignment, encoder
signs, signed field decoding, and rear-wheel reconstruction formula for manual
breaker-open motion on this attachment. It also rules out ordinary stationary
noise and a static
codec channel/sign swap as explanations for the earlier
`-0.170 m/s`, `1.059 rad/s` trigger. Because the VCU frames still lack source
timestamps, the cluster durations are host-receipt/operator observations and
not latency measurements.

Earlier manual turns performed while traction power was present are not clean
mapping evidence: the powered inner loop can react and rebound against the
hand-applied motion. The breaker-open run isolates the encoder mapping from that
closed-loop response. It does not validate the commanded motor direction or
uniquely identify the powered anomaly; PI residual state, firmware motor-control
behavior, parameters, and raised-wheel mechanics remain possible contributors.

## Experimental raw `0.50 m/s` profile

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_calibration_0p50_hold60_02.ndjson` | `7480a6ee456a60f8fd3d9307edf63a3c23532c05e456c54946dcf3ebcfa409ae` |

This explicitly authorized raised-wheel run used the experimental raw-
calibration mode with `acceptance_evidence=false`. It recorded, rather than
gated on, uncalibrated authorized-motion asymmetry. The normal wire command
still obeyed the strict integer `0.20 m/s^2` slew invariant. The session lasted
`65.588123261 s` and completed the requested profile:

- final pre-arm zero completed at `0.269019365 s`;
- the first actual nonzero command was `raw=20` at `0.369220952 s`;
- the first `raw=500` completed at `2.928519397 s`;
- 1,801 complete `raw=500` writes spanned `59.998732599 s`, and the wire stayed
  at 500 until the first downward-write attempt `60.038994143 s` after the
  first 500 completion;
- the first downward value was `raw=492` at `62.967601381 s`; and
- the normal ramp reached a complete zero write at `65.567618497 s`.

There were 1,965 complete normal host writes: nine zero and 1,956 nonzero. A
separate final emergency-zero host write also completed. The maximum observed
normal wire slew was `0.199901798 m/s^2` upward and
`0.199482443 m/s^2` downward, with zero integer-invariant violations. These are
host full-write facts, not VCU acknowledgements.

Using the same `0.322 m` rear-wheel reconstruction, the first receipt-aligned
sample outside the `0.005 m/s` per-wheel standstill envelope occurred with the
latest completed command at `raw=288`: forward `0.005 m/s`, yaw
`0.035 rad/s`, left `-0.000635 m/s`, and right `0.010635 m/s`. Both derived
wheels first exceeded positive `0.005 m/s` with the latest command at
`raw=307`: forward `0.077 m/s`, yaw `0.388 rad/s`, left `0.014532 m/s`, and
right `0.139468 m/s`; that began at least five consecutive such samples. Since
the VCU supplies no source timestamps and the command was continuously ramping,
these are host-observed onset brackets, not a measured fixed deadband or pure
command-to-motion delay.

The 60-second `raw=500` hold produced 1,200 accepted feedback frames:

| Field | Minimum | P05 | Median | Mean | P95 | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| forward, m/s | `0.445` | `0.464` | `0.490` | `0.4905125` | `0.517` | `0.543` |
| yaw, rad/s | `-0.306` | `-0.211` | `0.000` | `-0.0030175` | `0.211` | `0.400` |
| derived left, m/s | `0.389600` | `0.435591` | `0.492115` | `0.490998` | `0.544903` | `0.564470` |
| derived right, m/s | `0.439367` | `0.461732` | `0.488635` | `0.490027` | `0.522231` | `0.541837` |

All derived left/right hold samples were positive. These statistics characterize
the unloaded raised-wheel response under this raw profile; they are not
ground-loaded calibration or acceptance limits.

### Post-zero gate classification

The recorded summary is `feedback_limit_violation`, but the target ramp, full
60-second hold, and normal ramp-down had already completed. The status came from
the first post-zero observation. At a host receipt `20.108534 ms` after the
normal zero write, a checksum-valid frame reported forward `0.058 m/s`, lateral
zero, yaw `0.153 rad/s`, and derived rear speeds `0.033367/0.082633 m/s`.
The authorized-motion raw-calibration exception was no longer active, so the
immediate strict post-zero yaw gate (`0.023 rad/s`) classified that residual-
motion frame as `feedback_limit_violation` before the normal standstill-wait
path could complete. It also did not satisfy the `0.005 m/s` standstill
criteria.

The independent emergency-zero full write completed `175.453 us` after that
terminal RX. There is no later feedback in the command file. The status must
therefore not be misread as a hold-phase or wire-slew failure, but neither the
normal nor emergency full write is a stop ACK, and this file cannot quantify
the physical stop tail.

## Post-profile receive-only observation

| File | SHA-256 |
|---|---|
| `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/post_raw_calibration_0p50_hold60_02_10s.ndjson` | `70bd34f38b66f60167ddf02dbd820e61078f2833def47d18c318de0819147e61` |

The bounded `O_RDONLY` follow-up accepted 211 frames and sent no command. Nine
frames in the first shared receipt were an opening backlog; their motion values
included forward `0.013--0.028 m/s` and yaw `0.082--0.176 rad/s`, followed by
zero-valued frames. The summary recorded one checksum failure, one framing
failure, and 48 discarded opening bytes. The next 202 live frames covered
`9.981742785 s` and were all within `-0.001--0.001 m/s` forward, zero lateral,
and `-0.011--0.011 rad/s` yaw, with `FlagStop=0` and
`23.290--23.303 V`.

Every frame lacked a VCU source timestamp. The opening backlog cannot be placed
relative to either zero write, and the later live stationary stream only proves
a subsequent stationary observation. It does not provide a stop latency or
VCU acknowledgement.

## Dedicated raw-profile tool and eight completed profiles

After the earlier raw-calibration run, a separately reviewed, ROS-free
`wheeltec_raw_profile_capture` executable was used for eight one-profile-per-
invocation raised-bench captures. The executable is experimental, excluded from
the default build and install, and disconnected from the Phase-1 command path.
It uses an independent encoder with a `2.0 m/s` ceiling solely for these fixed
profiles; the Phase-1 and production codec ceiling remains `0.50 m/s`.

The execution binary was
`/tmp/auto_rover_raw_profile_release.w5sXa591/wheeltec_raw_profile_capture`,
SHA-256
`cfe3ae62e5479e1fd5b0859d584b9b87ce813b0bb2ec072df804ec77d6d23ced`.
That provenance comes from the orchestrator command log; the NDJSON files do
not self-attest the executable hash. The corresponding implementation snapshot
was:

| Source | SHA-256 |
|---|---|
| `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/include/auto_rover_vcu_wheeltec_serial/raw_profile.hpp` | `610a08ded4c19331856699ad33aca4fd5344effc78b06bd8944767cb6de83b24` |
| `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/src/raw_profile.cpp` | `54814effacfde68869c27c9534f643b5e028498ac1888ef754caf01d1fd68297` |
| `src/vehicle/adapters/auto_rover_vcu_wheeltec_serial/src/raw_profile_main.cpp` | `4bfca1c07c23964ec8cb72fa2aba0e65d83b222ec7df12e4ee356a3e211aea77` |
| `tools/vehicle/analyze_wheeltec_profile.py` | `d0349d8aa19ca9b13613137288b78433303313b98dc0a02af2e0d843c7a4e3f1` |

All eight CLI records pinned `/dev/ttyACM0`, character-device major/minor
`166:0`, owner/group `0:20`, USB VID/PID `1a86:55d4`, and USB serial `0002`.
Each used the same reviewed predecessor tokens:
`passive-capture-sha256:bf659ecdeedc6b52a86cbc6cbc58d3c6d82a180a4a43128c71133364e725e3ca`
and
`exact-zero-sha256:b48e5962f1bc82f234baa6a36abb9f9b73cb2e32bfbc91d8c9e52360de98b263`.

### Raw and analyzed evidence inventory

| Profile | Raw NDJSON and SHA-256 | Offline analysis JSON and SHA-256 |
|---|---|---|
| straight `0.50 m/s` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_0p50_hold60_01.ndjson`<br>`d1ceceac42312b35b2b50d97ebaf7bfa9f75fa4b7ef2c33fc1f5c12960469bf8` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_0p50_hold60_01.analysis.json`<br>`f08605b6b6b0229007f8b397799babaf956f804c26813a0dc17dde88edfc2812` |
| straight `1.00 m/s` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_1p00_hold60_01.ndjson`<br>`c85ff5bb47467682094de275a8cb714894476a21737974120af36fae174db42d` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_1p00_hold60_01.analysis.json`<br>`80d4dfeca6f45ead2f4ddd5b8073c4dd6b8f4fd0207713e33120776406c76526` |
| straight `1.50 m/s` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_1p50_hold60_01.ndjson`<br>`1cdcbcba7fd4c925dc11894b9bb0ae532ab8bf4ab3c5ce25806adf0b1866e4e3` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_1p50_hold60_01.analysis.json`<br>`8d24c6a0bd708331107798e0b9249bc38821342636bdcfea091dd4501fd2b46e` |
| straight `2.00 m/s` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_2p00_hold60_01.ndjson`<br>`b99d32a9b80fa486d9d85af5c4dcdd36fb10ee1d752251dc37fc3770c25f4dd2` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_straight_2p00_hold60_01.analysis.json`<br>`15ef51b2716698651dc32f8c4652c2e50ef6a8a0f015f0358a390e8bb754db28` |
| left `0.50 m/s`, R=`2.00 m` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_left_0p50_r2p00_hold60_01.ndjson`<br>`fe1f02ec8d36eb4898f2836e6c19eb8480d735c30a7214297716571026067cc8` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_left_0p50_r2p00_hold60_01.analysis.json`<br>`2dd53bb326f8ed4a1d3a71bbd1c53fa1c3d2cdcdd338535c0c518b76549153b1` |
| right `0.50 m/s`, R=`2.00 m` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_right_0p50_r2p00_hold60_01.ndjson`<br>`3f77af3863a738bc2bf31685ff226ed17daec660560d8145cfe7fdfdfe16c0d6` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_right_0p50_r2p00_hold60_01.analysis.json`<br>`aadfb91d5c9d772ccbf278f0eb7b6b302d941573a47fb20f296d1f3b7967d11a` |
| left `0.50 m/s`, R=`0.95 m` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_left_0p50_r0p95_hold60_01.ndjson`<br>`0eb2d21c2ec5da721a1f76b18ec55fbca8744b02b063368af1b3610dccecf667` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_left_0p50_r0p95_hold60_01.analysis.json`<br>`dfa32d0f0232c69f1885cc9bccd86126ca5545e8d2b3991147fa387ec394f910` |
| right `0.50 m/s`, R=`0.95 m` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_right_0p50_r0p95_hold60_01.ndjson`<br>`940d87625f121c16b129c2353d323fd1825663de2ef3db25c91e3d796ca83fd0` | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/raw_profile_right_0p50_r0p95_hold60_01.analysis.json`<br>`b65b9ef54a0bcfabbc52356dfa2bc83ecf4e2ad27e095dc5beb8f98ad270a8a8` |

Every raw hash was independently recomputed and matched the input hash in its
analysis JSON. Each file contains exactly one ordered CLI preflight/open/startup-
zero prefix. The startup exact-zero completed on its first host-write attempt,
before any read, in every run. All eight summaries report `status=completed`,
`zero_host_write_completed=true`, and `delivery_unconfirmed=false`. Session
durations were `67.481--83.314 s`; every hold covered
`59.9983--59.9996 s` with 1,200 or 1,201 valid feedback frames. Every valid
frame carried `FlagStop=0`; the eight summaries each report zero checksum,
framing, discarded-byte, and trailing-buffer parser counts.

These statements mean that the host completed the recorded command writes and
the final exact-zero write. They do not mean that the VCU acknowledged or
executed a command. The protocol has no ACK or command echo, and feedback has no
source timestamp or sequence number. The offline association is only the latest
completed host write preceding a host feedback receipt.

### Straight hold statistics

The table reports the complete 60-second hold population. Rear-wheel values use
the firmware-evidence `0.322 m` track and
`left/right = forward -/+ yaw * track / 2`.

| Command | Hold forward mean / median / range, m/s | Hold yaw mean / range, rad/s | Derived left/right mean, m/s | Absolute left-right difference mean / P95 / max, m/s |
|---:|---:|---:|---:|---:|
| `0.50` | `0.489943 / 0.490 / [0.432, 0.542]` | `-0.001312 / [-0.306, 0.364]` | `0.490155 / 0.489732` | `0.030240 / 0.071806 / 0.117208` |
| `1.00` | `0.989237 / 0.993 / [0.877, 1.053]` | `0.000027 / [-0.376, 0.541]` | `0.989233 / 0.989242` | `0.057936 / 0.121072 / 0.174202` |
| `1.50` | `1.490442 / 1.493 / [1.379, 1.576]` | `-0.003501 / [-0.447, 0.553]` | `1.491006 / 1.489878` | `0.051338 / 0.117208 / 0.178066` |
| `2.00` | `1.989551 / 1.997 / [1.874, 2.082]` | `-0.000767 / [-0.423, 0.576]` | `1.989674 / 1.989427` | `0.057922 / 0.132664 / 0.185472` |

The mean forward tracking error was approximately `-0.010 m/s` at every
straight target. Individual feedback values, including the `2.082 m/s` maximum
at the `2.00 m/s` command, were intentionally recorded rather than treated as
acceptance thresholds. Mean straight yaw was near zero, but the much wider
instantaneous yaw and derived-wheel distributions are real raised-wheel
observations requiring offline interpretation; they are not proof of ground-
loaded straightness.

### Turn hold statistics

| Profile | Commanded yaw, rad/s | Feedback forward mean / range, m/s | Feedback yaw mean / range, rad/s | Derived left/right mean, m/s | Absolute left-right difference mean, m/s |
|---|---:|---:|---:|---:|---:|
| left R=`2.00 m` | `0.250` | `0.489459 / [0.430, 0.545]` | `0.249099 / [-0.058, 0.576]` | `0.449354 / 0.529564` | `0.080289` |
| right R=`2.00 m` | `-0.250` | `0.489291 / [0.432, 0.542]` | `-0.249937 / [-0.517, 0.082]` | `0.529531 / 0.449052` | `0.080764` |
| left R=`0.95 m` | `0.526` | `0.489413 / [0.435, 0.543]` | `0.524910 / [0.211, 0.871]` | `0.404902 / 0.573924` | `0.169021` |
| right R=`0.95 m` | `-0.526` | `0.489732 / [0.424, 0.536]` | `-0.525393 / [-0.859, -0.117]` | `0.574320 / 0.405143` | `0.169176` |

The mean yaw signs and inner/outer derived-wheel ordering matched the
experimental `w=v/R` convention for both directions and radii. That is useful
mechanism evidence, but it does not establish steering-angle calibration,
front-wheel geometry, loaded minimum radius, tire slip, or Phase-1 curvature
acceptance.

### Onset and post-zero candidates

The analyzer found a five-frame, positive-time run with both derived wheels
above the observed pre-motion absolute envelope in every profile. The latest
preceding command was `0.338`, `0.266`, `0.243`, and `0.266 m/s` for the four
straight profiles, and `0.312`, `0.240`, `0.293`, and `0.266 m/s` for left/right
R=`2.00 m` and left/right R=`0.95 m`. The corresponding interval from the first
nonzero host write to the first candidate receipt was `1.760`, `1.400`, `1.260`,
`1.400`, `1.620`, `1.260`, `1.520`, and `1.400 s`. These are receipt-aligned
candidates during a moving command ramp, not deadband values or causal command-
to-motion delays.

All runs retained approximately two seconds of post-zero feedback. The last
receipt above the exact observed baseline envelope occurred `0.159`, `1.320`,
`2.040`, `2.020`, `1.900`, `1.060`, `0.100`, and `2.020 s` after the normal
zero write in the same profile order. Left R=`2.00 m` and right R=`0.95 m` did
not produce five consecutive return-to-baseline frames in that window. Other
runs did produce an earlier five-frame return candidate, sometimes followed by
later above-envelope samples. Because the observed baseline in these eight
runs was exactly zero and the VCU supplies no source time, this heuristic is
quantization-sensitive and non-monotonic. In particular, a late above-envelope
sample may be zero-near encoder quantization, not proof that a wheel physically
continued turning for two seconds. It is not a physical stop time, braking
guarantee, hold result, or zero acknowledgement.

The analysis artifacts listed above use the first successful normal exact-zero
TX after the last successful nonzero TX as the post-stop time origin. They
supersede earlier same-day analysis files that incorrectly selected the first
repeated `phase=post_stop` zero and shifted that origin by one command cycle.

## Experimental raw-profile boundary

The operator separately authorized raised-wheel experimental collection at
`0.50`, `1.0`, `1.5`, and `2.0 m/s`, one explicitly gated profile at a time.
That authorization does **not** alter the Phase-1 commissioning target and
ceiling, ROS configuration, public contracts, or production adapter limit of
`0.50 m/s`. It does not authorize ground motion.

The experimental raw-profile tool is a bench evidence instrument only. It is
not installed, has no ROS node or launch integration, and is not connected to
the Phase-1 or production execution path. Each requested speed requires its own
raw evidence and remains `UNVERIFIED`; a result at one profile cannot authorize
another profile or promote any limit into the production adapter. This record
now preserves the earlier raw-calibration `0.50 m/s` run and all eight completed
dedicated raw-profile captures through `2.0 m/s`; none is acceptance evidence.

After these captures, the operator requested a future experimental extension
through `6.0 m/s`. No command above `2.0 m/s` was executed or recorded in this
evidence set. That later request requires its own code, safety review, explicit
per-profile authorization, and evidence; it does not retroactively change these
runs or the production `0.50 m/s` ceiling.

## Disposition

The breaker observations, successful exact-zero session, fail-closed ramp
aborts, stationary-noise baseline, breaker-open manual mapping, earlier raw-
calibration run, and eight dedicated 60-second profiles are useful physical
mechanism evidence. They do not close the real VCU readiness gate or change the
production `0.50 m/s` ceiling.
Powered command-to-wheel response/asymmetry, installed firmware and subtype,
command acknowledgement, controller watchdog, loss/reconnect behavior,
braking/hold, independent release-grade E-stop behavior, and ground-loaded
motion remain `UNVERIFIED`.

Do not intentionally test cable loss or host death under a nonzero command: the
candidate source already predicts last-target retention. Real actuation remains
disabled by default, the real VCU is not connected to the phase-1 ROS loop, and
ground testing remains `NO-GO`. No perception component was tested in this
session.

## Later expanded matrix

This chronological record ends with the original eight dedicated profiles.
The operator subsequently authorized and completed a separate requested-scope
matrix of twelve straight profiles through `6.0 m/s` and forty outer-tier turn
profiles through `5.0 m/s`. That later evidence, aggregate manifest, and
threshold interpretation are recorded in the
[expanded raw-profile matrix](2026-08-11-wheeltec-expanded-raw-profile-matrix.md).
The later runs do not retroactively change the limitations or conclusions of
the earlier sessions recorded above.
