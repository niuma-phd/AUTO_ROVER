# Wheeltec Passive Feedback Capture

- Capture date: 2026-08-11 13:03-13:05 CST (+0800)
- Branch: `feat/nuc_ackermann`
- Base Git revision: `befaa4552e19740df489f77ef9356a5ab2a9eacc`
- Protocol status: `UNVERIFIED`
- Access mode: feedback only, `O_RDONLY`
- Physical command transmission: none
- Perception scope: excluded; no LiDAR, camera, FAST-LIVO2, or localization
  process was started, stopped, subscribed to, or tested by this capture

The operator initially stated that the independent emergency stop, lifted and
fixed vehicle, wheel clearance, and on-site spotter were confirmed before
physical access. The operator later clarified that no hardware emergency-stop
button is installed; the earlier button/E-stop statement is retracted. The
operator subsequently identified a separate latching main-power breaker that
cuts traction power while USB keeps the VCU logic alive. That later
clarification does not alter this capture: it remained receive-only and does
not itself authorize a command test. See the
[stop-gate correction](2026-08-11-wheeltec-estop-gate-correction.md) and later
[main-breaker follow-up](2026-08-11-wheeltec-breaker-raised-bench-follow-up.md).

## Device and permission gate

The observed direct device was `/dev/ttyACM0`, character-device major/minor
`166:0`, owned by `root:dialout`, with USB bridge identity
`1a86:55d4`, serial `0002`, and driver `cdc_acm`. No holder was reported by
`fuser` immediately before or after either capture.

The installed world-writable udev rule was backed up as
`/etc/udev/rules.d/wheeltec_controller3.rules.codex-backup-20260811`, SHA-256
`c07c159562084020b89fe8a90f5349e38653828aeffd0539dfd2f454b085ea3f`.
It was atomically replaced with the identity-pinned repository rule, SHA-256
`26cdf98a81ec8dae4dab44c602af3a562d35a5c2a0ff2df0131886c1d02bcc1e`,
installed as `root:root` mode `0644`. The current device node was changed to
`root:dialout` mode `0660` and rechecked before each open.

After the two passive captures, the udev daemon successfully reloaded the new
rule through a bounded privileged host-namespace command. No synthetic device
event was triggered because the current node had already been set to the exact
target owner/group and mode. A reconnect remains a stop-and-recheck condition:
the physical factory will independently reject any recreated node whose mode,
owner/group, major/minor, or USB identity does not match.

## Capture implementation identity

The bounded capture executable was built directly from the current working
tree with GCC C++14, `-O2 -Wall -Wextra -Wpedantic -Werror`. Its SHA-256 was
`50981bbf68ed4b254b997ef7f52e42a5e0616d2c97e1fca3e320ef83e3100b05`.
The CLI fixes physical access to feedback-only, supplies
`actuation_opt_in=false`, opens the TTY `O_RDONLY`, never constructs the
actuation adapter, and never calls `writeAll`. The transport itself also
rejects writes in this mode. Prior injection and PTY tests confirmed zero
outbound bytes.

The working tree was not clean, so the executable hash rather than the base
revision alone identifies the run. Relevant source hashes are:

| Source | SHA-256 |
|---|---|
| `transport.hpp` | `a66702a08ef1dcc39446000cf152ef07378745e8e58e16fa9a9de64a8f075cdb` |
| `feedback_capture.hpp` | `a31d4f76a74b82c6a72b8ba991662b5b32e111c0e2e5a8c26fe63ffa01cab301` |
| `transport.cpp` | `ebd4126eb751cd5a4c26af40f602904f40a2c71428d0e92b696fed442ef8aede` |
| `feedback_capture.cpp` | `7b94a3349b55596c2a746bdab13b33e58bc494634038271cbc343cf343d3f534` |
| `feedback_capture_main.cpp` | `2833d0aa20b6b884e6423171d9b33845a89a7dff565ec1bdc81d9174f6dd7b8c` |
| `codec.cpp` | `742a8b316d82df38aedb5b40723f386eb94e74568767f847b8cdcf05ff3e098d` |
| `stream_parser.cpp` | `724028c797848df03965f3fe318b6e79539f98b71a07a0de170875958297556c` |

## Raw evidence

The raw NDJSON files are kept outside the repository because they contain
high-volume physical evidence. Each file is mode `0600` and includes metadata,
the open result, every exact raw read chunk before parsing, each accepted raw
24-byte frame with local `CLOCK_MONOTONIC` receipt, and a terminal summary.

| Run | File | SHA-256 |
|---|---|---|
| passive 10 s #1 | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/passive_feedback_10s_01.ndjson` | `9e43dc4c87abbfdfddcdfe9668a6e76d7f7582ca0e9864d2c997f271b91fff7b` |
| passive 10 s #2 | `/home/nuc_x/AUTO_ROVER_BENCH_EVIDENCE/2026-08-11/passive_feedback_10s_02.ndjson` | `3e8b2b07f2f74742db17f8517f01bf9b72d958edf712917d481796ba301fe9de` |

## Results

Run 1 accepted 200 frames from 5,066 raw bytes. After 266 initially
unsynchronized bytes it locked to the candidate 24-byte layout in 40.6 ms.
All 200 accepted frames had header `0x7B`, tail `0x7D`, and a valid XOR over
bytes 0 through 21. There were zero checksum failures, zero framing failures,
and no trailing buffered bytes. The 199 live inter-frame intervals were
49.731-50.265 ms, mean 49.999 ms, giving 20.00055 Hz.

Run 2 accepted 212 frames from 5,136 raw bytes. The first two reads contained a
bounded backlog: 12 accepted frames shared two read receipts, with 48 partial
or invalid alignment bytes and one observed checksum/framing failure. Excluding
the opening burst and the next sub-10-ms receipt left 199 live frames over
9.8998 s. Their observed rate was 20.00047 Hz, with 49.761-50.196 ms intervals.
This opening burst demonstrates why receive receipt time is not a VCU source
timestamp and why backlog frames must not refresh a control watchdog as though
they were current samples.

Across both captures:

- the current VCU/USB path emitted feedback before any host TX, substantiating
  unsolicited continuous telemetry for this attachment and firmware state;
- stable live telemetry was approximately 20 Hz at the candidate 115200 8N1
  settings;
- feedback byte 1 was always zero; the later supplied source defines this as
  composite `FlagStop=0`, consistent with its current-cycle control-allowed
  state, but it is not an acknowledgement or a specific fault/enable reason;
- lateral speed was always `0.000 m/s`;
- stationary forward-speed values were `-0.001`, `0.000`, or `0.001 m/s`;
- stationary decoded yaw-rate values were quantized among approximately
  `-0.011`, `0.000`, `0.011`, and one `0.023 rad/s` sample;
- supply voltage was 23.402-23.407 V; and
- no VCU source timestamp or command sequence was present.

The stationary distributions are observations, not approved deadband,
standstill, or fault thresholds. The first run's 266 pre-lock bytes had no
`0xFA`, `0x7B`, or `0x7D` marker and were preserved rather than interpreted.
They may be stale pre-configuration USB/TTY data; the evidence does not assign
them a protocol meaning.

## Mechanism conclusion and remaining gate

For this connected controller, the evidence supports a 24-byte unsolicited
telemetry stream at approximately 20 Hz and the candidate signed/scaled motion
fields. The later source audit explains byte 1 as a composite, non-latched
`FlagStop`, but this capture did not exercise a physical inhibit transition or
bind the supplied HEX to the connected MCU. It therefore does not substantiate
the independent enable chain, a specific fault reason, gear, steering,
installed-firmware identity, command acknowledgement, or braking/hold
semantics. A successful host write would still be only an operating-system
delivery result, not a VCU acknowledgement.

No zero or nonzero command was sent in these runs. Exact-zero and bounded
straight-ramp testing require a separately reviewed command harness whose
safety-zero path cannot be blocked by logging, whose output is rate-limited and
slew-limited between every wire frame, and whose start gate proves fresh
standstill feedback. Phase-1 real closed-loop remains inhibited until trusted
control-enable and fault evidence exists.
