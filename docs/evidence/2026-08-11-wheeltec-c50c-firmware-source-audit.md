# Wheeltec C50C Firmware Source Audit

- Audit date: 2026-08-11 (Asia/Shanghai)
- Supplied archive:
  `/home/nuc_x/AUTO_ROVER/底盘VCU固件/WHEELTEC_C50X_2026.05.29.zip`
- Archive SHA-256:
  `d5ab5e0179736732b0e871090b8f158543c7583ef3a608b6fa01e2f88a80536d`
- Audit method: offline extraction to a temporary directory and static source,
  project, HEX, and documentation inspection
- Device access, firmware build, flashing, and modification: not performed
- Installed-firmware identity: `UNVERIFIED`

The archive was checked before extraction: its 360 entries contained no
absolute path, traversal path, drive-qualified path, or symbolic link. The
audit extracted 310 regular files to a temporary directory. No supplied source
file is copied into AUTO_ROVER.

## Artifact identity and limits

The active Keil project target is `Akm_Car`, built for STM32F407VE with the
`AKM_CAR` macro using an ARMCC 5.06u7 project. The supplied schematic names an
STM32F407VET6 and the source supports the C50C V1.0, V1.1, and V1.2 hardware
variants. These facts identify a source family, not the connected board or its
installed firmware.

The root and `OBJ` copies of `Akm_Car.hex` are byte-identical. Their SHA-256 is
`8f81854d15725869dd73ae098b3160192ab1c2240db6c8c934fb7ed7d6c03fda`;
an independent Intel HEX parse found valid record checksums. The project has no
current AXF, MAP, object set, reproducible build record, device readback, or
cryptographic release manifest. The two HEX files are produced by a copy step,
so their agreement is not independent source-to-binary evidence. The vehicle's
flashed image remains `UNVERIFIED`.

The application files state `All rights reserved`, while the archive has no
top-level license covering the Wheeltec application. The official desktop ROS
package also leaves its license value unresolved. AUTO_ROVER therefore records only
independently observed protocol and control facts; it does not copy or derive
implementation code from either source tree.

## Runtime vehicle selection

Although the selected build target is Ackermann, startup ADC input selects one
of ten Ackermann subtypes. Only subtype 9 sets the source-level maximum speed to
6.0 m/s; other subtypes use 3.5 m/s. Subtype 9 has:

- wheel track and wheelbase: `0.322 m` each;
- wheel diameter: `0.125 m`;
- gear ratio: `5.18`;
- encoder: 500 lines with four-times decoding; and
- source defaults `Kp=200`, `Ki=50`, subject to Flash override.

The operator's 6 m/s description and observed encoder quantization strongly
suggest subtype 9, but the ADC selection and running values must still be read
from the vehicle report or display. They are not inferred as an authorization
to raise the AUTO_ROVER 0.50 m/s commissioning ceiling.

## Serial protocol confirmed by source

UART1 and UART3 use 115200 baud, eight data bits, no parity, one stop bit, and
no flow control. A normal motion command is 11 bytes:

| Offset | Meaning |
|---|---|
| 0 | `0x7B` |
| 1 | mode; `0` is normal motion |
| 2 | unused but XOR-covered |
| 3..4 | signed big-endian forward speed, `/1000 m/s` |
| 5..6 | signed big-endian lateral speed, `/1000 m/s` |
| 7..8 | signed big-endian yaw rate, `/1000 rad/s` |
| 9 | XOR of bytes 0..8 |
| 10 | `0x7D` |

Ackermann mode consumes yaw rate, not a direct steering angle. The firmware
computes a turn radius and front-wheel steering command internally. The host
continues to enforce the project limit of `abs(curvature) <= 1/0.95 m`; the
firmware's catalogue `0.75 m` constant is not used by its active control path,
whose steering saturation would permit an even smaller radius.

The primary feedback frame is 24 bytes at a nominal 20 Hz:

| Offset | Meaning |
|---|---|
| 0 | `0x7B` |
| 1 | composite `FlagStop` |
| 2..3 | encoder-derived forward speed, `/1000 m/s` |
| 4..5 | lateral speed, `/1000 m/s` |
| 6..7 | rear-wheel-difference yaw rate, `/1000 rad/s` |
| 8..13 | raw accelerometer axes |
| 14..19 | raw gyroscope axes |
| 20..21 | battery voltage, `/1000 V` |
| 22 | XOR of bytes 0..21 |
| 23 | `0x7D` |

`FlagStop=0` means the current 100 Hz composite check permits motor actuation;
`FlagStop=1` means at least one of low voltage, the PE4 enable input, self-check
failure, or `SoftWare_Stop` currently inhibits it. It is not latched and is not
a command acknowledgement, command echo, gear, steering state, sequence,
timestamp, or specific fault code. Values other than 0 and 1 have no supported
meaning and must fail closed. Optional 19-byte ranger and 8-byte recharge frames
may be interleaved with the primary feedback stream.

The official `ws2` ROS package agrees with the basic 11/24-byte layout. It only
writes on `/cmd_vel` callbacks, does not consume `FlagStop` as an execution
gate, provides no sequence or acknowledgement, and has no robust command-age
or abnormal-exit protection. It is protocol corroboration, not a safe runtime
backend.

## Control, quantization, and delay

The motor task runs at 100 Hz and uses an incremental PI controller:

```text
e[k] = target[k] - encoder_speed[k]
PWM[k] = PWM[k-1] + Kp * (e[k] - e[k-1]) + Ki * e[k]
```

PWM is limited to ±16800 with a 10 kHz carrier. There is no explicit minimum
PWM, deadband compensation, feed-forward, derivative term, or starting kick.
The target smoothing step is `0.02 m/s` per 10 ms, approximately `2 m/s^2`, so
AUTO_ROVER's wire-level `0.20 m/s^2` envelope remains the tighter limit.

For subtype 9, one encoder count in a 10 ms window corresponds to approximately
`0.00379053 m/s` per wheel. Averaging two rear wheels and truncating to protocol
millimetres per second explains stationary forward readings of `±0.001 m/s`.
One count of left/right difference corresponds to approximately
`0.0117718 rad/s`, explaining the observed `±0.011 rad/s`. Feedback yaw is
therefore wheel-derived yaw rate, not IMU yaw or steering angle.

The source-level command-to-next-control-update bound is about 10 ms and the
feedback period is about 50 ms. Actual first motion depends on the incremental
PI state, Flash-overridden gains, battery, PWM/motor friction, and load. The
0.50 m/s commissioning target is not a measured deadband. Start/stop deadband,
hysteresis, and mechanical delay require raised-bench observation against the
recorded wire commands and feedback receipts.

## Safety-critical source findings

1. **Automatic startup motion.** For approximately 10--12 seconds after boot,
   the selected Ackermann build requests `0.2 m/s` as a motor self-test, then
   requests zero for roughly 0.3 seconds before normal command processing. A
   host watchdog cannot prevent motion initiated inside a restarting MCU.
2. **Command-loss stop is disabled by default.** `SecurityLevel` initializes to
   1, which retains the last target after serial loss. Only level 0 starts the
   approximately one-second lost-command counter, and any checksum-valid frame
   resets that counter even if its mode does not replace the normal motion
   target.
3. **The command parser survives a host reconnect.** The 11-byte UART callback
   keeps its receive count in a function-static variable. It resets only after
   eleven received bytes or when a different MCU UART invokes the callback; it
   has no inter-byte timeout and cannot observe a Linux close/reopen or process
   generation. A host partial write can therefore leave an unknown prefix in
   the MCU after the host exits. The next process completing a full zero-frame
   host write does not prove that the MCU parsed that frame as zero. Any reset
   or resynchronization procedure must be proven against the installed
   firmware while actuator power is independently isolated.
4. **CPU faults do not revoke PWM.** HardFault, MemManage, BusFault, and
   UsageFault handlers loop indefinitely, and the application has no active
   independent MCU watchdog. Timer PWM can retain its last register values.
5. **The enable input is software-polled.** The reviewed source does not prove
   that the physical emergency stop independently removes motor power or a
   hardware enable. That must be established electrically and by an approved
   test.
6. **Firmware limits are not the project safety envelope.** The internal
   steering path permits less than the project's 0.95 m radius, and a Flash
   `LineDiff` factor is insufficiently validated. Host-side speed, curvature,
   direction, freshness, and authorization gates remain mandatory.

## Disposition

The source resolves enough protocol ambiguity to finish the independent host
codec, parser, passive capture, and raised-bench command tool. It does not make
the real ROS closed loop or ground operation acceptable.

Before ground operation, the installed firmware identity and subtype must be
read back; startup self-motion must be removed; communication loss and MCU
faults must reach a demonstrated safe state; parser resynchronization after an
unclean/partial host write must be validated with actuator power isolated; and
the physical emergency-stop chain must be shown to act independently of the
CPU. The repository therefore compiles the physical-actuation release gate as
disabled. Changing it requires a separate safety ADR and acceptance evidence;
parameters and launch files cannot override it.
