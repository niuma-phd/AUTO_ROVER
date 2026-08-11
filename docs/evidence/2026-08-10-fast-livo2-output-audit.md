# FAST-LIVO2 ROS 1 Output Audit

- Audit date: 2026-08-10
- Intended phase-1 source: `main@3df020182aee52d81fd1a6b543bfb611c46d11bc`
- Current external checkout at audit time:
  `ROS2-Humble@f47b2c687649384238daee8d60ee172767883b99`
- Evidence type: read-only source audit and one existing offline Noetic bag replay
- Physical vehicle evidence: none

This record supports the localization adapter configuration. FAST-LIVO2 remains
an external deployment and no source or recorded data is copied into AUTO_ROVER.
The changing external checkout must not be used as an unpinned Noetic build
dependency.

## Confirmed ROS 1 graph contract

At the audited Noetic revision, `src/FAST-LIVO2/src/LIVMapper.cpp:186-210`
advertises `/aft_mapped_to_init` as `nav_msgs/Odometry`. The publisher at
`LIVMapper.cpp:1272-1289` sets:

```text
header.frame_id = camera_init
child_frame_id = aft_mapped
```

An already-running, input-disabled offline regression at the exact revision was
queried without changing any process. `rostopic type` returned
`nav_msgs/Odometry`, the publisher was `/laserMapping`, and all 3,985 recorded
outputs used the same frame/child pair. `aft_mapped` is a hard-coded provider
frame name; it is not `base_link` or the rear axle.

The adapter configuration therefore expects the exact topic, type, frame, child,
and source revision. A mismatch fails closed.

## Timestamp limitation

`LIVMapper.cpp:1276` assigns `ros::Time::now()` to the odometry header. It does
not publish the sensor sample or estimator state epoch. The algorithm retains an
internal `last_lio_update_time`, but that value is not carried by this message.

In the existing 3,985-message offline replay, publish time lagged the internal
state epoch by 0.071250200 to 0.262819767 seconds, with a mean of
0.142336746 seconds. This distribution is evidence for that replay only. It
cannot bound live sensor latency.

AUTO_ROVER therefore labels the provider stamp `PUBLISH_TIME`, leaves
measurement time unavailable, checks provider ordering, and independently uses
the subscriber's monotonic receipt time for freshness. A recent publish or
receipt time does not prove a recent sensor sample; live latency remains a
readiness item.

## Pose reference and axes

Executable pose composition strongly indicates an IMU-state reference:

- `src/FAST-LIVO2/src/LIVMapper.cpp:631-653` transforms a LiDAR point with the
  LiDAR-to-IMU extrinsic before applying `_state.rot_end` and `_state.pos_end`;
- `src/FAST-LIVO2/src/IMU_Processing.cpp:300-302,413-445` updates `_state` from
  IMU position and rotation;
- `src/FAST-LIVO2/src/LIVMapper.cpp:1261-1277` publishes `_state` directly.

Some legacy comments call the same value a LiDAR origin. Consequently
`imu_origin_math_inferred` is a high-confidence implementation inference, not a
measured public reference-point contract. The current MID360 configuration's
LiDAR-to-IMU translation is `[-0.011, -0.02329, 0.04412] m` with identity
rotation; it provides no IMU-to-rear-axle transform.

Gravity alignment maps estimated gravity to negative z, supporting positive z
up after successful initialization. ROS quaternion/right-handed conventions are
used. Vehicle forward/left alignment and initial yaw are not established. A
versioned, measured source/IMU-to-rear-axle transform is required, and an unknown
transform inhibits motion without an identity fallback.

The message publisher does not populate twist or covariance. Observed zero
arrays mean unavailable, not zero velocity or zero uncertainty. The adapter
publishes pose only and clears those validity bits.

## Frequency evidence

There is no odometry-rate parameter. A 5,000 Hz main-loop rate, 10-message
publisher queue, 10 Hz Livox setting, and 10 Hz soft-sync timer are not odometry
guarantees. Odometry is published from completed LIO handling and varies with
input eligibility, initialization, backlog, and compute time.

The existing offline replay produced 3,985 odometry messages at approximately
10 Hz. Provider-header intervals ranged from about 0.0100 to 0.2094 seconds.
This is replay evidence, not a live rate contract or a reason to weaken a
receiver watchdog.

## Configuration and readiness outcome

The software configuration records:

```yaml
source_revision: 3df020182aee52d81fd1a6b543bfb611c46d11bc
source_topic: /aft_mapped_to_init
source_message_type: nav_msgs/Odometry
expected_frame_id: camera_init
expected_child_frame_id: aft_mapped
pose_reference_point: imu_origin_math_inferred
timestamp_semantics: publisher_ros_time
measurement_time_available: false
twist_available: false
covariance_available: false
extrinsic_known: false
```

Live source frequency and latency, installed source-to-vehicle axes, and the
source-to-rear-axle extrinsic remain `UNVERIFIED`. Until those are supplied and
the extrinsic is explicitly marked known, the real localization adapter emits an
invalid `EgoState` and vehicle actuation remains inhibited.
