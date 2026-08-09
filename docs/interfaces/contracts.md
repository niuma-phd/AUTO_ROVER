# Interface Contract Registry

This registry names the semantic contracts between AUTO_ROVER modules before
their ROS 1 message definitions are frozen. It assigns ownership and review
requirements; it does not invent `.msg` fields. Detailed schemas are introduced
with tests when the corresponding phase-1 implementation begins.

Algorithm cores consume middleware-independent domain values from
`auto_rover_core`. Thin ROS 1 wrappers use `auto_rover_interfaces` and explicit
`auto_rover_ros1_conversions`; ROS messages do not leak into algorithm cores.

## Registry

| Contract | Semantic owner | Phase and ROS status | Required semantic documentation | Compatibility policy |
|---|---|---|---|---|
| `EgoState` | Perception/localization boundary | Phase 1; ROS 1 v1 is planned, not yet created | Pose and available motion estimates must identify their reference and child/control frames, SI units and sign conventions, source timestamp and clock, covariance or quality meaning, freshness, validity, and behavior when the provider is stale or degraded. | Provider-specific messages stay behind adapters. A released v1 semantic or field-layout break requires a V2 contract and converter. |
| `RoutePlan` | Planning boundary and off-board route producer | Phase 1; ROS 1 transport is planned, not yet created | The route frame, geometry units, route/segment identity, direction intent, creation/version time, provenance, validation state, vehicle-profile applicability, and rejection behavior must be defined before use. | File and transport representations share one versioned semantic contract. Incompatible changes require a new version and an explicit migration or converter. |
| `Trajectory` | On-board planning | Phase 1; ROS 1 v1 is planned, not yet created | The trajectory frame, point and curvature/speed units, sign conventions, explicit identity, generation/reference time, validity horizon, forward/reverse segmentation, vehicle-feasibility constraints, completion behavior, and invalid-input outcome must be documented. | Controllers depend only on the contract, not the planner. Released breaking changes require V2 plus converter; added optional semantics require explicit validity. |
| `MotionReference` | Control | Phase 1; ROS 1 v1 is planned, not yet created | The configured vehicle control frame, speed and curvature units, left/right and forward/reverse conventions, production time, declared validity duration, required input freshness, and fail-closed expiry behavior must be explicit. | It remains vehicle- and protocol-independent. Vendor commands, CAN fields, pedal values, and implicit `/cmd_vel` semantics are not added to this contract; breaking v1 changes use V2 plus converter. |
| `ChassisState` | Vehicle integration | Phase 1; ROS 1 v1 is planned, not yet created | Every normalized observation must identify its vehicle/control frame where applicable, SI units and signs, measurement/receipt time, source, per-value validity, whole-state freshness, enable/fault meaning, and unavailable-field behavior. | Adapters publish only substantiated semantics and retain vendor diagnostics at the edge. New shared meanings require cross-vehicle evidence; breaking released changes require V2 plus converter. |
| `VehicleProfile` | Vehicle integration with core validation | Phase 1; versioned configuration/domain schema, not required as a continuously published ROS message | The selected kinematic model and control frame, geometry and limit units, steering/direction conventions, capability declarations, configuration version, effective time and provenance, validation result, and failure policy must be explicit. Runtime message timestamp and age semantics do not apply unless a profile is transported dynamically; that transport must then define them. | Profiles are deployment data, never hard-coded planner/controller assumptions. Schema breaks require a new version and migration; a profile cannot silently change direction capability. |
| `SafetyState` | Safety supervision | Phase 1; ROS 1 representation is planned, not yet created | Safety-state meaning, source and receipt times, freshness, inhibit/latched-state semantics, authorization and recovery rules, diagnostic provenance, and fail-closed behavior must be specified. Frames and physical units are documented whenever a safety condition uses them. | Safety semantics cannot be weakened by an adapter or application. Any released semantic break requires a new version, migration, safety review, and ADR. |
| `EmergencyStop` | Safety supervision and command guard | Phase 1; independent ROS 1 control path is planned, not yet created | Request/source identity, time and clock, authenticity or authorization boundary, latch behavior, reset preconditions, acknowledgement, timeout handling, and hardware-chain independence must be defined. It has no implicit motion frame or physical unit. | It is never folded into `MotionReference` or auto-cleared. Breaking behavior requires a new version, converter/migration where meaningful, safety review, and ADR. |

`WorldModel` is a reserved contract name only. Phase 1 has no environment-model
schema or ROS message, and the name does not imply obstacle fields, map type,
update rate, frame, or validity policy. Its design and ownership must be accepted
before any implementation freezes that interface.

## Rules shared by every released contract

Before first use, the owning change must document:

- coordinate frame or an explicit statement that no frame applies;
- units, ranges, and sign conventions or an explicit statement that none apply;
- source timestamp, clock domain, receiver-age calculation, and ordering rules;
- required and optional values, validity representation, freshness limit, and
  behavior for absent, stale, non-finite, unsupported, or inconsistent input;
- semantic identity and versioning where replay or replacement can occur;
- producer and consumer responsibilities, including conversion ownership; and
- diagnostics, timeout behavior, and the fail-closed outcome.

ROS 1 `Header.seq` is not a semantic identifier. A published ROS 1 v1 message
must not receive a silent semantic or field-layout change that alters its MD5
contract. Introduce a V2 type, an explicit converter, compatibility tests, and
migration documentation. Public-interface changes require an ADR and review of
all affected producers, consumers, recordings, and deployment configurations.

An adapter may expose `/cmd_vel`, `ackermann_msgs`, or a vendor message only
after documenting that external contract's exact semantics. Such an adapter is
not permission to redefine the internal contracts above.
