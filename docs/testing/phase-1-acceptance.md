# Phase-1 Acceptance Evidence

Phase 1 is accepted against one versioned vehicle-and-scenario profile, not
against an informal claim that the vehicle looked stable. The profile, route,
software revisions, configuration, environment, operator procedure, and
recorded results form one reproducible evidence set.

## Acceptance profile

Create the acceptance profile before running a milestone test. Give it a
version and record:

- the vehicle profile revision and selected VCU-adapter revision;
- the AUTO_ROVER commit and every external dependency tag or SHA;
- the route fixture, map, localization source, parameter files, and test area;
- the test procedure, operator, date, and environmental conditions;
- the measurement source and calculation method for every metric; and
- the numeric pass criterion and observed result for every required field.

The profile must contain versioned numeric values for all of the following.
Units and aggregation methods are part of each value's definition.

| Required field | What the profile must define and record |
|---|---|
| Run count | Planned number of comparable runs, completed runs, and treatment of aborted runs. |
| Route-completion rate | A numeric completion criterion, numerator, denominator, and observed rate. |
| Lateral error | Reference frame, sampling interval, statistic or percentile, numeric limit, and observed value. |
| Heading error | Angle convention, sampling interval, statistic or percentile, numeric limit, and observed value. |
| Command rate and age | Expected command frequency, permitted age at receipt, measurement clock, and observed worst case. |
| Stale-input response | Which inputs are made stale, the numeric detection/response limits, and measured response time. |
| Stop time and distance | Initial-condition definition, stop condition, numeric limits, and measured time and distance. |
| Direction-change speed threshold | The configured numeric threshold, speed source, tolerance, and evidence that the change remained inhibited above it. |
| Hold-release conditions | Every required state/feedback condition, its numeric freshness or timing limit, and evidence that hold was not released early. |

There is no universal threshold that is safe or meaningful for every vehicle,
VCU, surface, route, or localization source. Contributors must not invent or
copy generic numbers merely to complete this document. Numeric criteria are
selected from audited vehicle capabilities, scenario risk, measured system
behavior, and an approved acceptance profile.

## Required scenarios

The evidence set must exercise the complete known-map path from an off-board
`RoutePlan` through `Trajectory`, `MotionReference`, guarded vehicle execution,
the selected VCU adapter, and normalized `ChassisState`. It must include:

- normal forward tracking and, when the vehicle profile supports it, reverse
  tracking;
- route completion and commanded stop;
- stale or invalid localization, trajectory, controller output, and chassis
  feedback;
- VCU fault or loss of autonomous-control enable;
- software emergency-stop latch and authorized recovery;
- direction-change inhibition until confirmed standstill and all configured
  capability conditions are valid; and
- replay or fake-VCU reproduction of the corresponding deterministic behavior.

Unsupported capabilities are explicitly identified and fail closed; they are
not silently omitted from results. Hardware emergency-stop operation is tested
under the vehicle's independent safety procedure and is never replaced by a
ROS-only test.

## Evidence and decision

Retain machine-readable logs or bag data, the exact configuration, summary
calculations, failures, reruns, and residual risks. Each result must be traceable
to its source data. A changed vehicle profile, adapter, safety policy, route,
localization configuration, acceptance criterion, or dependency revision
creates a new evidence version rather than overwriting the previous result.

Phase 1 passes only when every required field has a numeric criterion, every
required scenario has reproducible evidence, and all criteria pass. A reviewer
records the final decision and links the accepted evidence set from the
milestone issue.
