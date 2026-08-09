# Phase-1 Acceptance Evidence

Phase 1 is accepted against one versioned vehicle-and-scenario profile, not
against an informal claim that the vehicle looked stable. The profile, route,
software revisions, configuration, environment, operator procedure, and
recorded results form one reproducible evidence set.

## Acceptance profile

Create, review, and approve the acceptance profile before running a milestone
test. The approved profile is an immutable, versioned repository artifact; its
Git commit and blob SHA are recorded in the milestone issue before data is
collected. Give it a version and record:

- the vehicle profile revision and selected VCU-adapter revision;
- the AUTO_ROVER commit and every external dependency tag or SHA;
- the route fixture, map, localization source, parameter files, and test area;
- the test procedure, operator, date, and environmental conditions;
- the measurement source and calculation method for every metric; and
- the numeric pass criterion for every required field.

The profile must contain versioned numeric values for all of the following.
Units and aggregation methods are part of each value's definition.

| Required field | What the pre-test profile must define |
|---|---|
| Run count | Planned number of comparable runs and treatment of aborted runs. |
| Route-completion rate | A numeric completion criterion and the numerator/denominator rules. |
| Lateral error | Reference frame, sampling interval, statistic or percentile, and numeric limit. |
| Heading error | Angle convention, sampling interval, statistic or percentile, and numeric limit. |
| Command rate and age | Expected command frequency, permitted age at receipt, and measurement clock. |
| Stale-input response | Which inputs are made stale and the numeric detection/response limits. |
| Stop time and distance | Initial-condition definition, stop condition, and numeric limits. |
| Direction-change speed threshold | The configured numeric threshold, speed source, and tolerance. |
| Hold-release conditions | Every required state/feedback condition and its numeric freshness or timing limit. |

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

## Results, approval, and decision

Store test results separately from the pre-test profile. Each immutable result
set references the exact profile version, commit, and blob SHA and records
completed and aborted runs, every observed metric, machine-readable logs or bag
data, the exact configuration, summary calculations, failures, reruns, and
residual risks. Each result is traceable to its source data. Thresholds are never
edited to fit collected data: a changed vehicle profile, adapter, safety policy,
route, localization configuration, acceptance criterion, or dependency revision
requires a new pre-test profile version, approval, and new qualifying runs.

Phase 1 passes only when every required field has a numeric criterion, every
required scenario has reproducible evidence, and all criteria pass. A reviewer
cannot self-assign decision authority merely by contributing results. Before
testing, the repository owner names the acceptance decision authority and safety
reviewer in the milestone issue; the owner may retain either role or explicitly
delegate it there. The decision authority approves the pre-test profile before
data collection, and the authority plus safety reviewer record their final
dispositions in that same milestone issue after reviewing the separate result
set. The accepted evidence links those approval records.
