## Summary and scope

Describe the problem, the intended outcome, what changed, and the explicit
non-goals of this pull request.

## Affected area

- [ ] Perception
- [ ] Planning
- [ ] Control
- [ ] Vehicle integration or VCU adapter
- [ ] Safety
- [ ] Interfaces or ROS conversions
- [ ] Application bringup, deployment, or configuration
- [ ] Documentation, tooling, tests, or CI only

## Verification evidence

List the exact local checks, unit/integration tests, replay or simulation cases,
and real-vehicle evidence that were run. Include commands and results, not only
the words "tested" or "works".

## Interface and compatibility

Identify every affected public or internal contract, frame, unit, sign,
timestamp, validity, timeout, topic, parameter, configuration schema, recording,
or migration path. Explain why the change is compatible, or link the required
V2 contract and converter.

## Safety and vehicle behavior

Describe effects on freshness, watchdogs, limits, stopping, hold, direction or
gear changes, control enable, fault handling, and emergency stop. Identify every
new or changed hazard, its mitigation and residual risk, and link the recorded
safety disposition and relevant failure-path evidence.

## Dependencies and licensing

For every added, revised, or removed dependency, update the durable
`docs/governance/dependency-register.md` record in the same change and link it
here. Include the immutable pin, source, license, modifications, redistribution
impact, platform/interface/maintenance evidence, operational owner, and the
recorded dependency-owner and license-review approvals. Write "None" only when
the dependency graph and redistributed artifacts are unchanged.

## Documentation and decision record

List changed architecture, interface, vehicle, test, deployment, and operator
documentation. Link the ADR when this changes a public interface, safety
boundary, dependency direction, platform strategy, repository extraction, or
the project license or distribution policy.

## Required confirmations

- [ ] I ran the repository validator and relevant tests, or explained why a
  documented check does not apply.
- [ ] Vehicle- or vendor-specific protocol details did not leak into planning,
  control, `MotionReference`, or another vehicle-independent contract.
- [ ] This change does not weaken watchdogs, freshness checks, limits,
  fail-closed behavior, safety interlocks, or the independent hardware E-stop.
- [ ] Frames, units, signs, timestamps, validity, timeouts, and failure behavior
  are documented for every changed interface.
- [ ] No vehicle fact, acceptance threshold, dependency revision, or supported
  platform is asserted without reviewable evidence.
