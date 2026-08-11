# AUTO_ROVER Documentation

This index identifies the role of each architecture document and the authority
that governs accepted decisions.

## Authority order

When documents conflict, the newest relevant ADR with status `Accepted` wins,
followed by the approved architecture baseline. The conflict must be repaired
in the same pull request that introduces or discovers it. Summaries, registries,
and checklists must reflect the governing decision; they do not silently
override it.

## Architecture and decision records

- [Current architecture](../ARCHITECTURE.md) is the concise description of the
  architecture contributors must follow.
- [Approved architecture baseline](superpowers/specs/2026-08-10-auto-rover-architecture-design.md)
  preserves the original detailed baseline and its rationale.
- [Repository layout](architecture/repository-layout.md) records the target
  create-on-demand tree and dependency direction.
- [Architecture policy](architecture/architecture-policy.json) encodes selected
  approved decisions for automated validation.
- [Architecture Decision Records](adr/README.md) govern changes to decisions
  that have already been accepted.
- [ADR 0001: NUC Ackermann phase-1 contracts and safety boundary](adr/0001-nuc-ackermann-phase1-contracts-and-safety.md)
  proposes the ROS 1 known-map contracts, guarded execution path, and default-
  disabled candidate VCU boundary.
- [ADR 0002: Wheeltec experimental raw-profile capture boundary](adr/0002-wheeltec-experimental-raw-profile-capture.md)
  proposes the isolated raised-wheel profiles, explicit 6.0 m/s experimental
  catalog ceiling, startup/final-zero semantics, and bounded evidence-buffer
  tradeoff. It does not change the production 0.50 m/s ceiling.

## Evidence and contract registers

- [Interface contracts](interfaces/contracts.md) is the contract registry.
- [NUC Ackermann phase-1 contracts](interfaces/nuc-ackermann-phase1-contracts.md)
  proposes the first concrete schemas, clocks, frames, and failure semantics.
- [VCU adapter readiness](vehicles/vcu-adapter-readiness.md) is the evidence gate
  for the first VCU integration.
- [Phase-1 acceptance](testing/phase-1-acceptance.md) defines the measurement
  requirements for acceptance evidence.
- [Dependencies and licensing](governance/dependencies-and-licensing.md) records
  the review gate for external sources and licenses.
- [Dependency register](governance/dependency-register.md) records the proposed
  pinned phase-1 dependencies and their pending review evidence.
- [FAST-LIVO2 output audit](evidence/2026-08-10-fast-livo2-output-audit.md)
  records the pinned ROS 1 topic, reference-point inference, and timestamp limits.
- [Wheeltec static-source audit](evidence/2026-08-10-wheeltec-static-source-audit.md)
  records the isolated codec evidence and the remaining physical unknowns.
- [NUC Ackermann software verification](evidence/2026-08-11-nuc-ackermann-software-verification.md)
  records the pinned software-only build, sanitizer, Noetic, fake-loop, PTY,
  and proposed ROS interface MD5 evidence.
- [NUC Ackermann VCU bench preflight](evidence/2026-08-11-nuc-ackermann-vcu-bench-preflight.md)
  records the connected USB identity, perception-excluded software rerun, and
  the device/readiness gates that inhibited physical serial access.
- [Wheeltec passive feedback capture](evidence/2026-08-11-wheeltec-passive-feedback-capture.md)
  records the identity-gated, receive-only physical captures, raw evidence
  hashes, observed 20 Hz stream, and remaining command/closed-loop gates.
- [Wheeltec raised-bench command characterization](evidence/2026-08-11-wheeltec-raised-bench-characterization.md)
  records the exact-zero result, safely inhibited straight ramp, post-zero
  feedback, raw evidence hashes, and wire-slew correction gate.
- [Wheeltec main-breaker raised-bench follow-up](evidence/2026-08-11-wheeltec-breaker-raised-bench-follow-up.md)
  records breaker-open/reclose feedback, the post-reclose exact-zero result,
  two later wheel-feedback safety aborts, the untimed-backlog limitation, and a
  60-second powered-stationary noise baseline plus breaker-open manual
  rear-encoder mapping. It also records the earlier experimental raw
  `0.50 m/s` run and the later dedicated set of eight completed 60-second
  raised-wheel profiles: straight through `2.0 m/s` and left/right turns at
  R=`2.0 m` and R=`0.95 m`, with raw/analysis hashes and observation-only
  statistics. The separate production `0.50 m/s` ceiling and real-backend
  `NO-GO` remain unchanged.
- [Wheeltec expanded raised-bench raw-profile matrix](evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md)
  records the later `52/52` requested profile matrix: straight holds from
  `0.50` through `6.0 m/s` and outer-wheel-tier left/right turns through
  `5.0 m/s` at R=`2.0 m` and R=`0.95 m`. It preserves the aggregate manifest,
  fitted unloaded response, mirror statistics, evidence-derived diagnostic
  candidates, and the limits that remain unverified.
- [Wheeltec C50C firmware source audit](evidence/2026-08-11-wheeltec-c50c-firmware-source-audit.md)
  records the supplied STM32 source/HEX identity limits, confirmed serial
  fields, encoder quantization, controller timing, and firmware safety gaps.
- [Wheeltec hardware stop-gate correction](evidence/2026-08-11-wheeltec-estop-gate-correction.md)
  preserves the retracted E-stop-button attestation and later main-breaker
  clarification without extending the conditional raised-bench authorization
  to ground operation or release.
- [NUC Ackermann known-map runbook](applications/nuc_ackermann_known_map_runbook.md)
  describes the split default-safe ROS graph and fake-only operator workflow.
- [NUC Ackermann commissioning handoff](vehicles/nuc_ackermann_commissioning.md)
  separates completed software evidence from the restrained bench and
  low-speed field gates that still require the user and physical vehicle.
