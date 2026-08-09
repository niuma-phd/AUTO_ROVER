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

## Evidence and contract registers

- [Interface contracts](interfaces/contracts.md) is the contract registry.
- [VCU adapter readiness](vehicles/vcu-adapter-readiness.md) is the evidence gate
  for the first VCU integration.
- [Phase-1 acceptance](testing/phase-1-acceptance.md) defines the measurement
  requirements for acceptance evidence.
- [Dependencies and licensing](governance/dependencies-and-licensing.md) records
  the review gate for external sources and licenses.
