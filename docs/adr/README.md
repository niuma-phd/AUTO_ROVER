# Architecture Decision Records

Architecture Decision Records (ADRs) record reviewed changes to decisions that
AUTO_ROVER has already accepted. They preserve the decision context, alternatives,
consequences, migration impact, and verification evidence.

## ADR triggers

An ADR is required before changing any of these areas:

- public interfaces;
- safety boundaries;
- dependency direction;
- platform strategy;
- repository extraction;
- the project license or distribution policy.

## Numbering and filenames

Copy [the ADR template](0000-template.md) to the next unused sequential
four-digit number, beginning with `0001`, and use a descriptive filename such as
`0001-short-decision-title.md`. Never renumber an existing ADR or reuse the
number of a rejected or superseded ADR.

## Statuses

The only ADR statuses are:

- `Proposed`: the decision is under review and has no decision authority.
- `Accepted`: the decision is approved and governs the repository.
- `Superseded`: a later accepted ADR has replaced the decision; retain the ADR
  as history and identify its successor.
- `Rejected`: the proposal was considered but not approved; retain its number
  and rationale.

An ADR starts as `Proposed` and becomes either `Accepted` or `Rejected` through
review. An accepted ADR becomes `Superseded` only when a later ADR is accepted.

## Decision authority

Only the repository owner, or a maintainer the owner explicitly delegates in
the ADR pull request or linked issue, may change an ADR from `Proposed` to
`Accepted` or `Rejected`, or mark an accepted ADR `Superseded`. The ADR names
that decision authority and links the approving GitHub review or issue comment.
Specialist architecture, compatibility, safety, vehicle, security, or license
reviewers may be required by the subject, but they do not acquire decision
authority merely by contributing to the change.

## Process

1. Copy the template and assign the next sequential four-digit number.
2. Describe the context, decision, alternatives, consequences, compatibility or
   migration impact, and verification evidence.
3. Submit the proposed ADR with the implementation or policy change it governs.
4. Obtain the specialist reviews required by the trigger and explicit approval
   from the named decision authority. Record links to that evidence in the ADR.
5. The decision authority sets the reviewed status. When a new accepted ADR
   supersedes an earlier one, update both records in the same pull request.
6. Update every conflicting architecture summary, registry, checklist, policy,
   or other affected document in that same pull request.

The [documentation index](../README.md) defines authority precedence. The newest
relevant accepted ADR wins over the approved architecture baseline, and any
conflict must be repaired rather than left for readers to reconcile.

## Records

- [ADR 0001: NUC Ackermann phase-1 contracts and safety boundary](0001-nuc-ackermann-phase1-contracts-and-safety.md)
- [ADR 0002: Wheeltec experimental raw-profile capture boundary](0002-wheeltec-experimental-raw-profile-capture.md)
- [ADR 0003: Wheeltec power-isolated parser recovery and release sequencing](0003-wheeltec-power-isolated-parser-recovery.md)
- [ADR 0004: Pure Pursuit software-safety authorization feedback](0004-pure-pursuit-safety-authorization-feedback.md)
- [ADR 0005: Apache-2.0 license and reusable-module publication](0005-apache-2.0-license-and-reusable-module-publication.md)
- [ADR 0006: Bounded known-map planning resources](0006-bounded-known-map-planning-resources.md)
