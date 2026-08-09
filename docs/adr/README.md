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
- repository extraction.

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

## Process

1. Copy the template and assign the next sequential four-digit number.
2. Describe the context, decision, alternatives, consequences, compatibility or
   migration impact, and verification evidence.
3. Submit the proposed ADR with the implementation or policy change it governs.
4. Obtain architecture and compatibility review appropriate to the trigger.
5. Set the reviewed status. When a new accepted ADR supersedes an earlier one,
   update both records in the same pull request.
6. Update every conflicting architecture summary, registry, checklist, policy,
   or other affected document in that same pull request.

The [documentation index](../README.md) defines authority precedence. The newest
relevant accepted ADR wins over the approved architecture baseline, and any
conflict must be repaired rather than left for readers to reconcile.
