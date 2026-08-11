# ADR 0005: Apache-2.0 license and reusable-module publication

- Status: Accepted
- Date: 2026-08-11
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: [Repository-owner decision in issue #12](https://github.com/niuma-phd/AUTO_ROVER/issues/12) and [historical-author identity confirmation](https://github.com/niuma-phd/AUTO_ROVER/issues/12#issuecomment-5255132821)

## Context

AUTO_ROVER is publicly visible, but it previously reserved ordinary copyright
rights because no project license had been selected.  That prevented downstream
reuse and also kept the proposed package extractions behind the project-license
gate.  The repository owner has now explicitly selected Apache License 2.0 and
requested that reusable packages be published as separate public repositories.
The owner also confirmed that the historical commit identities `Tiger
<peifengliu001@nuaa.edu.cn>`, `niuma-phd <peifeng2025@163.com>`, and `pf
<pf@nuaa>` are all identities used by the same rights holder and that the
project-originated work under those identities may be licensed under this
decision.  The export keeps the historical metadata intact.

The repository also records or consumes material that the project does not own.
The Wheeltec firmware archive is all-rights-reserved vendor evidence and is not
in the repository.  FAST-LIVO2 is an external process whose manifest and source
tree have conflicting BSD/GPL-2.0-only declarations.  ROS 1, yaml-cpp, and
GoogleTest retain their respective upstream licenses.  Selecting a license for
AUTO_ROVER cannot relicense any of that material or cure uncertain provenance.

Public visibility is not a safety-readiness claim.  In particular, the
default-disabled Wheeltec integration, recorded bench behavior, and physical
activation gates remain governed by the safety ADRs and acceptance evidence,
not by the software license.

## Decision

AUTO_ROVER project-originated source code, ROS interfaces, documentation, tests,
launch files, and configuration files are made available under the Apache
License, Version 2.0, as attached in the root `LICENSE`.  Each project-authored
ROS package declares the SPDX identifier `Apache-2.0` in `package.xml`.
Contributions intentionally submitted for inclusion are handled by section 5 of
the license unless they are conspicuously marked otherwise or covered by a
separate written agreement.

This grant covers only rights held by the applicable AUTO_ROVER contributors.
Third-party and vendor material retains its own license and notices.  A package
manifest's `Apache-2.0` declaration describes the project-authored package; it
does not relicense linked dependencies, separately running providers, vendor
firmware, hardware protocols, trademarks, recorded facts, or operator-supplied
artifacts.  Unclear or unreviewed provenance fails closed for redistribution.

Reusable modules may be published under the `niuma-phd` GitHub account as one
public repository per reviewed ROS package.  AUTO_ROVER remains the canonical
integration and vehicle-acceptance source until a later ADR delegates release
authority to an extracted repository.  Each initial module publication must:

- identify the exact AUTO_ROVER source commit and package path;
- contain the package source, its package-specific tests and documentation, the
  unmodified Apache-2.0 text, and a README that states dependencies and maturity;
- preserve applicable copyright, patent, trademark, attribution, and modified-
  file notices;
- use a reviewed manifest rather than an unbounded copy of the monorepo; and
- avoid claiming independent compatibility, safety acceptance, or release
  stability that its build and test evidence has not established.

The reusable-module export set excludes vehicle-specific bringup and deployment
configuration, local or field evidence, raw recordings, generated build
artifacts, binaries, secrets and device identities, supplied vendor firmware or
archives, copied third-party source, the external FAST-LIVO2 checkout, and any
material whose authorship or license has not been reviewed.  These are export
exclusions: project-authored deployment and evidence documentation that remains
in AUTO_ROVER is still covered by the root license, but it is not automatically
part of a reusable package repository.

Apache-2.0 redistribution must include the license, mark modified files, retain
applicable notices, and carry forward any upstream `NOTICE` content if a future
accepted dependency introduces it.  The patent grant and termination terms in
section 3 apply as written.  No trademark permission, warranty, certification,
support promise, or permission to weaken a vehicle safety boundary is granted.

The repository owner is the ongoing license-compliance owner.  Dependency
changes continue to update the dependency register with exact revision,
license, obligations, notices, and approval evidence.  A distributable source,
binary, container, dataset, firmware bundle, or independent module release must
be checked against that register and the export manifest before publication.

## Alternatives considered

- Continuing with all rights reserved was rejected because it does not grant
  the requested reuse or redistribution rights.
- MIT and BSD-family licenses were not selected because the owner chose
  Apache-2.0 and its explicit patent grant and patent-litigation termination
  terms are useful for reusable robotics software.
- Applying Apache-2.0 to every adjacent artifact was rejected because the
  project cannot relicense vendor firmware, external source, or material of
  uncertain provenance.
- Treating extracted repositories as immediately authoritative independent
  releases was rejected because the current acceptance evidence belongs to the
  integrated, pinned monorepo.

## Consequences

Downstream users may use, modify, and redistribute AUTO_ROVER-originated work
under Apache-2.0, subject to its conditions.  Package metadata, the README, the
roadmap, and governance policy no longer contradict the root license.  Curated
public module repositories can be created without copying vehicle deployment or
unreviewed evidence into them.

Release work gains a permanent provenance obligation.  Incompatible or
uncertain third-party material cannot be bundled merely because the surrounding
project uses Apache-2.0.  In particular, the unresolved FAST-LIVO2 declaration
prevents source bundling until separately resolved; process separation and ROS
message interoperability do not decide redistribution rights.

The license disclaims warranty and liability, but those clauses do not replace
engineering safety controls.  Physical actuation remains compile-time frozen
and all vehicle-readiness gates remain unchanged.

## Compatibility and migration

This decision changes distribution metadata only.  It does not change a ROS
message or service layout, MD5 contract, C++ API, dependency direction, frame,
unit, sign convention, supported platform, vehicle limit, or safety semantic.
Existing downstream copies made before this decision do not acquire rights to
third-party content that AUTO_ROVER contributors did not own.

Initial extracted repositories are source publications tied to an AUTO_ROVER
commit.  Any later divergence, independent semantic versioning, release
authority, or compatibility promise requires an explicit migration and review;
breaking released ROS contracts still require V2 types and converters.

## Verification

- The root `LICENSE` matches the complete Apache License 2.0 text.
- All ten project-authored ROS package manifests declare `Apache-2.0`.
- README, roadmap, ADR index, and licensing governance link this accepted ADR
  and no longer state that the project-license decision is pending.
- Repository validation, XML parsing, Python compilation, and `git diff
  --check` pass.
- Before each public module push, its export manifest is reviewed for excluded
  material, its source commit is recorded, and its own license/package metadata
  and package-specific tests are checked.
