# Dependencies and Licensing

AUTO_ROVER adopts external software narrowly and deliberately. A library,
driver, algorithm, message package, container image, dataset, or copied source
is not added merely because it is convenient or widely used.

## Dependency record

Before a dependency enters a build, deployment, `.repos` manifest, test
fixture, or redistributed artifact, record all of the following in the change
that introduces it:

| Field | Required evidence |
|---|---|
| Name and purpose | What is used and which AUTO_ROVER boundary it serves. |
| Upstream and source URL | Canonical project location and the exact retrieval location. |
| Revision | A reviewed release tag or immutable commit SHA; floating branches are not accepted. |
| License | License name and SPDX identifier, with the upstream license text available for review. |
| Modifications | Local patches, generated bindings, copied files, or confirmation that the dependency is unmodified. |
| Redistribution obligations | Notices, source-offer, attribution, patent, network-use, or binary-distribution duties that apply. |
| Platform fit | Evidence for Ubuntu 20.04 and ROS 1 Noetic where phase-1 runtime use is proposed. |
| Interface fit | Why a thin adapter is sufficient and which public or internal contract it touches. |
| Maintenance evidence | Release activity, issue state, security posture, or a documented plan to maintain a pinned fork. |
| Owner | GitHub username responsible for upgrades, advisories, license compliance, and removal. |

Transitive dependencies and bundled assets are part of the same review. The
record must identify any dependency that executes on the vehicle, accepts
untrusted input, communicates over a network or bus, or participates in a
safety-relevant path.

## Durable register and approval

Dependency records live in `docs/governance/dependency-register.md`. That file
is created with the first proposed dependency rather than as an empty stub. It
contains one stable heading per dependency and the complete field set from the
table above. The introducing or revision pull request updates the register in
the same change as the pin, manifest, build, deployment, fixture, or copied
artifact. Removed dependencies remain in a historical section with the last
used revision and removal change; records are not silently deleted.

The repository owner, or a dependency maintainer explicitly delegated by the
owner in the pull request or linked issue, approves each add, revision, or
removal. License-sensitive changes also require a named license reviewer. The
register links the approving GitHub review or issue comment and records the
responsible operational owner. This document is the record schema until a later
ADR replaces it.

## Adoption rules

- Prefer a package boundary or thin adapter over copying third-party source
  into this repository.
- Pin source dependencies to reviewed tags or SHAs and pin deployment images
  and package sets reproducibly.
- Review changes to a pin as dependency changes, including compatibility,
  maintenance, security, and license impact.
- Keep vendor messages and implementation details behind their adapters; an
  external project's API does not automatically become an AUTO_ROVER public
  contract.
- Do not import an entire autonomy stack when a bounded component or original
  implementation satisfies the approved architecture.
- Preserve copyright notices and all applicable redistribution material.
- Reject a dependency when its provenance, license, obligations, maintainer,
  target-platform fit, or safety impact cannot be established.

Security advisories for pinned dependencies are triaged by the recorded owner.
Because Ubuntu 20.04 and ROS 1 Noetic are outside normal standard support, an
unchanged pin is not evidence that a deployed dependency remains acceptable.

## Project license and distribution

[ADR 0005](../adr/0005-apache-2.0-license-and-reusable-module-publication.md)
closes the project-license decision gate. AUTO_ROVER-originated source code,
interfaces, documentation, tests, launch files, and configuration files are
available under the root Apache License 2.0. Contributions intentionally
submitted for inclusion are governed by section 5 of that license unless they
are conspicuously marked otherwise or covered by a separate written agreement.

The project license reaches only rights held by the applicable contributors. It
does not relicense dependencies, linked system libraries, separately running
providers, vendor source or firmware, trademarks, operator-supplied artifacts,
recorded facts, or content of uncertain provenance. File-level notices and the
dependency register take precedence for those materials. Unresolved provenance
or license compatibility fails closed for copying, bundling, and redistribution.

Apache-2.0 source and object redistribution includes the license, marks modified
files, retains applicable copyright, patent, trademark, and attribution notices,
and carries forward any applicable upstream `NOTICE` content. A container,
binary, firmware bundle, dataset, or generated artifact receives its own
complete dependency and source-obligation review; the root license alone is not
a release checklist.

Reusable-package publications use a reviewed, bounded export manifest and
record the exact AUTO_ROVER source commit. They include package-specific source,
tests, documentation, license text, dependency/maturity notes, and applicable
notices. They exclude vehicle-specific bringup and deployment configuration,
local or field evidence, raw recordings, generated artifacts, binaries, secrets
and device identities, supplied vendor firmware or archives, copied third-party
source, external checkouts, and any unreviewed material. AUTO_ROVER remains the
canonical integration and acceptance source until another accepted ADR assigns
independent release authority.

The repository owner is the license decision and ongoing compliance authority
unless that authority is explicitly delegated in a pull request or linked
issue. Every dependency or distribution change updates the durable register,
identifies its operational owner and license reviewer, and links approval
evidence. Public visibility, an Apache-2.0 manifest, or a successful build is not
evidence of vehicle safety readiness.
