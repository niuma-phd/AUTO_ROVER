# AUTO_ROVER Repository Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the approved AUTO_ROVER architecture baseline into a documented,
testable, and governed GitHub repository without prematurely creating ROS
packages or inventing vehicle-specific facts.

**Architecture:** Keep the current monorepo documentation-first. Human-readable
documents explain the boundaries, while a small machine-readable policy and a
dependency-free Python validator prevent accidental scope drift. GitHub Actions
run the same tests as local development; GitHub settings, labels, and issues are
applied only after the foundation PR passes and merges.

**Tech Stack:** Markdown, JSON, Python 3.8 standard library, GitHub Actions,
GitHub CLI 2.79+, Git, PowerShell.

---

## Scope and file map

This plan creates documentation, governance, and repository validation only.
It deliberately creates no `package.xml`, `CMakeLists.txt`, ROS message, empty
ROS package, VCU adapter, `.repos` manifest, guessed acceptance threshold, or
placeholder `LICENSE`.

| Path | Responsibility |
|---|---|
| `README.md` | Public project entry point and phase-1 boundary |
| `ARCHITECTURE.md` | Developer-facing system data flow and dependency rules |
| `ROADMAP.md` | Ordered delivery stages, gates, and extraction criteria |
| `AGENTS.md` | Binding instructions for automated contributors |
| `CONTRIBUTING.md` | Branch, PR, testing, ADR, compatibility, and dependency policy |
| `SECURITY.md` | Noetic lifecycle risk and responsible security reporting |
| `.gitignore` | Ignore build, runtime, IDE, and large recording outputs |
| `.gitattributes` | Normalize source and documentation to LF |
| `.editorconfig` | Basic UTF-8, whitespace, and indentation rules |
| `docs/README.md` | Documentation authority and index |
| `docs/architecture/repository-layout.md` | Target tree and create-on-demand rule |
| `docs/architecture/architecture-policy.json` | Machine-readable approved decisions |
| `docs/interfaces/contracts.md` | Contract registry without prematurely creating messages |
| `docs/vehicles/vcu-adapter-readiness.md` | Evidence checklist for the first VCU |
| `docs/testing/phase-1-acceptance.md` | Required quantitative acceptance fields |
| `docs/governance/dependencies-and-licensing.md` | External-source and license gate |
| `docs/adr/README.md` | ADR workflow |
| `docs/adr/0000-template.md` | Copyable ADR template |
| `.github/PULL_REQUEST_TEMPLATE.md` | Review evidence checklist |
| `.github/ISSUE_TEMPLATE/*.yml` | Structured bug and change intake |
| `.github/workflows/repository-validation.yml` | Required repository-quality check |
| `tools/ci/validate_repository.py` | Deterministic repository validator |
| `tests/repository/test_validate_repository.py` | Validator regression tests |

### Task 1: Establish root documentation and repository policy

**Files:**
- Modify: `README.md`
- Create: `ARCHITECTURE.md`
- Create: `ROADMAP.md`
- Create: `AGENTS.md`
- Create: `CONTRIBUTING.md`
- Create: `SECURITY.md`
- Create: `.gitignore`
- Create: `.gitattributes`
- Create: `.editorconfig`
- Modify: `docs/superpowers/specs/2026-08-10-auto-rover-architecture-design.md`

- [ ] **Step 1: Set the repository-local Git identity**

Run:

```powershell
git config user.name "niuma-phd"
git config user.email "289723736+niuma-phd@users.noreply.github.com"
git config --get-regexp '^user\.(name|email)$'
```

Expected: the two values above are printed; no history is rewritten.

- [ ] **Step 2: Write the public entry documents**

`README.md` must contain, in this order:

```markdown
# AUTO_ROVER

Modular perception, planning, control, and vehicle-integration software for
autonomous ground vehicles.

AUTO_ROVER starts with one complete known-map navigation loop on Ubuntu 20.04
and ROS 1 Noetic. A native ROS 2 implementation follows only after the first
test platform is validated.

## Phase-1 data flow

Off-board RoutePlan -> on-board Trajectory -> MotionReference -> vehicle manager
-> VCU adapter -> VCU, with EgoState and normalized ChassisState feedback.

Phase 1 does not include online obstacle avoidance, environment modelling,
autonomous exploration, a runtime task manager, or universal vehicle support.

## Documentation

- [Architecture](ARCHITECTURE.md)
- [Roadmap](ROADMAP.md)
- [Documentation index](docs/README.md)
- [Contributing](CONTRIBUTING.md)
- [Security](SECURITY.md)

## Platform status

The initial runtime is Ubuntu 20.04 with ROS 1 Noetic. Both are outside normal
standard support, so dependencies and deployment environments must be pinned.

## License

No project-wide open-source license has been selected yet. Until a license is
added, copyright law reserves reuse rights; external dependencies retain their
own licenses.
```

`ARCHITECTURE.md` must include the approved Mermaid flow, the three autonomy
layers, the supporting interface/vehicle/safety roles, the ROS-independent core
and ROS 1 conversion boundary, the `MotionReference -> VehicleExecutionCommand`
boundary, fail-closed direction capabilities, dual watchdog ownership, and links
to the detailed approved design and contract registry.

`ROADMAP.md` must define these ordered stages with exit conditions:

1. repository foundation;
2. v1 contracts and ROS-independent core;
3. fake localization, fake VCU, route generation, and trajectory tracking;
4. first VCU audit and adapter;
5. known-map application integration;
6. versioned real-vehicle acceptance;
7. separately designed future environment modelling, exploration, coverage,
   and native ROS 2 work.

It must repeat the extraction gate: stable across at least two system releases,
independent build/tests, low synchronized-change rate, and an independent
consumer, maintainer, or release cadence. Future repository names remain
proposals, not repositories to create now.

- [ ] **Step 3: Write contributor and safety policy**

`AGENTS.md` must state the following binding rules:

```markdown
# AUTO_ROVER Agent Instructions

These instructions apply to the entire repository unless a deeper `AGENTS.md`
adds stricter local rules.

## Approved scope

- Phase 1 targets Ubuntu 20.04 and ROS 1 Noetic.
- Keep algorithm cores independent of ROS APIs.
- Use thin ROS 1 wrappers and explicit conversion code.
- Keep known-map navigation, exploration, and field coverage as separate
  application compositions.
- Do not add a runtime task manager for switching those applications.
- Do not implement phase-1 world modelling or local obstacle avoidance.
- Do not create empty ROS packages, speculative VCU adapters, or ROS 2 shells.

## Dependency boundaries

- Planning never depends on a localization implementation.
- Control never depends on a VCU protocol.
- VCU adapters contain no trajectory-tracking algorithm.
- Application bringup may depend on modules; modules never depend on an app.
- `/cmd_vel` is an adapter contract only when its exact semantics are documented.

## Interface and safety rules

- Document frame, units, sign, timestamp, validity, timeout, and failure behavior.
- Breaking a released ROS 1 message requires a new version and converter.
- Preserve a VCU's existing inner speed loop when it accepts vehicle-level targets.
- Direction changes fail closed; software E-stop remains latched until authorized
  reset; hardware E-stop remains independent.
- Never weaken watchdogs, freshness checks, limits, or safety interlocks to make a
  test pass.

## Workflow

- Work on a focused branch and merge through a pull request.
- Add or update tests before behavior changes.
- Run the repository validator and relevant package tests before every PR.
- Public-interface, safety-boundary, dependency-direction, or repository-split
  changes require an ADR.
- Pin third-party dependencies to reviewed tags or SHAs and record license impact.
```

`CONTRIBUTING.md` must specify `feat/*`, `fix/*`, `exp/*`, and `agent/*` branch
families, squash merge, required evidence, compatibility/ADR rules, dependency
review, and the exact local checks from Task 4.

`SECURITY.md` must explain that Noetic and Ubuntu 20.04 are outside normal
support, that networked deployment needs an explicit threat assessment, that
secrets and vehicle credentials never enter the repository, and that exploitable
security defects should use GitHub private vulnerability reporting rather than a
public issue. Non-sensitive functional safety defects may use the bug form.

- [ ] **Step 4: Add repository text conventions**

Create `.gitattributes`:

```gitattributes
* text=auto eol=lf
*.png binary
*.jpg binary
*.jpeg binary
*.gif binary
*.bag binary
*.pcd binary
```

Create `.editorconfig`:

```ini
root = true

[*]
charset = utf-8
end_of_line = lf
insert_final_newline = true
trim_trailing_whitespace = true
indent_style = space
indent_size = 2

[*.py]
indent_size = 4

[Makefile]
indent_style = tab
```

Create `.gitignore` with catkin/CMake outputs (`build/`, `devel/`, `install/`,
`log/`, `.catkin_tools/`), Python caches, coverage, editor state, OS metadata,
runtime logs, rosbags, PCD captures, and generated maps. Do not ignore source,
configuration, fixtures, or the global worktree location.

- [ ] **Step 5: Record written approval in the design baseline**

Change the spec status from:

```text
approved in discussion; awaiting review of this written baseline
```

to:

```text
approved in discussion and written review
```

- [ ] **Step 6: Check and commit Task 1**

Run:

```powershell
git diff --check
git status --short
git add README.md ARCHITECTURE.md ROADMAP.md AGENTS.md CONTRIBUTING.md SECURITY.md .gitignore .gitattributes .editorconfig docs/superpowers/specs/2026-08-10-auto-rover-architecture-design.md
git commit -m "docs: establish repository foundation"
```

Expected: one focused documentation/policy commit and a clean worktree.

### Task 2: Add authoritative supporting documentation

**Files:**
- Create: `docs/README.md`
- Create: `docs/architecture/repository-layout.md`
- Create: `docs/architecture/architecture-policy.json`
- Create: `docs/interfaces/contracts.md`
- Create: `docs/vehicles/vcu-adapter-readiness.md`
- Create: `docs/testing/phase-1-acceptance.md`
- Create: `docs/governance/dependencies-and-licensing.md`
- Create: `docs/adr/README.md`
- Create: `docs/adr/0000-template.md`

- [ ] **Step 1: Create the documentation authority index**

`docs/README.md` must distinguish authorities:

- `ARCHITECTURE.md`: concise current architecture;
- approved spec: original detailed baseline and rationale;
- `docs/interfaces/contracts.md`: contract registry;
- `docs/vehicles/vcu-adapter-readiness.md`: first-VCU evidence gate;
- `docs/testing/phase-1-acceptance.md`: measurement requirements;
- ADRs: changes to already accepted decisions.

When documents conflict, the newest accepted ADR wins, followed by the approved
architecture baseline; the conflict must be repaired in the same PR.

- [ ] **Step 2: Encode the approved policy as strict JSON**

Create `docs/architecture/architecture-policy.json` exactly as:

```json
{
  "schema_version": 1,
  "repository_strategy": "monorepo-first",
  "autonomy_layers": ["perception", "planning", "control"],
  "phase_1": {
    "runtime": {
      "os": "ubuntu-20.04",
      "middleware": "ros1-noetic"
    },
    "application": "known-map-navigation",
    "deployment_model": "single-purpose",
    "route_generation": "off-board",
    "onboard_planning": "route-plan-to-trajectory",
    "task_manager": false,
    "world_model": "reserved-boundary",
    "local_obstacle_avoidance": false
  },
  "migration": {
    "ros2": "native-after-phase-1",
    "dual_runtime_phase_1": false
  },
  "contracts": {
    "localization": "EgoState",
    "route": "RoutePlan",
    "trajectory": "Trajectory",
    "control_output": "MotionReference",
    "chassis_feedback": "ChassisState"
  },
  "vehicle_integration": {
    "universal_cmd_vel": false,
    "vcu_adapter_required": true,
    "chassis_feedback_required": true,
    "preserve_existing_vcu_inner_loop": true
  }
}
```

- [ ] **Step 3: Document repository, interface, vehicle, test, and license gates**

`repository-layout.md` must reproduce the target tree from the approved spec,
mark future directories as create-on-demand, and show this dependency graph:

```text
auto_rover_core -> algorithm core libraries
auto_rover_core + auto_rover_interfaces -> ROS 1 conversions and wrappers
algorithm cores + ROS 1 wrappers -> application bringup and integration
```

`contracts.md` must register `EgoState`, `RoutePlan`, `Trajectory`,
`MotionReference`, `ChassisState`, `VehicleProfile`, `SafetyState`, and
`EmergencyStop`; `WorldModel` remains a reserved name only. Each row must record
owner, phase, ROS-message status, frame/unit/time/validity requirements, and
compatibility policy. No `.msg` fields are invented in this task.

`vcu-adapter-readiness.md` must require evidence for protocol transport,
watchdog, command rates, signed-speed versus confirmed-gear direction policy,
standstill feedback, steering semantics, available chassis feedback, existing
inner loops, enable/override/fault states, braking/holding behavior, hardware
E-stop, and recorded replay fixtures. An adapter cannot be called ready until
the checklist is satisfied.

`phase-1-acceptance.md` must require versioned numeric values for run count,
route-completion rate, lateral/heading errors, command rate/age, stale-input
response, stop time/distance, direction-change speed threshold, and hold-release
conditions. It must explicitly prohibit invented universal thresholds.

`dependencies-and-licensing.md` must require tag/SHA pins, upstream/source URL,
license/SPDX identifier, modifications, redistribution obligations, ROS/platform
fit, maintenance evidence, and an owner. It must state that the absence of a
project `LICENSE` is intentional until the decision gate closes.

- [ ] **Step 4: Add the ADR process**

`docs/adr/README.md` must define sequential four-digit numbering, statuses
`Proposed`, `Accepted`, `Superseded`, and `Rejected`, and require ADRs for public
interfaces, safety boundaries, dependency direction, platform strategy, or
repository extraction.

`docs/adr/0000-template.md` must contain these headings:

```markdown
# ADR 0000: Decision title

- Status: Proposed
- Date: YYYY-MM-DD
- Owners: GitHub usernames

## Context
## Decision
## Alternatives considered
## Consequences
## Compatibility and migration
## Verification
```

- [ ] **Step 5: Validate links manually and commit Task 2**

Run:

```powershell
git diff --check
git status --short
git add docs
git commit -m "docs: organize architecture guidance"
```

Expected: supporting documents are non-empty, relative links resolve, and no ROS
package skeleton is present.

### Task 3: Add GitHub contribution metadata and CI workflow

**Files:**
- Create: `.github/PULL_REQUEST_TEMPLATE.md`
- Create: `.github/ISSUE_TEMPLATE/bug-report.yml`
- Create: `.github/ISSUE_TEMPLATE/change-proposal.yml`
- Create: `.github/ISSUE_TEMPLATE/config.yml`
- Create: `.github/workflows/repository-validation.yml`

- [ ] **Step 1: Add the pull-request checklist**

The template must request summary/scope, test evidence, affected layer, interface
compatibility, safety impact, dependency/license impact, documentation, and an
ADR link when applicable. It must include explicit confirmations that no
vehicle-specific protocol leaked upward and no safety behavior was weakened.

- [ ] **Step 2: Add structured issue forms**

`bug-report.yml` must collect description, reproduction, expected/actual
behavior, affected layer, platform, version/commit, logs, and safety impact.

`change-proposal.yml` must collect problem, proposed outcome, scope, non-goals,
affected contracts/layers, safety impact, alternatives, and acceptance evidence.

`config.yml` must disable blank issues and provide one contact link to GitHub's
private vulnerability reporting page for this repository.

- [ ] **Step 3: Add the stable required workflow**

Create `.github/workflows/repository-validation.yml`:

```yaml
name: Repository validation

on:
  pull_request:
  push:
    branches:
      - main

permissions:
  contents: read

concurrency:
  group: repository-validation-${{ github.ref }}
  cancel-in-progress: true

jobs:
  required:
    name: required
    runs-on: ubuntu-24.04
    timeout-minutes: 5

    steps:
      - name: Check out repository
        uses: actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1 # v7.0.1

      - name: Set up Python
        uses: actions/setup-python@5fda3b95a4ea91299a34e894583c3862153e4b97 # v7.0.0
        with:
          python-version: "3.8.18"

      - name: Run validator unit tests
        run: python -m unittest discover -s tests/repository -p "test_*.py" -v

      - name: Validate repository
        run: python tools/ci/validate_repository.py --root .
```

The action SHAs are verified live against the corresponding release tags. The
job/check name `required` must remain stable because branch protection uses it.

- [ ] **Step 4: Commit Task 3**

Run:

```powershell
git diff --check
git add .github
git commit -m "chore: add GitHub contribution workflows"
```

Expected: the workflow is committed before the branch is pushed, but it will not
run until the validator from Task 4 exists in the same branch.

### Task 4: Build the repository validator test-first

**Files:**
- Create: `tests/repository/test_validate_repository.py`
- Create: `tools/ci/validate_repository.py`

- [ ] **Step 1: Write validator tests before the implementation**

Use `unittest` and `tempfile.TemporaryDirectory`. A `make_valid_repository()`
fixture must create every required file, a valid architecture policy, simple
Issue Forms, and the required workflow. Add tests for:

1. valid fixture;
2. missing and empty required files;
3. prohibited placeholder markers while ignoring generated directories and the
   implementation-plan directory;
4. valid, missing, escaping, and wrong-case relative Markdown links;
5. links inside fenced code;
6. paired and unpaired backtick/tilde fences;
7. malformed and duplicate-key JSON;
8. changed required architecture-policy values;
9. Issue Form required keys and duplicate IDs;
10. workflow required keys and unsafe `pull_request_target`;
11. deterministic finding order.

Use string concatenation inside tests when constructing prohibited marker words,
so the validator does not report its own test fixtures.

- [ ] **Step 2: Run the tests and verify the expected failure**

Run:

```powershell
python -m unittest discover -s tests/repository -p "test_*.py" -v
```

Expected: import failure because `tools/ci/validate_repository.py` does not yet
exist. A passing result here means the test is not exercising the planned API.

- [ ] **Step 3: Implement the dependency-free validator**

Expose this public API and CLI:

```python
@dataclass(frozen=True, order=True)
class Finding:
    path: str
    line: int
    code: str
    message: str

def validate_repository(root: Path) -> List[Finding]:
    """Return deterministic repository-policy findings."""

def main(argv: Optional[Sequence[str]] = None) -> int:
    """Print findings and return 0 for success or 1 for violations."""
```

Implementation requirements:

- standard library only and Python 3.8 compatible;
- required-path existence/non-empty checks with exact case;
- UTF-8 decoding for first-party text;
- deterministic scans excluding `.git`, generated ROS/build outputs, virtual
  environments, `third_party`, and `docs/superpowers/plans`;
- placeholder-marker detection;
- state-machine Markdown fence validation;
- relative Markdown link/image resolution outside fenced code, including
  percent-decoding, repository-escape prevention, and Windows case checks;
- strict JSON parsing with duplicate-key rejection;
- recursive subset validation of `architecture-policy.json` against Task 2;
- honest limited YAML checks: no tabs, two-space indentation, unique top-level
  keys, required Issue Form keys/unique body IDs, and workflow keys/job shape;
- workflow rejection of `pull_request_target`, write-all permissions, and
  writable contents permission;
- output format `path:line: CODE message`, sorted by the dataclass fields.

- [ ] **Step 4: Run red-green verification**

Run:

```powershell
python -m unittest discover -s tests/repository -p "test_*.py" -v
python tools/ci/validate_repository.py --root .
python -m compileall -q tools/ci tests/repository
```

Expected: all tests pass, repository validation prints a success summary, and
compileall exits zero.

- [ ] **Step 5: Commit Task 4**

Run:

```powershell
git add tools/ci/validate_repository.py tests/repository/test_validate_repository.py
git commit -m "ci: validate repository structure"
```

Expected: tests and implementation land together after the red-green evidence.

### Task 5: Verify, review, publish, and merge the foundation

**Files:** all files from Tasks 1-4.

- [ ] **Step 1: Run the complete local verification gate**

Run:

```powershell
python -m unittest discover -s tests/repository -p "test_*.py" -v
python tools/ci/validate_repository.py --root .
python -m compileall -q tools/ci tests/repository
git diff --check origin/main...HEAD
git status --short
```

Expected: zero failed tests/findings, no compile errors, no whitespace errors,
and no uncommitted files.

- [ ] **Step 2: Self-review spec coverage**

Compare the changed files with all 13 sections of the approved spec. Confirm that
every phase-1 boundary has an authoritative document or machine-readable policy,
and confirm that no implementation package or guessed hardware fact was added.

- [ ] **Step 3: Push and open a draft PR with gh**

Run:

```powershell
git push -u origin agent/repository-foundation
gh pr create --repo niuma-phd/AUTO_ROVER --base main --head agent/repository-foundation --draft --title "Establish AUTO_ROVER repository foundation" --body "Adds the approved project documentation, machine-readable architecture policy, contribution templates, and a dependency-free required repository validation workflow. No ROS package, vehicle protocol, license, or acceptance threshold is invented."
```

- [ ] **Step 4: Obtain independent content and code reviews**

Review the complete diff for architecture/spec compliance, then review the
validator/tests for defects and false positives. Apply valid findings in focused
commits and rerun Step 1 after every correction.

- [ ] **Step 5: Wait for GitHub Actions and merge**

Run:

```powershell
gh pr checks --repo niuma-phd/AUTO_ROVER --watch
gh pr ready --repo niuma-phd/AUTO_ROVER
gh pr merge --repo niuma-phd/AUTO_ROVER --squash --delete-branch
```

Expected: `required` succeeds and the PR merges through GitHub. Fetch and verify
that `origin/main` contains the squash commit.

### Task 6: Apply repository governance and create the phase-1 backlog

**External state:** `niuma-phd/AUTO_ROVER` settings, labels, milestone, issues,
Actions policy, and `main` protection.

- [ ] **Step 1: Configure the repository with gh**

Patch the repository to use the approved description, disable the Wiki in favor
of versioned docs, keep Issues and Projects enabled, enable squash-only merging,
auto-merge, update-branch, and automatic head-branch deletion. Enable GitHub
private vulnerability reporting so `SECURITY.md` has a functioning private
channel. Add topics:
`autonomous-vehicles`, `robotics`, `ros`, `ros-noetic`, and
`ackermann-steering`. Keep the workflow token read-only.

- [ ] **Step 2: Restrict Actions to GitHub-owned actions**

Because the workflow uses only pinned `actions/checkout` and
`actions/setup-python`, set Actions to selected mode with GitHub-owned actions
allowed, verified/third-party actions disabled, and no custom patterns.

- [ ] **Step 3: Create additive labels and one milestone**

Keep the nine default labels and idempotently create:

```text
area:core, area:interfaces, area:perception, area:planning, area:control,
area:vehicle, area:safety, area:integration, phase:1, future,
priority:p0, priority:p1, priority:p2
```

Create `Phase 1 - Known-map navigation` without a guessed due date.

- [ ] **Step 4: Create exactly nine phase-1 issues**

Create issues in this order with measurable acceptance sections and the area,
phase, priority, and `enhancement`/`documentation` labels:

1. Define and freeze phase-1 v1 contracts.
2. Implement middleware-independent core geometry and validation.
3. Normalize the first LIO localization source into EgoState.
4. Load offline RoutePlan and generate Trajectory.
5. Track Trajectory and emit MotionReference.
6. Implement configurable Ackermann vehicle manager and first VCU adapter.
7. Implement watchdogs, fail-closed direction changes, and E-stop handling.
8. Integrate known-map navigation and replay/simulation acceptance.
9. Complete versioned real-vehicle phase-1 acceptance.

Do not create future ROS 2, environment-model, avoidance, exploration, or field
coverage issues yet; those are explicit boundaries, not active backlog noise.

- [ ] **Step 5: Protect main only after a successful main check exists**

Query the current `main` SHA and require an actual successful check run named
`required` on that SHA. If absent, wait for the push workflow instead of applying
protection. Then protect `main` with strict required status checks, linear
history, resolved conversations, no force-push, and no deletion. Use zero
required approvals for the single-maintainer repository and retain the admin
recovery bypass.

- [ ] **Step 6: Verify all external state**

Use `gh repo view`, `gh label list`, `gh issue list`, `gh api` for the milestone,
Actions policies, and branch protection, and `gh run list` for the main run.
Report exact URLs and any unavailable setting rather than assuming success.

## Final verification checklist

- [ ] Approved architecture status is updated.
- [ ] Root and supporting documents agree on phase-1 scope.
- [ ] Machine-readable policy matches the approved design.
- [ ] No empty ROS package, speculative adapter, license, or numeric vehicle
  threshold exists.
- [ ] Validator unit tests and repository validation pass locally and in Actions.
- [ ] Foundation PR is squash-merged.
- [ ] Repository is squash-only with read-only workflow token.
- [ ] Nine phase-1 issues and one milestone exist.
- [ ] `main` protection references a check that has already succeeded.
