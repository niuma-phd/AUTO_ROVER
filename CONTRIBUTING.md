# Contributing to AUTO_ROVER

Use `feat/*` for features, `fix/*` for fixes, `exp/*` for experiments, and
`agent/*` for automated-agent work. Keep commits focused and independently
reviewable. Open a pull request with a clear scope, relevant tests, and evidence
for any changed safety, timing, direction, stop, or vehicle behavior. Squash
merge is the default.

Run these exact checks before opening a pull request:

```text
python -m unittest discover -s tests/repository -p "test_*.py" -v
python tools/ci/validate_repository.py --root .
python -m compileall -q tools/ci tests/repository
git diff --check
```

Public ROS 1 message, service, or action changes need explicit compatibility
review. A released ROS 1 message cannot receive a semantic or field-layout
breaking edit: introduce a V2 type and converter. Add an ADR before changing a
public contract, safety or failure semantics, module/dependency boundary,
supported runtime, vehicle direction capability, or VCU control-loop ownership.

External dependencies must be reviewed for interface fit, maintenance state,
target ROS version, and license. Pin each accepted dependency to a reviewed tag
or SHA; do not copy third-party source until licensing is recorded.

Safety-relevant pull requests include test or replay evidence for the affected
failure path. Vehicle-facing changes identify the vehicle profile, inputs,
limits, timeout behavior, fail-closed outcome, and any required acceptance
scenario. Preserve the VCU encoder-level inner loop when one exists.
