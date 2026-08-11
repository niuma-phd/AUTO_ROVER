# ADR 0006: Bounded known-map planning resources

- Status: Proposed
- Date: 2026-08-11
- Owners: niuma-phd
- Decision authority: niuma-phd
- Approval evidence: Pending explicit repository-owner review

## Context

The known-map waypoint document is operator-controlled input that is parsed in
the on-board planning process. The strict schema already rejected malformed,
unknown, non-finite, and contract-invalid values, but it did not bound the file
before `yaml-cpp` parsed it. It also allowed an unbounded waypoint sequence and
unbounded identity strings before later validation. Those gaps permitted
unbounded parser, allocation, hashing, duplicate-position, and Hermite-probe
work from an input that could eventually request motion.

The formal vehicle-execution boundary accepts at most 4,096 trajectory points
and 256 bytes per identity. The planner previously allowed up to 1,000,000
points, so it could spend substantially more work producing a trajectory that
the guarded execution consumer was required to reject. The generated
`trajectory_id` also appends a plan version and fingerprint to `route_id`; a
256-byte route identifier would therefore exceed the downstream 256-byte
identity budget.

## Decision

Known-map planning uses compile-time resource limits declared in the public
ROS-free header `auto_rover_planning/resource_limits.hpp`. They are safety
contracts, not runtime parameters:

| Resource | Maximum | Enforcement point |
|---|---:|---|
| YAML document | 1,048,576 bytes | Before `YAML::Load` for both string and file entry points |
| YAML map key | 64 bytes | Before key insertion or diagnostic expansion |
| Numeric scalar | 64 bytes | Before regular-expression or numeric conversion work |
| Boolean scalar | 5 bytes | Before value comparison |
| `frame_id` | 256 UTF-8 bytes | Loader and generator |
| `route_id` | 210 UTF-8 bytes | Loader and generator |
| Vehicle `profile_id` used by the generator | 256 UTF-8 bytes | Generator |
| Route waypoints | 2,048 | Before reserve, duplicate checks, hashing, or Hermite work |
| Generated `trajectory_id` | 256 bytes | Immediately after construction and before sampling |
| Generated trajectory points | 4,096 total points | Before allocation and sampling |

The file entry point performs a bounded binary read and does not call
`YAML::LoadFile`. A file is read completely only while it remains within the
document budget, and parsing starts only after successful end-of-file. The
string entry point checks the encoded string size before invoking `yaml-cpp`.
No over-limit value is truncated.

The route identifier budget is 210 bytes because the generated identity adds
46 worst-case bytes: two separators, a 20-digit unsigned 64-bit plan version,
the eight-byte `fnv1a64-` marker, and sixteen hexadecimal fingerprint bytes.
The resulting worst-case identity is exactly 256 bytes.

The 2,048-waypoint limit bounds the loader's duplicate-position work and the
generator's per-segment feasibility probes. Hermite sampling uses at least two
intervals per segment, so a 2,048-waypoint route has a minimum of 4,095 output
points. Geometry or sampling resolution that requires more than the 4,096-point
execution budget is rejected even when the input waypoint count itself is
within its limit.

The generator repeats the route, frame, waypoint, and profile checks for direct
ROS-free callers so bypassing the YAML loader cannot bypass the resource
contract. Its point accounting starts with the first trajectory point and adds
each segment's interval count; exactly 4,096 total points are permitted and the
4,097th is rejected. Every resource failure returns an invalid trajectory with
an empty point sequence and cannot preserve a previously executable route
through the transactional reload path.

## Alternatives considered

- Relying only on the execution consumer's 4,096-point check was rejected
  because planning would still parse, validate, hash, allocate, and sample an
  input that cannot execute.
- Keeping the former 1,000,000-point generator limit was rejected because it is
  inconsistent with the Phase-1 execution envelope and consumes unnecessary
  memory and CPU before a mandatory downstream rejection.
- Using configurable limits was rejected because a launch-file edit could then
  silently widen a safety-relevant resource boundary.
- Checking a file's metadata size and then calling `YAML::LoadFile` was rejected
  because the file could change between the check and the parser read. The
  bounded read is the exact byte sequence later parsed.
- Truncating oversized files, strings, identities, waypoint lists, or
  trajectories was rejected because truncation changes route identity or
  geometry rather than reporting an invalid plan.

## Consequences

Malformed or excessive planning input now fails earlier with bounded memory and
work after the pre-parse document envelope. Planning and guarded execution share
the same 4,096-point maximum, and every generated identity fits the established
execution identity envelope.

Routes near a limit can still be rejected for independent reasons such as
duplicate positions, speed, curvature, sampling density, or vehicle-profile
validity. The waypoint limit does not promise that every 2,048-waypoint route
will fit in 4,096 trajectory points. The 1 MiB document envelope bounds input
size but is not a general proof that every possible `yaml-cpp` implementation
has constant memory use; the pinned parser revision and strict schema remain
part of the reviewed dependency boundary.

## Compatibility and migration

The Phase-1 contracts are unreleased, so no released ROS message layout or MD5
contract changes. This proposal tightens accepted planning input and changes no
field, frame, unit, sign convention, or dependency direction. An existing route
that exceeds a new limit must be split or simplified off board and assigned a
new, strictly increasing `plan_version`; it is never silently migrated on the
vehicle.

The planner rejects oversized input before publication instead of constructing
a candidate that the execution process would later reject. Downstream execution
keeps its independent 4,096-point and 256-byte checks.

## Verification

Planning tests cover exact-limit and one-byte/one-item-over cases for string and
file documents, ASCII and multibyte UTF-8 route identities, frame identities,
numeric scalars, map keys, waypoint sequences, worst-case generated identity,
and direct programmatic inputs. Generator tests construct exactly 4,096 output
points and then require the 4,097-point case to fail closed with no executable
points. Package compilation remains warning-clean under C++14, and repository
validation, Python compilation, and `git diff --check` remain required.
