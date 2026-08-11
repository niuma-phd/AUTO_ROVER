# Deterministic pure-core integration test

This test drives the phase-1 software loop without a ROS graph, wall-clock
sleep, serial device, or physical vehicle. It builds the existing C++14 core
implementations directly and uses injected monotonic and ROS timestamps.

The scenario loads a two-metre YAML route, plans a Hermite trajectory, converts
a synthetic non-identity FAST-LIVO2 source pose to the rear-axle pose, tracks
with Pure Pursuit, applies the safety supervisor and command guard, creates
vehicle execution commands, and integrates Fake VCU feedback. It also exercises
localization and output watchdogs, latched emergency-stop reset rules, and
disconnect/reconnect authorization loss.

On Ubuntu 20.04, install the repository's pinned compiler dependencies and
`libyaml-cpp-dev`, then run from the repository root:

```sh
cmake -S tests/integration -B /tmp/auto-rover-integration-build \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/auto-rover-integration-build --parallel
cmake -E chdir /tmp/auto-rover-integration-build \
  ctest --output-on-failure
```

The explicit working-directory change is required by the pinned CMake/CTest
3.16 toolchain on Ubuntu 20.04; that release does not support `ctest
--test-dir`.

The test executable is compiled with `-Wall -Wextra -Wpedantic -Werror` and has
no ROS dependency.
