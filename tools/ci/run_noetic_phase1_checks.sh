#!/usr/bin/env bash
set -euo pipefail

readonly REPOSITORY_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)"
readonly LOCK_FILE="${REPOSITORY_ROOT}/dependencies/noetic-focal-amd64.lock"
export PATH=/opt/ros/noetic/bin:/usr/bin:/bin
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1

# Noetic's catkin metadata is incompatible with the user-local CMake 4.x that
# may precede /usr/bin on developer machines. Always select the pinned Focal
# system toolchain before sourcing ROS.
source /opt/ros/noetic/setup.bash

if [[ "$(command -v cmake)" != "/usr/bin/cmake" ]]; then
  echo "refusing to build with a non-system CMake: $(command -v cmake)" >&2
  exit 1
fi

while IFS='=' read -r package expected_version; do
  [[ -z "${package}" || "${package}" == \#* ]] && continue
  actual_version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
  if [[ "${actual_version}" != "${expected_version}" ]]; then
    echo "dependency pin mismatch: ${package} expected ${expected_version}, got ${actual_version:-missing}" >&2
    exit 1
  fi
done < "${LOCK_FILE}"

check_root="$(mktemp -d /tmp/auto_rover_noetic.XXXXXXXX)"
cleanup() {
  case "${check_root}" in
    /tmp/auto_rover_noetic.*) rm -rf -- "${check_root}" ;;
    *) echo "refusing to remove unexpected check directory: ${check_root}" >&2 ;;
  esac
}
trap cleanup EXIT HUP INT TERM

cd "${REPOSITORY_ROOT}"
python3 -m unittest discover -s tests/repository -p "test_*.py" -v
python3 -m unittest discover -s tests/interfaces -p "test_*.py" -v
python3 -m unittest discover -s tests/ros_wrappers -p "test_*.py" -v
python3 -m unittest discover -s tests/scenarios -p "test_*.py" -v
python3 tools/ci/validate_repository.py --root .
python3 -m compileall -q tools/ci tests/repository
git diff --check

cmake -S tests/integration -B "${check_root}/integration" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build "${check_root}/integration" --parallel 2
integration_test_count="$(
  cmake -E chdir "${check_root}/integration" ctest -N |
    awk '/Total Tests:/ {print $3}'
)"
if [[ "${integration_test_count}" != "1" ]]; then
  echo "expected one pure-core integration test, found ${integration_test_count:-none}" >&2
  exit 1
fi
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  cmake -E chdir "${check_root}/integration" \
    ctest --output-on-failure

# Exercise the isolated UNVERIFIED codec/transport/watchdog implementation,
# including its pseudo-terminal path, under the same memory/undefined-behaviour
# instrumentation. This never selects or opens a physical controller device.
cmake -S tests/wheeltec -B "${check_root}/wheeltec" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build "${check_root}/wheeltec" --parallel 2
wheeltec_test_count="$(
  cmake -E chdir "${check_root}/wheeltec" ctest -N |
    awk '/Total Tests:/ {print $3}'
)"
if [[ "${wheeltec_test_count}" != "6" ]]; then
  echo "expected six Wheeltec sanitizer/injected/PTY tests, found ${wheeltec_test_count:-none}" >&2
  exit 1
fi
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  cmake -E chdir "${check_root}/wheeltec" \
    ctest --output-on-failure

mkdir -p "${check_root}/catkin"
catkin_make_isolated \
  -C "${check_root}/catkin" \
  --source "${REPOSITORY_ROOT}/src" \
  --build "${check_root}/catkin/build" \
  --devel "${check_root}/catkin/devel" \
  --install-space "${check_root}/catkin/install" \
  --install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCATKIN_ENABLE_TESTING=ON

# Build and execute catkin-managed tests first. In particular,
# catkin_add_gtest targets are excluded from the normal install build, so their
# executables must exist before the exhaustive raw CTest pass below.
catkin_make_isolated \
  -C "${check_root}/catkin" \
  --source "${REPOSITORY_ROOT}/src" \
  --build "${check_root}/catkin/build" \
  --devel "${check_root}/catkin/devel" \
  --install-space "${check_root}/catkin/install" \
  --install \
  --make-args run_tests

catkin_test_results "${check_root}/catkin/build"

# Several ROS-free packages intentionally use plain CTest executables instead
# of a catkin-specific test macro. catkin's run_tests target does not depend on
# those tests, so execute every isolated package test directory explicitly and
# make any failure fatal as a second, exhaustive package-level pass.
raw_ctest_count=0
raw_ctest_listing=""
while IFS= read -r package_build_dir; do
  package_test_listing="$(cmake -E chdir "${package_build_dir}" ctest -N)"
  raw_ctest_listing+=$'\n'"${package_test_listing}"
  package_test_count="$(
    awk '/Total Tests:/ {print $3}' <<< "${package_test_listing}"
  )"
  if [[ -n "${package_test_count}" && "${package_test_count}" -gt 0 ]]; then
    cmake -E chdir "${package_build_dir}" \
      "${check_root}/catkin/install/env.sh" \
      ctest --output-on-failure
    raw_ctest_count=$((raw_ctest_count + package_test_count))
  fi
done < <(
  find "${check_root}/catkin/build" -mindepth 2 -maxdepth 2 \
    -type f -name CMakeCache.txt -printf '%h\n' | sort
)
readonly expected_isolated_tests=(
  auto_rover_control_core_tests
  auto_rover_ros_wrapper_contract_tests
  auto_rover_core_tests
  fake_closed_loop.test
  auto_rover_localization_tests
  auto_rover_planning_tests
  auto_rover_ros1_conversions_tests
  auto_rover_safety_core_tests
  auto_rover_wheeltec_bench_tests
  auto_rover_wheeltec_feedback_capture_tests
  auto_rover_wheeltec_formal_backend_integration_tests
  auto_rover_wheeltec_physical_activation_tests
  auto_rover_wheeltec_raw_profile_tests
  auto_rover_wheeltec_vehicle_execution_node_contract_tests
  auto_rover_vcu_wheeltec_runtime_tests
  auto_rover_vcu_wheeltec_serial_tests
  auto_rover_vehicle_tests
  auto_rover_vehicle_execution_core_tests
  auto_rover_vehicle_execution_node_contract_tests
)
if [[ "${raw_ctest_count}" -lt "${#expected_isolated_tests[@]}" ]]; then
  echo "expected at least ${#expected_isolated_tests[@]} isolated package tests, found ${raw_ctest_count}" >&2
  exit 1
fi
for expected_test in "${expected_isolated_tests[@]}"; do
  if ! grep -Fq -- "${expected_test}" <<< "${raw_ctest_listing}"; then
    echo "isolated package test is not registered: ${expected_test}" >&2
    exit 1
  fi
done
