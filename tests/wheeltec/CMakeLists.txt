cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_wheeltec_sanitizer_tests LANGUAGES CXX)

enable_testing()

get_filename_component(AUTO_ROVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(WHEELTEC_PACKAGE
    "${AUTO_ROVER_ROOT}/src/vehicle/adapters/auto_rover_vcu_wheeltec_serial")

add_library(auto_rover_wheeltec_sanitizer_core STATIC
  "${WHEELTEC_PACKAGE}/src/adapter.cpp"
  "${WHEELTEC_PACKAGE}/src/bench.cpp"
  "${WHEELTEC_PACKAGE}/src/codec.cpp"
  "${WHEELTEC_PACKAGE}/src/feedback_capture.cpp"
  "${WHEELTEC_PACKAGE}/src/physical_activation.cpp"
  "${WHEELTEC_PACKAGE}/src/runtime.cpp"
  "${WHEELTEC_PACKAGE}/src/stream_parser.cpp"
  "${WHEELTEC_PACKAGE}/src/transport.cpp"
  "${WHEELTEC_PACKAGE}/src/vehicle_backend.cpp"
)
target_include_directories(auto_rover_wheeltec_sanitizer_core PUBLIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/include"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/include"
  "${WHEELTEC_PACKAGE}/include"
)
target_compile_features(auto_rover_wheeltec_sanitizer_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_wheeltec_sanitizer_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_executable(auto_rover_wheeltec_sanitizer_tests test_wheeltec.cpp)
target_compile_features(auto_rover_wheeltec_sanitizer_tests PUBLIC cxx_std_14)
target_compile_options(auto_rover_wheeltec_sanitizer_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_sanitizer_tests PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_codec_transport_watchdog_pty
  COMMAND auto_rover_wheeltec_sanitizer_tests
)

add_executable(auto_rover_wheeltec_physical_activation_tests
  test_physical_activation.cpp
)
target_compile_features(auto_rover_wheeltec_physical_activation_tests
  PUBLIC cxx_std_14
)
target_compile_options(auto_rover_wheeltec_physical_activation_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_physical_activation_tests PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_physical_activation_injected
  COMMAND auto_rover_wheeltec_physical_activation_tests
)

add_executable(auto_rover_wheeltec_runtime_tests test_runtime.cpp)
add_library(auto_rover_wheeltec_execution_test_support STATIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/src/geometry.cpp"
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/src/validation.cpp"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/src/command_guard.cpp"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/src/safety_supervisor.cpp"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/src/fake_vcu.cpp"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/src/vehicle_execution_core.cpp"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/src/vehicle_motion_manager.cpp"
)
target_include_directories(auto_rover_wheeltec_execution_test_support PUBLIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/include"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/include"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/include"
)
target_compile_features(auto_rover_wheeltec_execution_test_support
  PUBLIC cxx_std_14
)
target_compile_options(auto_rover_wheeltec_execution_test_support PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_compile_features(auto_rover_wheeltec_runtime_tests PUBLIC cxx_std_14)
target_compile_options(auto_rover_wheeltec_runtime_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_runtime_tests PRIVATE
  auto_rover_wheeltec_execution_test_support
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_runtime_backend_pty
  COMMAND auto_rover_wheeltec_runtime_tests
)

add_executable(auto_rover_wheeltec_feedback_capture_tests
  test_feedback_capture.cpp
)
target_compile_features(auto_rover_wheeltec_feedback_capture_tests
  PUBLIC cxx_std_14
)
target_compile_options(auto_rover_wheeltec_feedback_capture_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_feedback_capture_tests PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_feedback_capture_injected_pty
  COMMAND auto_rover_wheeltec_feedback_capture_tests
)

add_executable(auto_rover_wheeltec_bench_tests
  test_bench.cpp
)
target_compile_features(auto_rover_wheeltec_bench_tests PUBLIC cxx_std_14)
target_compile_options(auto_rover_wheeltec_bench_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_bench_tests PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_bench_characterization_injected_pty
  COMMAND auto_rover_wheeltec_bench_tests
)

add_library(auto_rover_wheeltec_raw_profile_experimental STATIC
  "${WHEELTEC_PACKAGE}/src/raw_profile.cpp"
)
target_include_directories(auto_rover_wheeltec_raw_profile_experimental PUBLIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/include"
  "${WHEELTEC_PACKAGE}/include"
)
target_compile_features(auto_rover_wheeltec_raw_profile_experimental
  PUBLIC cxx_std_14
)
target_compile_options(auto_rover_wheeltec_raw_profile_experimental PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_raw_profile_experimental PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_executable(auto_rover_wheeltec_raw_profile_tests
  test_raw_profile.cpp
)
target_compile_features(auto_rover_wheeltec_raw_profile_tests PUBLIC cxx_std_14)
target_compile_options(auto_rover_wheeltec_raw_profile_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_raw_profile_tests PRIVATE
  auto_rover_wheeltec_raw_profile_experimental
  auto_rover_wheeltec_sanitizer_core
)

add_test(NAME wheeltec_raw_profile_capture_injected
  COMMAND auto_rover_wheeltec_raw_profile_tests
)

add_executable(wheeltec_feedback_capture
  "${WHEELTEC_PACKAGE}/src/feedback_capture_main.cpp"
)
target_compile_features(wheeltec_feedback_capture PUBLIC cxx_std_14)
target_compile_options(wheeltec_feedback_capture PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_feedback_capture PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_executable(wheeltec_bench_characterize
  "${WHEELTEC_PACKAGE}/src/bench_main.cpp"
)
target_compile_features(wheeltec_bench_characterize PUBLIC cxx_std_14)
target_compile_options(wheeltec_bench_characterize PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_bench_characterize PRIVATE
  auto_rover_wheeltec_sanitizer_core
)

add_executable(wheeltec_raw_profile_capture
  "${WHEELTEC_PACKAGE}/src/raw_profile_main.cpp"
)
target_compile_features(wheeltec_raw_profile_capture PUBLIC cxx_std_14)
target_compile_options(wheeltec_raw_profile_capture PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_raw_profile_capture PRIVATE
  auto_rover_wheeltec_raw_profile_experimental
  auto_rover_wheeltec_sanitizer_core
)
