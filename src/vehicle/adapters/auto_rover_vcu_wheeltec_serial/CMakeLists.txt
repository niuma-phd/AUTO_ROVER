cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_vcu_wheeltec_serial LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  auto_rover_ros1_conversions
  auto_rover_vehicle
  roscpp
)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_vcu_wheeltec_serial
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    auto_rover_ros1_conversions
    auto_rover_vehicle
    roscpp
)

add_library(auto_rover_vcu_wheeltec_serial
  src/adapter.cpp
  src/bench.cpp
  src/codec.cpp
  src/feedback_capture.cpp
  src/physical_activation.cpp
  src/runtime.cpp
  src/stream_parser.cpp
  src/transport.cpp
  src/vehicle_backend.cpp
)
target_include_directories(auto_rover_vcu_wheeltec_serial
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(auto_rover_vcu_wheeltec_serial PUBLIC cxx_std_14)
target_compile_options(auto_rover_vcu_wheeltec_serial
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_vcu_wheeltec_serial
  PUBLIC
    ${auto_rover_core_LIBRARIES}
)

add_executable(wheeltec_feedback_capture
  src/feedback_capture_main.cpp
)
target_compile_features(wheeltec_feedback_capture PUBLIC cxx_std_14)
target_compile_options(wheeltec_feedback_capture
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_feedback_capture
  auto_rover_vcu_wheeltec_serial
)

add_executable(wheeltec_bench_characterize
  src/bench_main.cpp
)
target_compile_features(wheeltec_bench_characterize PUBLIC cxx_std_14)
target_compile_options(wheeltec_bench_characterize
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_bench_characterize
  auto_rover_vcu_wheeltec_serial
)

add_executable(wheeltec_vehicle_execution_node
  src/wheeltec_vehicle_execution_node.cpp
)
target_include_directories(wheeltec_vehicle_execution_node
  BEFORE
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(wheeltec_vehicle_execution_node PUBLIC cxx_std_14)
target_compile_options(wheeltec_vehicle_execution_node
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
add_dependencies(wheeltec_vehicle_execution_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(wheeltec_vehicle_execution_node
  PRIVATE
    auto_rover_vcu_wheeltec_serial
    ${catkin_LIBRARIES}
    Threads::Threads
)

# Intentionally separate from the exported Phase-1 adapter library.  This
# raised-bench-only raw capture target is UNVERIFIED, opt-in, and not installed.
add_library(auto_rover_wheeltec_raw_profile_experimental EXCLUDE_FROM_ALL STATIC
  src/raw_profile.cpp
)
target_include_directories(auto_rover_wheeltec_raw_profile_experimental
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(auto_rover_wheeltec_raw_profile_experimental
  PUBLIC cxx_std_14
)
target_compile_options(auto_rover_wheeltec_raw_profile_experimental
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_wheeltec_raw_profile_experimental
  auto_rover_vcu_wheeltec_serial
)

add_executable(wheeltec_raw_profile_capture EXCLUDE_FROM_ALL
  src/raw_profile_main.cpp
)
target_compile_features(wheeltec_raw_profile_capture PUBLIC cxx_std_14)
target_compile_options(wheeltec_raw_profile_capture
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(wheeltec_raw_profile_capture
  auto_rover_wheeltec_raw_profile_experimental
  auto_rover_vcu_wheeltec_serial
)

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_vcu_wheeltec_serial_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_wheeltec.cpp
  )
  target_compile_options(auto_rover_vcu_wheeltec_serial_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_vcu_wheeltec_serial_tests
    auto_rover_vcu_wheeltec_serial
  )
  add_test(
    NAME auto_rover_vcu_wheeltec_serial_tests
    COMMAND auto_rover_vcu_wheeltec_serial_tests
  )

  add_executable(auto_rover_wheeltec_physical_activation_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_physical_activation.cpp
  )
  target_compile_features(auto_rover_wheeltec_physical_activation_tests
    PUBLIC cxx_std_14
  )
  target_compile_options(auto_rover_wheeltec_physical_activation_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_wheeltec_physical_activation_tests
    auto_rover_vcu_wheeltec_serial
  )
  add_test(
    NAME auto_rover_wheeltec_physical_activation_tests
    COMMAND auto_rover_wheeltec_physical_activation_tests
  )

  add_executable(auto_rover_vcu_wheeltec_runtime_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_runtime.cpp
  )
  target_compile_options(auto_rover_vcu_wheeltec_runtime_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_vcu_wheeltec_runtime_tests
    auto_rover_vcu_wheeltec_serial
    ${catkin_LIBRARIES}
  )
  add_test(
    NAME auto_rover_vcu_wheeltec_runtime_tests
    COMMAND auto_rover_vcu_wheeltec_runtime_tests
  )

  # This is intentionally a catkin-only cross-package regression.  It links
  # the protocol-independent execution core to the formal Wheeltec backend and
  # uses only an injected ByteTransport; no physical path can be opened.
  add_executable(auto_rover_wheeltec_formal_backend_integration_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_formal_backend_integration.cpp
  )
  target_compile_features(
    auto_rover_wheeltec_formal_backend_integration_tests PUBLIC cxx_std_14
  )
  target_compile_options(
    auto_rover_wheeltec_formal_backend_integration_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  add_dependencies(auto_rover_wheeltec_formal_backend_integration_tests
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(
    auto_rover_wheeltec_formal_backend_integration_tests
    PRIVATE
      auto_rover_vcu_wheeltec_serial
      ${catkin_LIBRARIES}
  )
  add_test(
    NAME auto_rover_wheeltec_formal_backend_integration_tests
    COMMAND auto_rover_wheeltec_formal_backend_integration_tests
  )
  set(_wheeltec_test_library_dirs
    ${CATKIN_DEVEL_PREFIX}/lib
    ${catkin_LIBRARY_DIRS}
  )
  string(REPLACE ";" ":" _wheeltec_test_library_path
    "${_wheeltec_test_library_dirs}"
  )
  set_tests_properties(
    auto_rover_vcu_wheeltec_runtime_tests
    auto_rover_wheeltec_formal_backend_integration_tests
    PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${_wheeltec_test_library_path}:$ENV{LD_LIBRARY_PATH}"
  )

  add_executable(auto_rover_wheeltec_feedback_capture_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_feedback_capture.cpp
  )
  target_compile_options(auto_rover_wheeltec_feedback_capture_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_wheeltec_feedback_capture_tests
    auto_rover_vcu_wheeltec_serial
  )
  add_test(
    NAME auto_rover_wheeltec_feedback_capture_tests
    COMMAND auto_rover_wheeltec_feedback_capture_tests
  )

  add_executable(auto_rover_wheeltec_bench_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_bench.cpp
  )
  target_compile_options(auto_rover_wheeltec_bench_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_wheeltec_bench_tests
    auto_rover_vcu_wheeltec_serial
  )
  add_test(
    NAME auto_rover_wheeltec_bench_tests
    COMMAND auto_rover_wheeltec_bench_tests
  )

  add_executable(auto_rover_wheeltec_raw_profile_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec/test_raw_profile.cpp
  )
  target_compile_features(auto_rover_wheeltec_raw_profile_tests
    PUBLIC cxx_std_14
  )
  target_compile_options(auto_rover_wheeltec_raw_profile_tests
    PRIVATE -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_wheeltec_raw_profile_tests
    auto_rover_wheeltec_raw_profile_experimental
    auto_rover_vcu_wheeltec_serial
  )
  add_test(
    NAME auto_rover_wheeltec_raw_profile_tests
    COMMAND auto_rover_wheeltec_raw_profile_tests
  )

  find_package(PythonInterp 3 REQUIRED)
  add_test(
    NAME auto_rover_wheeltec_vehicle_execution_node_contract_tests
    COMMAND ${PYTHON_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/wheeltec
      -p test_vehicle_execution_node_contract.py
      -v
  )
  set_tests_properties(
    auto_rover_wheeltec_vehicle_execution_node_contract_tests
    PROPERTIES ENVIRONMENT PYTHONDONTWRITEBYTECODE=1
  )
endif()

install(TARGETS
  auto_rover_vcu_wheeltec_serial
  wheeltec_feedback_capture
  wheeltec_bench_characterize
  wheeltec_vehicle_execution_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "raw_profile.hpp" EXCLUDE
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
