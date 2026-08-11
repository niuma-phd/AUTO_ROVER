cmake_minimum_required(VERSION 3.10)

project(auto_rover_pure_core_integration LANGUAGES CXX)

get_filename_component(AUTO_ROVER_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

find_package(yaml-cpp REQUIRED)

add_library(auto_rover_integration_pure_core STATIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/src/geometry.cpp"
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/src/validation.cpp"
  "${AUTO_ROVER_ROOT}/src/perception/auto_rover_localization/src/fast_livo_odometry_adapter_core.cpp"
  "${AUTO_ROVER_ROOT}/src/planning/auto_rover_planning/src/waypoint_loader.cpp"
  "${AUTO_ROVER_ROOT}/src/planning/auto_rover_planning/src/trajectory_generator.cpp"
  "${AUTO_ROVER_ROOT}/src/control/auto_rover_control/src/pure_pursuit.cpp"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/src/command_guard.cpp"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/src/safety_supervisor.cpp"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/src/vehicle_motion_manager.cpp"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/src/fake_vcu.cpp")

target_include_directories(auto_rover_integration_pure_core PUBLIC
  "${AUTO_ROVER_ROOT}/src/core/auto_rover_core/include"
  "${AUTO_ROVER_ROOT}/src/perception/auto_rover_localization/include"
  "${AUTO_ROVER_ROOT}/src/planning/auto_rover_planning/include"
  "${AUTO_ROVER_ROOT}/src/control/auto_rover_control/include"
  "${AUTO_ROVER_ROOT}/src/safety/auto_rover_safety/include"
  "${AUTO_ROVER_ROOT}/src/vehicle/auto_rover_vehicle/include")

target_compile_features(auto_rover_integration_pure_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_integration_pure_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(auto_rover_integration_pure_core PUBLIC yaml-cpp)

add_executable(auto_rover_full_loop_integration test_full_loop.cpp)
target_compile_features(auto_rover_full_loop_integration PRIVATE cxx_std_14)
target_compile_options(auto_rover_full_loop_integration PRIVATE
  -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(auto_rover_full_loop_integration PRIVATE
  auto_rover_integration_pure_core)

enable_testing()
add_test(NAME deterministic_pure_core_full_loop
  COMMAND auto_rover_full_loop_integration)
