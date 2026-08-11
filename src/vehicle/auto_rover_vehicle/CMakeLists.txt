cmake_minimum_required(VERSION 3.10)
project(auto_rover_vehicle LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  auto_rover_ros1_conversions
  auto_rover_safety
  roscpp
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    auto_rover_vehicle_core
    auto_rover_vehicle_execution_core
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    auto_rover_ros1_conversions
    auto_rover_safety
    roscpp
    std_srvs
)

add_library(auto_rover_vehicle_core
  src/fake_vcu.cpp
  src/vehicle_motion_manager.cpp
)
target_include_directories(auto_rover_vehicle_core
  PUBLIC
    include
    ${auto_rover_core_INCLUDE_DIRS}
    ${auto_rover_safety_INCLUDE_DIRS}
)
target_compile_features(auto_rover_vehicle_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_vehicle_core PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Werror
)
target_link_libraries(auto_rover_vehicle_core
  PUBLIC
    ${auto_rover_core_LIBRARIES}
    ${auto_rover_safety_LIBRARIES}
)

add_library(auto_rover_vehicle_execution_core
  src/vehicle_execution_core.cpp
)
target_include_directories(auto_rover_vehicle_execution_core
  PUBLIC
    include
    ${auto_rover_core_INCLUDE_DIRS}
    ${auto_rover_safety_INCLUDE_DIRS}
)
target_compile_features(auto_rover_vehicle_execution_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_vehicle_execution_core PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Werror
)
target_link_libraries(auto_rover_vehicle_execution_core
  PUBLIC
    auto_rover_vehicle_core
    ${auto_rover_core_LIBRARIES}
    ${auto_rover_safety_LIBRARIES}
)

add_executable(vehicle_execution_node
  src/vehicle_execution_node.cpp
)
target_include_directories(vehicle_execution_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(vehicle_execution_node PUBLIC cxx_std_14)
target_compile_options(vehicle_execution_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Werror
)
add_dependencies(vehicle_execution_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(vehicle_execution_node
  PRIVATE
    auto_rover_vehicle_execution_core
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_vehicle_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/vehicle/test_vehicle.cpp
  )
  target_compile_features(auto_rover_vehicle_tests PUBLIC cxx_std_14)
  target_compile_options(auto_rover_vehicle_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
  )
  target_link_libraries(auto_rover_vehicle_tests
    auto_rover_vehicle_core
  )
  add_test(NAME auto_rover_vehicle_tests COMMAND auto_rover_vehicle_tests)

  add_executable(auto_rover_vehicle_execution_core_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/vehicle_ros/test_vehicle_execution_core.cpp
  )
  target_compile_features(auto_rover_vehicle_execution_core_tests
    PUBLIC cxx_std_14
  )
  target_compile_options(auto_rover_vehicle_execution_core_tests PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
  )
  target_link_libraries(auto_rover_vehicle_execution_core_tests
    auto_rover_vehicle_execution_core
  )
  add_test(
    NAME auto_rover_vehicle_execution_core_tests
    COMMAND auto_rover_vehicle_execution_core_tests
  )

  find_package(PythonInterp 3 REQUIRED)
  add_test(
    NAME auto_rover_vehicle_execution_node_contract_tests
    COMMAND ${PYTHON_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/vehicle_ros
      -p test_*.py
      -v
  )
  set_tests_properties(
    auto_rover_vehicle_execution_node_contract_tests
    PROPERTIES ENVIRONMENT PYTHONDONTWRITEBYTECODE=1
  )
endif()

install(TARGETS
    auto_rover_vehicle_core
    auto_rover_vehicle_execution_core
    vehicle_execution_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
