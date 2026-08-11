cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_control LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  auto_rover_ros1_conversions
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_control_core
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    auto_rover_ros1_conversions
    roscpp
)

add_library(auto_rover_control_core src/pure_pursuit.cpp)
target_include_directories(auto_rover_control_core
  PUBLIC
    include
    ${auto_rover_core_INCLUDE_DIRS}
)
target_compile_features(auto_rover_control_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_control_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_control_core
  PUBLIC
    ${auto_rover_core_LIBRARIES}
)

add_executable(pure_pursuit_node
  src/pure_pursuit_node.cpp
)
target_include_directories(pure_pursuit_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(pure_pursuit_node PUBLIC cxx_std_14)
target_compile_options(pure_pursuit_node
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
add_dependencies(pure_pursuit_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pure_pursuit_node
  PRIVATE
    auto_rover_control_core
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_control_core_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/control/test_pure_pursuit.cpp
  )
  target_compile_features(auto_rover_control_core_tests PUBLIC cxx_std_14)
  target_compile_options(auto_rover_control_core_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror
  )
  target_link_libraries(auto_rover_control_core_tests
    auto_rover_control_core
  )
  add_test(NAME auto_rover_control_core_tests
           COMMAND auto_rover_control_core_tests)

  find_package(PythonInterp 3 REQUIRED)
  add_test(
    NAME auto_rover_ros_wrapper_contract_tests
    COMMAND ${PYTHON_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/ros_wrappers
      -p test_*.py
      -v
  )
  set_tests_properties(auto_rover_ros_wrapper_contract_tests PROPERTIES
    ENVIRONMENT PYTHONDONTWRITEBYTECODE=1
  )
endif()

install(TARGETS
    auto_rover_control_core
    pure_pursuit_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
