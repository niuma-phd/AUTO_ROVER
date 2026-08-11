cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_planning LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  auto_rover_ros1_conversions
  roscpp
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_planning
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    auto_rover_ros1_conversions
    roscpp
)

add_library(auto_rover_planning
  src/route_repository.cpp
  src/trajectory_generator.cpp
  src/waypoint_loader.cpp
)
target_include_directories(auto_rover_planning
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${auto_rover_core_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)
target_compile_features(auto_rover_planning PUBLIC cxx_std_14)
target_compile_options(auto_rover_planning
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(auto_rover_planning
  PUBLIC
    ${auto_rover_core_LIBRARIES}
    yaml-cpp
)

add_executable(route_planner_node
  src/route_planner_node.cpp
)
target_include_directories(route_planner_node
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(route_planner_node PUBLIC cxx_std_14)
target_compile_options(route_planner_node
  PRIVATE -Wall -Wextra -Wpedantic -Werror
)
add_dependencies(route_planner_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(route_planner_node
  PRIVATE
    auto_rover_planning
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(auto_rover_planning_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/planning/test_planning.cpp
  )
  if(TARGET auto_rover_planning_tests)
    target_compile_options(auto_rover_planning_tests
      PRIVATE -Wall -Wextra -Wpedantic -Werror
    )
    target_link_libraries(auto_rover_planning_tests auto_rover_planning)
  endif()
endif()

install(TARGETS
    auto_rover_planning
    route_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
