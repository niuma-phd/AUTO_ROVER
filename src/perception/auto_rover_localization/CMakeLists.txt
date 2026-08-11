cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_localization LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  auto_rover_ros1_conversions
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_localization_core
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    auto_rover_ros1_conversions
    nav_msgs
    roscpp
)

add_library(auto_rover_localization_core
  src/fast_livo_odometry_adapter_core.cpp
)
target_include_directories(auto_rover_localization_core
  PUBLIC
    include
    ${auto_rover_core_INCLUDE_DIRS}
)
target_compile_features(auto_rover_localization_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_localization_core
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
target_link_libraries(auto_rover_localization_core
  PUBLIC
    ${auto_rover_core_LIBRARIES}
)

add_executable(fast_livo_odometry_node
  src/fast_livo_odometry_node.cpp
)
target_include_directories(fast_livo_odometry_node
  PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(fast_livo_odometry_node PUBLIC cxx_std_14)
target_compile_options(fast_livo_odometry_node
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)
add_dependencies(fast_livo_odometry_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fast_livo_odometry_node
  PRIVATE
    auto_rover_localization_core
    ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_localization_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/localization/test_fast_livo_odometry_adapter_core.cpp
  )
  target_compile_options(auto_rover_localization_tests
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Werror
  )
  target_link_libraries(auto_rover_localization_tests
    PRIVATE
      auto_rover_localization_core
  )
  add_test(
    NAME auto_rover_localization_tests
    COMMAND auto_rover_localization_tests
  )
endif()

install(TARGETS
    auto_rover_localization_core
    fast_livo_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
