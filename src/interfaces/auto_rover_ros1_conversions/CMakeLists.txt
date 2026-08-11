cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_ros1_conversions LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  auto_rover_core
  auto_rover_interfaces
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_ros1_conversions
  CATKIN_DEPENDS
    auto_rover_core
    auto_rover_interfaces
    geometry_msgs
    roscpp
    std_msgs
)

add_library(auto_rover_ros1_conversions src/conversions.cpp)
target_include_directories(auto_rover_ros1_conversions
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(auto_rover_ros1_conversions PUBLIC cxx_std_14)
target_compile_options(auto_rover_ros1_conversions PRIVATE
  -Wall -Wextra -Wpedantic
)
add_dependencies(auto_rover_ros1_conversions
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(auto_rover_ros1_conversions ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(auto_rover_ros1_conversions_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/interfaces/test_ros1_conversions.cpp
  )
  if(TARGET auto_rover_ros1_conversions_tests)
    add_dependencies(auto_rover_ros1_conversions_tests ${catkin_EXPORTED_TARGETS})
    target_link_libraries(auto_rover_ros1_conversions_tests
      auto_rover_ros1_conversions
      ${catkin_LIBRARIES}
    )
    target_compile_features(auto_rover_ros1_conversions_tests PUBLIC cxx_std_14)
  endif()
endif()

install(TARGETS auto_rover_ros1_conversions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
