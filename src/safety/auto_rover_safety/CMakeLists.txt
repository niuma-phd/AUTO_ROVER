cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_safety LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS auto_rover_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES auto_rover_safety_core
  CATKIN_DEPENDS auto_rover_core
)

add_library(auto_rover_safety_core
  src/command_guard.cpp
  src/safety_supervisor.cpp
)
target_include_directories(auto_rover_safety_core
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_compile_features(auto_rover_safety_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_safety_core PRIVATE
  -Wall -Wextra -Wpedantic
)
target_link_libraries(auto_rover_safety_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_safety_core_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/safety/test_safety.cpp
  )
  target_compile_features(auto_rover_safety_core_tests PUBLIC cxx_std_14)
  target_link_libraries(auto_rover_safety_core_tests
    auto_rover_safety_core
    ${catkin_LIBRARIES}
  )
  add_test(NAME auto_rover_safety_core_tests
           COMMAND auto_rover_safety_core_tests)
endif()

install(TARGETS auto_rover_safety_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
