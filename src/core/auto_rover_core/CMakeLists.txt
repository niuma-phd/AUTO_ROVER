cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_core LANGUAGES CXX)

find_package(catkin QUIET)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES auto_rover_core
  )
endif()

add_library(auto_rover_core
  src/geometry.cpp
  src/validation.cpp
)
target_include_directories(auto_rover_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(auto_rover_core PUBLIC cxx_std_14)
target_compile_options(auto_rover_core PRIVATE -Wall -Wextra -Wpedantic)

if(CATKIN_ENABLE_TESTING)
  add_executable(auto_rover_core_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/core/test_core.cpp
  )
  target_link_libraries(auto_rover_core_tests auto_rover_core)
  add_test(NAME auto_rover_core_tests COMMAND auto_rover_core_tests)
endif()

install(TARGETS auto_rover_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
