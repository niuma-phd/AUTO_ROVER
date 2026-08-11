cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_known_map_bringup)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS scripts/fake_fast_livo_source.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(
    PROGRAMS test/fake_closed_loop_rostest.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  add_rostest(test/fake_closed_loop.test)
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
