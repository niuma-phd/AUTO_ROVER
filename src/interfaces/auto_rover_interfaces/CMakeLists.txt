cmake_minimum_required(VERSION 3.0.2)
project(auto_rover_interfaces)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
    ChassisState.msg
    EgoState.msg
    EmergencyStop.msg
    MotionReference.msg
    RoutePlan.msg
    RouteWaypoint.msg
    SafetyState.msg
    Trajectory.msg
    TrajectoryPoint.msg
)

add_service_files(
  FILES
    ArmVehicle.srv
    AssertEmergencyStop.srv
    ReloadRoute.srv
    ResetEmergencyStop.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
