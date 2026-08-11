uint8 COMPLETION_UNKNOWN=0
uint8 COMPLETION_STOP_AND_HOLD=1

std_msgs/Header header
uint32 schema_version
string trajectory_id
string route_id
uint64 plan_version
string vehicle_profile_id
duration valid_for
uint8 completion_behavior
bool valid
auto_rover_interfaces/TrajectoryPoint[] points
