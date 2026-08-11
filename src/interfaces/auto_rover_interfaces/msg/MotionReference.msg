uint8 DIRECTION_UNKNOWN=0
uint8 DIRECTION_FORWARD=1
uint8 DIRECTION_REVERSE=2

std_msgs/Header header
uint32 schema_version
uint64 command_id
string producer_generation_id
string trajectory_id
uint8 direction
float64 target_speed_mps
float64 target_curvature_inv_m
duration valid_for
bool route_complete
bool valid
