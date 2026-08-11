uint8 TIME_SOURCE_UNKNOWN=0
uint8 TIME_SOURCE_SAMPLE=1
uint8 TIME_SOURCE_PUBLISH=2
uint8 TIME_SOURCE_RECEIPT=3

uint32 POSE_COVARIANCE_VALID=1
uint32 TWIST_VALID=2
uint32 TWIST_COVARIANCE_VALID=4

std_msgs/Header header
uint32 schema_version
uint64 state_id
uint8 time_source
string reference_frame
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
uint32 valid_mask
string source_id
bool valid
