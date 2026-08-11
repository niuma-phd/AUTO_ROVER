uint8 TIME_SOURCE_UNKNOWN=0
uint8 TIME_SOURCE_SAMPLE=1
uint8 TIME_SOURCE_PUBLISH=2
uint8 TIME_SOURCE_RECEIPT=3

uint8 GEAR_UNKNOWN=0
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 GEAR_LOW=5
uint8 GEAR_OTHER=6

uint8 FAULT_UNKNOWN=0
uint8 FAULT_OK=1
uint8 FAULT_DEGRADED=2
uint8 FAULT=3
uint8 FAULT_ESTOP=4

uint32 MEASURED_SPEED_VALID=1
uint32 YAW_RATE_VALID=2
uint32 STEERING_ANGLE_VALID=4
uint32 GEAR_VALID=8
uint32 CONTROL_ENABLED_VALID=16
uint32 FAULT_VALID=32
uint32 VOLTAGE_VALID=64

std_msgs/Header header
uint32 schema_version
uint64 state_id
uint8 time_source
float64 measured_speed_mps
float64 yaw_rate_radps
float64 steering_tire_angle_rad
uint8 gear_state
bool control_enabled
uint8 fault_state
float64 supply_voltage_v
uint32 valid_mask
string source_id
bool valid
