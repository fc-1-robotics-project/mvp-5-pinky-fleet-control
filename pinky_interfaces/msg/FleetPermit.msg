# A short-lived motion permit sent repeatedly by the fleet manager.
uint8 MODE_HOLD=0
uint8 MODE_RUN=1
uint8 MODE_ESTOP=2

std_msgs/Header header
string robot_id
string controller_id
uint64 sequence
uint8 mode
builtin_interfaces/Duration ttl
string lease_id
string[] allowed_zone_ids
