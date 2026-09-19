# Robot-local gate state reported to the fleet manager.
std_msgs/Header header
string robot_id
uint8 gate_mode
bool permit_fresh
bool command_fresh
bool output_enabled
string status
string controller_id
uint64 last_permit_sequence
string active_lease_id
