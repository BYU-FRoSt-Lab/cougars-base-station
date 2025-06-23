std_msgs/Header header

# 0 for acoustic modem, 1 for radio
uint8 connection_type

# id of vehicle connections
uint32[] vehicle_ids

# connection status
bool[] connections

# time since last ping response
uint32[] last_ping