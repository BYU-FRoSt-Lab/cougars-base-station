std_msgs/Header header

# 0 for acoustic modem, 1 for radio
uint8 connection_type

# connection status
bool[] connections

# time since last ping response
uint32[] last_ping