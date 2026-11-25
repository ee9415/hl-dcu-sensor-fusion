# length is bytes (32); DLC on the bus is code 13.
std_msgs/Header header
# Host-local handoff only: protects queued frames even when ROS time freezes.
uint64 created_monotonic_ns
# Active path deadline; driver must check immediately before a physical write.
uint64 valid_until_monotonic_ns
uint32 can_id
bool is_extended
bool is_fd
bool bitrate_switch
uint8 length
uint8[32] data
