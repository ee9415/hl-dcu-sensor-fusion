# One atomic capture bundle. Individual diagnostic topics carry the same header.
# stamp is a host-anchored capture estimate, not a synchronized device wall clock.
std_msgs/Header header
string session
uint64 sequence
sensor_msgs/Image rgb
sensor_msgs/Image drivable_mask
sensor_msgs/Image lane_mask
sensor_msgs/Image depth
sensor_msgs/CameraInfo rgb_camera_info
sensor_msgs/CameraInfo depth_camera_info
vision_msgs/Detection2DArray detections
# Source epochs, original device timestamps, geometry contract, spatial objects.
string metadata_json
