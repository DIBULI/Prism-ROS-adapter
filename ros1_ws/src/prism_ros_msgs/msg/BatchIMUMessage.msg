# One sensor per message, in acquisition order, without averaging or resampling.
# header.stamp equals timestamp_ns[0]; frame_id identifies the sensor.
# All seven arrays have equal nonzero length, at most imu_batch_size.
# timestamp_ns uses the legacy Imu header clock, in nanoseconds.
std_msgs/Header header
uint64[] timestamp_ns
float64[] acceleration_x
float64[] acceleration_y
float64[] acceleration_z
float64[] angular_velocity_x
float64[] angular_velocity_y
float64[] angular_velocity_z
