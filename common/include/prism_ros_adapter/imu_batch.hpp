#pragma once

namespace prism_ros_adapter {

// ROS1 and ROS2 share the same wire fields. Preserve every original timestamp
// and double-precision SI value, including discontinuities (reported by health).
template <typename Message, typename Sample>
void appendImuSample(Message& message, const Sample& sample) {
  message.timestamp_ns.push_back(sample.timestamp_ns);
  message.acceleration_x.push_back(sample.acceleration_m_s2[0]);
  message.acceleration_y.push_back(sample.acceleration_m_s2[1]);
  message.acceleration_z.push_back(sample.acceleration_m_s2[2]);
  message.angular_velocity_x.push_back(sample.angular_velocity_rad_s[0]);
  message.angular_velocity_y.push_back(sample.angular_velocity_rad_s[1]);
  message.angular_velocity_z.push_back(sample.angular_velocity_rad_s[2]);
}

}  // namespace prism_ros_adapter
