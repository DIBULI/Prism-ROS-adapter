#include "prism_ros_adapter/driver.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <prism_ros_msgs/CameraFrameMetadata.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

std::string topic(const std::string& prefix, const std::string& suffix) {
  if (prefix.empty() || prefix == "/") return "/" + suffix;
  return prefix.back() == '/' ? prefix + suffix : prefix + "/" + suffix;
}

ros::Time rosTime(uint64_t timestamp_ns) {
  ros::Time output;
  output.fromNSec(timestamp_ns);
  return output;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

class PrismRos1Node {
 public:
  PrismRos1Node() : private_node_("~") {
    prism_ros_adapter::DriverConfig config;
    private_node_.param("device_serial", config.device_serial, std::string{});
    private_node_.param("camera_enabled", config.enable_camera, true);
    private_node_.param("board_imu_enabled", config.enable_board_imu, true);
    private_node_.param("lidar_enabled", config.enable_lidar, true);
    int camera_fps = 0;
    int imu_sensor_count = 0;
    int imu_rate_hz = 0;
    private_node_.param("camera_fps", camera_fps, 0);
    private_node_.param("imu_sensor_count", imu_sensor_count, 0);
    private_node_.param("imu_rate_hz", imu_rate_hz, 0);
    config.camera_fps = static_cast<uint32_t>(std::max(0, camera_fps));
    config.imu_sensor_count =
        static_cast<uint32_t>(std::max(0, imu_sensor_count));
    config.imu_rate_hz = static_cast<uint32_t>(std::max(0, imu_rate_hz));
    private_node_.param("require_synchronized_timestamps",
                        config.require_synchronized_timestamps, true);
    std::string lidar_model = "mid360";
    private_node_.param("lidar_model", lidar_model, lidar_model);
    if (lidar_model == "mid360") {
      config.lidar_model = prism_ros_adapter::LidarModel::Mid360;
    } else if (lidar_model == "mid360s") {
      config.lidar_model = prism_ros_adapter::LidarModel::Mid360S;
    } else {
      throw std::invalid_argument("~lidar_model must be mid360 or mid360s");
    }

    private_node_.param("topic_prefix", topic_prefix_, std::string("/prism"));
    private_node_.param("camera_frame_prefix", camera_frame_prefix_,
                        std::string("prism_camera_"));
    private_node_.param("board_imu_frame_prefix", board_imu_frame_prefix_,
                        std::string("prism_imu_"));
    private_node_.param("lidar_frame", lidar_frame_,
                        std::string("prism_lidar"));
    private_node_.param("lidar_imu_frame", lidar_imu_frame_,
                        std::string("prism_lidar_imu"));

    for (size_t i = 0; i < camera_publishers_.size(); ++i) {
      camera_publishers_[i] = node_.advertise<sensor_msgs::CompressedImage>(
          topic(topic_prefix_, "camera" + std::to_string(i) +
                                   "/image/compressed"),
          2);
    }
    camera_metadata_publisher_ =
        node_.advertise<prism_ros_msgs::CameraFrameMetadata>(
            topic(topic_prefix_, "camera/metadata"), 8);
    for (size_t i = 0; i < board_imu_publishers_.size(); ++i) {
      board_imu_publishers_[i] = node_.advertise<sensor_msgs::Imu>(
          topic(topic_prefix_, "imu" + std::to_string(i) + "/data"), 512);
    }
    lidar_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        topic(topic_prefix_, "lidar/points"), 4);
    lidar_imu_publisher_ = node_.advertise<sensor_msgs::Imu>(
        topic(topic_prefix_, "lidar/imu"), 256);
    diagnostics_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/diagnostics", 4);

    prism_ros_adapter::DriverCallbacks callbacks;
    callbacks.camera = [this](const auto& value) { publishCamera(value); };
    callbacks.board_imu = [this](const auto& value) { publishBoardImu(value); };
    callbacks.lidar_points =
        [this](const auto& value) { publishLidar(value); };
    callbacks.lidar_imu =
        [this](const auto& value) { publishLidarImu(value); };
    callbacks.status = [this](const auto& value) { publishStatus(value); };
    callbacks.log = [](prism_ros_adapter::LogLevel level,
                       const std::string& text) {
      if (level == prism_ros_adapter::LogLevel::Error) {
        ROS_ERROR_STREAM(text);
      } else if (level == prism_ros_adapter::LogLevel::Warning) {
        ROS_WARN_STREAM(text);
      } else {
        ROS_INFO_STREAM(text);
      }
    };
    driver_ = std::make_unique<prism_ros_adapter::Driver>(
        std::move(config), std::move(callbacks));
  }

  void run() { driver_->run([]() { return ros::ok(); }); }

 private:
  void publishCamera(const prism_ros_adapter::CameraFrameSet& frame) {
    const ros::Time stamp = rosTime(frame.timestamp_ns);
    for (size_t i = 0; i < camera_publishers_.size(); ++i) {
      sensor_msgs::CompressedImage message;
      message.header.stamp = stamp;
      message.header.frame_id = camera_frame_prefix_ + std::to_string(i);
      message.format = "jpeg";
      message.data = frame.jpeg[i];
      camera_publishers_[i].publish(message);
    }

    prism_ros_msgs::CameraFrameMetadata metadata;
    metadata.header.stamp = stamp;
    metadata.header.frame_id = "prism_camera_trigger";
    metadata.host_frame_id = frame.host_frame_id;
    metadata.carrier_frame_id = frame.carrier_frame_id;
    std::copy(frame.exposure_us.begin(), frame.exposure_us.end(),
              metadata.exposure_us.begin());
    std::copy(frame.analog_gain_x1024.begin(), frame.analog_gain_x1024.end(),
              metadata.analog_gain_x1024.begin());
    std::copy(frame.digital_gain_x1024.begin(), frame.digital_gain_x1024.end(),
              metadata.digital_gain_x1024.begin());
    camera_metadata_publisher_.publish(metadata);
  }

  void publishBoardImu(const prism_ros_adapter::BoardImuSample& sample) {
    if (sample.sensor_id >= board_imu_publishers_.size()) return;
    sensor_msgs::Imu message;
    message.header.stamp = rosTime(sample.timestamp_ns);
    message.header.frame_id =
        board_imu_frame_prefix_ + std::to_string(sample.sensor_id);
    message.orientation_covariance[0] = -1.0;
    message.linear_acceleration.x = sample.acceleration_m_s2[0];
    message.linear_acceleration.y = sample.acceleration_m_s2[1];
    message.linear_acceleration.z = sample.acceleration_m_s2[2];
    message.angular_velocity.x = sample.angular_velocity_rad_s[0];
    message.angular_velocity.y = sample.angular_velocity_rad_s[1];
    message.angular_velocity.z = sample.angular_velocity_rad_s[2];
    board_imu_publishers_[sample.sensor_id].publish(message);
  }

  void publishLidar(const prism_ros_adapter::LidarPointBatch& batch) {
    sensor_msgs::PointCloud2 message;
    message.header.stamp = rosTime(batch.timestamp_ns);
    message.header.frame_id = lidar_frame_;
    message.height = 1;
    message.width = static_cast<uint32_t>(batch.points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 16;
    message.row_step = message.point_step * message.width;
    message.fields.resize(5);
    const std::array<std::string, 5> names{"x", "y", "z", "intensity", "tag"};
    const std::array<uint32_t, 5> offsets{0, 4, 8, 12, 13};
    for (size_t i = 0; i < message.fields.size(); ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = offsets[i];
      message.fields[i].count = 1;
      message.fields[i].datatype =
          i < 3 ? sensor_msgs::PointField::FLOAT32
                : sensor_msgs::PointField::UINT8;
    }
    message.data.assign(message.row_step, 0u);
    for (size_t i = 0; i < batch.points.size(); ++i) {
      const auto& point = batch.points[i];
      uint8_t* output = message.data.data() + i * message.point_step;
      std::memcpy(output + 0, &point.x_m, sizeof(float));
      std::memcpy(output + 4, &point.y_m, sizeof(float));
      std::memcpy(output + 8, &point.z_m, sizeof(float));
      output[12] = point.reflectivity;
      output[13] = point.tag;
    }
    lidar_publisher_.publish(message);
  }

  void publishLidarImu(const prism_ros_adapter::LidarImuSample& sample) {
    sensor_msgs::Imu message;
    message.header.stamp = rosTime(sample.timestamp_ns);
    message.header.frame_id = lidar_imu_frame_;
    message.orientation_covariance[0] = -1.0;
    message.linear_acceleration.x = sample.acceleration_m_s2[0];
    message.linear_acceleration.y = sample.acceleration_m_s2[1];
    message.linear_acceleration.z = sample.acceleration_m_s2[2];
    message.angular_velocity.x = sample.angular_velocity_rad_s[0];
    message.angular_velocity.y = sample.angular_velocity_rad_s[1];
    message.angular_velocity.z = sample.angular_velocity_rad_s[2];
    lidar_imu_publisher_.publish(message);
  }

  void publishStatus(const prism_ros_adapter::DriverStatus& status) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus diagnostic;
    diagnostic.name = "Prism USB adapter";
    diagnostic.hardware_id = status.product_serial;
    diagnostic.level = status.error.empty()
                           ? diagnostic_msgs::DiagnosticStatus::OK
                           : diagnostic_msgs::DiagnosticStatus::ERROR;
    diagnostic.message = status.error.empty() ? status.state : status.error;
    diagnostic.values.push_back(
        keyValue("usb3_connected", status.usb3_connected ? "true" : "false"));
    diagnostic.values.push_back(keyValue(
        "sensor_board_online", status.sensor_board_online ? "true" : "false"));
    diagnostic.values.push_back(
        keyValue("sensor_board_time_synced",
                 status.sensor_board_time_synced ? "true" : "false"));
    diagnostic.values.push_back(
        keyValue("camera_frame_sets", std::to_string(status.camera_frame_sets)));
    diagnostic.values.push_back(
        keyValue("imu0_samples", std::to_string(status.board_imu_samples[0])));
    diagnostic.values.push_back(
        keyValue("imu1_samples", std::to_string(status.board_imu_samples[1])));
    diagnostic.values.push_back(
        keyValue("lidar_points", std::to_string(status.lidar_points)));
    diagnostic.values.push_back(keyValue(
        "lidar_raw_timestamp_selected",
        std::to_string(status.lidar_raw_timestamp_selected)));
    diagnostic.values.push_back(keyValue(
        "lidar_sdk_timestamp_selected",
        std::to_string(status.lidar_sdk_timestamp_selected)));
    diagnostic.values.push_back(keyValue(
        "dropped_unsynchronized",
        std::to_string(status.dropped_unsynchronized)));
    diagnostic.values.push_back(keyValue(
        "dropped_dispatch",
        std::to_string(status.dropped_camera_dispatch +
                       status.dropped_lidar_dispatch +
                       status.dropped_imu_dispatch)));
    array.status.push_back(std::move(diagnostic));
    diagnostics_publisher_.publish(array);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string topic_prefix_;
  std::string camera_frame_prefix_;
  std::string board_imu_frame_prefix_;
  std::string lidar_frame_;
  std::string lidar_imu_frame_;
  std::array<ros::Publisher, 4> camera_publishers_;
  ros::Publisher camera_metadata_publisher_;
  std::array<ros::Publisher, 2> board_imu_publishers_;
  ros::Publisher lidar_publisher_;
  ros::Publisher lidar_imu_publisher_;
  ros::Publisher diagnostics_publisher_;
  std::unique_ptr<prism_ros_adapter::Driver> driver_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "prism_ros_driver");
  try {
    PrismRos1Node node;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    node.run();
    return 0;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
}
