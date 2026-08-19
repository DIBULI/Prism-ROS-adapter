#include "prism_ros_adapter/driver.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <prism_ros_msgs/msg/camera_frame_metadata.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

std::string topic(const std::string& prefix, const std::string& suffix) {
  if (prefix.empty() || prefix == "/") return "/" + suffix;
  return prefix.back() == '/' ? prefix + suffix : prefix + "/" + suffix;
}

builtin_interfaces::msg::Time rosTime(uint64_t timestamp_ns) {
  builtin_interfaces::msg::Time output;
  output.sec = static_cast<int32_t>(timestamp_ns / 1000000000ull);
  output.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ull);
  return output;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key,
                                        const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

class PrismRos2Node : public rclcpp::Node {
 public:
  PrismRos2Node() : rclcpp::Node("prism_ros_driver") {
    prism_ros_adapter::DriverConfig config;
    config.device_serial = declare_parameter<std::string>("device_serial", "");
    config.enable_camera = declare_parameter<bool>("camera_enabled", true);
    config.enable_board_imu =
        declare_parameter<bool>("board_imu_enabled", true);
    config.enable_lidar = declare_parameter<bool>("lidar_enabled", true);
    config.camera_fps = static_cast<uint32_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("camera_fps", 0)));
    config.imu_sensor_count = static_cast<uint32_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("imu_sensor_count", 0)));
    config.imu_rate_hz = static_cast<uint32_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("imu_rate_hz", 0)));
    config.require_synchronized_timestamps =
        declare_parameter<bool>("require_synchronized_timestamps", true);
    const std::string lidar_model =
        declare_parameter<std::string>("lidar_model", "mid360");
    if (lidar_model == "mid360") {
      config.lidar_model = prism_ros_adapter::LidarModel::Mid360;
    } else if (lidar_model == "mid360s") {
      config.lidar_model = prism_ros_adapter::LidarModel::Mid360S;
    } else {
      throw std::invalid_argument("lidar_model must be mid360 or mid360s");
    }

    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "/prism");
    camera_frame_prefix_ =
        declare_parameter<std::string>("camera_frame_prefix", "prism_camera_");
    board_imu_frame_prefix_ = declare_parameter<std::string>(
        "board_imu_frame_prefix", "prism_imu_");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "prism_lidar");
    lidar_imu_frame_ =
        declare_parameter<std::string>("lidar_imu_frame", "prism_lidar_imu");

    // MJPEG frames can exceed one megabyte in noisy scenes. Reliable delivery
    // avoids losing the complete DDS sample when one UDP fragment is dropped.
    auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(2))
                          .reliable()
                          .durability_volatile();
    for (size_t i = 0; i < camera_publishers_.size(); ++i) {
      camera_publishers_[i] =
          create_publisher<sensor_msgs::msg::CompressedImage>(
              topic(topic_prefix_, "camera" + std::to_string(i) +
                                       "/image/compressed"),
              camera_qos);
    }
    camera_metadata_publisher_ =
        create_publisher<prism_ros_msgs::msg::CameraFrameMetadata>(
            topic(topic_prefix_, "camera/metadata"),
            rclcpp::SensorDataQoS().keep_last(8));
    for (size_t i = 0; i < board_imu_publishers_.size(); ++i) {
      board_imu_publishers_[i] = create_publisher<sensor_msgs::msg::Imu>(
          topic(topic_prefix_, "imu" + std::to_string(i) + "/data"),
          rclcpp::SensorDataQoS().keep_last(512));
    }
    lidar_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        topic(topic_prefix_, "lidar/points"),
        rclcpp::SensorDataQoS().keep_last(4));
    lidar_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        topic(topic_prefix_, "lidar/imu"),
        rclcpp::SensorDataQoS().keep_last(256));
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(4).reliable());

    prism_ros_adapter::DriverCallbacks callbacks;
    callbacks.camera = [this](const auto& value) { publishCamera(value); };
    callbacks.board_imu = [this](const auto& value) { publishBoardImu(value); };
    callbacks.lidar_points =
        [this](const auto& value) { publishLidar(value); };
    callbacks.lidar_imu =
        [this](const auto& value) { publishLidarImu(value); };
    callbacks.status = [this](const auto& value) { publishStatus(value); };
    callbacks.log = [this](prism_ros_adapter::LogLevel level,
                           const std::string& text) {
      if (level == prism_ros_adapter::LogLevel::Error) {
        RCLCPP_ERROR(get_logger(), "%s", text.c_str());
      } else if (level == prism_ros_adapter::LogLevel::Warning) {
        RCLCPP_WARN(get_logger(), "%s", text.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", text.c_str());
      }
    };
    driver_ = std::make_unique<prism_ros_adapter::Driver>(
        std::move(config), std::move(callbacks));
  }

  void run() { driver_->run([]() { return rclcpp::ok(); }); }

 private:
  void publishCamera(const prism_ros_adapter::CameraFrameSet& frame) {
    const auto stamp = rosTime(frame.timestamp_ns);
    for (size_t i = 0; i < camera_publishers_.size(); ++i) {
      sensor_msgs::msg::CompressedImage message;
      message.header.stamp = stamp;
      message.header.frame_id = camera_frame_prefix_ + std::to_string(i);
      message.format = "jpeg";
      message.data = frame.jpeg[i];
      camera_publishers_[i]->publish(message);
    }

    prism_ros_msgs::msg::CameraFrameMetadata metadata;
    metadata.header.stamp = stamp;
    metadata.header.frame_id = "prism_camera_trigger";
    metadata.host_frame_id = frame.host_frame_id;
    metadata.carrier_frame_id = frame.carrier_frame_id;
    metadata.exposure_us = frame.exposure_us;
    metadata.analog_gain_x1024 = frame.analog_gain_x1024;
    metadata.digital_gain_x1024 = frame.digital_gain_x1024;
    camera_metadata_publisher_->publish(metadata);
  }

  void publishBoardImu(const prism_ros_adapter::BoardImuSample& sample) {
    if (sample.sensor_id >= board_imu_publishers_.size()) return;
    sensor_msgs::msg::Imu message;
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
    board_imu_publishers_[sample.sensor_id]->publish(message);
  }

  void publishLidar(const prism_ros_adapter::LidarPointBatch& batch) {
    sensor_msgs::msg::PointCloud2 message;
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
          i < 3 ? sensor_msgs::msg::PointField::FLOAT32
                : sensor_msgs::msg::PointField::UINT8;
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
    lidar_publisher_->publish(message);
  }

  void publishLidarImu(const prism_ros_adapter::LidarImuSample& sample) {
    sensor_msgs::msg::Imu message;
    message.header.stamp = rosTime(sample.timestamp_ns);
    message.header.frame_id = lidar_imu_frame_;
    message.orientation_covariance[0] = -1.0;
    message.linear_acceleration.x = sample.acceleration_m_s2[0];
    message.linear_acceleration.y = sample.acceleration_m_s2[1];
    message.linear_acceleration.z = sample.acceleration_m_s2[2];
    message.angular_velocity.x = sample.angular_velocity_rad_s[0];
    message.angular_velocity.y = sample.angular_velocity_rad_s[1];
    message.angular_velocity.z = sample.angular_velocity_rad_s[2];
    lidar_imu_publisher_->publish(message);
  }

  void publishStatus(const prism_ros_adapter::DriverStatus& status) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "Prism USB adapter";
    diagnostic.hardware_id = status.product_serial;
    diagnostic.level = status.error.empty()
                           ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
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
    diagnostics_publisher_->publish(array);
  }

  std::string topic_prefix_;
  std::string camera_frame_prefix_;
  std::string board_imu_frame_prefix_;
  std::string lidar_frame_;
  std::string lidar_imu_frame_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr, 4>
      camera_publishers_;
  rclcpp::Publisher<prism_ros_msgs::msg::CameraFrameMetadata>::SharedPtr
      camera_metadata_publisher_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr, 2>
      board_imu_publishers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr lidar_imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  std::unique_ptr<prism_ros_adapter::Driver> driver_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PrismRos2Node>();
    std::thread spinner([node]() { rclcpp::spin(node); });
    try {
      node->run();
    } catch (...) {
      rclcpp::shutdown();
      if (spinner.joinable()) spinner.join();
      throw;
    }
    rclcpp::shutdown();
    if (spinner.joinable()) spinner.join();
    return 0;
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("prism_ros_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
