#include "prism_ros_adapter/driver.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <prism_ros_msgs/msg/camera_frame_metadata.hpp>
#include <prism_ros_msgs/srv/control_streams.hpp>
#include <prism_ros_msgs/srv/get_device_configuration.hpp>
#include <prism_ros_msgs/srv/get_device_info.hpp>
#include <prism_ros_msgs/srv/get_exposure.hpp>
#include <prism_ros_msgs/srv/get_lidar_network.hpp>
#include <prism_ros_msgs/srv/get_lidar_status.hpp>
#include <prism_ros_msgs/srv/get_stream_state.hpp>
#include <prism_ros_msgs/srv/get_wifi_hotspot.hpp>
#include <prism_ros_msgs/srv/probe_lidar_network.hpp>
#include <prism_ros_msgs/srv/set_camera_exposure.hpp>
#include <prism_ros_msgs/srv/set_device_configuration.hpp>
#include <prism_ros_msgs/srv/set_exposure_limits.hpp>
#include <prism_ros_msgs/srv/set_lidar_network.hpp>
#include <prism_ros_msgs/srv/set_target_brightness.hpp>
#include <prism_ros_msgs/srv/set_wifi_hotspot.hpp>
#include <prism_ros_msgs/srv/sync_system_time.hpp>
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

template <typename Response>
void fillExposure(Response& response,
                  const prism_ros_adapter::ExposureState& state) {
  response.automatic_camera_mask = state.automatic_camera_mask;
  response.target_brightness = state.target_brightness;
  std::copy(state.manual_exposure_time_us.begin(),
            state.manual_exposure_time_us.end(),
            response.manual_exposure_time_us.begin());
  std::copy(state.gain_x1024.begin(), state.gain_x1024.end(),
            response.gain_x1024.begin());
}

template <typename Response>
void fillExposureLimits(Response& response,
                        const prism_ros_adapter::ExposureLimitsState& state) {
  response.min_exposure_time_us = state.min_exposure_time_us;
  response.max_exposure_time_us = state.max_exposure_time_us;
  response.effective_max_exposure_time_us =
      state.effective_max_exposure_time_us;
  response.min_gain_x1024 = state.min_gain_x1024;
  response.max_gain_x1024 = state.max_gain_x1024;
}

template <typename Response>
void failService(Response& response, const std::exception& error) {
  response.success = false;
  response.message = error.what();
}

template <typename Message>
void fillDeviceInfo(Message& message,
                    const prism_ros_adapter::DeviceState& state) {
  message.host_sdk_version = state.host_sdk_version;
  message.agent_version = state.agent_version;
  message.sensor_board_version = state.sensor_board_version;
  message.combined_version = state.combined_version;
  message.agent_protocol_version = state.agent_protocol_version;
  message.product_serial = state.product_serial;
  message.usb_serial = state.usb_serial;
  message.vendor_id = state.vendor_id;
  message.product_id = state.product_id;
  message.info_version = state.info_version;
  message.usb_speed = state.usb_speed;
  message.usb3_connected = state.usb3_connected;
  message.sensor_board_online = state.sensor_board_online;
  message.sensor_board_time_synced = state.sensor_board_time_synced;
  message.detected_camera_count = state.detected_camera_count;
  message.detected_imu_count = state.detected_imu_count;
  message.camera_present_mask = state.camera_present_mask;
  message.camera_streaming_mask = state.camera_streaming_mask;
  message.imu_present_mask = state.imu_present_mask;
  message.imu_receiving_mask = state.imu_receiving_mask;
  message.imu_time_synced_mask = state.imu_time_synced_mask;
  message.imu_init_error_mask = state.imu_init_error_mask;
  std::copy(state.imu_init_error_reason.begin(),
            state.imu_init_error_reason.end(),
            message.imu_init_error_reason.begin());
  message.camera_fps = state.camera_fps;
  message.imu_fps = state.imu_fps;
  message.sensor_board_error_code = state.sensor_board_error_code;
  message.sensor_board_error_flags = state.sensor_board_error_flags;
  message.sensor_board_error = state.sensor_board_error;
}

template <typename Message>
void fillDeviceConfiguration(
    Message& message,
    const prism_ros_adapter::DeviceConfigurationState& state) {
  message.camera_fps = state.camera_fps;
  message.imu_rate_hz = state.imu_rate_hz;
  message.mjpeg_quality = state.mjpeg_quality;
  message.generation = state.generation;
  message.persisted = state.persisted;
}

template <typename Message>
void fillLidarStatus(Message& message,
                     const prism_ros_adapter::LidarStatusState& state) {
  message.available = state.available;
  message.enabled = state.enabled;
  message.connected = state.connected;
  message.receiving = state.receiving;
  message.model = state.model;
  message.device_type = state.device_type;
  message.handle = state.handle;
  message.packet_count = state.packet_count;
  message.point_count = state.point_count;
  message.dropped_point_count = state.dropped_point_count;
  message.serial = state.serial;
  message.lidar_ip = state.lidar_ip;
  message.error = state.error;
}

template <typename Message>
void fillLidarNetwork(Message& message,
                      const prism_ros_adapter::LidarNetworkState& state) {
  message.enabled = state.enabled;
  message.host_ip = state.host_ip;
  message.netmask = state.netmask;
  message.lidar_ip = state.lidar_ip;
  message.interface_present = state.interface_present;
  message.link_up = state.link_up;
  message.address_applied = state.address_applied;
  message.same_subnet = state.same_subnet;
  message.target_reachable = state.target_reachable;
  message.persisted = state.persisted;
  message.error_code = state.error_code;
  message.generation = state.generation;
  message.interface_name = state.interface_name;
  message.error = state.error;
}

template <typename Message>
void fillStreamState(Message& message,
                     const prism_ros_adapter::StreamState& state) {
  message.camera_enabled = state.camera_enabled;
  message.board_imu_enabled = state.board_imu_enabled;
  message.lidar_enabled = state.lidar_enabled;
  message.camera_active = state.camera_active;
  message.board_imu_active = state.board_imu_active;
  message.lidar_active = state.lidar_active;
  message.lidar_model = state.lidar_model;
}

template <typename Message>
void fillWifiHotspot(Message& message,
                     const prism_ros_adapter::WifiHotspotState& state) {
  message.present = state.present;
  message.enabled = state.enabled;
  message.running = state.running;
  message.access_point_running = state.access_point_running;
  message.dhcp_running = state.dhcp_running;
  message.persisted = state.persisted;
  message.error_code = state.error_code;
  message.interface_name = state.interface_name;
  message.ssid = state.ssid;
  message.address = state.address;
  message.error = state.error;
}

prism_ros_adapter::StreamCommand parseStreamCommand(
    const std::string& command) {
  if (command == "start") return prism_ros_adapter::StreamCommand::Start;
  if (command == "stop") return prism_ros_adapter::StreamCommand::Stop;
  if (command == "restart") return prism_ros_adapter::StreamCommand::Restart;
  throw std::invalid_argument("stream command must be start, stop, or restart");
}

class PrismRos2Node : public rclcpp::Node {
 public:
  PrismRos2Node() : rclcpp::Node("prism_ros_driver") {
    prism_ros_adapter::DriverConfig config;
    config.device_serial = declare_parameter<std::string>("device_serial", "");
    config.enable_camera = declare_parameter<bool>("camera_enabled", true);
    config.enable_board_imu =
        declare_parameter<bool>("board_imu_enabled", true);
    config.enable_lidar = declare_parameter<bool>("lidar_enabled", false);
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
    config.lidar_network_apply_on_start =
        declare_parameter<bool>("lidar_network_apply_on_start", false);
    config.lidar_network_enabled =
        declare_parameter<bool>("lidar_network_enabled", true);
    config.lidar_host_ip =
        declare_parameter<std::string>("lidar_host_ip", "192.168.1.5");
    config.lidar_netmask =
        declare_parameter<std::string>("lidar_netmask", "255.255.255.0");
    config.lidar_ip =
        declare_parameter<std::string>("lidar_ip", "192.168.1.3");

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
    createServices();
  }

  void run() { driver_->run([]() { return rclcpp::ok(); }); }

 private:
  void createServices() {
    using GetExposure = prism_ros_msgs::srv::GetExposure;
    using SetTargetBrightness = prism_ros_msgs::srv::SetTargetBrightness;
    using SetCameraExposure = prism_ros_msgs::srv::SetCameraExposure;
    using SetExposureLimits = prism_ros_msgs::srv::SetExposureLimits;
    using SyncSystemTime = prism_ros_msgs::srv::SyncSystemTime;
    using GetDeviceInfo = prism_ros_msgs::srv::GetDeviceInfo;
    using GetDeviceConfiguration =
        prism_ros_msgs::srv::GetDeviceConfiguration;
    using SetDeviceConfiguration =
        prism_ros_msgs::srv::SetDeviceConfiguration;
    using GetLidarStatus = prism_ros_msgs::srv::GetLidarStatus;
    using GetLidarNetwork = prism_ros_msgs::srv::GetLidarNetwork;
    using SetLidarNetwork = prism_ros_msgs::srv::SetLidarNetwork;
    using ProbeLidarNetwork = prism_ros_msgs::srv::ProbeLidarNetwork;
    using GetStreamState = prism_ros_msgs::srv::GetStreamState;
    using ControlStreams = prism_ros_msgs::srv::ControlStreams;
    using GetWifiHotspot = prism_ros_msgs::srv::GetWifiHotspot;
    using SetWifiHotspot = prism_ros_msgs::srv::SetWifiHotspot;
    get_exposure_service_ =
        create_service<GetExposure>(
            topic(topic_prefix_, "camera/get_exposure"),
            [this](const std::shared_ptr<GetExposure::Request>,
                   std::shared_ptr<GetExposure::Response> response) {
              try {
                fillExposure(*response, driver_->getExposure());
                fillExposureLimits(*response, driver_->getExposureLimits());
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_target_brightness_service_ =
        create_service<SetTargetBrightness>(
            topic(topic_prefix_, "camera/set_target_brightness"),
            [this](const std::shared_ptr<SetTargetBrightness::Request> request,
                   std::shared_ptr<SetTargetBrightness::Response> response) {
              try {
                fillExposure(
                    *response,
                    driver_->setTargetBrightness(request->target_brightness));
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_camera_exposure_service_ =
        create_service<SetCameraExposure>(
            topic(topic_prefix_, "camera/set_exposure"),
            [this](const std::shared_ptr<SetCameraExposure::Request> request,
                   std::shared_ptr<SetCameraExposure::Response> response) {
              try {
                const auto mode =
                    request->automatic
                        ? prism_ros_adapter::CameraExposureMode::Automatic
                        : prism_ros_adapter::CameraExposureMode::Manual;
                fillExposure(
                    *response,
                    driver_->setCameraExposure(
                        request->camera_index, mode, request->exposure_time_us,
                        request->gain_x1024));
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_exposure_limits_service_ =
        create_service<SetExposureLimits>(
            topic(topic_prefix_, "camera/set_exposure_limits"),
            [this](const std::shared_ptr<SetExposureLimits::Request> request,
                   std::shared_ptr<SetExposureLimits::Response> response) {
              try {
                fillExposureLimits(
                    *response,
                    driver_->setExposureLimits(
                        request->min_exposure_time_us,
                        request->max_exposure_time_us,
                        request->min_gain_x1024, request->max_gain_x1024));
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    sync_system_time_service_ =
        create_service<SyncSystemTime>(
            topic(topic_prefix_, "system/sync_time"),
            [this](const std::shared_ptr<SyncSystemTime::Request> request,
                   std::shared_ptr<SyncSystemTime::Response> response) {
              if (!request->confirm) {
                response->success = false;
                response->message =
                    "set confirm=true to pause streams and synchronize the "
                    "device to the host clock";
                return;
              }
              try {
                const auto state = driver_->synchronizeSystemTime();
                response->before_offset_us = state.before_offset_us;
                response->applied_correction_us =
                    state.applied_correction_us;
                response->after_offset_us = state.after_offset_us;
                response->round_trip_us = state.round_trip_us;
                response->jitter_us = state.jitter_us;
                response->correction_passes = state.correction_passes;
                response->system_time_set = state.system_time_set;
                response->ptp_hardware_clock_set =
                    state.ptp_hardware_clock_set;
                response->hardware_clock_set = state.hardware_clock_set;
                response->verified = state.verified;
                response->rtc_device = state.rtc_device;
                response->success = state.verified;
                response->message = state.verified
                                        ? "device synchronized to host clock"
                                        : "time synchronization was not verified";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_device_info_service_ =
        create_service<GetDeviceInfo>(
            topic(topic_prefix_, "device/get_info"),
            [this](const std::shared_ptr<GetDeviceInfo::Request>,
                   std::shared_ptr<GetDeviceInfo::Response> response) {
              try {
                fillDeviceInfo(response->info, driver_->getDeviceInfo());
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_device_configuration_service_ =
        create_service<GetDeviceConfiguration>(
            topic(topic_prefix_, "device/get_configuration"),
            [this](const std::shared_ptr<GetDeviceConfiguration::Request>,
                   std::shared_ptr<GetDeviceConfiguration::Response> response) {
              try {
                fillDeviceConfiguration(response->configuration,
                                        driver_->getDeviceConfiguration());
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_device_configuration_service_ =
        create_service<SetDeviceConfiguration>(
            topic(topic_prefix_, "device/set_configuration"),
            [this](
                const std::shared_ptr<SetDeviceConfiguration::Request> request,
                std::shared_ptr<SetDeviceConfiguration::Response> response) {
              if (!request->confirm) {
                response->success = false;
                response->message =
                    "set confirm=true to persist the selected device fields";
                return;
              }
              try {
                const auto state = driver_->setDeviceConfiguration(
                    request->set_camera_fps, request->camera_fps,
                    request->set_imu_rate_hz, request->imu_rate_hz,
                    request->set_mjpeg_quality, request->mjpeg_quality);
                fillDeviceConfiguration(response->configuration, state);
                response->success = state.persisted;
                response->message = state.persisted
                                        ? "device configuration persisted"
                                        : "device configuration was not persisted";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_lidar_status_service_ =
        create_service<GetLidarStatus>(
            topic(topic_prefix_, "lidar/get_status"),
            [this](const std::shared_ptr<GetLidarStatus::Request>,
                   std::shared_ptr<GetLidarStatus::Response> response) {
              try {
                fillLidarStatus(response->status, driver_->getLidarStatus());
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_lidar_network_service_ =
        create_service<GetLidarNetwork>(
            topic(topic_prefix_, "lidar/get_network"),
            [this](const std::shared_ptr<GetLidarNetwork::Request>,
                   std::shared_ptr<GetLidarNetwork::Response> response) {
              try {
                const auto state = driver_->getLidarNetwork();
                fillLidarNetwork(response->status, state);
                response->success = state.error_code == 0;
                response->message = state.error.empty() ? "ok" : state.error;
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_lidar_network_service_ =
        create_service<SetLidarNetwork>(
            topic(topic_prefix_, "lidar/set_network"),
            [this](const std::shared_ptr<SetLidarNetwork::Request> request,
                   std::shared_ptr<SetLidarNetwork::Response> response) {
              if (!request->confirm) {
                response->success = false;
                response->message =
                    "set confirm=true to persist the LiDAR network settings";
                return;
              }
              try {
                const auto state = driver_->setLidarNetwork(
                    request->enabled, request->host_ip, request->netmask,
                    request->lidar_ip);
                fillLidarNetwork(response->status, state);
                response->success = state.persisted && state.error_code == 0;
                response->message =
                    state.error.empty()
                        ? (state.persisted ? "LiDAR network persisted"
                                           : "LiDAR network was not persisted")
                        : state.error;
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    probe_lidar_network_service_ =
        create_service<ProbeLidarNetwork>(
            topic(topic_prefix_, "lidar/probe_network"),
            [this](const std::shared_ptr<ProbeLidarNetwork::Request>,
                   std::shared_ptr<ProbeLidarNetwork::Response> response) {
              try {
                const auto state = driver_->probeLidarNetwork();
                fillLidarNetwork(response->status, state);
                response->success =
                    state.error_code == 0 && state.target_reachable;
                response->message =
                    state.error.empty()
                        ? (state.target_reachable ? "LiDAR is reachable"
                                                  : "LiDAR is not reachable")
                        : state.error;
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_stream_state_service_ =
        create_service<GetStreamState>(
            topic(topic_prefix_, "streams/get_state"),
            [this](const std::shared_ptr<GetStreamState::Request>,
                   std::shared_ptr<GetStreamState::Response> response) {
              try {
                fillStreamState(response->state, driver_->getStreamState());
                response->success = true;
                response->message = "ok";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    control_streams_service_ =
        create_service<ControlStreams>(
            topic(topic_prefix_, "streams/control"),
            [this](const std::shared_ptr<ControlStreams::Request> request,
                   std::shared_ptr<ControlStreams::Response> response) {
              try {
                const auto command = parseStreamCommand(request->command);
                fillStreamState(
                    response->state,
                    driver_->controlStreams(command, request->camera,
                                            request->board_imu,
                                            request->lidar));
                response->success = true;
                response->message = "stream command completed";
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    get_wifi_hotspot_service_ =
        create_service<GetWifiHotspot>(
            topic(topic_prefix_, "wifi/get_hotspot"),
            [this](const std::shared_ptr<GetWifiHotspot::Request>,
                   std::shared_ptr<GetWifiHotspot::Response> response) {
              try {
                const auto state = driver_->getWifiHotspot();
                fillWifiHotspot(response->status, state);
                response->success = state.error_code == 0;
                response->message = state.error.empty() ? "ok" : state.error;
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
    set_wifi_hotspot_service_ =
        create_service<SetWifiHotspot>(
            topic(topic_prefix_, "wifi/set_hotspot"),
            [this](const std::shared_ptr<SetWifiHotspot::Request> request,
                   std::shared_ptr<SetWifiHotspot::Response> response) {
              if (!request->confirm) {
                response->success = false;
                response->message =
                    "set confirm=true to persist the Wi-Fi hotspot setting";
                return;
              }
              try {
                const auto state = driver_->setWifiHotspot(request->enabled);
                fillWifiHotspot(response->status, state);
                response->success = state.persisted && state.error_code == 0;
                response->message =
                    state.error.empty()
                        ? (state.persisted ? "Wi-Fi hotspot setting persisted"
                                           : "Wi-Fi setting was not persisted")
                        : state.error;
              } catch (const std::exception& error) {
                failService(*response, error);
              }
            });
  }

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
    message.point_step = 20;
    message.row_step = message.point_step * message.width;
    message.fields.resize(6);
    const std::array<std::string, 6> names{
        "x", "y", "z", "intensity", "tag", "offset_time"};
    const std::array<uint32_t, 6> offsets{0, 4, 8, 12, 13, 16};
    const std::array<uint8_t, 6> datatypes{
        sensor_msgs::msg::PointField::FLOAT32,
        sensor_msgs::msg::PointField::FLOAT32,
        sensor_msgs::msg::PointField::FLOAT32,
        sensor_msgs::msg::PointField::UINT8,
        sensor_msgs::msg::PointField::UINT8,
        sensor_msgs::msg::PointField::UINT32};
    for (size_t i = 0; i < message.fields.size(); ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = offsets[i];
      message.fields[i].count = 1;
      message.fields[i].datatype = datatypes[i];
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
      std::memcpy(output + 16, &point.offset_time_ns, sizeof(uint32_t));
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
  rclcpp::Service<prism_ros_msgs::srv::GetExposure>::SharedPtr
      get_exposure_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetTargetBrightness>::SharedPtr
      set_target_brightness_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetCameraExposure>::SharedPtr
      set_camera_exposure_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetExposureLimits>::SharedPtr
      set_exposure_limits_service_;
  rclcpp::Service<prism_ros_msgs::srv::SyncSystemTime>::SharedPtr
      sync_system_time_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetDeviceInfo>::SharedPtr
      get_device_info_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetDeviceConfiguration>::SharedPtr
      get_device_configuration_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetDeviceConfiguration>::SharedPtr
      set_device_configuration_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetLidarStatus>::SharedPtr
      get_lidar_status_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetLidarNetwork>::SharedPtr
      get_lidar_network_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetLidarNetwork>::SharedPtr
      set_lidar_network_service_;
  rclcpp::Service<prism_ros_msgs::srv::ProbeLidarNetwork>::SharedPtr
      probe_lidar_network_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetStreamState>::SharedPtr
      get_stream_state_service_;
  rclcpp::Service<prism_ros_msgs::srv::ControlStreams>::SharedPtr
      control_streams_service_;
  rclcpp::Service<prism_ros_msgs::srv::GetWifiHotspot>::SharedPtr
      get_wifi_hotspot_service_;
  rclcpp::Service<prism_ros_msgs::srv::SetWifiHotspot>::SharedPtr
      set_wifi_hotspot_service_;
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
