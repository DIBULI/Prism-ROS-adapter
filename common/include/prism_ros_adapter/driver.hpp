#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace prism_ros_adapter {

enum class LidarModel {
  Mid360,
  Mid360S,
};

enum class LogLevel {
  Info,
  Warning,
  Error,
};

struct DriverConfig {
  std::string device_serial;
  bool enable_camera = true;
  bool enable_board_imu = true;
  bool enable_lidar = true;
  uint32_t camera_fps = 0;
  uint32_t imu_sensor_count = 0;
  uint32_t imu_rate_hz = 0;
  LidarModel lidar_model = LidarModel::Mid360;
  bool require_synchronized_timestamps = true;
};

enum class CameraExposureMode {
  Automatic,
  Manual,
};

struct ExposureState {
  uint8_t automatic_camera_mask = 0;
  uint8_t target_brightness = 0;
  std::array<uint32_t, 4> manual_exposure_time_us{};
  std::array<uint32_t, 4> gain_x1024{};
};

struct ExposureLimitsState {
  uint32_t min_exposure_time_us = 0;
  uint32_t max_exposure_time_us = 0;
  uint32_t effective_max_exposure_time_us = 0;
  uint32_t min_gain_x1024 = 0;
  uint32_t max_gain_x1024 = 0;
};

struct SystemTimeSyncState {
  int64_t before_offset_us = 0;
  int64_t applied_correction_us = 0;
  int64_t after_offset_us = 0;
  int64_t round_trip_us = 0;
  int64_t jitter_us = 0;
  uint32_t correction_passes = 0;
  bool system_time_set = false;
  bool ptp_hardware_clock_set = false;
  bool hardware_clock_set = false;
  bool verified = false;
  std::string rtc_device;
};

struct CameraFrameSet {
  uint64_t timestamp_ns = 0;
  uint32_t host_frame_id = 0;
  uint32_t carrier_frame_id = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  std::array<std::vector<uint8_t>, 4> jpeg;
  std::array<uint32_t, 4> exposure_us{};
  std::array<uint32_t, 4> analog_gain_x1024{};
  std::array<uint32_t, 4> digital_gain_x1024{};
};

struct BoardImuSample {
  uint8_t sensor_id = 0;
  uint32_t sample_id = 0;
  uint64_t timestamp_ns = 0;
  bool fsync_event = false;
  bool sample_gap = false;
  std::array<double, 3> acceleration_m_s2{};
  std::array<double, 3> angular_velocity_rad_s{};
  double temperature_c = 0.0;
};

struct LidarPoint {
  float x_m = 0.0F;
  float y_m = 0.0F;
  float z_m = 0.0F;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
};

struct LidarPointBatch {
  uint32_t batch_id = 0;
  uint64_t timestamp_ns = 0;
  uint64_t timestamp_raw = 0;
  uint16_t time_interval_100ns = 0;
  std::vector<LidarPoint> points;
};

struct LidarImuSample {
  uint32_t sample_id = 0;
  uint64_t timestamp_ns = 0;
  uint64_t timestamp_raw_ns = 0;
  std::array<double, 3> acceleration_m_s2{};
  std::array<double, 3> angular_velocity_rad_s{};
};

struct DriverStatus {
  bool running = false;
  bool usb3_connected = false;
  bool sensor_board_online = false;
  bool sensor_board_time_synced = false;
  std::string product_serial;
  std::string state;
  std::string error;
  uint64_t camera_frame_sets = 0;
  std::array<uint64_t, 2> board_imu_samples{};
  uint64_t lidar_batches = 0;
  uint64_t lidar_points = 0;
  uint64_t lidar_imu_samples = 0;
  uint64_t lidar_raw_timestamp_selected = 0;
  uint64_t lidar_sdk_timestamp_selected = 0;
  uint64_t dropped_unsynchronized = 0;
  uint64_t dropped_camera_dispatch = 0;
  uint64_t dropped_lidar_dispatch = 0;
  uint64_t dropped_imu_dispatch = 0;
};

struct DriverCallbacks {
  std::function<void(const CameraFrameSet&)> camera;
  std::function<void(const BoardImuSample&)> board_imu;
  std::function<void(const LidarPointBatch&)> lidar_points;
  std::function<void(const LidarImuSample&)> lidar_imu;
  std::function<void(const DriverStatus&)> status;
  std::function<void(LogLevel, const std::string&)> log;
};

class Driver {
 public:
  Driver(DriverConfig config, DriverCallbacks callbacks);
  ~Driver();

  Driver(const Driver&) = delete;
  Driver& operator=(const Driver&) = delete;

  void run(const std::function<bool()>& keep_running);
  void requestStop() noexcept;

  ExposureState getExposure();
  ExposureState setTargetBrightness(uint8_t target_brightness);
  ExposureState setCameraExposure(uint8_t camera_index,
                                  CameraExposureMode mode,
                                  uint32_t exposure_time_us,
                                  uint32_t gain_x1024);
  ExposureLimitsState getExposureLimits();
  ExposureLimitsState setExposureLimits(uint32_t min_exposure_time_us,
                                        uint32_t max_exposure_time_us,
                                        uint32_t min_gain_x1024,
                                        uint32_t max_gain_x1024);
  SystemTimeSyncState synchronizeSystemTime();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace prism_ros_adapter
