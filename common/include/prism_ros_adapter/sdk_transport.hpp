#pragma once

#include "prism/usb_sdk.hpp"
#ifdef PRISM_ROS_RKLOCAL
#include "prism/rklocal_sdk.hpp"
#endif

namespace prism_ros_adapter::sdk {

#ifdef PRISM_ROS_RKLOCAL
using Client = prism::rklocal::Client;
inline constexpr const char* kTransport = "rklocal";
#else
using Client = prism::Client;
inline constexpr const char* kTransport = "usb";
#endif

// RK-local assembles and ACKs video in its receive thread, even when raw
// events are enabled. A second ACK from the ROS assembler would be incorrect.
inline void acknowledgeVideo(Client& client, uint32_t frame_id) {
#ifdef PRISM_ROS_RKLOCAL
  (void)client;
  (void)frame_id;
#else
  client.sendVideoAck(frame_id);
#endif
}

#ifdef PRISM_ROS_RKLOCAL
// The public stream helpers accept only a USB Client. Keep protocol parsing
// in the SDK and adapt just their lifecycle for the RK-local Client.
class ImuStream {
 public:
  ImuStream(Client& client, prism::ImuSampleHandler handler)
      : client_(client), handler_(std::move(handler)) {}
  ~ImuStream() { try { stop(); } catch (...) {} }
  ImuStream(const ImuStream&) = delete;
  ImuStream& operator=(const ImuStream&) = delete;
  void start(uint32_t sensors, uint32_t rate) {
    if (active_) return;
    const auto status = client_.startImu(sensors, rate);
    // Track a started session even if validation fails, so cleanup stops it.
    active_ = true;
    if (status.sensors != sensors ||
        (rate != 0 && status.nominal_rate_hz != rate)) {
      throw std::runtime_error("agent acknowledged different IMU stream settings");
    }
  }
  void stop() {
    if (!active_) return;
    active_ = false;
    if (client_.isOpen()) client_.stopImu();
  }
  bool handleFrame(const prism::Frame& frame) {
    if (frame.type != prism::FrameType::ImuSample) return false;
    if (active_ && handler_) handler_(prism::parseImuSample(frame));
    return true;
  }
 private:
  Client& client_;
  prism::ImuSampleHandler handler_;
  bool active_ = false;
};

class LidarStream {
 public:
  LidarStream(Client& client, prism::LidarPointBatchHandler points,
              prism::LidarImuSampleHandler imu)
      : client_(client), points_(std::move(points)), imu_(std::move(imu)) {}
  ~LidarStream() { try { stop(); } catch (...) {} }
  LidarStream(const LidarStream&) = delete;
  LidarStream& operator=(const LidarStream&) = delete;
  void start(prism::LidarModel model) {
    if (active_) return;
    const auto status = client_.startLidar(model);
    active_ = status.enabled;
    if (!status.enabled || status.model != model) {
      throw std::runtime_error("agent acknowledged different LiDAR settings");
    }
  }
  void stop() {
    if (!active_) return;
    active_ = false;
    if (client_.isOpen()) client_.stopLidar();
  }
  bool handleFrame(const prism::Frame& frame) {
    if (frame.type == prism::FrameType::LidarPoints) {
      if (active_ && points_) points_(prism::parseLidarPointBatch(frame));
      return true;
    }
    if (frame.type == prism::FrameType::LidarImuSample) {
      if (active_ && imu_) imu_(prism::parseLidarImuSample(frame));
      return true;
    }
    return false;
  }
 private:
  Client& client_;
  prism::LidarPointBatchHandler points_;
  prism::LidarImuSampleHandler imu_;
  bool active_ = false;
};
#else
using ImuStream = prism::ImuStream;
using LidarStream = prism::LidarStream;
#endif

}  // namespace prism_ros_adapter::sdk
