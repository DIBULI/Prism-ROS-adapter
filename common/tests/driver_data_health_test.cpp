// Exercise the real receive handlers and warning callback without opening any
// USB/socket/SDK client. This test TU contains driver.cpp exactly once.
#if defined(__GNUC__) && !defined(__clang__)
// Including the implementation for private-handler tests makes GCC diagnose
// its existing anonymous DispatchQueue type; this TU does not export Impl.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#include "../src/driver.cpp"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <cstdio>
#include <iostream>

namespace prism_ros_adapter {
struct DriverDataHealthTest {
  static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
  }
  static bool has(const std::vector<std::string>& logs, const std::string& stream,
                  const std::string& issue) {
    for (const auto& log : logs)
      if (log.find("stream=" + stream + " ") != std::string::npos &&
          log.find(issue) != std::string::npos) return true;
    return false;
  }
  static std::string sentence(const std::string& body) {
    unsigned crc = 0;
    for (unsigned char c : body) crc ^= c;
    char suffix[5]; std::snprintf(suffix, sizeof(suffix), "*%02X", crc);
    return "$" + body + suffix;
  }
  static void run() {
    using S = SensorDataHealth::Stream;
    using I = SensorDataHealth::Issue;
    std::vector<std::string> logs;
    DriverCallbacks callbacks;
    callbacks.log = [&](LogLevel level, const std::string& text) {
      require(level == LogLevel::Warning, "wrong warning severity");
      logs.push_back(text);
    };
    DriverConfig config;
    config.require_synchronized_timestamps = false;
    Driver::Impl driver(config, callbacks);
    for (auto stream : {S::BoardImu0, S::BoardImu1}) driver.health.reset(stream, 0, 800);
    for (size_t i = 0; i < 4; ++i) driver.health.reset(static_cast<S>(i), 0, 10);

    prism::ImuSample imu;
    imu.timestamp_synced = true;
    imu.sample_id = 65535; imu.timestamp_us = 1000000;
    driver.dispatchBoardImu(imu);
    imu.sample_id = 0; imu.timestamp_us += 1254;
    driver.dispatchBoardImu(imu);
    driver.reportHealth(true);
    require(logs.empty(), "real board handler falsely reports 16-bit rollover");
    imu.sample_id = 3; imu.timestamp_us += 3 * 1254; imu.sample_gap = true;
    driver.dispatchBoardImu(imu);
    imu.sensor_id = 1; imu.sample_id = 10;
    driver.dispatchBoardImu(imu);
    imu.sample_id = 12; imu.timestamp_us += 2 * 1254;
    driver.dispatchBoardImu(imu);

    prism::capture::CameraFrameSet frame;
    frame.metadata.valid = true;
    frame.metadata.trigger_time_ns = 1000000000;
    frame.frame_id = 1;
    for (auto& jpeg : frame.jpeg) jpeg = {0xff, 0xd8, 0xff, 0xd9};
    driver.publishCameraFrame(frame);
    frame.frame_id = 3; frame.metadata.trigger_time_ns += 200000000;
    driver.publishCameraFrame(frame);
    prism::capture::CameraChunkResult partial;
    partial.discarded_diagnostics.emplace_back();
    partial.discarded_diagnostics.back().cameras[0].state = prism::capture::CameraTransferState::Complete;
    partial.discarded_diagnostics.back().cameras[2].state = prism::capture::CameraTransferState::Partial;
    driver.camera_mask = 5; // Only slots 0 and 2 exist; absent 1/3 must not warn.
    driver.checkCameraDiagnostics(partial);

    prism::LidarPointBatch batch;
    batch.timestamp_synced = true; batch.timestamp_utc_us = 2000000;
    batch.batch_id = 1; batch.points.emplace_back();
    driver.dispatchLidarPoints(batch);
    batch.batch_id = 3; batch.timestamp_utc_us += 10000;
    driver.dispatchLidarPoints(batch);
    prism::LidarImuSample lidar_imu;
    lidar_imu.timestamp_synced = true; lidar_imu.timestamp_utc_us = 2000000;
    lidar_imu.sample_id = 1;
    driver.dispatchLidarImu(lidar_imu);
    lidar_imu.sample_id = 4; lidar_imu.timestamp_utc_us += 10000;
    driver.dispatchLidarImu(lidar_imu);

    prism::RoverRtcmChunkView rover;
    rover.sequence = 1; driver.checkRoverRtcm(rover);
    rover.sequence = 3; rover.dropped_bytes = 128; rover.flags = 1;
    driver.checkRoverRtcm(rover);
    prism::GnssRawBatch raw;
    raw.session = 1;
    prism::GnssRawRecord record;
    record.channel = prism::GnssRawChannel::Receiver; record.sequence = 1;
    raw.records.push_back(record);
    record.channel = prism::GnssRawChannel::CorsRtcm; record.sequence = 2;
    raw.records.push_back(record);
    record.channel = prism::GnssRawChannel::Receiver; record.sequence = 3;
    raw.records.push_back(record);
    driver.checkRawBatch(raw);
    driver.reportHealth(true);
    require(!has(logs, "gnss+cors/raw_dispatch", "sequence_missing"), "interleaved raw channels produce false loss");
    record.channel = prism::GnssRawChannel::CorsRtcm; record.sequence = 5;
    record.flags = 5; record.lost_bytes = 64;
    raw.records = {record}; raw.gap = true;
    driver.checkRawBatch(raw);

    prism::GnssObservations observations;
    observations.session = 1; observations.cursor = 1; observations.device_monotonic_ms = 1000;
    observations.records = {{1, 1000, sentence("GNGGA,120000.1,3100.0000,N,12100.0000,E,5,12,0.8,20,M,10,M,,")}};
    driver.dispatchReceiverBatch(observations);
    observations.records.clear(); observations.device_monotonic_ms = 1100;
    driver.dispatchReceiverBatch(observations); // Poll same position, not duplicate sensor data.
    driver.reportHealth(true);
    require(!has(logs, "gnss/receiver+fix", "timestamp_duplicate"), "polling generated duplicate warning");
    observations.device_monotonic_ms = 4000;
    driver.dispatchReceiverBatch(observations); // Previously received GGA is now stale.
    observations.cursor = 3; observations.records = {{3, 4000, "$invalid*00"}};
    observations.gap = true;
    driver.dispatchReceiverBatch(observations);

    GnssReceptionStatusState reception;
    reception.uart_frame_error_count = 20; driver.checkReception(reception);
    reception.uart_frame_error_count = 22; reception.fifo_overflow_count = 3;
    reception.raw_data_seen = true; driver.checkReception(reception);
    TimeSyncRtkStatusState rtk;
    rtk.linked = true; rtk.device_status_fresh = true; rtk.uptime_ms = 100;
    driver.checkRtk(rtk);
    rtk.uptime_ms = 200; rtk.rtcm_errors = 2; rtk.upstream_drops = 17;
    driver.checkRtk(rtk);
    driver.reportHealth(true);

    require(has(logs, "imu0", "sequence_missing+=2"), "IMU0 gap not logged");
    require(has(logs, "imu1", "sequence_missing+=1"), "IMU1 gap not logged");
    require(has(logs, "imu0", "source_gap_flag"), "source flag not logged");
    for (unsigned i = 0; i < 4; ++i)
      require(has(logs, "camera" + std::to_string(i), "sequence_missing+=1"), "camera gap not logged");
    require(has(logs, "camera2", "incomplete_frame+=1"), "partial camera not logged");
    require(!has(logs, "camera1", "incomplete_frame"), "absent camera logged as incomplete");
    require(has(logs, "camera/metadata", "incomplete_frame+=1"), "metadata loss not logged");
    require(has(logs, "lidar/points", "sequence_missing+=1"), "point batch gap not logged");
    require(has(logs, "lidar/imu", "sequence_missing+=2"), "LiDAR IMU gap not logged");
    require(has(logs, "gnss/rover_rtcm", "lost_bytes+=128"), "rover counter not logged");
    require(has(logs, "rtk/cors_rtcm", "lost_bytes+=64"), "CORS record loss not logged");
    require(has(logs, "gnss+cors/raw_dispatch", "cache_gap"), "raw cache gap not logged");
    require(has(logs, "gnss/observations", "sequence_missing+=1"), "observation gap not logged");
    require(has(logs, "gnss/receiver+fix", "status_stale"), "position staleness not logged");
    require(has(logs, "gnss/reception", "uart_error+=2"), "reception counter not logged");
    require(has(logs, "rtk/status", "upstream_drop+=17") && !has(logs, "rtk/status", "uart_error"),
            "PA2 drops misclassified");
    const auto before_persistent_flag = logs.size();
    rover.sequence = 4; driver.checkRoverRtcm(rover); driver.reportHealth(true);
    require(logs.size() == before_persistent_flag, "persistent rover loss flag repeats old loss");
    rover.sequence = 5; rover.dropped_bytes = 192;
    driver.checkRoverRtcm(rover); driver.reportHealth(true);
    require(has(logs, "gnss/rover_rtcm", "lost_bytes+=64"), "new rover loss delta not counted");

    // Real bounded queue: block one callback, overflow, then release. No timing
    // races or hardware needed. The handler exception must reach the log too.
    std::promise<void> entered, release;
    auto released = release.get_future().share();
    std::atomic<bool> first{true};
    DispatchQueue<int> queue(1, [&](const int&) {
      if (first.exchange(false)) {
        entered.set_value(); released.wait(); throw std::runtime_error("synthetic publisher failure");
      }
    });
    queue.start(); queue.push(1); entered.get_future().wait();
    queue.push(2); queue.push(3);
    release.set_value(); queue.stop(); // joins in-flight callback
    driver.checkDispatch(S::Transport, queue);
    driver.reportHealth(true);
    require(has(logs, "transport", "dispatch_drop+=1") && has(logs, "transport", "publish_callback_error+=1"),
            "real dispatch queue loss/exception not logged");
    // Delayed queries can release a burst of complete camera sets. All eight
    // staged sets must drain; overflow reporting remains tested above.
    std::promise<void> burst_entered, burst_release, burst_done;
    auto burst_ready = burst_release.get_future().share();
    auto burst_finished = burst_done.get_future();
    std::atomic<unsigned> burst_count{0};
    DispatchQueue<int> burst(8, [&](const int&) {
      if (burst_count.load() == 0) { burst_entered.set_value(); burst_ready.wait(); }
      if (++burst_count == 9) burst_done.set_value();
    });
    burst.start(); burst.push(0); burst_entered.get_future().wait();
    for (int n = 1; n <= 8; ++n) burst.push(n);
    require(burst.depth() == 8 && burst.peak() == 8 && burst.dropped() == 0,
            "bounded burst queue accounting incorrect");
    burst_release.set_value();
    require(burst_finished.wait_for(std::chrono::seconds(2)) == std::future_status::ready,
            "camera burst did not drain");
    burst.stop();
    require(burst_count == 9 && burst.depth() == 0 && burst.dropped() == 0,
            "camera burst silently lost data");
    for (const auto& log : logs) std::cout << log << '\n';
    std::cout << "Driver receive-path fault injection passed; no device opened\n";
  }
};
} // namespace prism_ros_adapter

int main() {
  try { prism_ros_adapter::DriverDataHealthTest::run(); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
