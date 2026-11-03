#pragma once

#include <prism/usb/profile_video.hpp>
#include <array>
#include <optional>
#include <string>

namespace prism_ros_adapter {

// One independent image, never a fabricated four-camera frame set. Source
// stamps are device-clock values, NOT attested UTC or host arrival times.
struct ProfileCameraImage {
  uint8_t camera = 0, format = 0, source_bits = 0, bayer = 0, timing_mode = 0;
  uint32_t transfer_id = 0, source_frame_id = 0, trigger_sequence = 0;
  uint32_t width = 0, height = 0, exposure_us = 0, gain_x1024 = 0;
  uint64_t source_timestamp_us = 0;
  uint64_t acquisition_epoch = 0; // Adapter start epoch; group IDs may restart.
  bool source_frame_valid = false, timestamp_utc_valid = false;
  bool image_published = false;
  std::vector<uint8_t> data;
};

struct ProfileCameraJob {
  prism::capture::ProfileImage image;
  uint64_t epoch = 0;
};

inline ProfileCameraImage decodeProfileCamera(const prism::capture::ProfileImage& image) {
  ProfileCameraImage out;
  out.camera = image.camera; out.format = image.format;
  out.transfer_id = image.frame_id; out.width = image.width; out.height = image.height;
  out.source_timestamp_us = image.timestamp_us;
  if (image.timestamp_us > 2147483647ULL * 1000000ULL + 999999ULL)
    throw std::runtime_error("profile timestamp exceeds common ROS1/ROS2 time range");
  if (image.camera >= 4) throw std::runtime_error("profile camera outside range");
  if (image.format == prism::capture::kRaw8Rice) {
    auto raw = prism::capture::decodeRawImage(image.encoded, image.format);
    if (raw.camera != image.camera || raw.width != image.width || raw.height != image.height ||
        raw.source_timestamp_us != image.timestamp_us)
      throw std::runtime_error("RAW8 envelope/header identity mismatch");
    out.source_bits = raw.source_bits; out.bayer = raw.bayer;
    out.timing_mode = raw.timing_mode; out.source_frame_id = raw.frame_id;
    out.source_frame_valid = true; out.trigger_sequence = raw.trigger_sequence;
    out.exposure_us = raw.exposure_us; out.gain_x1024 = raw.gain_x1024;
    out.data = std::move(raw.pixels);
  } else if (image.format == 2 && image.width == 1280 && image.height == 1024 &&
             image.encoded.size() >= 4 && image.encoded[0] == 0xff && image.encoded[1] == 0xd8 &&
             image.encoded[image.encoded.size()-2] == 0xff && image.encoded.back() == 0xd9) {
    // The independent JPEG envelope has no source-frame/group/UTC fields.
    out.data = image.encoded;
  } else throw std::runtime_error("unsupported profile image or JPEG framing");
  return out;
}

// SDK validates chunk geometry/size/order. Track retirement separately so an
// interrupted camera cannot hold credits, hide replacement or block healthy
// lanes. Expiry uses host monotonic time, never the sensor clock.
class ProfileCameraAssembler {
 public:
  struct Result {
    std::optional<prism::capture::ProfileImage> image;
    std::vector<uint32_t> retired;
    unsigned incomplete = 0;
    std::string error;
  };
  Result ingest(const prism::VideoChunkView& chunk, uint64_t now_ns) {
    Result out;
    if (chunk.camera_id >= 4) { out.error = "invalid profile camera ID"; return out; }
    auto& s = slots_[chunk.camera_id];
    const auto retire = [&](uint32_t id) {
      if (!s.retired || *s.retired != id) { out.retired.push_back(id); s.retired = id; }
    };
    if ((chunk.flags & 1u) && s.active) {
      retire(s.id); ++out.incomplete; s.assembler = {}; s.active = false;
    }
    if (!s.active && s.retired && *s.retired == chunk.frame_id) {
      out.error = "replayed retired profile transfer"; return out;
    }
    if (chunk.flags & 1u) { s.id = chunk.frame_id; s.active = true; s.started_ns = now_ns; }
    try { out.image = s.assembler.ingest(chunk); }
    catch (const std::exception& e) {
      out.error = e.what();
      if (s.active) { retire(s.id); ++out.incomplete; }
      if (!s.active || chunk.frame_id != s.id) retire(chunk.frame_id);
      s.active = false; s.assembler = {}; return out;
    }
    if (out.image) { retire(out.image->frame_id); s.active = false; }
    return out;
  }
  Result expire(uint8_t camera, uint64_t now_ns) {
    Result out; auto& s = slots_.at(camera);
    if (s.active && now_ns >= s.started_ns && now_ns-s.started_ns >= 5'000'000'000ULL) {
      out.retired.push_back(s.id); s.retired = s.id; s.active = false;
      s.assembler = {}; out.incomplete = 1;
    }
    return out;
  }
  void reset() { slots_ = {}; }
  Result finish(uint8_t camera) {
    Result out; auto& s = slots_.at(camera);
    if (s.active) {
      out.retired.push_back(s.id); out.incomplete = 1;
      out.error = "incomplete profile transfer at STOP: " + std::to_string(s.id);
      s.retired = s.id; s.active = false; s.assembler = {};
    }
    return out;
  }
 private:
  struct Slot {
    prism::capture::ProfileImageAssembler assembler;
    bool active = false;
    uint32_t id = 0;
    uint64_t started_ns = 0;
    std::optional<uint32_t> retired;
  };
  std::array<Slot,4> slots_{};
};
} // namespace prism_ros_adapter
