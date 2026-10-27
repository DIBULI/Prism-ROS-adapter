#pragma once

#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace prism_ros_adapter {

// Foxy predates the public rclcpp acknowledgment API. Detect the actual API
// instead of inferring support from a distribution name or version number.
template <typename Publisher, typename = void>
struct HasPublicationAckWait : std::false_type {};

template <typename Publisher>
struct HasPublicationAckWait<Publisher, std::void_t<decltype(
    std::declval<Publisher&>().wait_for_all_acked(std::chrono::milliseconds(100)))>>
    : std::true_type {};

// Used only by a stream's publication worker, never by the SDK receive thread.
// Bounds the burst of unconfirmed DDS samples without changing timestamps or
// source cadence. An ACK is DDS reception, not proof of application/disk delivery.
class PublicationAckWindow {
 public:
  explicit PublicationAckWindow(std::size_t limit = 1) : limit_(limit) {
    if (!limit) throw std::invalid_argument("ACK window must be positive");
  }

  template <typename Wait>
  bool published(bool reliable, std::size_t subscribers, Wait&& wait) {
    if (!reliable || !subscribers) {
      pending_ = 0;
      return true;
    }
    if (pending_ < limit_) ++pending_;
    if (pending_ < limit_) return true;
    // On timeout retain the pending fence; the next publication checks again.
    // Callers report the failure, but must not skip other camera lanes.
    if (!wait(std::chrono::milliseconds(100))) return false;
    pending_ = 0;
    return true;
  }

 private:
  const std::size_t limit_;
  std::size_t pending_ = 0;
};

} // namespace prism_ros_adapter
