#pragma once

#include "prism_ros_adapter/driver.hpp"
#include <prism/usb/network.hpp>

#include <arpa/inet.h>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace prism_ros_adapter {

inline std::string resolvedTransport(const DriverConfig& config,
                                     bool rklocal_build) {
  return config.transport == "auto" ? (rklocal_build ? "rklocal" : "usb")
                                     : config.transport;
}

inline void validateConnectionConfig(const DriverConfig& config,
                                     bool rklocal_build) {
  const auto transport = resolvedTransport(config, rklocal_build);
  if (rklocal_build) {
    if (transport != "rklocal")
      throw std::invalid_argument("RK-local build requires transport=auto or rklocal; use the host build for USB/network");
    if (!config.device_serial.empty())
      throw std::invalid_argument("device_serial is not supported by RK-local");
    if (config.rklocal_socket.empty() || config.rklocal_socket.front() != '/')
      throw std::invalid_argument("rklocal_socket must be an absolute socket path");
  } else if (transport != "usb" && transport != "network") {
    throw std::invalid_argument("transport must be auto, usb or network in a host build");
  }
  if (transport != "network") {
    if (!config.network_host.empty())
      throw std::invalid_argument("network_host requires transport=network (no implicit fallback)");
    return;
  }
  in_addr ipv4{};
  in6_addr ipv6{};
  if (config.network_host.find('\0') != std::string::npos ||
      (inet_pton(AF_INET, config.network_host.c_str(), &ipv4) != 1 &&
       inet_pton(AF_INET6, config.network_host.c_str(), &ipv6) != 1))
    throw std::invalid_argument("network_host must be a numeric IPv4 or IPv6 address, without a URL or port");
  if (config.network_port < 1 || config.network_port > 65535)
    throw std::invalid_argument("network_port must be 1..65535");
  if (config.network_connect_timeout_ms < 100 ||
      config.network_connect_timeout_ms > 60000)
    throw std::invalid_argument("network_connect_timeout_ms must be 100..60000");
  const auto& name = config.network_token_env;
  if (name.empty() || name.find_first_not_of(
          "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_") != std::string::npos ||
      (name.front() >= '0' && name.front() <= '9'))
    throw std::invalid_argument("network_token_env must name an environment variable, not contain the password");
}

// Shared by the real driver and factory tests. Credentials are read only for
// an explicit network connection and cleared on both success and exceptions.
template <typename Client>
Client openNetworkClient(const DriverConfig& config) {
  validateConnectionConfig(config, false);
  if (resolvedTransport(config, false) != "network")
    throw std::invalid_argument("explicit network transport required");
  struct Options : prism::NetworkOptions {
    ~Options() { std::fill(token.begin(), token.end(), '\0'); }
  } options;
  options.host = config.network_host;
  options.port = static_cast<uint16_t>(config.network_port);
  options.connect_timeout_ms = static_cast<uint32_t>(config.network_connect_timeout_ms);
  const char* token = std::getenv(config.network_token_env.c_str());
  if (!token || !*token)
    throw std::invalid_argument("network password environment variable is missing or empty");
  options.token = token;
  if (options.token.find_first_of("\r\n") != std::string::npos)
    throw std::invalid_argument("network password must not contain line breaks");
  return Client::openNetwork(options);
}

} // namespace prism_ros_adapter
