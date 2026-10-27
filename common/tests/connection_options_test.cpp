#include "prism_ros_adapter/connection_options.hpp"
#include <functional>
#include <iostream>

using namespace prism_ros_adapter;
static void check(bool ok) { if (!ok) throw std::runtime_error("connection test failed"); }
static void rejects(const std::function<void()>& fn) {
  try { fn(); } catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("invalid configuration accepted");
}
struct FakeClient {
  static inline unsigned calls = 0;
  static inline bool fail = false;
  static FakeClient openNetwork(const prism::NetworkOptions& o) {
    ++calls;
    check(o.host == "127.0.0.1" && o.port == 7443 && o.connect_timeout_ms == 5000);
    check(o.token == "MiXeD password");
    if (fail) throw std::runtime_error("simulated rejected authentication");
    return {};
  }
};
int main() {
  DriverConfig c;
  validateConnectionConfig(c, false);
  validateConnectionConfig(c, true);
  check(resolvedTransport(c, false) == "usb");
  check(resolvedTransport(c, true) == "rklocal");
  c.transport = "network";
  rejects([&] { validateConnectionConfig(c, false); });
  c.network_host = "127.0.0.1";
  validateConnectionConfig(c, false);
  rejects([&] { validateConnectionConfig(c, true); });
  c.network_host = "::1";
  validateConnectionConfig(c, false);
  for (const std::string host : {"", "hostname", "https://192.168.1.5", "192.168.1.5:7443", "999.1.1.1"}) {
    c.network_host = host;
    rejects([&] { validateConnectionConfig(c, false); });
  }
  c.network_host = "127.0.0.1";
  for (auto port : {int64_t(-1), int64_t(0), int64_t(65536), INT64_MAX}) {
    c.network_port = port;
    rejects([&] { validateConnectionConfig(c, false); });
  }
  c.network_port = 7443;
  for (auto timeout : {int64_t(-1), int64_t(99), int64_t(60001), INT64_MAX}) {
    c.network_connect_timeout_ms = timeout;
    rejects([&] { validateConnectionConfig(c, false); });
  }
  c.network_connect_timeout_ms = 5000;
  for (const std::string env : {"", "0NAME", "a=b", "a b", "a\nb"}) {
    c.network_token_env = env;
    rejects([&] { validateConnectionConfig(c, false); });
  }
  c.network_token_env = "PRISM_ADAPTER_CONNECTION_TEST_TOKEN";
  unsetenv(c.network_token_env.c_str());
  rejects([&] { openNetworkClient<FakeClient>(c); });
  setenv(c.network_token_env.c_str(), "", 1);
  rejects([&] { openNetworkClient<FakeClient>(c); });
  setenv(c.network_token_env.c_str(), "secret\nline", 1);
  rejects([&] { openNetworkClient<FakeClient>(c); });
  check(FakeClient::calls == 0);
  setenv(c.network_token_env.c_str(), "MiXeD password", 1);
  openNetworkClient<FakeClient>(c);
  check(FakeClient::calls == 1);
  FakeClient::fail = true;
  try { openNetworkClient<FakeClient>(c); check(false); }
  catch (const std::runtime_error& e) {
    check(std::string(e.what()) == "simulated rejected authentication");
  }
  check(FakeClient::calls == 2); // no retry/fallback
  unsetenv(c.network_token_env.c_str());
  c.transport = "usb";
  rejects([&] { validateConnectionConfig(c, false); });
  c.network_host.clear();
  validateConnectionConfig(c, false);
  rejects([&] { openNetworkClient<FakeClient>(c); });
  c.transport = "rklocal";
  rejects([&] { validateConnectionConfig(c, false); });
  validateConnectionConfig(c, true);
  c.rklocal_socket = "relative.sock";
  rejects([&] { validateConnectionConfig(c, true); });
  c.transport = "typo";
  rejects([&] { validateConnectionConfig(c, false); });
  std::cout << "PASS connection selection, address/port/timeout validation, secret isolation, no fallback\n";
}
