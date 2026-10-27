#ifdef NDEBUG
#undef NDEBUG
#endif
#include "prism_ros_adapter/publication_ack.hpp"
#include <cassert>
#include <iostream>

struct LegacyPublisher {};
struct AckPublisher { bool wait_for_all_acked(std::chrono::milliseconds) const; };
static_assert(!prism_ros_adapter::HasPublicationAckWait<LegacyPublisher>::value);
static_assert(prism_ros_adapter::HasPublicationAckWait<AckPublisher>::value);

int main() {
  using prism_ros_adapter::PublicationAckWindow;
  bool invalid=false;
  try { PublicationAckWindow zero(0); } catch(const std::invalid_argument&) {invalid=true;}
  assert(invalid);
  unsigned calls=0;
  auto ack=[&](auto timeout){assert(timeout==std::chrono::milliseconds(100));++calls;return true;};
  PublicationAckWindow imu(64);
  for(unsigned n=1;n<=192;++n){assert(imu.published(true,1,ack));assert(calls==n/64);}
  PublicationAckWindow camera(1);
  for(unsigned n=0;n<10;++n)assert(camera.published(true,1,ack));
  assert(calls==13);
  auto before=calls;
  for(unsigned n=0;n<200;++n){assert(camera.published(false,1,ack));assert(imu.published(true,0,ack));}
  assert(calls==before); // Best-effort/no-subscriber never waits.
  for(unsigned n=0;n<63;++n)assert(imu.published(true,1,ack));
  auto timeout=[&](auto ms){assert(ms==std::chrono::milliseconds(100));++calls;return false;};
  assert(!imu.published(true,1,timeout));
  assert(!imu.published(true,1,timeout)); // No false-success reset after timeout.
  assert(imu.published(true,1,ack));
  before=calls;
  for(unsigned n=0;n<63;++n)assert(imu.published(true,1,ack));
  assert(calls==before && imu.published(true,1,ack) && calls==before+1);
  std::cout<<"PASS bounded publication ACK batching, timeouts, recovery, no-subscriber and best-effort\n";
}
