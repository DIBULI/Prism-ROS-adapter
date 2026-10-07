#ifdef NDEBUG
#undef NDEBUG
#endif
#include "prism_ros_adapter/lidar_speed_control.hpp"
#include <cassert>
#include <iostream>

struct Client {
  unsigned reads=0, writes=0;
  bool fail=false;
  prism::LidarSpeedMode mode=prism::LidarSpeedMode::Normal;
  prism::LidarSpeedStatus lidarSpeedStatus(prism::LidarModel model, uint32_t timeout) {
    ++reads;
    assert(model==prism::LidarModel::Mid360S && timeout>0 && timeout<=10000);
    if (fail) throw std::runtime_error("readback failed");
    return {model, mode, 35};
  }
  prism::LidarSpeedStatus setLidarSpeedMode(prism::LidarModel model,
                                         prism::LidarSpeedMode value, uint32_t timeout) {
    ++writes;
    assert(model==prism::LidarModel::Mid360S && timeout>0 && timeout<=10000);
    mode=value;
    if (fail) throw std::runtime_error("write readback failed");
    return {model, mode, 35};
  }
};

int main() {
  using prism_ros_adapter::LidarModel;
  using prism_ros_adapter::detail::lidarSpeedOperation;
  Client c;
  auto rejects=[&](LidarModel model, bool busy, uint32_t timeout,
                   std::optional<uint8_t> mode) {
    const auto before=c.reads+c.writes;
    bool failed=false;
    try { (void)lidarSpeedOperation(c,model,busy,timeout,mode); }
    catch(const std::exception&) { failed=true; }
    assert(failed && c.reads+c.writes==before);
  };
  for(auto model:{LidarModel::Mid360,LidarModel::Xt32}) {
    rejects(model,false,3000,std::nullopt);
    rejects(model,false,5000,2);
  }
  for(auto timeout:{0u,10001u,0xffffffffu}) {
    rejects(LidarModel::Mid360S,false,timeout,std::nullopt);
    rejects(LidarModel::Mid360S,false,timeout,1);
  }
  for(uint8_t mode:{0,3,255}) rejects(LidarModel::Mid360S,false,5000,mode);
  rejects(LidarModel::Mid360S,true,3000,std::nullopt);
  rejects(LidarModel::Mid360S,true,5000,2);
  auto s=lidarSpeedOperation(c,LidarModel::Mid360S,false,1);
  assert(s.model==2 && s.mode==1 && s.device_type==35 && c.reads==1);
  for(uint8_t mode:{2,1}) {
    s=lidarSpeedOperation(c,LidarModel::Mid360S,false,10000,mode);
    assert(s.mode==mode);
    assert(lidarSpeedOperation(c,LidarModel::Mid360S,false,3000).mode==mode);
  }
  c.fail=true;
  const auto writes=c.writes;
  bool failed=false;
  try { (void)lidarSpeedOperation(c,LidarModel::Mid360S,false,5000,2); }
  catch(const std::runtime_error&) { failed=true; }
  assert(failed && c.writes==writes+1); // No retry and no invented success.
  c.fail=false;
  assert(lidarSpeedOperation(c,LidarModel::Mid360S,false,3000).mode==2);
  std::cout << "LiDAR speed: model/mode/timeout/idle guards, readback and no-retry passed\n";
}
