// Actual ROS cleanup and SDK close under a synthetic UNIX command peer.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#include "../src/driver.cpp"
#include <cassert>
#include <iostream>
extern "C" int prism_rklocal_run_control(int (*)(const char*));
extern "C" int prism_control_send(int,uint8_t,uint32_t,const uint8_t*,uint32_t);
extern "C" const char* prism_control_sensor_version(){return "0.5.2";}
static std::atomic<unsigned> starts{0},stops{0};
static unsigned fault=0;
static bool earlier=false;
extern "C" int prism_control_dispatch(int fd,uint8_t t,uint32_t seq,const uint8_t*,uint32_t){
  using prism::FrameType;
  assert(FrameType(t)==FrameType::VideoStart||FrameType(t)==FrameType::VideoStop);
  const bool on=FrameType(t)==FrameType::VideoStart;if(on)++starts;else ++stops;
  std::vector<uint8_t> b(20);b[0]=on;b[1]=4;b[2]=2;b[5]=20;b[9]=12;b[14]=144;b[15]=1;b[16]=7;
  auto response=FrameType::VideoStatus;
  if(!on){if(fault==1){b.assign(50,0);b[0]=65;response=FrameType::Error;}
    else if(fault==2)b[0]=1;
    else if(fault==3)b.resize(3);}
  return prism_control_send(fd,uint8_t(response),seq,b.data(),b.size());
}
namespace prism_ros_adapter {
struct DriverDataHealthTest {
  static int exercise(const char* path){
    prism::rklocal::ClientOptions o;o.socket_path=path;o.raw_camera_imu_frames=true;
    auto c=sdk::Client::open(o);c.setKeepaliveEnabled(false);c.startVideo1280x1024(10);
    DriverConfig config;DriverCallbacks callbacks;Driver::Impl d(config,callbacks);
    d.capture_start_issued=true;
    std::unique_ptr<sdk::ImuStream> imu;
    std::unique_ptr<sdk::LidarStream> lidar;
    bool video=true,imu_started=true,lidar_started=false;
    Driver::Impl::ControlContext context{c,imu,lidar,2,video,imu_started,lidar_started};
    if(earlier){
      bool failed=false;
      try{d.runIdleOperation<int>(context,"fixture",[](auto&){assert(false);return 0;});}
      catch(const std::exception&){failed=true;}
      assert(failed&&!d.fatal_control_error.empty()&&stops==1&&starts==1);
    }
    auto error=d.cleanup(c,nullptr,nullptr,false,imu_started,video);
    assert(!c.isOpen()&&starts==1&&stops==1);
    if(fault)assert(error.find("STOP unconfirmed")!=std::string::npos);
    else assert(error.empty()&&!d.capture_start_issued);
    if(earlier)assert(error.find("no automatic retry")!=std::string::npos);
    return 0;
  }
};
}
int main(){
  for(fault=0;fault<=3;++fault){starts=stops=0;earlier=false;
    assert(!prism_rklocal_run_control(prism_ros_adapter::DriverDataHealthTest::exercise));}
  for(fault=1;fault<=3;++fault){starts=stops=0;earlier=true;
    assert(!prism_rklocal_run_control(prism_ros_adapter::DriverDataHealthTest::exercise));}
  std::cout<<"PASS actual ROS aggregate STOP once, remote/error/malformed acknowledgement propagated, no close retry or restart; not independent hardware idle proof\n";
}
