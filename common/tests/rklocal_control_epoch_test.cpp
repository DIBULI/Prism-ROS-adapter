// Actual driver idle/control lifecycle and UNIX SDK. No hardware/clock change.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#include "../src/driver.cpp"
#include <cassert>
#include <iostream>
extern "C" int prism_rklocal_run_control(int (*)(const char*));
extern "C" int prism_control_send(int,uint8_t,uint32_t,const uint8_t*,uint32_t);
extern "C" const char* prism_control_sensor_version(){return "0.5.2";}
using Bytes=std::vector<uint8_t>;
using prism::FrameType;
static std::atomic<unsigned> starts{0},stops{0},imu_starts{0};
static unsigned fault=0;
static bool mixed=false;
static Bytes original;
static void put(Bytes& b,size_t at,uint32_t v,unsigned n=4){for(unsigned i=0;i<n;++i)b.at(at+i)=uint8_t(v>>(8*i));}
static void send(int fd,FrameType t,uint32_t seq,const Bytes& b){assert(!prism_control_send(fd,uint8_t(t),seq,b.data(),b.size()));}
static Bytes makeImage(){
  using namespace prism::capture;
  Bytes b(80),pixels(kRaw8Bytes,23);
  for(size_t i=0;i<kRaw8Bytes;i+=256)b.insert(b.end(),{0,23,23});
  std::memcpy(b.data(),"PRG8",4);b[4]=1;b[5]=80;b[6]=8;b[36]=2;b[38]=4;
  put(b,8,5120);put(b,12,3072);put(b,16,kRaw8Bytes);put(b,20,rawCrc32(pixels.data(),pixels.size()));
  put(b,24,1);put(b,28,1800);put(b,32,1024);put(b,40,5000000);put(b,52,1);
  put(b,48,rawCrc32(b.data()+80,b.size()-80));put(b,56,b.size()-80);put(b,60,256);
  put(b,64,kRaw8Bytes/256);put(b,76,rawHeaderCrc32(b.data(),80));return b;
}
static void chunks(int fd,bool jpeg){
  const Bytes b=jpeg?Bytes{255,216,255,217}:original;
  for(size_t at=0;at<b.size();){
    const size_t n=std::min(size_t(32768),b.size()-at);Bytes c(36+n);c[0]=jpeg?1:0;c[1]=jpeg?2:0x82;
    put(c,2,4|(at?0:1)|((at+n==b.size())?2:0),2);put(c,4,jpeg?1280:5120);put(c,8,jpeg?1024:3072);
    // Source/trigger/transfer IDs intentionally restart identically each epoch.
    put(c,12,jpeg?101:100);put(c,16,b.size());put(c,20,at);put(c,24,n);put(c,28,5000000);
    std::copy_n(b.data()+at,n,c.data()+36);send(fd,FrameType::VideoChunk,0,c);at+=n;
    if(fault==1&&!jpeg)break;
  }
}
extern "C" int prism_control_dispatch(int fd,uint8_t t,uint32_t seq,const uint8_t*,uint32_t){
  const auto type=FrameType(t);
  // Any late ACK or a second stop for the same epoch fails this fixture.
  assert(type==FrameType::VideoStart||type==FrameType::VideoStop||type==FrameType::ImuStart||type==FrameType::ImuStop);
  const bool on=type==FrameType::VideoStart||type==FrameType::ImuStart;
  if(type==FrameType::VideoStart)++starts;
  if(type==FrameType::ImuStart)++imu_starts;
  if(!on){
    assert(stops<starts);++stops;
    chunks(fd,false);if(mixed)chunks(fd,true);
    for(unsigned i=0;i<8;++i){Bytes imu(44);imu[0]=i%2;put(imu,4,stops*100+i);put(imu,8,5000123+i*1250);
      send(fd,FrameType::ImuSample,0,imu);}
  }
  if(type==FrameType::ImuStart||type==FrameType::ImuStop){
    Bytes b(12);b[0]=on;b[1]=2;put(b,2,800,2);put(b,4,44);send(fd,FrameType::ImuStatus,seq,b);
  }else{
    Bytes b(20);b[0]=on;b[1]=4;put(b,2,2,2);put(b,4,mixed?1280:5120);put(b,8,mixed?1024:3072);
    put(b,12,25u*1024u*1024u);put(b,16,mixed?3:7);send(fd,FrameType::VideoStatus,seq,b);
  }
  return 0;
}
namespace prism_ros_adapter {
struct DriverDataHealthTest {
  static int exercise(const char* path){
    prism::rklocal::ClientOptions o;o.socket_path=path;o.raw_camera_imu_frames=true;
    auto client=sdk::Client::open(o);client.setKeepaliveEnabled(false);
    std::vector<uint64_t> epochs;std::vector<uint32_t> samples;
    unsigned points=0,controls=0;
    DriverConfig cfg;cfg.require_synchronized_timestamps=false;
    cfg.camera_fps=2;cfg.enable_navigation=false;cfg.enable_gnss_raw=false;
    DriverCallbacks cb;
    cb.profile_camera=[&](const auto& im){
      std::this_thread::sleep_for(std::chrono::milliseconds(140));
      if(fault==2)throw std::runtime_error("synthetic old-epoch publisher failed");
      assert(im.transfer_id==(im.camera?101u:100u)&&im.source_timestamp_us==5000000);
      if(!im.camera){assert(im.source_frame_id==1&&im.trigger_sequence==1&&im.data.size()==5120u*3072u);
        for(auto p:im.data)assert(p==23);}
      epochs.push_back(im.acquisition_epoch);
    };
    cb.board_imu=[&](const auto& s){samples.push_back(s.sample_id);};
    cb.lidar_points=[&](const auto& p){points+=p.points.size();};
    Driver::Impl d(cfg,cb);d.device_model=mixed?prism::DeviceModel::Prism1315:prism::DeviceModel::Prism415RSV0;
    d.startDispatchers();
    std::unique_ptr<sdk::ImuStream> imu;std::unique_ptr<sdk::LidarStream> lidar;
    bool video=false,imu_on=false,lidar_on=false;
    Driver::Impl::ControlContext ctx{client,imu,lidar,2,video,imu_on,lidar_on};
    d.startConfiguredStreams(ctx);assert(starts==1&&imu_starts==1&&d.profile_epoch==1);
    const unsigned per_epoch=mixed?2:1;
    const unsigned rounds=(fault==1||fault==2)?1:2;
    for(unsigned round=1;round<=rounds;++round){
      LidarPointBatch cloud;cloud.timestamp_ns=5000000000ULL;cloud.points.resize(3);
      assert(d.lidar_frame_accumulator.append(std::move(cloud)).empty());
      bool failed=false;
      try{d.runIdleOperation<int>(ctx,"synthetic idle operation",[&](auto&){
        // Must run only after the old epoch's callbacks, including a slow one.
        assert(epochs.size()==round*per_epoch&&samples.size()==round*8&&points==round*3);
        assert(d.profile_epoch==round&&starts==round&&stops==round);
        ++controls;if(fault==3)throw std::runtime_error("synthetic setting rejected");return 7;
      });}catch(const std::exception&){failed=true;}
      if(fault==1||fault==2){
        assert(failed&&controls==0&&starts==1&&stops==1&&!d.fatal_control_error.empty());
        break;
      }
      assert(failed==(fault==3)&&controls==round&&d.fatal_control_error.empty());
      assert(starts==round+1&&imu_starts==round+1&&d.profile_epoch==round+1);
      for(unsigned j=0;j<epochs.size();++j)assert(epochs[j]==1+j/per_epoch);
    }
    const auto cleanup_error=d.cleanup(client,lidar.get(),imu.get(),lidar_on,imu_on,video);
    d.stopDispatchers();
    assert(!client.isOpen()&&stops==starts);
    if(fault==0||fault==3){
      assert(cleanup_error.empty());d.ensureFinalPublicationHealthy();
      assert(epochs.size()==3*per_epoch&&samples.size()==24);
      for(unsigned j=0;j<epochs.size();++j)assert(epochs[j]==1+j/per_epoch);
      for(unsigned j=0;j<samples.size();++j)assert(samples[j]==100*(1+j/8)+j%8);
    }
    return 0;
  }
};
}
int main(){original=makeImage();for(bool product:{false,true}){mixed=product;
  for(fault=0;fault<4;++fault){starts=stops=imu_starts=0;
    assert(!prism_rklocal_run_control(prism_ros_adapter::DriverDataHealthTest::exercise));}}
  std::cout<<"PASS actual control cycles 415RSV0/1315: prior epoch RAW/JPEG/IMU/LiDAR fully published before settings/restart, reused IDs stay distinct, partial/callback failure blocks operation/restart, setting rejection resumes only after clean drain\n";
}
