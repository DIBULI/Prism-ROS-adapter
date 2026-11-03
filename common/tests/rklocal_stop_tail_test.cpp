// Real ROS driver + C/C++ UNIX receiver. Synthetic GPU-format originals only;
// no hardware, GPU execution, TF performance or independent-idle claim.
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
static unsigned mode=0;
static std::atomic<unsigned> starts{0},stops{0};
struct ExpectedQueueFaultDisconnect {};
static void put(Bytes& b,size_t at,uint32_t v,unsigned n=4){for(unsigned i=0;i<n;++i)b.at(at+i)=uint8_t(v>>(8*i));}
static void send(int fd,FrameType t,uint32_t seq,const Bytes& b){
  const auto rc=prism_control_send(fd,uint8_t(t),seq,b.data(),b.size());
  if(rc)std::cerr<<"peer send failed mode="<<mode<<" type="<<unsigned(t)<<" bytes="<<b.size()<<" rc="<<rc<<std::endl;
  // The deliberately undersized-queue case must fail closed immediately. A
  // peer still emitting tail data can then see EPIPE; that is not a success
  // assertion for the client (exercise separately requires its exact error).
  if(rc&&mode==3)throw ExpectedQueueFaultDisconnect{};
  assert(!rc);
}
static Bytes image(uint8_t cam){
  using namespace prism::capture;
  Bytes b(80),pixels(kRaw8Bytes,cam+11);
  for(size_t i=0;i<kRaw8Bytes;i+=256) {
    if(mode==5){b.push_back(8);b.insert(b.end(),256,uint8_t(cam+11));}
    else b.insert(b.end(),{0,uint8_t(cam+11),uint8_t(cam+11)});
  }
  std::memcpy(b.data(),"PRG8",4);b[4]=1;b[5]=80;b[6]=8;b[36]=2;b[37]=cam;b[38]=4;
  put(b,8,5120);put(b,12,3072);put(b,16,kRaw8Bytes);put(b,20,rawCrc32(pixels.data(),pixels.size()));
  put(b,24,17+cam);put(b,28,1800);put(b,32,1024);put(b,40,5000000);put(b,52,73);
  put(b,48,rawCrc32(b.data()+80,b.size()-80));put(b,56,b.size()-80);put(b,60,256);
  put(b,64,kRaw8Bytes/256);put(b,76,rawHeaderCrc32(b.data(),80));return b;
}
static std::array<Bytes,4> originals;
static void chunks(int fd,uint8_t cam,uint32_t id,bool partial=false,bool corrupt=false){
  auto b=originals.at(cam);if(corrupt)b.back()^=1;
  for(size_t at=0;at<b.size();){
    const size_t n=std::min(size_t(32768),b.size()-at);Bytes c(36+n);c[0]=cam;c[1]=0x82;
    put(c,2,4|(at?0:1)|((at+n==b.size())?2:0),2);put(c,4,5120);put(c,8,3072);
    put(c,12,id);put(c,16,b.size());put(c,20,at);put(c,24,n);put(c,28,5000000);
    std::copy_n(b.data()+at,n,c.data()+36);send(fd,FrameType::VideoChunk,0,c);at+=n;
    if(partial)break;
  }
}
extern "C" int prism_control_dispatch(int fd,uint8_t t,uint32_t seq,const uint8_t*,uint32_t){
  try {
  // No late credits, destructor STOP retry or automatic restart is allowed.
  assert(FrameType(t)==FrameType::VideoStart||FrameType(t)==FrameType::VideoStop);
  const bool on=FrameType(t)==FrameType::VideoStart;if(on)++starts;else ++stops;
  if(!on){
    const unsigned count=(mode==0||mode==5)?14:1;
    for(unsigned i=0;i<count;++i)chunks(fd,i%4,100+i,mode==1,mode==2);
    for(unsigned i=0;i<64;++i){Bytes imu(44);imu[0]=i%2;put(imu,4,1000+i);put(imu,8,5000123+i*1250);
      send(fd,FrameType::ImuSample,0,imu);}
    Bytes rtcm(27);put(rtcm,0,1,2);put(rtcm,2,24,2);put(rtcm,4,55);put(rtcm,12,3);
    rtcm[24]=0xd3;rtcm[25]=8;rtcm[26]=9;send(fd,FrameType::RoverRtcm,0,rtcm);
  }
  Bytes b(20);b[0]=on;b[1]=4;put(b,2,2,2);put(b,4,5120);put(b,8,3072);put(b,12,25u*1024u*1024u);put(b,16,7);
  send(fd,FrameType::VideoStatus,seq,b);return 0;
  } catch(const ExpectedQueueFaultDisconnect&) {
    assert(mode==3);return 0;
  }
}
namespace prism_ros_adapter {
struct DriverDataHealthTest {
  static int exercise(const char* path){
    prism::rklocal::ClientOptions o;o.socket_path=path;o.raw_camera_imu_frames=true;
    if(mode==3)o.raw_frame_queue_bytes=1024; // a single chunk cannot fit, explicit failure
    if(mode==5)o.raw_frame_queue_bytes=1024u*1024u; // >210MiB tail through 1MiB queue
    auto c=sdk::Client::open(o);c.setKeepaliveEnabled(false);c.startPrism415rsv0Video(2);
    std::vector<uint32_t> images,samples;unsigned rtcm=0,points=0;
    DriverConfig cfg;cfg.require_synchronized_timestamps=false;
    DriverCallbacks cb;
    cb.profile_camera=[&](const auto& im){
      if(images.empty())std::this_thread::sleep_for(std::chrono::milliseconds(120));
      if(mode==4)throw std::runtime_error("synthetic final subscriber failure");
      assert(im.source_bits==8&&im.data.size()==5120u*3072u&&im.trigger_sequence==73&&
        im.source_frame_id==17u+im.camera&&im.source_timestamp_us==5000000&&im.acquisition_epoch==9);
      for(auto p:im.data)assert(p==im.camera+11);
      images.push_back(im.transfer_id);
    };
    cb.board_imu=[&](const auto& s){samples.push_back(s.sample_id);};
    cb.rover_rtcm=[&](const auto& r){assert(r.sequence==55&&r.data==Bytes({0xd3,8,9}));++rtcm;};
    cb.lidar_points=[&](const auto& p){points+=p.points.size();};
    Driver::Impl d(cfg,cb);d.profile_camera=true;d.profile_epoch=9;d.capture_start_issued=true;
    d.startDispatchers();
    LidarPointBatch cloud;cloud.timestamp_ns=5000000000ULL;cloud.points.resize(3);
    assert(d.lidar_frame_accumulator.append(std::move(cloud)).empty());
    const auto error=d.cleanup(c,nullptr,nullptr,false,true,true);
    d.stopDispatchers();
    bool publish_failed=false;try{d.ensureFinalPublicationHealthy();}catch(...){publish_failed=true;}
    assert(!c.isOpen()&&starts==1&&stops==1&&points==3);
    if(mode==3){assert(error.find("raw event queue failed")!=std::string::npos);return 0;}
    assert(samples.size()==64&&rtcm==1);
    for(unsigned i=0;i<64;++i)assert(samples[i]==1000+i);
    if(mode==1)assert(error.find("incomplete/corrupt")!=std::string::npos&&images.empty());
    else assert(error.empty());
    if(mode==2||mode==4)assert(publish_failed&&images.empty());
    else assert(!publish_failed);
    if(mode==0||mode==5){assert(images.size()==14&&d.profile_dispatch.peak()<=4&&d.profile_dispatch.dropped()==0);
      for(unsigned i=0;i<14;++i)assert(images[i]==100+i);}
    return 0;
  }
};
}
int main(){
  for(mode=0;mode<6;++mode){starts=stops=0;
    std::cerr<<"STOP tail case "<<mode<<" begin"<<std::endl;
    for(uint8_t c=0;c<4;++c)originals[c]=image(c);
    assert(!prism_rklocal_run_control(prism_ros_adapter::DriverDataHealthTest::exercise));}
  std::cout<<"PASS actual STOP tail: 14 full RAW8 CRC images, uncompressed-block >210MiB tail through fixed1MiB queue +64 ordered IMU +RTCM +pending LiDAR; bounded slow publisher; truncated/corrupt/oversized-chunk/callback faults explicit; no late ACK/retry/restart\n";
}
