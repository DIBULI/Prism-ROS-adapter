// Real UNIX C/C++ SDK receiver + ROS profile assembler/ACK helper. Synthetic
// sensor bytes only: not Mali execution, TF performance or hardware acceptance.
#include "prism_ros_adapter/sdk_transport.hpp"
#include "prism_ros_adapter/profile_camera.hpp"
#include <atomic>
#include <cassert>
#include <cstring>
#include <iostream>
using prism::FrameType;
using Bytes=std::vector<uint8_t>;
using namespace prism_ros_adapter;
extern "C" int prism_rklocal_run_control(int (*)(const char*));
extern "C" int prism_control_send(int,uint8_t,uint32_t,const uint8_t*,uint32_t);
extern "C" const char* prism_control_sensor_version(){return "0.5.2";}
static std::atomic<unsigned> starts{0},stops{0},acks{0};
static uint32_t fps=2,options=7,bad=0;
static void put(Bytes& b,size_t at,uint32_t v,unsigned n=4){for(unsigned i=0;i<n;++i)b.at(at+i)=uint8_t(v>>(8*i));}
static uint32_t get(const uint8_t* b,size_t at,unsigned n=4){uint32_t v=0;for(unsigned i=0;i<n;++i)v|=uint32_t(b[at+i])<<(8*i);return v;}
static void send(int fd,FrameType t,uint32_t seq,const Bytes& b){assert(!prism_control_send(fd,uint8_t(t),seq,b.data(),uint32_t(b.size())));}
static Bytes image(uint8_t cam){
  using namespace prism::capture;
  Bytes b(80),pixels(kRaw8Bytes,cam+11);
  for(size_t i=0;i<kRaw8Bytes;i+=256)b.insert(b.end(),{0,uint8_t(cam+11),uint8_t(cam+11)});
  std::memcpy(b.data(),"PRG8",4);b[4]=1;b[5]=80;b[6]=8;b[36]=2;b[37]=cam;b[38]=4;
  put(b,8,5120);put(b,12,3072);put(b,16,kRaw8Bytes);put(b,20,rawCrc32(pixels.data(),pixels.size()));
  put(b,24,17+cam);put(b,28,1800);put(b,32,1024);put(b,40,5000000);put(b,52,73);
  put(b,48,rawCrc32(b.data()+80,b.size()-80));put(b,56,b.size()-80);put(b,60,256);
  put(b,64,kRaw8Bytes/256);put(b,76,rawHeaderCrc32(b.data(),80));return b;
}
static void chunks(int fd,uint8_t cam,uint32_t id,bool partial=false){
  bool jpeg=options==3 && (cam&1);Bytes b=jpeg?Bytes{255,216,255,217}:image(cam);
  for(size_t at=0;at<b.size();){
    size_t n=std::min(size_t(32768),b.size()-at);Bytes c(36+n);c[0]=cam;c[1]=jpeg?2:0x82;
    put(c,2,4|(at?0:1)|((at+n==b.size())?2:0),2);put(c,4,jpeg?1280:5120);put(c,8,jpeg?1024:3072);
    put(c,12,id);put(c,16,b.size());put(c,20,at);put(c,24,n);put(c,28,5000000);
    std::copy_n(b.data()+at,n,c.data()+36);send(fd,FrameType::VideoChunk,0,c);at+=n;
    if(partial)break;
  }
}
extern "C" int prism_control_dispatch(int fd,uint8_t t,uint32_t seq,const uint8_t* p,uint32_t n){
  if(FrameType(t)==FrameType::StreamAck){assert(n==8&&p[0]==1);unsigned index=acks++;
    assert(get(p,4)==(index<4?100+index:index==4?201:200));return 0;}
  assert(FrameType(t)==FrameType::VideoStart||FrameType(t)==FrameType::VideoStop);
  bool on=FrameType(t)==FrameType::VideoStart;if(on)++starts;else ++stops;
  if(on){assert(n==20&&get(p,0)==(options==7?5120u:1280u)&&get(p,4)==(options==7?3072u:1024u));
    assert(get(p,8)==fps&&get(p,12)==25u*1024u*1024u&&get(p,16)==options);}
  Bytes b(20);b[0]=on;b[1]=4;put(b,2,fps?fps:10,2);put(b,4,options==7?5120:1280);
  put(b,8,options==7?3072:1024);put(b,12,25u*1024u*1024u);put(b,16,options);
  if(on){
    if(bad==1)put(b,2,3,2);
    if(bad==2)put(b,16,1);
    if(bad==3)b[1]=3;
    if(bad==4)put(b,4,0);
    if(bad==5)put(b,8,4096);
    if(bad==6)put(b,12,1);
    if(bad==7)b[0]=2;
  }
  send(fd,FrameType::VideoStatus,seq,b);
  if(on&&!bad){
    for(uint8_t cam=0;cam<4;++cam)chunks(fd,cam,100+cam);
    chunks(fd,0,200,true);chunks(fd,2,201);chunks(fd,2,201); // replay not a second credit
    Bytes imu(44);imu[0]=1;put(imu,4,777);put(imu,8,5000123);send(fd,FrameType::ImuSample,0,imu);
  }return 0;
}
template<class F> static void rejects(F fn){bool caught=false;try{fn();}catch(const std::exception&){caught=true;}assert(caught);}
static int exercise(const char* path){
  prism::rklocal::ClientOptions o;o.socket_path=path;o.raw_camera_imu_frames=true;
  auto c=sdk::Client::open(o);c.setKeepaliveEnabled(false);
  for(auto invalid:{0u,3u,1500u,UINT32_MAX})rejects([&]{c.startPrism415rsv0Video(invalid);});
  rejects([&]{c.startProfileVideoGpu(31);});assert(!starts);
  auto start=[&]{return options==7?c.startPrism415rsv0Video(fps):c.startProfileVideoGpu(fps);};
  if(bad){rejects(start);assert(starts==1);c.stopVideo();c.close();assert(stops==1);return 0;}
  auto s=start();assert(s.enabled&&s.fps==(fps?fps:10));
  rejects(start);assert(starts==1); // no second START while active
  ProfileCameraAssembler assembler;unsigned count=0;bool imu=false;
  while(!imu){
    auto f=c.readFrame(2000);
    if(f.type==FrameType::ImuSample){assert(get(f.payload.data(),4)==777);imu=true;break;}
    assert(f.type==FrameType::VideoChunk);auto chunk=prism::parseVideoChunkView(f);
    auto r=assembler.ingest(chunk,1);
    for(auto id:r.retired)sdk::acknowledgeProfileVideo(c,id);
    if(r.image){auto decoded=decodeProfileCamera(*r.image);++count;
      if(decoded.source_bits){assert(decoded.source_bits==8&&decoded.source_frame_id==17u+decoded.camera&&
        decoded.trigger_sequence==73&&decoded.transfer_id!=decoded.source_frame_id&&!decoded.timestamp_utc_valid);
        for(auto pixel:decoded.data)assert(pixel==decoded.camera+11);}}
  }
  assert(count==5);auto expired=assembler.expire(0,5000000001ULL);assert(expired.incomplete==1);
  for(auto id:expired.retired)sdk::acknowledgeProfileVideo(c,id);
  sdk::acknowledgeVideo(c,999); // legacy helper must remain NOOP for RK-local
  c.hello();assert(acks==6&&c.droppedRawFrames()==0);
  c.stopVideo();c.close();assert(stops==1);return 0;
}
static int no_raw(const char* path){
  prism::rklocal::ClientOptions o;o.socket_path=path;auto c=sdk::Client::open(o);c.setKeepaliveEnabled(false);
  rejects([&]{c.startPrism415rsv0Video(1);});rejects([&]{c.startProfileVideoGpu(10);});
  c.close();assert(!starts&&!stops);return 0;
}
static void run(int(*fn)(const char*)){starts=stops=acks=0;assert(!prism_rklocal_run_control(fn));}
int main(){run(no_raw);for(uint32_t mode:{7u,3u}){options=mode;for(uint32_t rate:{1u,2u}){fps=rate;bad=0;run(exercise);}}
  options=3;fps=0;run(exercise);options=7;fps=2;for(bad=1;bad<=7;++bad)run(exercise);
  std::cout<<"PASS actual RK-local socket profile1/2/saved mixed GPU + exact START + ROS independent ACK/expiry/replay/fullpixelsCRC + IMU + legacy ACK isolation\n";
}
