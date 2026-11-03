// Real driver/decoder/dispatch implementation; no device or socket is opened.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#include "../src/driver.cpp"
#include <iostream>

namespace prism_ros_adapter {
struct DriverDataHealthTest {
  static void require(bool ok,const char* text) { if(!ok)throw std::runtime_error(text); }
  static void put(std::vector<uint8_t>& b,size_t at,uint32_t n) {
    for(unsigned i=0;i<4;++i)b.at(at+i)=uint8_t(n>>(8*i));
  }
  static prism::capture::ProfileImage fixture(uint8_t cam,uint32_t transfer,uint32_t source) {
    using namespace prism::capture;
    ProfileImage image; image.camera=cam;image.frame_id=transfer;image.format=kRaw8Rice;
    image.width=5120;image.height=3072;image.timestamp_us=5000000;
    std::vector<uint8_t> pixels(kRaw8Bytes); auto& b=image.encoded;b.resize(80);
    for(size_t i=0;i<pixels.size();++i)pixels[i]=uint8_t(i*13+cam);
    for(size_t i=0;i<pixels.size();i+=256) {
      b.push_back(8);b.insert(b.end(),pixels.begin()+i,pixels.begin()+i+256);
    }
    std::memcpy(b.data(),"PRG8",4);b[4]=1;b[5]=80;b[6]=8;b[36]=2;b[37]=cam;b[38]=4;
    put(b,8,5120);put(b,12,3072);put(b,16,kRaw8Bytes);put(b,20,rawCrc32(pixels.data(),pixels.size()));
    put(b,24,source);put(b,28,1800);put(b,32,1024);put(b,40,5000000);put(b,52,73);
    put(b,48,rawCrc32(b.data()+80,b.size()-80));put(b,56,b.size()-80);
    put(b,60,256);put(b,64,kRaw8Bytes/256);put(b,76,rawHeaderCrc32(b.data(),80));
    return image;
  }
  static prism::VideoChunkView chunk(const prism::capture::ProfileImage& im,size_t offset,size_t count) {
    prism::VideoChunkView c;
    c.camera_id=im.camera;c.format=im.format;c.frame_id=im.frame_id;
    c.width=im.width;c.height=im.height;c.timestamp_us=im.timestamp_us;
    c.encoded_size=im.encoded.size();c.chunk_offset=offset;c.chunk_size=count;
    c.data=im.encoded.data()+offset;c.data_size=count;
    c.flags=4|(offset==0?1:0)|(offset+count==im.encoded.size()?2:0);return c;
  }
  template<class F> static void rejects(F fn,const char* why) {
    bool failed=false;try{fn();}catch(const std::exception&){failed=true;}require(failed,why);
  }
  static void run() {
    ProfileCameraAssembler assembly;
    for(uint8_t cam=0;cam<4;++cam) {
      auto im=fixture(cam,100+cam,17+cam*9);
      auto a=assembly.ingest(chunk(im,0,500),0);
      require(!a.image&&a.retired.empty(),"partial retirement");
      auto z=assembly.ingest(chunk(im,500,im.encoded.size()-500),10);
      require(z.image&&z.retired==std::vector<uint32_t>{100u+cam},"exact completion credit");
      auto decoded=decodeProfileCamera(*z.image);
      require(decoded.transfer_id==100u+cam&&decoded.source_frame_id==17u+cam*9&&
              decoded.trigger_sequence==73&&decoded.source_frame_valid,"IDs conflated");
      require(decoded.source_bits==8&&decoded.source_timestamp_us==5000000&&
              !decoded.timestamp_utc_valid&&decoded.timing_mode==4,"clock/depth semantics");
      require(decoded.data.size()==5120u*3072u,"native dimensions");
      for(size_t i=0;i<decoded.data.size();++i)require(decoded.data[i]==uint8_t(i*13+cam),"native bytes altered");
      auto duplicate=assembly.ingest(chunk(im,0,im.encoded.size()),20);
      require(!duplicate.image&&duplicate.retired.empty()&&!duplicate.error.empty(),"duplicate credit/image");
      im.timestamp_us++;
      rejects([&]{decodeProfileCamera(im);},"envelope time mismatch");
      im.timestamp_us--; im.encoded.back()^=1;
      rejects([&]{decodeProfileCamera(im);},"encoded CRC failure");
      im.encoded.back()^=1; put(im.encoded,20,1);
      put(im.encoded,76,prism::capture::rawHeaderCrc32(im.encoded.data(),80));
      rejects([&]{decodeProfileCamera(im);},"full pixel CRC failure");
    }
    auto pending=fixture(0,200,50);
    assembly.ingest(chunk(pending,0,500),100);
    auto healthy=fixture(2,201,99);
    auto h=assembly.ingest(chunk(healthy,0,healthy.encoded.size()),101);
    require(h.image&&h.retired==std::vector<uint32_t>{201},"missing camera blocks healthy lane");
    pending.frame_id=202;
    auto replacement=assembly.ingest(chunk(pending,0,500),102);
    require(replacement.incomplete==1&&replacement.retired==std::vector<uint32_t>{200},"hidden partial replacement");
    auto expired=assembly.expire(0,5000000102ULL);
    require(expired.incomplete==1&&expired.retired==std::vector<uint32_t>{202},"expiry credit");
    require(assembly.expire(0,6000000102ULL).retired.empty(),"duplicate expiry credit");
    pending.frame_id=204;assembly.ingest(chunk(pending,0,500),1);
    auto tail=assembly.finish(0);
    require(tail.incomplete==1&&!tail.error.empty()&&tail.retired==std::vector<uint32_t>{204},"STOP partial unreported");
    require(assembly.finish(0).retired.empty(),"STOP partial retired twice");
    assembly.reset();
    require(assembly.ingest(chunk(pending,0,pending.encoded.size()),1).image.has_value(),"restart clears retirement epoch");
    prism::capture::ProfileImage jpeg;
    jpeg.camera=3;jpeg.format=2;jpeg.frame_id=203;jpeg.width=1280;jpeg.height=1024;
    jpeg.timestamp_us=900;jpeg.encoded={255,216,255,217};
    const auto mixed=decodeProfileCamera(jpeg);
    require(mixed.data==jpeg.encoded&&!mixed.source_frame_valid&&mixed.trigger_sequence==0&&
            mixed.source_bits==0&&!mixed.timestamp_utc_valid,"mixed JPEG invented native metadata");
    jpeg.format=0x81; rejects([&]{decodeProfileCamera(jpeg);},"CPU fallback admitted");

    std::vector<ProfileCameraImage> delivered;
    DriverCallbacks cb;cb.profile_camera=[&](const auto& im){delivered.push_back(im);};
    DriverConfig config; config.require_synchronized_timestamps=false;
    Driver::Impl driver(config,cb);
    auto original=fixture(1,333,9);
    driver.publishProfileCamera(ProfileCameraJob{original,7});
    require(delivered.size()==1&&delivered[0].image_published&&delivered[0].acquisition_epoch==7&&
            delivered[0].data.size()==5120u*3072u,"actual driver callback");
    driver.config.require_synchronized_timestamps=true;
    driver.publishProfileCamera(ProfileCameraJob{original,8});
    require(delivered.size()==2&&!delivered[1].image_published&&delivered[1].data.empty()&&
            delivered[1].source_timestamp_us==5000000&&driver.dropped_unsynchronized==1,"explicit sync policy");
    original.encoded.back()^=1; driver.publishProfileCamera(ProfileCameraJob{original,8});
    require(delivered.size()==2&&driver.profile_decode_errors[1]==1,"corruption silently published");
    original.encoded.back()^=1; driver.config.require_synchronized_timestamps=false;
    driver.publishProfileCamera(ProfileCameraJob{original,9});
    require(delivered.size()==3,"corrupt lane halts healthy future image");

    std::promise<void> entered,release; auto gate=release.get_future().share();
    std::vector<int> consumed;
    DispatchQueue<int> queue(2,[&](int n){if(n==1){entered.set_value();gate.wait();}consumed.push_back(n);},true);
    queue.start(); queue.push(1);
    if(entered.get_future().wait_for(std::chrono::seconds(2))!=std::future_status::ready) {
      release.set_value();throw std::runtime_error("dispatch did not start");
    }
    queue.push(2);queue.push(3);
    bool overflow=false;try{queue.push(4);}catch(const std::runtime_error&){overflow=true;}
    release.set_value();queue.stop();
    require(overflow&&consumed==std::vector<int>({1,2,3})&&queue.dropped()==0&&queue.peak()==2,
            "bounded queue must reject new work and drain, not evict");
    std::promise<void> tail_entered,tail_release;auto tail_gate=tail_release.get_future().share();
    std::vector<int> tail_consumed;
    DispatchQueue<int> ordinary(1,[&](int n){if(n==1){tail_entered.set_value();tail_gate.wait();}tail_consumed.push_back(n);});
    ordinary.start();ordinary.push(1);tail_entered.get_future().wait();
    rejects([&]{ordinary.waitUntilIdle(std::chrono::steady_clock::now()+std::chrono::milliseconds(20));},
            "empty queue incorrectly ignores in-flight callback");
    ordinary.push(2);
    ordinary.beginDrain(std::chrono::steady_clock::now()+std::chrono::milliseconds(20));
    rejects([&]{ordinary.push(3);},"bounded tail deadline not enforced");
    tail_release.set_value();ordinary.stop();
    ordinary.waitUntilIdle(std::chrono::steady_clock::now()+std::chrono::milliseconds(20));
    ordinary.endDrain();
    require(tail_consumed==std::vector<int>({1,2})&&ordinary.dropped()==0,"STOP discarded accepted ordinary queue samples");
    DriverCallbacks bad; bad.profile_camera=[](const auto&){throw std::runtime_error("last publication failed");};
    Driver::Impl failing(config,bad);
    jpeg.format=2;
    failing.profile_dispatch.start();failing.profile_dispatch.push(ProfileCameraJob{jpeg,1});
    failing.profile_dispatch.stop();
    require(failing.profile_dispatch.callbackErrors()==1,"final drain callback failure lost");
    rejects([&]{failing.ensureProfilePublicationHealthy();},"false success after final callback failure");
    std::cout<<"PASS ROS native8 4 cameras/all pixels/CRC/separate IDs/epoch/clock/missing/mixed/replay/expiry/strict queue\n";
  }
};
}
int main(){try{prism_ros_adapter::DriverDataHealthTest::run();return 0;}
catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
