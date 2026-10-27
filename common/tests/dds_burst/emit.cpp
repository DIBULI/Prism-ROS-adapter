// Loopback-only DDS regression: no SDK or device operations.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>
#include <iostream>
#include <atomic>
#include "prism_ros_adapter/publication_ack.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
  if (argc != 5) return 2;
  const auto camera_depth = std::stoul(argv[1]), imu_depth = std::stoul(argv[2]);
  const auto ack_chunk=std::stoul(argv[4]);
  std::atomic<size_t> ack_timeouts{0};
  prism_ros_adapter::PublicationAckWindow imu_ack(ack_chunk?ack_chunk:1);
  std::array<prism_ros_adapter::PublicationAckWindow,4> camera_ack;
  if (!camera_depth || camera_depth > 64 || !imu_depth || imu_depth > 8192) return 2;
  std::ifstream input(argv[3], std::ios::binary);
  std::vector<uint8_t> jpeg((std::istreambuf_iterator<char>(input)), {});
  if (jpeg.empty() || jpeg.size() > 2*1024*1024) return 2;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("prism_dds_burst_emitter");
  std::array<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr,4> cameras;
  for (size_t i=0;i<4;++i) cameras[i]=node->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/dds_probe/camera"+std::to_string(i),rclcpp::QoS(camera_depth).reliable().durability_volatile());
  auto imu=node->create_publisher<sensor_msgs::msg::Imu>("/dds_probe/imu",
    rclcpp::QoS(imu_depth).reliable().durability_volatile());
  auto actual=imu->get_actual_qos().get_rmw_qos_profile();
  std::cout<<"IMU actual history="<<actual.history<<" depth="<<actual.depth
           <<" reliability="<<actual.reliability<<std::endl;
  auto deadline=std::chrono::steady_clock::now()+8s;
  auto ready=[&](){for(auto &p:cameras)if(p->get_subscription_count()!=1)return false;return imu->get_subscription_count()==1;};
  while(!ready() && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(10ms);
  if(!ready())return 3;
  auto start=std::chrono::steady_clock::now()+200ms;
  std::thread imu_thread([&]{
    for(size_t burst=0;burst<12;++burst){
      std::this_thread::sleep_until(start+burst*600ms);
      for(size_t j=0;j<480;++j){
        size_t n=burst*480+j;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp=rclcpp::Time(1000000000LL+n*1250000LL);
        msg.linear_acceleration.x=n;
        imu->publish(msg);
        if(ack_chunk && !imu_ack.published(true,imu->get_subscription_count(),
           [&](auto timeout){return imu->wait_for_all_acked(timeout);}))++ack_timeouts;
      }
    }
  });
  for(size_t burst=0;burst<12;++burst){
    std::this_thread::sleep_until(start+burst*600ms);
    for(size_t j=0;j<6;++j)for(size_t c=0;c<4;++c){
      size_t n=burst*6+j;
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp=rclcpp::Time(1000000000LL+n*100000000LL);
      msg.header.frame_id=std::to_string(c);
      msg.format="jpeg";msg.data=jpeg;cameras[c]->publish(msg);
      if(ack_chunk && !camera_ack[c].published(true,cameras[c]->get_subscription_count(),
         [&](auto timeout){return cameras[c]->wait_for_all_acked(timeout);}))++ack_timeouts;
    }
  }
  imu_thread.join();
  for(auto &p:cameras)if(!p->wait_for_all_acked(5s))return 4;
  if(!imu->wait_for_all_acked(5s))return 4;
  std::this_thread::sleep_for(1s);
  std::cout<<"sent72frames/lane and5760IMU jpegbytes="<<jpeg.size()<<std::endl;
  std::cout<<"ack_timeouts="<<ack_timeouts.load()<<std::endl;
  rclcpp::shutdown();
  return ack_timeouts?5:0;
}
