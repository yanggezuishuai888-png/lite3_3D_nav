#include "lite3_sensor_gateway/codecs.hpp"
#include <chrono>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
using namespace std::chrono;
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("gateway_test_fixture");
  auto mode = n->declare_parameter<std::string>("mode", "publish");
  auto qos = rclcpp::SensorDataQoS();
  if (mode == "stress") {
    auto color = n->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", qos);
    auto aligned = n->create_publisher<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", qos);
    auto color_info = n->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", qos);
    auto aligned_info = n->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/aligned_depth_to_color/camera_info", qos);
    sensor_msgs::msg::Image rgb, depth;
    rgb.header.frame_id = "camera_color_optical_frame";
    rgb.width = depth.width = 640;
    rgb.height = depth.height = 480;
    rgb.encoding = "rgb8";
    rgb.step = 1920;
    rgb.data.resize(rgb.step * rgb.height);
    depth.header.frame_id = rgb.header.frame_id;
    depth.encoding = "16UC1";
    depth.step = 1280;
    depth.data.resize(depth.step * depth.height);
    std::mt19937 random(42);
    for (auto &byte : rgb.data)
      byte = static_cast<uint8_t>(random());
    for (auto &byte : depth.data)
      byte = static_cast<uint8_t>(random());
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = rgb.header.frame_id;
    info.width = rgb.width;
    info.height = rgb.height;
    info.k[0] = info.k[4] = 450;
    info.k[8] = 1;
    std::this_thread::sleep_for(seconds(2));
    auto next = steady_clock::now();
    auto end = next + seconds(15);
    int frames = 0;
    while (rclcpp::ok() && steady_clock::now() < end) {
      rgb.header.stamp = depth.header.stamp = info.header.stamp = n->now();
      color->publish(rgb);
      aligned->publish(depth);
      color_info->publish(info);
      aligned_info->publish(info);
      ++frames;
      next += milliseconds(70);
      std::this_thread::sleep_until(next);
    }
    printf("stress published %d RGB-D pairs\n", frames);
    rclcpp::shutdown();
    return 0;
  }
  std::set<std::string> seen;
  bool stale = false;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  if (mode == "check") {
    subs.push_back(n->create_subscription<sensor_msgs::msg::Imu>(
        "/lite3/imu/data", qos, [&](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          if (m->angular_velocity.x == 3.25 && m->orientation_covariance[0] == -1 &&
              m->header.stamp.sec != 123)
            seen.insert("imu");
        }));
    subs.push_back(n->create_subscription<nav_msgs::msg::Odometry>(
        "/lite3/leg_odom2", qos, [&](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          if (m->twist.twist.linear.x == .75 && m->child_frame_id == "base_test")
            seen.insert("odom");
        }));
    subs.push_back(n->create_subscription<geometry_msgs::msg::Twist>(
        "/lite3/handle_state", qos, [&](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          if (m->angular.z == -.25)
            seen.insert("handle");
        }));
    subs.push_back(n->create_subscription<std_msgs::msg::Float64>(
        "/lite3/ultrasound_distance", qos, [&](std_msgs::msg::Float64::ConstSharedPtr m) {
          if (m->data == 2.5)
            seen.insert("ultrasound");
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::Image>(
        "/lite3/camera/depth/image_rect_raw", qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (m->width == 424 && m->height == 240 && m->data.size() == 203520) {
            bool ok = true;
            for (size_t i = 0; i < m->data.size(); ++i)
              if (m->data[i] != (i * 31 + i / 253) % 256)
                ok = false;
            if (ok)
              seen.insert("depth");
          }
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/lite3/camera/depth/camera_info", qos,
        [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          if (m->k[0] == 123 && m->d.size() == 5)
            seen.insert("info");
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::Image>(
        "/lite3/camera/color/image_raw", qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (m->width == 64 && m->height == 48 && m->encoding == "rgb8" &&
              m->header.frame_id == "color_test" && m->header.stamp.sec == 123 &&
              m->data.size() == 9216 && m->data[1234] == 1234 % 251)
            seen.insert("color");
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::Image>(
        "/lite3/camera/aligned_depth_to_color/image_raw", qos,
        [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          if (m->width == 64 && m->height == 48 && m->encoding == "16UC1" &&
              m->header.frame_id == "color_test" && m->header.stamp.sec == 123 &&
              m->data.size() == 6144 && m->data[1234] == 1234 % 251)
            seen.insert("aligned");
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/lite3/camera/color/camera_info", qos,
        [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          if (m->width == 64 && m->height == 48 && m->header.stamp.sec == 123 && m->k[0] == 456)
            seen.insert("color_info");
        }));
    subs.push_back(n->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/lite3/camera/aligned_depth_to_color/camera_info", qos,
        [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          if (m->width == 64 && m->height == 48 && m->header.stamp.sec == 123 && m->k[0] == 456)
            seen.insert("aligned_info");
        }));
    subs.push_back(n->create_subscription<std_msgs::msg::Float64>(
        "/lite3/camera/depth/scale_m", qos, [&](std_msgs::msg::Float64::ConstSharedPtr m) {
          if (m->data == .001)
            seen.insert("scale");
        }));
    subs.push_back(n->create_subscription<std_msgs::msg::Bool>(
        "/lite3/sensor_status/imu_isalive", qos, [&](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data)
            seen.insert("status");
          else if (seen.count("status"))
            stale = true;
        }));
    auto end = steady_clock::now() + seconds(18);
    while (rclcpp::ok() && steady_clock::now() < end) {
      rclcpp::spin_some(n);
      if (seen.size() == 12 && stale) {
        puts("integration passed: 12 streams and stale status");
        rclcpp::shutdown();
        return 0;
      }
      std::this_thread::sleep_for(milliseconds(5));
    }
    for (auto &s : seen)
      printf("seen %s\n", s.c_str());
    fprintf(stderr, "integration failed: streams=%zu stale=%d\n", seen.size(), stale);
    rclcpp::shutdown();
    return 1;
  }
  auto imu = n->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
  auto odom = n->create_publisher<nav_msgs::msg::Odometry>("/leg_odom2", qos);
  auto handle = n->create_publisher<geometry_msgs::msg::Twist>("/handle_state", qos);
  auto ultrasound =
      n->create_publisher<std_msgs::msg::Float64>("/us_publisher/ultrasound_distance", qos);
  auto depth = n->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", qos);
  auto info = n->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", qos);
  auto color = n->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", qos);
  auto color_info = n->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", qos);
  auto aligned = n->create_publisher<sensor_msgs::msg::Image>(
      "/camera/aligned_depth_to_color/image_raw", qos);
  auto aligned_info = n->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/aligned_depth_to_color/camera_info", qos);
  auto status = n->create_publisher<std_msgs::msg::Bool>("/sensor_status/imu_isalive", qos);
  sensor_msgs::msg::Imu a;
  a.header.stamp.sec = 123;
  a.header.frame_id = "imu_test";
  a.angular_velocity.x = 3.25;
  a.orientation_covariance[0] = -1;
  nav_msgs::msg::Odometry b;
  b.header = a.header;
  b.child_frame_id = "base_test";
  b.twist.twist.linear.x = .75;
  geometry_msgs::msg::Twist c;
  c.angular.z = -.25;
  std_msgs::msg::Float64 d;
  d.data = 2.5;
  std_msgs::msg::Bool live;
  live.data = true;
  sensor_msgs::msg::Image im;
  im.header = a.header;
  im.header.frame_id = "depth_test";
  im.width = 424;
  im.height = 240;
  im.encoding = "16UC1";
  im.step = 848;
  im.data.resize(im.step * im.height);
  for (size_t i = 0; i < im.data.size(); ++i)
    im.data[i] = (i * 31 + i / 253) % 256;
  sensor_msgs::msg::CameraInfo ci;
  ci.header = im.header;
  ci.width = 424;
  ci.height = 240;
  ci.k[0] = 123;
  ci.d.resize(5);
  sensor_msgs::msg::Image rgb;
  rgb.header = a.header;
  rgb.header.frame_id = "color_test";
  rgb.width = 64;
  rgb.height = 48;
  rgb.encoding = "rgb8";
  rgb.step = 192;
  rgb.data.resize(rgb.step * rgb.height);
  for (size_t i = 0; i < rgb.data.size(); ++i)
    rgb.data[i] = i % 251;
  sensor_msgs::msg::Image aligned_image = rgb;
  aligned_image.encoding = "16UC1";
  aligned_image.step = 128;
  aligned_image.data.resize(aligned_image.step * aligned_image.height);
  for (size_t i = 0; i < aligned_image.data.size(); ++i)
    aligned_image.data[i] = i % 251;
  sensor_msgs::msg::CameraInfo color_ci;
  color_ci.header = rgb.header;
  color_ci.width = rgb.width;
  color_ci.height = rgb.height;
  color_ci.k[0] = 456;
  auto end = steady_clock::now() + seconds(8);
  while (rclcpp::ok() && steady_clock::now() < end) {
    imu->publish(a);
    odom->publish(b);
    handle->publish(c);
    ultrasound->publish(d);
    depth->publish(im);
    info->publish(ci);
    color->publish(rgb);
    color_info->publish(color_ci);
    aligned->publish(aligned_image);
    aligned_info->publish(color_ci);
    status->publish(live);
    rclcpp::spin_some(n);
    std::this_thread::sleep_for(milliseconds(50));
  }
  rclcpp::shutdown();
  return 0;
}
