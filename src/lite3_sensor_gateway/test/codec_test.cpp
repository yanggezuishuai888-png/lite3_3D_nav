#include "lite3_sensor_gateway/codecs.hpp"
#include <iostream>
#include <limits>
using namespace lite3_gateway;
void check(bool x) {
  if (!x)
    throw std::runtime_error("assertion failed");
}
int main() {
  try {
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "imu";
    imu.header.stamp.sec = 15;
    imu.orientation.w = 1;
    imu.angular_velocity.x = 3.2;
    imu.orientation_covariance[0] = -1;
    imu.linear_acceleration_covariance[8] = 42;
    check(decode_imu(encode(imu)) == imu);
    nav_msgs::msg::Odometry odom;
    odom.header = imu.header;
    odom.child_frame_id = "base";
    odom.pose.pose.orientation.w = 1;
    odom.pose.covariance[35] = 8;
    odom.twist.covariance[30] = 9;
    odom.twist.twist.linear.y = -3;
    check(decode_odom(encode(odom)) == odom);
    sensor_msgs::msg::CameraInfo info;
    info.header = imu.header;
    info.width = 424;
    info.height = 240;
    info.d = {0.1, 0.2};
    info.k[0] = 100;
    info.r[8] = 1;
    info.p[11] = 3;
    info.roi.do_rectify = true;
    info.roi.x_offset = 8;
    check(decode_info(encode(info)) == info);
    sensor_msgs::msg::Image im;
    im.header = imu.header;
    im.width = 2;
    im.height = 2;
    im.step = 6;
    im.encoding = "16UC1";
    im.is_bigendian = 1;
    im.data = {0, 1, 0, 2, 0, 0, 0, 3, 0, 4, 0, 0};
    double scale = 0;
    check(decode_depth(encode_depth(im, .001), scale) == im && scale == .001);
    sensor_msgs::msg::Image color;
    color.header = imu.header;
    color.width = 640;
    color.height = 480;
    color.encoding = "rgb8";
    color.step = 1920;
    color.data.resize(color.step * color.height);
    for (size_t i = 0; i < color.data.size(); ++i)
      color.data[i] = (i * 31 + i / 253) % 256;
    check(decode_color(encode_color(color)) == color);
    color.encoding = "16UC1";
    bool rejected = false;
    try {
      encode_color(color);
    } catch (...) {
      rejected = true;
    }
    check(rejected);
    rejected = false;
    im.step = 1;
    try {
      encode_depth(im, .001);
    } catch (...) {
      rejected = true;
    }
    check(rejected);
    rejected = false;
    try {
      decode_scalar(encode_scalar(std::numeric_limits<double>::quiet_NaN()));
    } catch (...) {
      rejected = true;
    }
    check(rejected);
    rejected = false;
    auto bytes = encode(imu);
    bytes.push_back(0);
    try {
      decode_imu(bytes);
    } catch (...) {
      rejected = true;
    }
    check(rejected);
    check(decode_bool(encode_bool(true)));
    std::cout << "codec tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
