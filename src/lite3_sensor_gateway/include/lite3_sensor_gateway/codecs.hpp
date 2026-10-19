#pragma once
#include "lite3_sensor_gateway/protocol.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
namespace lite3_gateway {
Bytes encode(const sensor_msgs::msg::Imu &m);
Bytes encode(const nav_msgs::msg::Odometry &m);
Bytes encode(const geometry_msgs::msg::Twist &m);
Bytes encode(const sensor_msgs::msg::CameraInfo &m);
Bytes encode_depth(const sensor_msgs::msg::Image &m, double scale);
Bytes encode_color(const sensor_msgs::msg::Image &m);
sensor_msgs::msg::Imu decode_imu(const Bytes &b);
nav_msgs::msg::Odometry decode_odom(const Bytes &b);
geometry_msgs::msg::Twist decode_twist(const Bytes &b);
sensor_msgs::msg::CameraInfo decode_info(const Bytes &b);
sensor_msgs::msg::Image decode_depth(const Bytes &b, double &scale);
sensor_msgs::msg::Image decode_color(const Bytes &b);
Bytes encode_scalar(double x);
double decode_scalar(const Bytes &b);
Bytes encode_bool(bool x);
bool decode_bool(const Bytes &b);
bool is_camera_internal_tf(const geometry_msgs::msg::TransformStamped &transform);
Bytes encode_camera_tf(const tf2_msgs::msg::TFMessage &m);
tf2_msgs::msg::TFMessage decode_camera_tf(const Bytes &b);
uint64_t stamp_ns(const builtin_interfaces::msg::Time &stamp);
} // namespace lite3_gateway
