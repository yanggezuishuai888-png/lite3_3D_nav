#include "lite3_sensor_gateway/codecs.hpp"
#include <cmath>
#include <set>
/*
在 ROS 消息与网关协议字段之间转换，保留消息数据和协方差；验证深度图、内参和数值格式。
*/
namespace lite3_gateway {
namespace {
double finite(Reader &r) {
  double x = r.f64();
  if (!std::isfinite(x))
    throw std::runtime_error("non-finite field");
  return x;
}
void header(Writer &w, const std_msgs::msg::Header &h) {
  w.u32(static_cast<uint32_t>(h.stamp.sec));
  w.u32(h.stamp.nanosec);
  w.string(h.frame_id);
}
void header(Reader &r, std_msgs::msg::Header &h) {
  auto sec = r.u32();
  h.stamp.sec = sec <= 0x7fffffff ? static_cast<int32_t>(sec)
                                  : static_cast<int32_t>(int64_t(sec) - 0x100000000LL);
  h.stamp.nanosec = r.u32();
  if (h.stamp.nanosec >= 1000000000)
    throw std::runtime_error("invalid nanoseconds");
  h.frame_id = r.string();
}
template <class V> void xyz(Writer &w, const V &v) {
  w.f64(v.x);
  w.f64(v.y);
  w.f64(v.z);
}
template <class V> void xyz(Reader &r, V &v) {
  v.x = finite(r);
  v.y = finite(r);
  v.z = finite(r);
}
template <class A> void array(Writer &w, const A &a) {
  for (double x : a)
    w.f64(x);
}
template <class A> void array(Reader &r, A &a) {
  for (double &x : a)
    x = finite(r);
}
void quat(Writer &w, const geometry_msgs::msg::Quaternion &q) {
  xyz(w, q);
  w.f64(q.w);
}
void quat(Reader &r, geometry_msgs::msg::Quaternion &q) {
  xyz(r, q);
  q.w = finite(r);
}
void validate_image(const sensor_msgs::msg::Image &m, double scale) {
  if (!m.width || !m.height || m.width > 8192 || m.height > 8192 || m.encoding != "16UC1" ||
      m.is_bigendian > 1 || m.step < uint64_t(m.width) * 2 ||
      uint64_t(m.step) * m.height != m.data.size() || m.data.size() > kMaxMessage - 8192 ||
      !std::isfinite(scale) || scale <= 0)
    throw std::runtime_error("invalid depth image; expected bounded 16UC1");
}
void validate_color(const sensor_msgs::msg::Image &m) {
  if (!m.width || !m.height || m.width > 8192 || m.height > 8192 ||
      (m.encoding != "rgb8" && m.encoding != "bgr8") || m.is_bigendian > 1 ||
      m.step < uint64_t(m.width) * 3 || uint64_t(m.step) * m.height != m.data.size() ||
      m.data.size() > kMaxMessage - 8192)
    throw std::runtime_error("invalid color image; expected bounded rgb8/bgr8");
}
} // namespace
uint64_t stamp_ns(const builtin_interfaces::msg::Time &s) {
  return s.sec < 0 ? 0 : uint64_t(s.sec) * 1000000000ULL + s.nanosec;
}
Bytes encode(const sensor_msgs::msg::Imu &m) {
  Writer w;
  header(w, m.header);
  quat(w, m.orientation);
  array(w, m.orientation_covariance);
  xyz(w, m.angular_velocity);
  array(w, m.angular_velocity_covariance);
  xyz(w, m.linear_acceleration);
  array(w, m.linear_acceleration_covariance);
  return w.data;
}
sensor_msgs::msg::Imu decode_imu(const Bytes &b) {
  Reader r(b);
  sensor_msgs::msg::Imu m;
  header(r, m.header);
  quat(r, m.orientation);
  array(r, m.orientation_covariance);
  xyz(r, m.angular_velocity);
  array(r, m.angular_velocity_covariance);
  xyz(r, m.linear_acceleration);
  array(r, m.linear_acceleration_covariance);
  r.finish();
  return m;
}
Bytes encode(const nav_msgs::msg::Odometry &m) {
  Writer w;
  header(w, m.header);
  w.string(m.child_frame_id);
  xyz(w, m.pose.pose.position);
  quat(w, m.pose.pose.orientation);
  array(w, m.pose.covariance);
  xyz(w, m.twist.twist.linear);
  xyz(w, m.twist.twist.angular);
  array(w, m.twist.covariance);
  return w.data;
}
nav_msgs::msg::Odometry decode_odom(const Bytes &b) {
  Reader r(b);
  nav_msgs::msg::Odometry m;
  header(r, m.header);
  m.child_frame_id = r.string();
  xyz(r, m.pose.pose.position);
  quat(r, m.pose.pose.orientation);
  array(r, m.pose.covariance);
  xyz(r, m.twist.twist.linear);
  xyz(r, m.twist.twist.angular);
  array(r, m.twist.covariance);
  r.finish();
  return m;
}
Bytes encode(const geometry_msgs::msg::Twist &m) {
  Writer w;
  xyz(w, m.linear);
  xyz(w, m.angular);
  return w.data;
}
geometry_msgs::msg::Twist decode_twist(const Bytes &b) {
  Reader r(b);
  geometry_msgs::msg::Twist m;
  xyz(r, m.linear);
  xyz(r, m.angular);
  r.finish();
  return m;
}
Bytes encode_scalar(double x) {
  Writer w;
  w.f64(x);
  return w.data;
}
double decode_scalar(const Bytes &b) {
  Reader r(b);
  auto v = finite(r);
  r.finish();
  return v;
}
Bytes encode_bool(bool x) {
  return Bytes{uint8_t(x)};
}
bool decode_bool(const Bytes &b) {
  if (b.size() != 1 || b[0] > 1)
    throw std::runtime_error("invalid bool");
  return b[0] != 0;
}
Bytes encode(const sensor_msgs::msg::CameraInfo &m) {
  if (m.d.size() > 64)
    throw std::runtime_error("too many distortion coefficients");
  Writer w;
  header(w, m.header);
  w.u32(m.height);
  w.u32(m.width);
  w.string(m.distortion_model);
  w.u32(m.d.size());
  array(w, m.d);
  array(w, m.k);
  array(w, m.r);
  array(w, m.p);
  w.u32(m.binning_x);
  w.u32(m.binning_y);
  w.u32(m.roi.x_offset);
  w.u32(m.roi.y_offset);
  w.u32(m.roi.height);
  w.u32(m.roi.width);
  w.u8(m.roi.do_rectify);
  return w.data;
}
sensor_msgs::msg::CameraInfo decode_info(const Bytes &b) {
  Reader r(b);
  sensor_msgs::msg::CameraInfo m;
  header(r, m.header);
  m.height = r.u32();
  m.width = r.u32();
  if (m.width > 8192 || m.height > 8192)
    throw std::runtime_error("invalid calibration dimensions");
  m.distortion_model = r.string();
  auto count = r.u32();
  if (count > 64)
    throw std::runtime_error("invalid calibration array");
  m.d.resize(count);
  array(r, m.d);
  array(r, m.k);
  array(r, m.r);
  array(r, m.p);
  m.binning_x = r.u32();
  m.binning_y = r.u32();
  m.roi.x_offset = r.u32();
  m.roi.y_offset = r.u32();
  m.roi.height = r.u32();
  m.roi.width = r.u32();
  auto flag = r.u8();
  if (flag > 1)
    throw std::runtime_error("invalid ROI flag");
  m.roi.do_rectify = flag;
  r.finish();
  return m;
}
Bytes encode_depth(const sensor_msgs::msg::Image &m, double scale) {
  validate_image(m, scale);
  Writer w;
  header(w, m.header);
  w.u32(m.height);
  w.u32(m.width);
  w.string(m.encoding);
  w.u8(m.is_bigendian);
  w.u32(m.step);
  w.f64(scale);
  w.bytes(m.data);
  return w.data;
}
sensor_msgs::msg::Image decode_depth(const Bytes &b, double &scale) {
  Reader r(b);
  sensor_msgs::msg::Image m;
  header(r, m.header);
  m.height = r.u32();
  m.width = r.u32();
  m.encoding = r.string();
  m.is_bigendian = r.u8();
  m.step = r.u32();
  scale = finite(r);
  m.data = r.bytes();
  r.finish();
  validate_image(m, scale);
  return m;
}
Bytes encode_color(const sensor_msgs::msg::Image &m) {
  validate_color(m);
  Writer w;
  header(w, m.header);
  w.u32(m.height);
  w.u32(m.width);
  w.string(m.encoding);
  w.u8(m.is_bigendian);
  w.u32(m.step);
  w.bytes(m.data);
  return w.data;
}
sensor_msgs::msg::Image decode_color(const Bytes &b) {
  Reader r(b);
  sensor_msgs::msg::Image m;
  header(r, m.header);
  m.height = r.u32();
  m.width = r.u32();
  m.encoding = r.string();
  m.is_bigendian = r.u8();
  m.step = r.u32();
  m.data = r.bytes();
  r.finish();
  validate_color(m);
  return m;
}
bool is_camera_internal_tf(const geometry_msgs::msg::TransformStamped &t) {
  return (t.header.frame_id == "camera_link" &&
          (t.child_frame_id == "camera_depth_frame" ||
           t.child_frame_id == "camera_color_frame")) ||
         (t.header.frame_id == "camera_depth_frame" &&
          t.child_frame_id == "camera_depth_optical_frame") ||
         (t.header.frame_id == "camera_color_frame" &&
          t.child_frame_id == "camera_color_optical_frame");
}
namespace {
void validate_camera_tf(const geometry_msgs::msg::TransformStamped &t) {
  const auto &v = t.transform.translation;
  const auto &q = t.transform.rotation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!is_camera_internal_tf(t) || !std::isfinite(v.x) || !std::isfinite(v.y) ||
      !std::isfinite(v.z) || !std::isfinite(norm) || norm < 0.9 || norm > 1.1)
    throw std::runtime_error("invalid camera internal TF");
}
} // namespace
Bytes encode_camera_tf(const tf2_msgs::msg::TFMessage &m) {
  if (m.transforms.empty() || m.transforms.size() > 4)
    throw std::runtime_error("invalid camera TF count");
  Writer w;
  w.u8(m.transforms.size());
  std::set<std::string> children;
  for (const auto &t : m.transforms) {
    validate_camera_tf(t);
    if (!children.insert(t.child_frame_id).second)
      throw std::runtime_error("duplicate camera TF child");
    w.string(t.header.frame_id);
    w.string(t.child_frame_id);
    xyz(w, t.transform.translation);
    quat(w, t.transform.rotation);
  }
  return w.data;
}
tf2_msgs::msg::TFMessage decode_camera_tf(const Bytes &b) {
  Reader r(b);
  const auto count = r.u8();
  if (!count || count > 4)
    throw std::runtime_error("invalid camera TF count");
  tf2_msgs::msg::TFMessage m;
  std::set<std::string> children;
  for (unsigned i = 0; i < count; ++i) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = r.string();
    t.child_frame_id = r.string();
    xyz(r, t.transform.translation);
    quat(r, t.transform.rotation);
    validate_camera_tf(t);
    if (!children.insert(t.child_frame_id).second)
      throw std::runtime_error("duplicate camera TF child");
    m.transforms.push_back(std::move(t));
  }
  r.finish();
  return m;
}
} // namespace lite3_gateway
