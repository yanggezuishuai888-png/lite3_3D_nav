#include "lite3_sensor_gateway/codecs.hpp"
#include "lite3_sensor_gateway/net.hpp"
#include <cmath>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
using namespace lite3_gateway;
/*
只接收指定 IP 的 UDP，检查数据包并重组图像，发布 /lite3/... ROS 话题。
监测各路数据新鲜度，生成网关诊断。
*/
namespace {
const char *status_names[] = {"imu", "odom", "odom2", "joint", "ultrasound", "realsense", "lidar"};
struct Stream {
  uint64_t sequence = 0, count = 0, gaps = 0, source = 0, sent = 0;
  double received = 0;
  ClockDomain clock = ClockDomain::Unknown;
};
class Receiver : public rclcpp::Node {
  std::unique_ptr<Socket> telemetry_, images_;
  sockaddr_in peer_{};
  Reassembler small_, large_;
  uint64_t session_ = 0, invalid_ = 0, foreign_ = 0, old_ = 0;
  std::set<uint64_t> retired_;
  std::deque<uint64_t> retired_order_;
  std::map<Type, Stream> streams_;
  bool source_time_, camera_source_time_;
  double stale_;
  std::string prefix_;
  sensor_msgs::msg::CameraInfo info_, color_info_, aligned_info_;
  bool have_info_ = false, have_color_info_ = false, have_aligned_info_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr handle_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ultrasound_, scale_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_, color_, aligned_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr calibration_, color_calibration_,
      aligned_calibration_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> statuses_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr poll_, status_timer_;
  template <class M> void timestamp(M &m, bool source) {
    if (!source)
      m.header.stamp = now();
  }
  void repeat_info(const sensor_msgs::msg::CameraInfo &info, bool have,
                   const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr &publisher,
                   const sensor_msgs::msg::Image &image) {
    if (have && info.width == image.width && info.height == image.height &&
        info.header.frame_id == image.header.frame_id) {
      auto copy = info;
      copy.header.stamp = image.header.stamp;
      publisher->publish(copy);
    }
  }
  void false_statuses() {
    std_msgs::msg::Bool b;
    b.data = false;
    for (auto &p : statuses_)
      p->publish(b);
  }
  void publish(const Message &m) {
    switch (m.env.type) {
    case Type::Heartbeat: {
      Reader r(m.payload);
      if (r.string() != "lite3_sensor_gateway/1")
        throw std::runtime_error("unknown heartbeat");
      r.finish();
      break;
    }
    case Type::Imu: {
      auto v = decode_imu(m.payload);
      timestamp(v, source_time_);
      imu_->publish(v);
      break;
    }
    case Type::Odom: {
      auto v = decode_odom(m.payload);
      timestamp(v, source_time_);
      odom_->publish(v);
      break;
    }
    case Type::Handle:
      handle_->publish(decode_twist(m.payload));
      break;
    case Type::Ultrasound: {
      std_msgs::msg::Float64 v;
      v.data = decode_scalar(m.payload);
      ultrasound_->publish(v);
      break;
    }
    case Type::CameraInfo: {
      info_ = decode_info(m.payload);
      have_info_ = true;
      auto copy = info_;
      timestamp(copy, camera_source_time_);
      calibration_->publish(copy);
      break;
    }
    case Type::ColorInfo: {
      color_info_ = decode_info(m.payload);
      have_color_info_ = true;
      auto copy = color_info_;
      timestamp(copy, camera_source_time_);
      color_calibration_->publish(copy);
      break;
    }
    case Type::AlignedInfo: {
      aligned_info_ = decode_info(m.payload);
      have_aligned_info_ = true;
      auto copy = aligned_info_;
      timestamp(copy, camera_source_time_);
      aligned_calibration_->publish(copy);
      break;
    }
    case Type::Depth: {
      double scale;
      auto v = decode_depth(m.env.flags ? decompress_payload(m.payload) : m.payload, scale);
      timestamp(v, camera_source_time_);
      repeat_info(info_, have_info_, calibration_, v);
      std_msgs::msg::Float64 units;
      units.data = scale;
      scale_->publish(units);
      depth_->publish(v);
      break;
    }
    case Type::Color: {
      auto v = decode_color(m.payload);
      timestamp(v, camera_source_time_);
      repeat_info(color_info_, have_color_info_, color_calibration_, v);
      color_->publish(v);
      break;
    }
    case Type::AlignedDepth: {
      double scale;
      auto v = decode_depth(m.env.flags ? decompress_payload(m.payload) : m.payload, scale);
      timestamp(v, camera_source_time_);
      repeat_info(aligned_info_, have_aligned_info_, aligned_calibration_, v);
      std_msgs::msg::Float64 units;
      units.data = scale;
      scale_->publish(units);
      aligned_depth_->publish(v);
      break;
    }
    default: {
      std_msgs::msg::Bool v;
      v.data = decode_bool(m.payload);
      statuses_.at(static_cast<int>(m.env.type) - 16)->publish(v);
      break;
    }
    }
  }
  void receive(Socket &socket, bool image) {
    // Bound work per executor tick so diagnostics cannot be starved by a flooded socket.
    for (int i = 0; i < 256; ++i) {
      uint8_t buffer[kDatagramSize + 1];
      sockaddr_in from{};
      socklen_t len = sizeof(from);
      auto n = recvfrom(socket.fd(), buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&from),
                        &len);
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
          ++invalid_;
        break;
      }
      if (from.sin_addr.s_addr != peer_.sin_addr.s_addr) {
        ++foreign_;
        continue;
      }
      try {
        auto f = parse_fragment(Bytes(buffer, buffer + n));
        if (image != is_image(f.env.type))
          throw std::runtime_error("wrong channel");
        // All non-image streams fit a datagram. Reject bogus huge telemetry allocations.
        if (!image && f.total > kChunkSize)
          throw std::runtime_error("oversized telemetry");
        if (f.env.session != session_) {
          if (f.env.type != Type::Heartbeat || retired_.count(f.env.session)) {
            ++old_;
            continue;
          }
          Message hello;
          if (!small_.accept(f, steady_seconds(), hello))
            continue;
          publish(hello);
          if (session_) {
            retired_.insert(session_);
            retired_order_.push_back(session_);
          }
          if (retired_order_.size() > 128) {
            retired_.erase(retired_order_.front());
            retired_order_.pop_front();
          }
          session_ = f.env.session;
          small_.clear();
          large_.clear();
          streams_.clear();
          have_info_ = false;
          have_color_info_ = false;
          have_aligned_info_ = false;
          false_statuses();
        }
        auto &s = streams_[f.env.type];
        if (f.env.sequence <= s.sequence) {
          ++old_;
          continue;
        }
        Message message;
        if (!(image ? large_ : small_).accept(f, steady_seconds(), message))
          continue;
        publish(message);
        if (s.sequence && message.env.sequence > s.sequence + 1)
          s.gaps += message.env.sequence - s.sequence - 1;
        s.sequence = message.env.sequence;
        ++s.count;
        s.source = message.env.source_ns;
        s.sent = message.env.sent_ns;
        s.clock = message.env.clock;
        s.received = steady_seconds();
      } catch (const std::exception &) {
        ++invalid_;
      }
    }
  }
  static void value(diagnostic_msgs::msg::DiagnosticStatus &s, const std::string &key,
                    const std::string &v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = v;
    s.values.push_back(kv);
  }
  void report() {
    auto t = steady_seconds();
    small_.expire(t);
    large_.expire(t);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus link;
    link.name = prefix_ + "/gateway";
    link.hardware_id = "lite3_udp";
    bool online =
        streams_.count(Type::Heartbeat) && t - streams_.at(Type::Heartbeat).received < stale_;
    link.level = online ? 1 : 2;
    link.message = online ? "UDP online; clocks/TF NOT validated" : "No recent sender heartbeat";
    value(link, "timestamp_policy",
          source_time_ ? "source (requires synchronized source clock)"
                       : "receive (not measurement time)");
    value(link, "camera_timestamp_policy",
          camera_source_time_ ? "source (preserves RGB-D pairing)" : "receive");
    value(link, "session", std::to_string(session_));
    value(link, "invalid_packets", std::to_string(invalid_));
    value(link, "foreign_packets", std::to_string(foreign_));
    value(link, "old_packets", std::to_string(old_));
    value(link, "incomplete_expired", std::to_string(small_.expired + large_.expired));
    value(link, "assembly_evicted", std::to_string(small_.evicted + large_.evicted));
    value(link, "assembly_bytes", std::to_string(small_.allocated() + large_.allocated()));
    array.status.push_back(link);
    for (int id : {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22}) {
      auto type = static_cast<Type>(id);
      auto it = streams_.find(type);
      bool fresh = it != streams_.end() && t - it->second.received < stale_;
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.name = prefix_ + "/" +
               (id >= 16 ? std::string("status_") + status_names[id - 16] : type_name(type));
      s.hardware_id = "lite3_udp";
      s.level = fresh ? 0 : 1;
      s.message = fresh ? "Received recently" : "No recent sample";
      if (it != streams_.end()) {
        auto &v = it->second;
        value(s, "age_seconds", std::to_string(t - v.received));
        value(s, "source_stamp_ns", std::to_string(v.source));
        value(s, "source_clock_domain", std::to_string(static_cast<int>(v.clock)));
        value(s, "sender_system_ns", std::to_string(v.sent));
        value(s, "received", std::to_string(v.count));
        value(s, "sequence_gaps", std::to_string(v.gaps));
      }
      array.status.push_back(s);
      if (id >= 16 && !fresh) {
        std_msgs::msg::Bool b;
        b.data = false;
        statuses_[id - 16]->publish(b);
      }
    }
    diagnostics_->publish(array);
  }

public:
  Receiver() : Node("lite3_gateway_receiver") {
    const auto bind = declare_parameter<std::string>("bind_ip", "10.42.0.1");
    const auto sender = declare_parameter<std::string>("sender_ip", "10.42.0.2");
    int tp = declare_parameter<int>("telemetry_port", 45001),
        ip = declare_parameter<int>("image_port", 45003);
    stale_ = declare_parameter<double>("stale_timeout", 2.5);
    if (tp < 1024 || ip < 1024 || tp > 65535 || ip > 65535 || tp == ip || !std::isfinite(stale_) ||
        stale_ < 1.1)
      throw std::runtime_error("invalid port/stale_timeout");
    prefix_ = declare_parameter<std::string>("output_prefix", "/lite3");
    if (prefix_.empty() || prefix_[0] != '/' || prefix_.back() == '/')
      throw std::runtime_error("output_prefix must be absolute with no trailing slash");
    auto policy = declare_parameter<std::string>("timestamp_policy", "receive");
    if (policy != "receive" && policy != "source")
      throw std::runtime_error("invalid timestamp policy");
    source_time_ = policy == "source";
    auto camera_policy = declare_parameter<std::string>("camera_timestamp_policy", "source");
    if (camera_policy != "receive" && camera_policy != "source")
      throw std::runtime_error("invalid camera timestamp policy");
    camera_source_time_ = camera_policy == "source";
    peer_ = address(sender, 0);
    telemetry_.reset(new Socket(bind, tp));
    images_.reset(new Socket(bind, ip));
    auto sensor = rclcpp::SensorDataQoS().keep_last(5);
    auto reliable = rclcpp::QoS(1).reliable();
    imu_ = create_publisher<sensor_msgs::msg::Imu>(prefix_ + "/imu/data", sensor);
    odom_ = create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/leg_odom2", sensor);
    handle_ = create_publisher<geometry_msgs::msg::Twist>(prefix_ + "/handle_state", sensor);
    ultrasound_ =
        create_publisher<std_msgs::msg::Float64>(prefix_ + "/ultrasound_distance", sensor);
    depth_ = create_publisher<sensor_msgs::msg::Image>(prefix_ + "/camera/depth/image_rect_raw",
                                                       rclcpp::SensorDataQoS().keep_last(2));
    calibration_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        prefix_ + "/camera/depth/camera_info", reliable);
    color_ = create_publisher<sensor_msgs::msg::Image>(
        prefix_ + "/camera/color/image_raw", rclcpp::SensorDataQoS().keep_last(2));
    color_calibration_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        prefix_ + "/camera/color/camera_info", reliable);
    aligned_depth_ = create_publisher<sensor_msgs::msg::Image>(
        prefix_ + "/camera/aligned_depth_to_color/image_raw",
        rclcpp::SensorDataQoS().keep_last(2));
    aligned_calibration_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        prefix_ + "/camera/aligned_depth_to_color/camera_info", reliable);
    scale_ = create_publisher<std_msgs::msg::Float64>(prefix_ + "/camera/depth/scale_m", reliable);
    for (auto name : status_names)
      statuses_.push_back(create_publisher<std_msgs::msg::Bool>(
          prefix_ + "/sensor_status/" + name + "_isalive", reliable));
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        prefix_ + "/gateway/status", rclcpp::QoS(5));
    poll_ = create_wall_timer(std::chrono::milliseconds(2), [this] {
      receive(*telemetry_, false);
      receive(*images_, true);
    });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { report(); });
    RCLCPP_WARN(
        get_logger(),
        "Read-only receiver; timestamp_policy=%s camera_timestamp_policy=%s; TF and motion-time synchronization pending",
        policy.c_str(), camera_policy.c_str());
  }
};
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Receiver>());
  } catch (const std::exception &e) {
    fprintf(stderr, "receiver: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
