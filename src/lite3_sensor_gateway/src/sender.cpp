#include "lite3_sensor_gateway/codecs.hpp"
#include "lite3_sensor_gateway/net.hpp"
#include "lite3_sensor_gateway/time_sync.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
using namespace lite3_gateway;

/*
狗端 ROS 节点。订阅 IMU、里程计、超声波、遥控状态、存活状态、
RGB、深度图和相机内参；通过两个 UDP 端口发送。图像限速、深度可压缩；
待发送数据只保留最新样本。没有速度命令输出
*/
namespace {
const char *status_names[] = {"imu", "odom", "odom2", "joint", "ultrasound", "realsense", "lidar"};
struct Job {
  Envelope env;
  std::function<Bytes()> encode;
};
struct RateLimiter {
  double tokens = 2.0, last = steady_seconds();
  bool admit(double rate) {
    const double now = steady_seconds();
    tokens = std::min(2.0, tokens + (now - last) * rate);
    last = now;
    if (tokens < 1.0)
      return false;
    tokens -= 1.0;
    return true;
  }
};
// A single latest sample per stream: slow links cannot grow the queue indefinitely.
class Channel {
  Socket socket_;
  sockaddr_in destination_;
  bool compress_;
  double bytes_per_second_;
  std::string error_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::map<Type, Job> pending_;
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::map<Type, uint64_t> sequences_;
  Type last_type_ = Type::Heartbeat;
  void run() {
    while (running_) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return !running_ || !pending_.empty(); });
        if (!running_)
          break;
        auto it = pending_.upper_bound(last_type_);
        if (it == pending_.end())
          it = pending_.begin();
        last_type_ = it->first;
        job = std::move(it->second);
        pending_.erase(it);
      }
      try {
        auto data = job.encode();
        if (compress_ && is_depth(job.env.type)) {
          auto compressed = compress_payload(data);
          if (compressed.size() < data.size()) {
            data = std::move(compressed);
            job.env.flags = 1;
          }
        }
        job.env.sequence = ++sequences_[job.env.type];
        job.env.sent_ns = system_ns();
        auto packets = fragment(job.env, data);
        auto deadline = std::chrono::steady_clock::now();
        const size_t burst_bytes = bytes_per_second_ > 0
                                       ? std::clamp<size_t>(bytes_per_second_ * 0.002,
                                                            kDatagramSize, 64 * 1024)
                                       : 0;
        size_t bytes_since_pause = 0;
        bool complete = true;
        for (const auto &packet : packets) {
          if (!running_) {
            complete = false;
            break;
          }
          bool delivered = false;
          int send_error = ECANCELED;
          for (int attempt = 0; attempt < 50 && running_; ++attempt) {
            if (socket_.send(packet, destination_)) {
              delivered = true;
              break;
            }
            send_error = errno;
            if (send_error != EAGAIN && send_error != EWOULDBLOCK)
              break;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
          }
          if (!delivered) {
            ++errors;
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = strerror(send_error);
            complete = false;
            break;
          }
          if (bytes_per_second_ > 0) {
            deadline += std::chrono::nanoseconds(
                static_cast<int64_t>(packet.size() * 1e9 / bytes_per_second_));
            bytes_since_pause += packet.size();
            if (bytes_since_pause >= burst_bytes) {
              std::unique_lock<std::mutex> lock(mutex_);
              ready_.wait_until(lock, deadline, [this] { return !running_; });
              bytes_since_pause = 0;
            }
          }
        }
        if (complete)
          ++sent;
      } catch (const std::exception &e) {
        ++errors;
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = e.what();
      }
    }
  }

public:
  std::atomic<uint64_t> overwritten{0}, sent{0}, errors{0};
  Channel(const std::string &bind, const std::string &peer, int port, bool compress, double mbps)
      : socket_(bind, 0), destination_(address(peer, port)), compress_(compress),
        bytes_per_second_(mbps * 1e6 / 8) {
    thread_ = std::thread([this] { run(); });
  }
  ~Channel() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
    }
    ready_.notify_all();
    thread_.join();
  }
  std::string last_error() {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }
  void put(Job job) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.count(job.env.type))
      ++overwritten;
    pending_[job.env.type] = std::move(job);
    ready_.notify_one();
  }
};
class Sender : public rclcpp::Node {
  // Channels are declared last so workers join before the rest of the node is destroyed.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr probe_timer_;
  uint64_t session_;
  double depth_scale_, depth_rate_, color_rate_;
  RateLimiter depth_limiter_, aligned_limiter_, color_limiter_;
  ClockDomain motion_clock_, camera_clock_;
  rclcpp::Clock motion_stamp_clock_{RCL_STEADY_TIME};
  TimeMapper mapper_;
  PeerClock peer_clock_;
  std::unique_ptr<Socket> probe_;
  sockaddr_in probe_peer_{};
  uint64_t probe_sequence_ = 0, pending_probe_ = 0, pending_t1_ = 0;
  uint64_t probes_sent_ = 0, probes_received_ = 0, probes_rejected_ = 0;
  double last_probe_ = 0, last_report_ = 0;
  bool clock_step_latched_ = false;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_, color_calibration_, aligned_calibration_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> camera_transforms_;
  std::unique_ptr<Channel> telemetry_, images_;
  ClockDomain clock_parameter(const char *name, int value) {
    int v = declare_parameter<int>(name, value);
    if (v < 0 || v > 2)
      throw std::runtime_error("clock domain must be 0 unknown, 1 steady, 2 system");
    return static_cast<ClockDomain>(v);
  }
  void queue(Type type, uint64_t stamp, ClockDomain clock, std::function<Bytes()> fn,
             uint64_t raw_stamp = 0) {
    Envelope e;
    e.type = type;
    e.session = session_;
    e.source_ns = stamp;
    e.raw_source_ns = raw_stamp;
    e.clock = clock;
    (is_image(type) ? images_ : telemetry_)->put(Job{e, std::move(fn)});
  }
  bool time_ready() {
    auto t = std::chrono::steady_clock::now().time_since_epoch();
    auto peer_steady = std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
    return !clock_step_latched_ &&
           mapper_.fresh(motion_stamp_clock_.now().nanoseconds()) &&
           peer_clock_.qualified(peer_steady);
  }
  bool camera_stamp_valid(uint64_t stamp) const {
    const auto wall = system_ns();
    return stamp && stamp <= wall + 250000000ULL &&
           (stamp >= wall || wall - stamp <= 250000000ULL);
  }
  void poll_probe() {
    for (int i = 0; i < 8; ++i) {
      uint8_t buffer[kDatagramSize + 1];
      sockaddr_in from{};
      socklen_t len = sizeof(from);
      auto n = recvfrom(probe_->fd(), buffer, sizeof(buffer), 0,
                        reinterpret_cast<sockaddr *>(&from), &len);
      if (n < 0)
        break;
      if (from.sin_addr.s_addr != probe_peer_.sin_addr.s_addr ||
          from.sin_port != probe_peer_.sin_port)
        continue;
      const auto t4 = system_ns();
      const auto steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch()).count();
      try {
        auto f = parse_fragment(Bytes(buffer, buffer + n));
        if (f.env.type != Type::TimeResponse || checksum(f.data) != f.crc ||
            f.env.session != session_ ||
            f.env.sequence != pending_probe_ || f.count != 1 ||
            f.total != f.data.size())
          continue;
        Reader r(f.data);
        auto t1 = r.u64(), t2 = r.u64(), t3 = r.u64();
        r.finish();
        if (t1 != pending_t1_ || t1 != f.env.source_ns)
          continue;
        pending_probe_ = 0;
        if (peer_clock_.observe(t1, t2, t3, t4, steady))
          ++probes_received_;
        else {
          ++probes_rejected_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "time probe rejected: t2-t1=%lldus t3-t2=%lldus t4-t3=%lldus",
                               static_cast<long long>((static_cast<int64_t>(t2) - static_cast<int64_t>(t1)) / 1000),
                               static_cast<long long>((static_cast<int64_t>(t3) - static_cast<int64_t>(t2)) / 1000),
                               static_cast<long long>((static_cast<int64_t>(t4) - static_cast<int64_t>(t3)) / 1000));
        }
      } catch (const std::exception &) {
      }
    }
  }
  void send_probe() {
    Envelope e;
    e.type = Type::TimeRequest;
    e.session = session_;
    e.sequence = ++probe_sequence_;
    e.clock = ClockDomain::System;
    e.source_ns = pending_t1_ = system_ns();
    e.sent_ns = pending_t1_;
    pending_probe_ = e.sequence;
    Writer w;
    w.u64(pending_t1_);
    if (probe_->send(fragment(e, w.data).front(), probe_peer_))
      ++probes_sent_;
  }
  void clock_tick() {
    const auto before = motion_stamp_clock_.now().nanoseconds();
    const auto wall = system_ns();
    const auto after = motion_stamp_clock_.now().nanoseconds();
    if (mapper_.update(before, wall, after)) {
      clock_step_latched_ = true;
      peer_clock_.reset();
      RCLCPP_ERROR(get_logger(), "System clock stepped; restart camera and gateway after time sync");
    }
    auto t = steady_seconds();
    if (t - last_probe_ >= 0.5) {
      send_probe();
      last_probe_ = t;
    }
    if (t - last_report_ >= 1.0) {
      queue(Type::Heartbeat, 0, ClockDomain::Unknown, [] {
        Writer w;
        w.string("lite3_sensor_gateway/2");
        return w.data;
      });
      ClockStatusData status;
      status.qualified = time_ready();
      status.peer_offset_ns = peer_clock_.offset_ns();
      status.peer_bound_ns = peer_clock_.bound_ns();
      status.peer_rtt_ns = peer_clock_.rtt_ns();
      status.mapping_uncertainty_ns = mapper_.uncertainty_ns();
      const auto peer_steady = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch()).count();
      status.probe_age_ns = peer_clock_.age_ns(peer_steady);
      status.clock_jumps = mapper_.jumps();
      queue(Type::ClockStatus, wall, ClockDomain::System,
            [status] { return encode_clock_status(status); });
      if (camera_transforms_.size() == 4) {
        tf2_msgs::msg::TFMessage transforms;
        for (const auto &entry : camera_transforms_)
          transforms.transforms.push_back(entry.second);
        queue(Type::CameraTf, 0, ClockDomain::Unknown,
              [transforms] { return encode_camera_tf(transforms); });
      }
      if (status.qualified) {
        if (calibration_) {
          auto m = calibration_;
          queue(Type::CameraInfo, stamp_ns(m->header.stamp), ClockDomain::System,
                [m] { return encode(*m); });
        }
        if (color_calibration_) {
          auto m = color_calibration_;
          queue(Type::ColorInfo, stamp_ns(m->header.stamp), ClockDomain::System,
                [m] { return encode(*m); });
        }
        if (aligned_calibration_) {
          auto m = aligned_calibration_;
          queue(Type::AlignedInfo, stamp_ns(m->header.stamp), ClockDomain::System,
                [m] { return encode(*m); });
        }
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
          "time_valid=%d offset_ms=%.3f bound_ms=%.3f probes=%lu/%lu rejected=%lu jumps=%lu UDP telemetry=%lu images=%lu overwritten=%lu errors=%lu",
          status.qualified, status.peer_offset_ns / 1e6, status.peer_bound_ns / 1e6,
          probes_received_, probes_sent_, probes_rejected_, status.clock_jumps,
          telemetry_->sent.load(), images_->sent.load(),
          telemetry_->overwritten.load() + images_->overwritten.load(),
          telemetry_->errors.load() + images_->errors.load());
      last_report_ = t;
    }
  }
  template <class M> void queue_motion(Type type, typename M::ConstSharedPtr m) {
    if (!time_ready())
      return;
    const auto raw = stamp_ns(m->header.stamp);
    const auto steady = motion_stamp_clock_.now().nanoseconds();
    uint64_t mapped;
    if (!mapper_.convert(raw, steady, mapped))
      return;
    auto copy = *m;
    copy.header.stamp = rclcpp::Time(mapped, RCL_SYSTEM_TIME);
    queue(type, mapped, ClockDomain::System, [copy] { return encode(copy); }, raw);
  }
  template <class M, class F> void subscribe(const std::string &topic, F fn) {
    subscriptions_.push_back(
        create_subscription<M>(topic, rclcpp::SensorDataQoS().keep_last(2), fn));
  }

public:
  Sender() : Node("lite3_gateway_sender") {
    const auto bind = declare_parameter<std::string>("bind_ip", "10.42.0.2"),
               peer = declare_parameter<std::string>("pc_ip", "10.42.0.1");
    const int tp = declare_parameter<int>("telemetry_port", 45001),
              ip = declare_parameter<int>("image_port", 45003);
    const double mbps = declare_parameter<double>("image_max_mbps", 30.0);
    depth_rate_ = declare_parameter<double>("depth_max_hz", 15.0);
    color_rate_ = declare_parameter<double>("color_max_hz", 15.0);
    depth_scale_ = declare_parameter<double>("depth_scale_m", 0.001);
    if (tp < 1024 || ip < 1024 || tp > 65535 || ip > 65535 || tp == ip || !std::isfinite(mbps) ||
        mbps <= 0 || !std::isfinite(depth_rate_) || depth_rate_ <= 0 ||
        !std::isfinite(color_rate_) || color_rate_ <= 0 ||
        !std::isfinite(depth_scale_) || depth_scale_ <= 0)
      throw std::runtime_error("invalid gateway port/rate/scale");
    motion_clock_ = clock_parameter("motion_clock_domain", 1);
    camera_clock_ = clock_parameter("camera_clock_domain", 2);
    if (motion_clock_ != ClockDomain::Steady || camera_clock_ != ClockDomain::System)
      throw std::runtime_error("time sync requires steady motion and system camera stamps");
    std::random_device random;
    session_ = (uint64_t(random()) << 32) ^ random() ^ system_ns();
    if (!session_)
      session_ = 1;
    telemetry_.reset(new Channel(bind, peer, tp, false, 0));
    probe_.reset(new Socket(bind, 0));
    probe_peer_ = address(peer, tp);
    images_.reset(
        new Channel(bind, peer, ip, declare_parameter<bool>("compress_depth", true), mbps));
    const auto camera_tf_topic = declare_parameter<std::string>("camera_tf_topic", "/tf_static");
    subscriptions_.push_back(create_subscription<tf2_msgs::msg::TFMessage>(
        camera_tf_topic, rclcpp::QoS(10).reliable().transient_local(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          for (const auto &transform : m->transforms) {
            if (is_camera_internal_tf(transform))
              camera_transforms_[transform.child_frame_id] = transform;
          }
        }));
    subscribe<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic", "/imu/data"),
                                     [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                                       queue_motion<sensor_msgs::msg::Imu>(Type::Imu, m);
                                     });
    subscribe<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic", "/leg_odom2"),
                                       [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                                         queue_motion<nav_msgs::msg::Odometry>(Type::Odom, m);
                                       });
    subscribe<std_msgs::msg::Float64>(
        declare_parameter<std::string>("ultrasound_topic", "/us_publisher/ultrasound_distance"),
        [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          queue(Type::Ultrasound, 0, ClockDomain::Unknown, [m] { return encode_scalar(m->data); });
        });
    subscribe<geometry_msgs::msg::Twist>(
        declare_parameter<std::string>("handle_topic", "/handle_state"),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          queue(Type::Handle, 0, ClockDomain::Unknown, [m] { return encode(*m); });
        });
    auto prefix = declare_parameter<std::string>("status_prefix", "/sensor_status");
    for (int i = 0; i < 7; ++i) {
      auto type = static_cast<Type>(16 + i);
      subscribe<std_msgs::msg::Bool>(prefix + "/" + status_names[i] + "_isalive",
                                     [this, type](std_msgs::msg::Bool::ConstSharedPtr m) {
                                       queue(type, 0, ClockDomain::Unknown,
                                             [m] { return encode_bool(m->data); });
                                     });
    }
    if (declare_parameter<bool>("enable_depth", true)) {
      subscribe<sensor_msgs::msg::Image>(
          declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw"),
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
            if (!time_ready() || !camera_stamp_valid(stamp_ns(m->header.stamp)) ||
                !depth_limiter_.admit(depth_rate_))
              return;
            double scale = depth_scale_;
            queue(Type::Depth, stamp_ns(m->header.stamp), camera_clock_,
                  [m, scale] { return encode_depth(*m, scale); });
          });
      subscribe<sensor_msgs::msg::CameraInfo>(
          declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info"),
          [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { calibration_ = m; });
    }
    if (declare_parameter<bool>("enable_color", false)) {
      subscribe<sensor_msgs::msg::Image>(
          declare_parameter<std::string>("color_topic", "/camera/color/image_raw"),
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
            if (!time_ready() || !camera_stamp_valid(stamp_ns(m->header.stamp)) ||
                !color_limiter_.admit(color_rate_))
              return;
            queue(Type::Color, stamp_ns(m->header.stamp), camera_clock_,
                  [m] { return encode_color(*m); });
          });
      subscribe<sensor_msgs::msg::CameraInfo>(
          declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info"),
          [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { color_calibration_ = m; });
    }
    if (declare_parameter<bool>("enable_aligned_depth", false)) {
      subscribe<sensor_msgs::msg::Image>(
          declare_parameter<std::string>("aligned_depth_topic",
                                         "/camera/aligned_depth_to_color/image_raw"),
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
            if (!time_ready() || !camera_stamp_valid(stamp_ns(m->header.stamp)) ||
                !aligned_limiter_.admit(depth_rate_))
              return;
            double scale = depth_scale_;
            queue(Type::AlignedDepth, stamp_ns(m->header.stamp), camera_clock_,
                  [m, scale] { return encode_depth(*m, scale); });
          });
      subscribe<sensor_msgs::msg::CameraInfo>(
          declare_parameter<std::string>("aligned_depth_info_topic",
                                         "/camera/aligned_depth_to_color/camera_info"),
          [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { aligned_calibration_ = m; });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { clock_tick(); });
    probe_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this] { poll_probe(); });
    RCLCPP_INFO(get_logger(), "Read-only gateway: %s -> %s UDP %d/%d; no control output",
                bind.c_str(), peer.c_str(), tp, ip);
  }
};
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Sender>());
  } catch (const std::exception &e) {
    fprintf(stderr, "sender: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
