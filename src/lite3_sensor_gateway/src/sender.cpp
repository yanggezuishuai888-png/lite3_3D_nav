#include "lite3_sensor_gateway/codecs.hpp"
#include "lite3_sensor_gateway/net.hpp"
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
        for (const auto &packet : packets) {
          if (!running_)
            break;
          if (!socket_.send(packet, destination_)) {
            ++errors;
            std::lock_guard<std::mutex> lock(mutex_);
            error_ = strerror(errno);
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
  uint64_t session_;
  double depth_scale_, depth_rate_, color_rate_;
  double last_depth_ = 0, last_aligned_ = 0, last_color_ = 0;
  ClockDomain motion_clock_, camera_clock_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_, color_calibration_, aligned_calibration_;
  std::unique_ptr<Channel> telemetry_, images_;
  ClockDomain clock_parameter(const char *name, int value) {
    int v = declare_parameter<int>(name, value);
    if (v < 0 || v > 2)
      throw std::runtime_error("clock domain must be 0 unknown, 1 steady, 2 system");
    return static_cast<ClockDomain>(v);
  }
  void queue(Type type, uint64_t stamp, ClockDomain clock, std::function<Bytes()> fn) {
    Envelope e;
    e.type = type;
    e.session = session_;
    e.source_ns = stamp;
    e.clock = clock;
    (is_image(type) ? images_ : telemetry_)->put(Job{e, std::move(fn)});
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
    camera_clock_ = clock_parameter("camera_clock_domain", 0);
    std::random_device random;
    session_ = (uint64_t(random()) << 32) ^ random() ^ system_ns();
    if (!session_)
      session_ = 1;
    telemetry_.reset(new Channel(bind, peer, tp, false, 0));
    images_.reset(
        new Channel(bind, peer, ip, declare_parameter<bool>("compress_depth", true), mbps));
    subscribe<sensor_msgs::msg::Imu>(declare_parameter<std::string>("imu_topic", "/imu/data"),
                                     [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                                       queue(Type::Imu, stamp_ns(m->header.stamp), motion_clock_,
                                             [m] { return encode(*m); });
                                     });
    subscribe<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic", "/leg_odom2"),
                                       [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                                         queue(Type::Odom, stamp_ns(m->header.stamp), motion_clock_,
                                               [m] { return encode(*m); });
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
            double t = steady_seconds();
            if (t - last_depth_ < 1.0 / depth_rate_)
              return;
            last_depth_ = t;
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
            double t = steady_seconds();
            if (t - last_color_ < 1.0 / color_rate_)
              return;
            last_color_ = t;
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
            double t = steady_seconds();
            if (t - last_aligned_ < 1.0 / depth_rate_)
              return;
            last_aligned_ = t;
            double scale = depth_scale_;
            queue(Type::AlignedDepth, stamp_ns(m->header.stamp), camera_clock_,
                  [m, scale] { return encode_depth(*m, scale); });
          });
      subscribe<sensor_msgs::msg::CameraInfo>(
          declare_parameter<std::string>("aligned_depth_info_topic",
                                         "/camera/aligned_depth_to_color/camera_info"),
          [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { aligned_calibration_ = m; });
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      queue(Type::Heartbeat, 0, ClockDomain::Unknown, [] {
        Writer w;
        w.string("lite3_sensor_gateway/1");
        return w.data;
      });
      if (calibration_) {
        auto m = calibration_;
        queue(Type::CameraInfo, stamp_ns(m->header.stamp), camera_clock_,
              [m] { return encode(*m); });
      }
      if (color_calibration_) {
        auto m = color_calibration_;
        queue(Type::ColorInfo, stamp_ns(m->header.stamp), camera_clock_,
              [m] { return encode(*m); });
      }
      if (aligned_calibration_) {
        auto m = aligned_calibration_;
        queue(Type::AlignedInfo, stamp_ns(m->header.stamp), camera_clock_,
              [m] { return encode(*m); });
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "UDP telemetry=%lu images=%lu overwritten=%lu errors=%lu last_error=%s / %s",
          telemetry_->sent.load(), images_->sent.load(),
          telemetry_->overwritten.load() + images_->overwritten.load(),
          telemetry_->errors.load() + images_->errors.load(), telemetry_->last_error().c_str(),
          images_->last_error().c_str());
    });
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
