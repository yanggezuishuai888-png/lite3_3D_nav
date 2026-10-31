#include "lite3_sensor_gateway/frame_trace.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 3 || argc > 6) {
    std::fprintf(stderr, "usage: image_rate_probe TOPIC SECONDS [TRACE_CSV] [QOS_DEPTH] [reliable|best_effort]\n");
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("image_rate_probe");
  using Clock = std::chrono::steady_clock;
  std::vector<int64_t> received, stamps;
  std::vector<int64_t> ages;
  auto trace = std::make_unique<lite3_gateway::FrameTrace>(argc >= 4 ? argv[3] : "");
  const int qos_depth = argc >= 5 ? std::atoi(argv[4]) : 2;
  if (qos_depth < 1 || qos_depth > 100) {
    std::fprintf(stderr, "QOS_DEPTH must be 1..100\n");
    return 2;
  }
  const auto type = std::string(argv[1]).find("aligned_depth") != std::string::npos
                        ? lite3_gateway::Type::AlignedDepth : lite3_gateway::Type::Color;
  const char *stage = std::string(argv[1]).find("/lite3/") == 0
                          ? "pc_subscriber" : "dog_ros_subscriber";
  int64_t first_age = 0, last_age = 0, min_age = std::numeric_limits<int64_t>::max();
  int64_t max_age = std::numeric_limits<int64_t>::min();
  size_t stale_stamps = 0;
  size_t invalid_ticks = 0, valid_ticks = 0;
  bool gate_seen = false, gate_valid = false;
  auto gate = node->create_subscription<std_msgs::msg::Bool>(
      "/lite3/gateway/time_valid", rclcpp::QoS(10),
      [&](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (!gate_seen || message->data != gate_valid) {
          const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch()).count();
          std::printf("gate wall=%.3f valid=%d\n", now_ns / 1e9, message->data);
          gate_seen = true;
          gate_valid = message->data;
        }
        message->data ? ++valid_ticks : ++invalid_ticks;
      });
  auto qos = rclcpp::SensorDataQoS().keep_last(qos_depth);
  if (argc == 6) {
    if (std::string(argv[5]) == "reliable")
      qos.reliable();
    else if (std::string(argv[5]) != "best_effort") {
      std::fprintf(stderr, "invalid QoS reliability\n");
      return 2;
    }
  }
  auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
      argv[1], qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr image) {
        received.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
                               Clock::now().time_since_epoch()).count());
        stamps.push_back(int64_t(image->header.stamp.sec) * 1000000000LL +
                         image->header.stamp.nanosec);
        const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch()).count();
        const auto age = wall_ns - stamps.back();
        ages.push_back(age);
        trace->record(stage, type, 0, stamps.back(), image->data.size());
        if (stamps.size() == 1)
          first_age = age;
        last_age = age;
        min_age = std::min(min_age, age);
        max_age = std::max(max_age, age);
        stale_stamps += age < -250000000 || age > 250000000;
      });
  const auto until = Clock::now() + std::chrono::seconds(std::max(1, std::atoi(argv[2])));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && Clock::now() < until)
    executor.spin_once(std::chrono::milliseconds(100));
  const auto quantile = [](std::vector<int64_t> values, double fraction) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size() - 1,
                           static_cast<size_t>(fraction * (values.size() - 1)))];
  };
  const auto report = [&](const char *label, const std::vector<int64_t> &values) {
    if (values.size() < 2) {
      std::printf("%s samples=%zu\n", label, values.size());
      return;
    }
    int64_t max_gap = 0;
    size_t over_150 = 0, over_300 = 0;
    std::vector<int64_t> gaps;
    for (size_t i = 1; i < values.size(); ++i) {
      const auto gap = values[i] - values[i - 1];
      gaps.push_back(gap);
      max_gap = std::max(max_gap, gap);
      over_150 += gap > 150000000;
      over_300 += gap > 300000000;
      if (gap > 300000000)
        std::printf("%s gap=%.1fms at=%.3f\n", label, gap / 1e6, values[i] / 1e9);
    }
    std::printf("%s samples=%zu rate=%.2fHz p99_gap=%.1fms max_gap=%.1fms over150=%zu over300=%zu\n",
                label, values.size(),
                (values.size() - 1) * 1e9 / (values.back() - values.front()),
                quantile(gaps, 0.99) / 1e6, max_gap / 1e6, over_150, over_300);
  };
  report("receive", received);
  report("header", stamps);
  std::printf("gate valid_ticks=%zu invalid_ticks=%zu\n", valid_ticks, invalid_ticks);
  if (!ages.empty()) {
    const auto midpoint = ages.size() / 2;
    std::vector<int64_t> first_half(ages.begin(), ages.begin() + midpoint);
    std::vector<int64_t> second_half(ages.begin() + midpoint, ages.end());
    std::printf("stamp_age_ms median=%.2f p95=%.2f p99=%.2f max=%.2f first_half_med=%.2f second_half_med=%.2f first=%.2f last=%.2f min=%.2f out_of_250ms=%zu\n",
                quantile(ages, 0.5) / 1e6, quantile(ages, 0.95) / 1e6,
                quantile(ages, 0.99) / 1e6, max_age / 1e6,
                quantile(first_half, 0.5) / 1e6, quantile(second_half, 0.5) / 1e6,
                first_age / 1e6, last_age / 1e6, min_age / 1e6, stale_stamps);
  }
  rclcpp::shutdown();
}
