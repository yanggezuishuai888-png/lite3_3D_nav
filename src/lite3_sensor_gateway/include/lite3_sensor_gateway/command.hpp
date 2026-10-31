#pragma once
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>

namespace lite3_gateway {
inline bool valid_command(const geometry_msgs::msg::Twist &m) {
  return std::isfinite(m.linear.x) && std::abs(m.linear.x) <= 1.0 &&
         std::isfinite(m.linear.y) && std::abs(m.linear.y) <= 0.5 &&
         std::isfinite(m.angular.z) && std::abs(m.angular.z) <= 1.5 &&
         m.linear.z == 0.0 && m.angular.x == 0.0 && m.angular.y == 0.0;
}

inline bool fresh_command(uint64_t stamp_ns, uint64_t now_ns) {
  return stamp_ns && stamp_ns <= now_ns + 50000000ULL &&
         (stamp_ns >= now_ns || now_ns - stamp_ns <= 250000000ULL);
}
} // namespace lite3_gateway
