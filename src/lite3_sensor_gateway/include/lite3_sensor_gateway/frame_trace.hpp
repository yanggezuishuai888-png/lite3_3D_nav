#pragma once
#include "lite3_sensor_gateway/protocol.hpp"
#include <chrono>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace lite3_gateway {
class FrameTrace {
  std::FILE *file_ = nullptr;
  std::mutex mutex_;

public:
  explicit FrameTrace(const std::string &path) {
    if (path.empty())
      return;
    file_ = std::fopen(path.c_str(), "w");
    if (!file_)
      throw std::runtime_error("cannot open image trace: " + path);
    std::setvbuf(file_, nullptr, _IOFBF, 256 * 1024);
    std::fprintf(file_, "stage,type,sequence,stamp_ns,wall_ns,steady_ns,bytes\n");
  }
  ~FrameTrace() {
    if (file_)
      std::fclose(file_);
  }
  FrameTrace(const FrameTrace &) = delete;
  FrameTrace &operator=(const FrameTrace &) = delete;
  void record(const char *stage, Type type, uint64_t sequence, uint64_t stamp, size_t bytes) {
    if (!file_)
      return;
    const auto wall = std::chrono::system_clock::now().time_since_epoch();
    const auto steady = std::chrono::steady_clock::now().time_since_epoch();
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(steady).count();
    std::lock_guard<std::mutex> lock(mutex_);
    std::fprintf(file_, "%s,%u,%llu,%llu,%lld,%lld,%zu\n", stage,
                 static_cast<unsigned>(type), static_cast<unsigned long long>(sequence),
                 static_cast<unsigned long long>(stamp), static_cast<long long>(wall_ns),
                 static_cast<long long>(steady_ns), bytes);
  }
};
} // namespace lite3_gateway
