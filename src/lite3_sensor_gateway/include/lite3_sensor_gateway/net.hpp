#pragma once
#include "lite3_sensor_gateway/protocol.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
namespace lite3_gateway {
inline double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t system_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
inline sockaddr_in address(const std::string &ip, int port) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  if (port < 0 || port > 65535 || inet_pton(AF_INET, ip.c_str(), &a.sin_addr) != 1)
    throw std::runtime_error("invalid IPv4 address/port");
  a.sin_port = htons(port);
  return a;
}
class Socket {
  int fd_ = -1;

public:
  Socket(const std::string &ip, int port) {
    auto a = address(ip, port);
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
      throw std::runtime_error("socket failed");
    int size = 4 * 1024 * 1024;
    setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
    if (bind(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0 ||
        fcntl(fd_, F_SETFL, O_NONBLOCK) < 0) {
      int e = errno;
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("UDP bind/nonblock failed: " + std::string(strerror(e)));
    }
  }
  ~Socket() {
    if (fd_ >= 0)
      close(fd_);
  }
  Socket(const Socket &) = delete;
  Socket &operator=(const Socket &) = delete;
  int fd() const { return fd_; }
  bool send(const Bytes &b, const sockaddr_in &a) {
    return sendto(fd_, b.data(), b.size(), 0, reinterpret_cast<const sockaddr *>(&a), sizeof(a)) ==
           static_cast<ssize_t>(b.size());
  }
};
} // namespace lite3_gateway
