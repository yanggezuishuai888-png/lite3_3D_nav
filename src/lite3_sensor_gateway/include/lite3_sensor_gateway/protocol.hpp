#pragma once
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace lite3_gateway {
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "Wire format requires IEEE-754 binary64");
using Bytes = std::vector<uint8_t>;
constexpr size_t kHeaderSize = 56, kDatagramSize = 1200, kChunkSize = kDatagramSize - kHeaderSize;
constexpr size_t kMaxMessage = 16 * 1024 * 1024;
enum class Type : uint8_t {
  Heartbeat = 1,
  Imu = 2,
  Odom = 3,
  Ultrasound = 4,
  Handle = 5,
  Depth = 6,
  CameraInfo = 7,
  Color = 8,
  AlignedDepth = 9,
  ColorInfo = 10,
  AlignedInfo = 11,
  StatusImu = 16,
  StatusOdom = 17,
  StatusOdom2 = 18,
  StatusJoint = 19,
  StatusUltrasound = 20,
  StatusCamera = 21,
  StatusLidar = 22
};
inline bool is_image(Type t) {
  return t == Type::Depth || t == Type::Color || t == Type::AlignedDepth;
}
inline bool is_depth(Type t) { return t == Type::Depth || t == Type::AlignedDepth; }
enum class ClockDomain : uint8_t { Unknown = 0, Steady = 1, System = 2 };
bool valid_type(Type t);
const char *type_name(Type t);
class Writer {
public:
  Bytes data;
  void u8(uint8_t x) { data.push_back(x); }
  void u16(uint16_t x) {
    u8(x >> 8);
    u8(x);
  }
  void u32(uint32_t x) {
    u16(x >> 16);
    u16(x);
  }
  void u64(uint64_t x) {
    u32(x >> 32);
    u32(x);
  }
  void f64(double x) {
    uint64_t b;
    std::memcpy(&b, &x, 8);
    u64(b);
  }
  void string(const std::string &s) {
    if (s.size() > 4096)
      throw std::runtime_error("string too long");
    u32(s.size());
    data.insert(data.end(), s.begin(), s.end());
  }
  void bytes(const Bytes &v) {
    u32(v.size());
    data.insert(data.end(), v.begin(), v.end());
  }
};
class Reader {
  const Bytes &b_;
  size_t pos_ = 0;

public:
  explicit Reader(const Bytes &b) : b_(b) {}
  size_t remaining() const { return b_.size() - pos_; }
  uint8_t u8() {
    if (!remaining())
      throw std::runtime_error("truncated payload");
    return b_[pos_++];
  }
  uint16_t u16() {
    uint16_t x = u8();
    return (x << 8) | u8();
  }
  uint32_t u32() {
    uint32_t x = u16();
    return (x << 16) | u16();
  }
  uint64_t u64() {
    uint64_t x = u32();
    return (x << 32) | u32();
  }
  double f64() {
    uint64_t b = u64();
    double x;
    std::memcpy(&x, &b, 8);
    return x;
  }
  std::string string() {
    auto n = u32();
    if (n > 4096 || n > remaining())
      throw std::runtime_error("invalid string length");
    std::string s(b_.begin() + pos_, b_.begin() + pos_ + n);
    pos_ += n;
    return s;
  }
  Bytes bytes() {
    auto n = u32();
    if (n > kMaxMessage || n > remaining())
      throw std::runtime_error("invalid byte array length");
    Bytes v(b_.begin() + pos_, b_.begin() + pos_ + n);
    pos_ += n;
    return v;
  }
  void finish() {
    if (remaining())
      throw std::runtime_error("trailing payload data");
  }
};
struct Envelope {
  Type type = Type::Heartbeat;
  ClockDomain clock = ClockDomain::Unknown;
  uint8_t flags = 0;
  uint64_t session = 0, sequence = 0, sent_ns = 0, source_ns = 0;
};
struct Fragment {
  Envelope env;
  uint32_t total = 0, crc = 0;
  uint16_t index = 0, count = 0;
  Bytes data;
};
struct Message {
  Envelope env;
  Bytes payload;
};
uint32_t checksum(const Bytes &b);
std::vector<Bytes> fragment(const Envelope &e, const Bytes &payload);
Fragment parse_fragment(const Bytes &packet);
Bytes compress_payload(const Bytes &b);
Bytes decompress_payload(const Bytes &b);
class Reassembler {
  struct Pending {
    Fragment first;
    Bytes data;
    std::vector<bool> seen;
    size_t received = 0;
    double started = 0;
  };
  std::map<std::pair<uint8_t, uint64_t>, Pending> pending_;
  size_t allocated_ = 0;
  double timeout_;
  uint64_t session_ = 0;

public:
  uint64_t expired = 0, duplicates = 0, evicted = 0;
  explicit Reassembler(double timeout = 0.25) : timeout_(timeout) {}
  bool accept(const Fragment &f, double now, Message &out);
  void expire(double now);
  void clear() {
    pending_.clear();
    allocated_ = 0;
    session_ = 0;
  }
  size_t allocated() const { return allocated_; }
};
} // namespace lite3_gateway
