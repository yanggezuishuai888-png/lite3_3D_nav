#include "lite3_sensor_gateway/protocol.hpp"
#include <algorithm>
#include <zlib.h>
/*
定义 UDP 协议：消息类型、会话号、序号、时间信息、分片、CRC 校验、压缩和有界重组。
*/
namespace lite3_gateway {
bool valid_type(Type t) {
  auto n = static_cast<uint8_t>(t);
  return (n >= 1 && n <= 11) || (n >= 16 && n <= 22);
}
const char *type_name(Type t) {
  switch (t) {
  case Type::Heartbeat:
    return "heartbeat";
  case Type::Imu:
    return "imu";
  case Type::Odom:
    return "odom";
  case Type::Ultrasound:
    return "ultrasound";
  case Type::Handle:
    return "handle";
  case Type::Depth:
    return "depth";
  case Type::CameraInfo:
    return "camera_info";
  case Type::Color:
    return "color";
  case Type::AlignedDepth:
    return "aligned_depth";
  case Type::ColorInfo:
    return "color_info";
  case Type::AlignedInfo:
    return "aligned_info";
  default:
    return "sensor_status";
  }
}
uint32_t checksum(const Bytes &b) {
  return static_cast<uint32_t>(crc32(0, b.data(), b.size()));
}
std::vector<Bytes> fragment(const Envelope &e, const Bytes &payload) {
  if (!valid_type(e.type) || payload.empty() || payload.size() > kMaxMessage || !e.session ||
      !e.sequence)
    throw std::runtime_error("invalid message");
  const size_t count = (payload.size() + kChunkSize - 1) / kChunkSize;
  const auto crc = checksum(payload);
  std::vector<Bytes> result;
  result.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    size_t offset = i * kChunkSize, len = std::min(kChunkSize, payload.size() - offset);
    Writer w;
    w.u32(0x4c334757);
    w.u8(1);
    w.u8(static_cast<uint8_t>(e.type));
    w.u8(static_cast<uint8_t>(e.clock));
    w.u8(e.flags);
    w.u64(e.session);
    w.u64(e.sequence);
    w.u64(e.sent_ns);
    w.u64(e.source_ns);
    w.u32(payload.size());
    w.u32(crc);
    w.u16(i);
    w.u16(count);
    w.u16(len);
    w.u16(0);
    w.data.insert(w.data.end(), payload.begin() + offset, payload.begin() + offset + len);
    result.push_back(std::move(w.data));
  }
  return result;
}
Fragment parse_fragment(const Bytes &packet) {
  if (packet.size() < kHeaderSize || packet.size() > kDatagramSize)
    throw std::runtime_error("invalid datagram size");
  Reader r(packet);
  Fragment f;
  if (r.u32() != 0x4c334757 || r.u8() != 1)
    throw std::runtime_error("unknown protocol");
  f.env.type = static_cast<Type>(r.u8());
  f.env.clock = static_cast<ClockDomain>(r.u8());
  f.env.flags = r.u8();
  f.env.session = r.u64();
  f.env.sequence = r.u64();
  f.env.sent_ns = r.u64();
  f.env.source_ns = r.u64();
  f.total = r.u32();
  f.crc = r.u32();
  f.index = r.u16();
  f.count = r.u16();
  auto len = r.u16();
  auto reserved = r.u16();
  if (!valid_type(f.env.type) || static_cast<uint8_t>(f.env.clock) > 2 || f.env.flags > 1 ||
      reserved || !f.env.session || !f.env.sequence || !f.total || f.total > kMaxMessage)
    throw std::runtime_error("invalid header");
  if (f.env.flags && !is_depth(f.env.type))
    throw std::runtime_error("compression only allowed for depth");
  if (f.count != (f.total + kChunkSize - 1) / kChunkSize || f.index >= f.count ||
      len != std::min(kChunkSize, size_t(f.total) - size_t(f.index) * kChunkSize) ||
      r.remaining() != len)
    throw std::runtime_error("invalid fragment layout");
  f.data.assign(packet.begin() + kHeaderSize, packet.end());
  return f;
}
Bytes compress_payload(const Bytes &b) {
  if (b.empty() || b.size() > kMaxMessage)
    throw std::runtime_error("oversized compression input");
  uLongf n = compressBound(b.size());
  Writer header;
  header.u32(b.size());
  Bytes out = header.data;
  out.resize(n + 4);
  if (compress2(out.data() + 4, &n, b.data(), b.size(), 1) != Z_OK)
    throw std::runtime_error("compression failed");
  out.resize(n + 4);
  return out;
}
Bytes decompress_payload(const Bytes &b) {
  Reader reader(b);
  const auto expected = reader.u32();
  if (!expected || expected > kMaxMessage || !reader.remaining())
    throw std::runtime_error("invalid decompressed size");
  Bytes out(expected);
  uLongf n = out.size();
  if (uncompress(out.data(), &n, b.data() + 4, b.size() - 4) != Z_OK || n != expected)
    throw std::runtime_error("invalid compressed payload");
  out.resize(n);
  return out;
}
void Reassembler::expire(double now) {
  for (auto i = pending_.begin(); i != pending_.end();) {
    if (now - i->second.started > timeout_) {
      allocated_ -= i->second.data.size();
      i = pending_.erase(i);
      ++expired;
    } else
      ++i;
  }
}
bool Reassembler::accept(const Fragment &f, double now, Message &out) {
  expire(now);
  if (session_ != f.env.session) {
    clear();
    session_ = f.env.session;
  }
  auto key = std::make_pair(static_cast<uint8_t>(f.env.type), f.env.sequence);
  auto it = pending_.find(key);
  if (it == pending_.end()) {
    while (!pending_.empty() && (pending_.size() >= 4 || allocated_ + f.total > 2 * kMaxMessage)) {
      auto oldest =
          std::min_element(pending_.begin(), pending_.end(), [](const auto &a, const auto &b) {
            return a.second.started < b.second.started;
          });
      allocated_ -= oldest->second.data.size();
      pending_.erase(oldest);
      ++evicted;
    }
    Pending p;
    p.first = f;
    p.first.data.clear();
    p.data.resize(f.total);
    p.seen.resize(f.count);
    p.started = now;
    allocated_ += f.total;
    it = pending_.emplace(key, std::move(p)).first;
  }
  auto &p = it->second;
  const auto &a = p.first;
  if (a.total != f.total || a.crc != f.crc || a.count != f.count || a.env.flags != f.env.flags ||
      a.env.clock != f.env.clock || a.env.source_ns != f.env.source_ns ||
      a.env.sent_ns != f.env.sent_ns)
    throw std::runtime_error("conflicting fragment headers");
  if (p.seen[f.index]) {
    ++duplicates;
    return false;
  }
  p.seen[f.index] = true;
  ++p.received;
  std::copy(f.data.begin(), f.data.end(), p.data.begin() + size_t(f.index) * kChunkSize);
  if (p.received != p.seen.size())
    return false;
  out.env = f.env;
  out.payload = std::move(p.data);
  allocated_ -= f.total;
  pending_.erase(it);
  if (checksum(out.payload) != f.crc)
    throw std::runtime_error("CRC mismatch");
  return true;
}
} // namespace lite3_gateway
