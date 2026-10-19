#pragma once
#include "lite3_sensor_gateway/protocol.hpp"
#include <cstdint>

namespace lite3_gateway {
struct ClockStatusData {
  bool qualified = false;
  int64_t peer_offset_ns = 0;
  uint64_t peer_bound_ns = 0, peer_rtt_ns = 0, mapping_uncertainty_ns = 0;
  uint64_t probe_age_ns = 0, clock_jumps = 0;
};
Bytes encode_clock_status(const ClockStatusData &status);
ClockStatusData decode_clock_status(const Bytes &bytes);

class TimeMapper {
  int64_t offset_ns_ = 0;
  uint64_t steady_ns_ = 0, uncertainty_ns_ = 0, jumps_ = 0;
  bool initialized_ = false;

public:
  bool update(uint64_t steady_before_ns, uint64_t system_ns, uint64_t steady_after_ns,
              uint64_t jump_threshold_ns = 2000000);
  bool convert(uint64_t source_steady_ns, uint64_t current_steady_ns, uint64_t &system_ns) const;
  bool fresh(uint64_t current_steady_ns) const;
  uint64_t uncertainty_ns() const { return uncertainty_ns_; }
  uint64_t jumps() const { return jumps_; }
};

class PeerClock {
  int64_t offset_ns_ = 0;
  uint64_t bound_ns_ = 0, rtt_ns_ = 0, last_good_steady_ns_ = 0;
  unsigned good_samples_ = 0;

public:
  bool observe(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4,
               uint64_t receive_steady_ns, uint64_t max_bound_ns = 5000000);
  bool qualified(uint64_t current_steady_ns) const;
  void reset();
  int64_t offset_ns() const { return offset_ns_; }
  uint64_t bound_ns() const { return bound_ns_; }
  uint64_t rtt_ns() const { return rtt_ns_; }
  uint64_t age_ns(uint64_t now) const;
};
} // namespace lite3_gateway
