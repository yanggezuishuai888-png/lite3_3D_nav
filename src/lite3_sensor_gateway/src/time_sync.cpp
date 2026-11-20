#include "lite3_sensor_gateway/time_sync.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace lite3_gateway {
Bytes encode_clock_status(const ClockStatusData &s) {
  Writer w;
  w.u8(s.qualified);
  w.u64(static_cast<uint64_t>(s.peer_offset_ns));
  w.u64(s.peer_bound_ns);
  w.u64(s.peer_rtt_ns);
  w.u64(s.mapping_uncertainty_ns);
  w.u64(s.probe_age_ns);
  w.u64(s.clock_jumps);
  w.u64(s.probes_sent);
  w.u64(s.probes_received);
  w.u64(s.probes_rejected);
  w.u64(s.probe_responses_ignored);
  w.u64(s.good_samples);
  return w.data;
}
ClockStatusData decode_clock_status(const Bytes &bytes) {
  Reader r(bytes);
  ClockStatusData s;
  auto value = r.u8();
  if (value > 1)
    throw std::runtime_error("invalid clock status");
  s.qualified = value;
  auto encoded_offset = r.u64();
  s.peer_offset_ns = encoded_offset <= INT64_MAX
                         ? static_cast<int64_t>(encoded_offset)
                         : static_cast<int64_t>(encoded_offset - uint64_t(INT64_MAX) - 1) + INT64_MIN;
  s.peer_bound_ns = r.u64();
  s.peer_rtt_ns = r.u64();
  s.mapping_uncertainty_ns = r.u64();
  s.probe_age_ns = r.u64();
  s.clock_jumps = r.u64();
  // Accept the original ClockStatus payload during a rolling receiver-first
  // deployment. A new sender appends probe observability counters.
  if (r.remaining()) {
    if (r.remaining() != 5 * sizeof(uint64_t))
      throw std::runtime_error("invalid extended clock status");
    s.probes_sent = r.u64();
    s.probes_received = r.u64();
    s.probes_rejected = r.u64();
    s.probe_responses_ignored = r.u64();
    s.good_samples = r.u64();
  }
  r.finish();
  return s;
}

bool TimeMapper::update(uint64_t before, uint64_t system, uint64_t after,
                        uint64_t jump_threshold) {
  if (after < before || after - before > 4000000 || system > INT64_MAX)
    return false;
  const auto midpoint = before + (after - before) / 2;
  if (midpoint > INT64_MAX)
    return false;
  const auto offset = static_cast<int64_t>(system) - static_cast<int64_t>(midpoint);
  bool jumped = initialized_ && std::llabs(offset - offset_ns_) >
                                   static_cast<int64_t>(jump_threshold);
  if (jumped)
    ++jumps_;
  offset_ns_ = offset;
  steady_ns_ = midpoint;
  uncertainty_ns_ = (after - before + 1) / 2;
  initialized_ = true;
  return jumped;
}
bool TimeMapper::fresh(uint64_t now) const {
  return initialized_ && now >= steady_ns_ && now - steady_ns_ <= 2000000000ULL &&
         uncertainty_ns_ <= 2000000;
}
bool TimeMapper::convert(uint64_t source, uint64_t now, uint64_t &system) const {
  if (!fresh(now) || !source || source > now + 100000000ULL ||
      (now > source && now - source > 1000000000ULL) || source > INT64_MAX)
    return false;
  const auto converted = static_cast<int64_t>(source) + offset_ns_;
  if (converted <= 0)
    return false;
  system = static_cast<uint64_t>(converted);
  return true;
}

bool PeerClock::observe(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4,
                        uint64_t steady, uint64_t max_bound,
                        uint64_t qualification_timeout) {
  // A previously qualified estimate may ride through a short scheduling or
  // network outlier. If the last accepted probe is already stale, however,
  // reacquisition must start from the first of three good samples again.
  if (last_good_steady_ns_ &&
      (steady < last_good_steady_ns_ ||
       steady - last_good_steady_ns_ > qualification_timeout))
    good_samples_ = 0;

  if (t4 < t1 || t3 < t2 || t4 - t1 < t3 - t2 || t4 - t1 > 100000000 ||
      std::max({t1, t2, t3, t4}) > INT64_MAX)
    return false;

  const auto outbound = static_cast<int64_t>(t2) - static_cast<int64_t>(t1);
  const auto inbound = static_cast<int64_t>(t3) - static_cast<int64_t>(t4);
  const auto rtt = (t4 - t1) - (t3 - t2);
  const auto offset = outbound / 2 + inbound / 2;
  const auto bound = static_cast<uint64_t>(std::llabs(offset)) + (rtt + 1) / 2;

  // Do not let one latency spike erase a valid clock estimate or publish the
  // rejected bound as if it were usable. qualified() will still fail closed
  // when no acceptable probe has arrived for the holdover timeout.
  if (bound > max_bound)
    return false;

  offset_ns_ = offset;
  bound_ns_ = bound;
  rtt_ns_ = rtt;
  last_good_steady_ns_ = steady;
  good_samples_ = std::min(good_samples_ + 1, 3U);
  return true;
}
bool PeerClock::qualified(uint64_t now, uint64_t qualification_timeout) const {
  return good_samples_ >= 3 && last_good_steady_ns_ && now >= last_good_steady_ns_ &&
         now - last_good_steady_ns_ <= qualification_timeout;
}
void PeerClock::reset() { good_samples_ = 0; }
uint64_t PeerClock::age_ns(uint64_t now) const {
  return last_good_steady_ns_ && now >= last_good_steady_ns_
             ? now - last_good_steady_ns_
             : UINT64_MAX;
}
} // namespace lite3_gateway
