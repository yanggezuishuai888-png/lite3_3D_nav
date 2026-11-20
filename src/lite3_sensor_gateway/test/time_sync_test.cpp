#include "lite3_sensor_gateway/time_sync.hpp"
#include <iostream>
using namespace lite3_gateway;
void check(bool okay) {
  if (!okay)
    throw std::runtime_error("time sync assertion failed");
}
int main() {
  try {
    TimeMapper mapper;
    uint64_t system = 0;
    check(!mapper.convert(5000000000ULL, 5000000000ULL, system));
    check(!mapper.update(5000000000ULL, 1700000000000000000ULL, 5000001000ULL));
    check(mapper.convert(4900000000ULL, 5000001000ULL, system));
    check(system == 1699999999900000000ULL - 500);
    check(!mapper.convert(2000000000ULL, 5000001000ULL, system));
    check(!mapper.fresh(8000000000ULL));
    check(mapper.update(6000000000ULL, 1700000001200000000ULL, 6000001000ULL));
    check(mapper.jumps() == 1);

    PeerClock peer;
    const uint64_t t1 = 1700000000000000000ULL;
    for (unsigned i = 0; i < 3; ++i) {
      check(peer.observe(t1, t1 + 1000000, t1 + 1100000, t1 + 2200000,
                         6000000000ULL + i * 500000000ULL));
    }
    check(peer.qualified(7000000000ULL));
    check(peer.bound_ns() <= 5000000 && peer.rtt_ns() == 2100000);
    const auto accepted_bound = peer.bound_ns();
    const auto accepted_rtt = peer.rtt_ns();

    // One latency outlier must not create a multi-second data blackout. Keep
    // the last accepted estimate until its bounded holdover expires.
    check(!peer.observe(t1, t1 + 10000000, t1 + 10100000, t1 + 12000000,
                        7500000000ULL));
    check(peer.qualified(7500000000ULL));
    check(peer.bound_ns() == accepted_bound && peer.rtt_ns() == accepted_rtt);

    // A malformed single response is handled the same way.
    check(!peer.observe(t1, t1 + 20000000, t1 + 20100000, t1 + 2200000,
                        8000000000ULL));
    check(peer.qualified(8000000000ULL));

    // Once holdover has expired, three fresh good probes are required again.
    check(!peer.qualified(10000000000ULL));
    check(peer.observe(t1, t1 + 1000000, t1 + 1100000, t1 + 2200000,
                       11000000000ULL));
    check(!peer.qualified(11000000000ULL));
    check(peer.observe(t1, t1 + 1000000, t1 + 1100000, t1 + 2200000,
                       11500000000ULL));
    check(!peer.qualified(11500000000ULL));
    check(peer.observe(t1, t1 + 1000000, t1 + 1100000, t1 + 2200000,
                       12000000000ULL));
    check(peer.qualified(12000000000ULL));

    // A configured five-second holdover preserves a qualified estimate across
    // the observed 2.5-3 second scheduling tail without accepting bad probes.
    PeerClock held_peer;
    for (unsigned i = 0; i < 3; ++i) {
      check(held_peer.observe(t1, t1 + 1000000, t1 + 1100000, t1 + 2200000,
                              20000000000ULL + i * 500000000ULL, 5000000,
                              5000000000ULL));
    }
    check(held_peer.qualified(25000000000ULL, 5000000000ULL));
    check(!held_peer.qualified(26000000001ULL, 5000000000ULL));

    ClockStatusData status;
    status.qualified = true;
    status.peer_offset_ns = -12345;
    status.peer_bound_ns = 4321;
    status.clock_jumps = 2;
    status.probes_sent = 20;
    status.probes_received = 18;
    status.probes_rejected = 2;
    status.probe_responses_ignored = 1;
    status.good_samples = 3;
    const auto decoded = decode_clock_status(encode_clock_status(status));
    check(decoded.qualified && decoded.peer_offset_ns == -12345 &&
          decoded.peer_bound_ns == 4321 && decoded.clock_jumps == 2 &&
          decoded.probes_sent == 20 && decoded.probes_received == 18 &&
          decoded.probes_rejected == 2 && decoded.probe_responses_ignored == 1 &&
          decoded.good_samples == 3);

    // New receivers remain compatible with an old sender during a
    // receiver-first rolling deployment.
    Writer legacy;
    legacy.u8(1);
    legacy.u64(static_cast<uint64_t>(-12345));
    legacy.u64(4321);
    legacy.u64(2100);
    legacy.u64(100);
    legacy.u64(500000000);
    legacy.u64(2);
    const auto old_decoded = decode_clock_status(legacy.data);
    check(old_decoded.qualified && old_decoded.peer_offset_ns == -12345 &&
          old_decoded.probes_sent == 0 && old_decoded.good_samples == 0);
    std::cout << "time sync tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
