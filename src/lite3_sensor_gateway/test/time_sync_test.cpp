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
    check(!peer.qualified(10000000000ULL));
    check(!peer.observe(t1, t1 + 20000000, t1 + 20100000, t1 + 2200000,
                        11000000000ULL));
    check(!peer.qualified(11000000000ULL));
    check(!peer.observe(t1, t1 + 10000000, t1 + 10100000, t1 + 12000000,
                        11500000000ULL));
    check(peer.bound_ns() > 5000000);

    ClockStatusData status;
    status.qualified = true;
    status.peer_offset_ns = -12345;
    status.peer_bound_ns = 4321;
    status.clock_jumps = 2;
    const auto decoded = decode_clock_status(encode_clock_status(status));
    check(decoded.qualified && decoded.peer_offset_ns == -12345 &&
          decoded.peer_bound_ns == 4321 && decoded.clock_jumps == 2);
    std::cout << "time sync tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
