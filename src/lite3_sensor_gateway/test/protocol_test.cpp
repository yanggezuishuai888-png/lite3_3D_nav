#include "lite3_sensor_gateway/protocol.hpp"
#include <algorithm>
#include <iostream>
using namespace lite3_gateway;
void check(bool x) {
  if (!x)
    throw std::runtime_error("assertion failed");
}
template <class F> void rejects(F fn) {
  bool rejected = false;
  try {
    fn();
  } catch (const std::exception &) {
    rejected = true;
  }
  check(rejected);
}
int main() {
  try {
    Envelope e;
    e.type = Type::Depth;
    e.session = 0x0102030405060708ULL;
    e.sequence = 10;
    e.source_ns = 123;
    Bytes b(9000);
    for (size_t i = 0; i < b.size(); ++i)
      b[i] = i % 251;
    auto packets = fragment(e, b);
    check(packets[0].size() == 1200 && packets[0][0] == 'L' && packets[0][3] == 'W' &&
          packets[0][8] == 1 && packets[0][15] == 8);
    Reassembler a;
    Message m;
    check(!a.accept(parse_fragment(packets.back()), 0, m));
    check(!a.accept(parse_fragment(packets.back()), 0, m));
    check(a.duplicates == 1);
    bool complete = false;
    for (size_t i = 0; i + 1 < packets.size(); ++i)
      complete = a.accept(parse_fragment(packets[i]), 0.01, m);
    check(complete && m.payload == b && a.allocated() == 0);
    auto bad = packets;
    bad[1].back() ^= 1;
    rejects([&] {
      for (auto &p : bad)
        a.accept(parse_fragment(p), 1, m);
    });
    auto truncated = packets[0];
    truncated.pop_back();
    rejects([&] { parse_fragment(truncated); });
    auto huge = packets[0];
    huge[40] = 0xff;
    rejects([&] { parse_fragment(huge); });
    check(!a.accept(parse_fragment(packets[0]), 2, m));
    a.expire(3);
    check(a.expired == 1 && a.allocated() == 0);
    for (int i = 0; i < 6; ++i) {
      e.sequence = 20 + i;
      auto p = fragment(e, b);
      a.accept(parse_fragment(p[0]), 4, m);
    }
    check(a.evicted == 2 && a.allocated() == 4 * b.size());
    check(decompress_payload(compress_payload(b)) == b);
    rejects([&] { decompress_payload(Bytes{1, 2, 3}); });
    Writer w;
    w.u32(0x01020304);
    check(w.data == Bytes({1, 2, 3, 4}));
    std::cout << "protocol tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
