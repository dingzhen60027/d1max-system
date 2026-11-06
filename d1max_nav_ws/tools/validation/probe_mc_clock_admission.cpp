// Passive production-core reproducer. No ROS node, SDK, grant or motion call.
// The two positive packets are exact endpoints from the 2026-10-12 live
// read-only probe; they are NOT a synthesized navigation-success simulation.
#include "execution_transport_core.hpp"
#include <array>
#include <iostream>
#include <string>

struct Packet {
  const char* name;
  uint64_t raw;
  double mapped, receipt, observer;
};

int main() {
  const std::array<Packet, 5> packets{{
    {"observed_first", 1140788198032ULL, 1791782598.1336393,
      1791782598.0737355, 1791782598.0937316},
    {"observed_last", 1144788178775ULL, 1791782602.13362,
      1791782602.0737438, 1791782602.0744348},
    {"stale_receipt", 1144788178775ULL, 1791782602.13362,
      1791782602.0737438, 1791782603.0744348},
    {"excessive_mapping_lead", 1144788178775ULL, 1791782602.3737438,
      1791782602.0737438, 1791782602.0744348},
    {"future_receipt", 1144788178775ULL, 1791782602.13362,
      1791782602.0737438, 1791782602.0},
  }};
  std::cout << "{\"scope\":\"production_transport_no_ROS_no_SDK_no_grants\",\"samples\":[";
  bool first = true;
  for (const auto& p : packets) {
    d1monitor::execution3::Transport writer("passive_probe", "live", false);
    writer.configureMcCaptureBound(.25, "source_delta_host_anchor_approximate");
    writer.mc(p.raw, p.mapped, p.receipt, .02, .01, p.observer, "sdk:0");
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"name\":\"" << p.name << "\",\"accepted_capture\":"
      << (writer.mcSourceFresh(p.observer, "sdk:0") ? "true" : "false") << '}';
  }
  std::cout << "]}\n";
}
