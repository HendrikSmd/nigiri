#pragma once

#include <cstdint>

namespace nigiri::routing::para {

struct bmc_raptor_route_label {

  bool        dominates(bmc_raptor_route_label const& other) const;
  static bool dominates(bmc_raptor_route_label const& l1, bmc_raptor_route_label const& l2);

  // --- Block 1: 64 bits (8 bytes) ---
  std::uint32_t transport_idx_;              // 32 bits
  std::uint16_t enter_stop_idx_;              // 16 bits
  std::int16_t  transport_day_offset_;       // 16 bits

  // --- Block 2: 48 bits (6 bytes) ---
  std::uint32_t parent_bag_idx_;              // 32 bits
  std::uint16_t departure_;                  // 16 bits
  // 2 Bytes Padding
};

} // nigiri::routing::raptor::para