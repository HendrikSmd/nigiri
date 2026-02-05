#pragma once
#include "nigiri/types.h"

namespace nigiri::routing::para {

struct bmc_raptor_label {

  bool        dominates_non_destination(bmc_raptor_label const& o) const;
  static bool dominates_non_destination(bmc_raptor_label const& l1, bmc_raptor_label const& l2);

  bool        dominates_destination(bmc_raptor_label const& o) const;
  static bool dominates_destination(bmc_raptor_label const& l1, bmc_raptor_label const& l2);

  // --- Block 1: 64 bits (8 bytes) ---
  std::uint32_t route_idx;              // 32 bits
  std::uint16_t enter_stop_idx;         // 16 bits
  std::uint16_t arrival_;               // 16 bits

  // --- Block 2: 64 bits (8 bytes) ---
  std::uint32_t parent_bag_idx;         // 32 bits
  std::uint16_t arrival_with_transfer_; // 16 bits
  std::uint16_t departure_ : 11;        // 11 bits (0-2047)
  std::uint16_t is_footpath : 1;        // 1 bit
  std::uint16_t has_parent  : 1;        // 1 bit
  std::uint16_t reserved    : 3 {0};        // 3 bits padding to fill 16
};

} // nigiri::routing::raptor::para