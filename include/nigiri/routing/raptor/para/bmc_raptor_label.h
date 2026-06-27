#pragma once
#include "nigiri/types.h"

namespace nigiri::routing::para {

struct bmc_raptor_label {

  bool        dominates_non_destination(bmc_raptor_label const& o) const;
  static bool dominates_non_destination(bmc_raptor_label const& l1, bmc_raptor_label const& l2);

  bool        dominates_destination(bmc_raptor_label const& o) const;
  static bool dominates_destination(bmc_raptor_label const& l1, bmc_raptor_label const& l2);

  static bool equals(bmc_raptor_label const& l1, bmc_raptor_label const& l2);

  std::uint16_t arrival_;
  std::uint16_t arrival_with_transfer_;
  std::uint16_t departure_;
};

} // nigiri::routing::raptor::para