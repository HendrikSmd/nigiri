#pragma once

#include <cstdint>

namespace nigiri::routing::para {

struct bmc_raptor_route_label {

  bool        dominates(bmc_raptor_route_label const& other) const;
  static bool dominates(bmc_raptor_route_label const& l1, bmc_raptor_route_label const& l2);
  static bool equals(bmc_raptor_route_label const& l1, bmc_raptor_route_label const& l2);

  std::uint32_t transport_idx_;
  std::int16_t  transport_day_offset_;
  std::uint16_t departure_;
};

} // nigiri::routing::raptor::para