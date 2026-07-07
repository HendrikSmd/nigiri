#pragma once
#include "nigiri/types.h"

namespace nigiri::routing {

template <size_t Size>
struct arrival_label {
  std::uint16_t arrival_;
  std::uint16_t arrival_with_transfer_;
  std::uint16_t departure_;
  cista::bitset<Size> active_days_;
};

template <size_t Size>
struct route_label {
  std::uint16_t departure_;
  std::int16_t transport_day_offset_;
  std::uint32_t transport_idx_;
  cista::bitset<Size> active_days_;
};

} // nigiri::routing