#pragma once
#include "nigiri/types.h"

namespace nigiri::routing {

template <size_t Size>
struct route_label {
  std::uint16_t arrival_;
  std::uint16_t arrival_with_transfer_;
  std::uint16_t departure_;
  cista::bitset<Size> const& active_days_;
};

template <size_t Size>
struct route_label_by_value {
  route_label_by_value() = default;
  route_label_by_value(route_label<Size> const& l)
      : arrival_{l.arrival_},
        arrival_with_transfer_{l.arrival_with_transfer_},
        departure_{l.departure_},
        active_days_{l.active_days_} {}

  std::uint16_t arrival_;
  std::uint16_t arrival_with_transfer_;
  std::uint16_t departure_;
  cista::bitset<Size> active_days_;
};

} // nigiri::routing