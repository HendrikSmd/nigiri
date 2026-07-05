#pragma once

#include <vector>

#include "nigiri/common/standard_flat_matrix.h"
#include "nigiri/routing/label.h"
#include "nigiri/timetable.h"

namespace nigiri::routing::gpu {

struct gpu_timetable;

template <size_t Size>
struct device_route_label {
  std::uint16_t arrival_;
  std::uint16_t arrival_with_transfer_;
  std::uint16_t departure_;
  cista::bitset<Size> active_days_;
};

std::vector<route_label_by_value<64>> get_earliest_sufficient_transports_gpu(
    timetable const& tt,
    gpu_timetable const& gtt,
    simple_flat_matrix<std::vector<route_label<64>>> const& M,
    size_t k,
    std::vector<route_idx_t> const& R);

}  // namespace nigiri::routing::gpu
