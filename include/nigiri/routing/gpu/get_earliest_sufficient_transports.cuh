#pragma once

#include <vector>

#include "nigiri/common/standard_flat_matrix.h"
#include "nigiri/routing/label.h"
#include "nigiri/timetable.h"

namespace nigiri::routing::gpu {

struct gpu_timetable;

std::vector<route_label<64>> get_earliest_sufficient_transports_gpu(
    timetable const& tt,
    gpu_timetable const& gtt,
    simple_flat_matrix<std::vector<arrival_label<64>>> const& M,
    size_t k,
    std::vector<route_idx_t> const& R);

}  // namespace nigiri::routing::gpu
