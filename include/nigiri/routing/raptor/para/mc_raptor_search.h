#pragma once

#include "nigiri/common/interval.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/search.h"
#include "nigiri/timetable.h"

#include <string_view>
#include <vector>

#include "mc_raptor_state.h"

namespace nigiri::routing::para {
/*
std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  location_idx_t from,
                                                  std::string_view start_time,
                                                  std::string_view end_time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask,
                                                  bitvec const& transfer_mask,
                                                  bool use_start_footpaths);

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  location_idx_t from,
                                                  interval<unixtime_t> time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask,
                                                  bitvec const& transfer_mask,
                                                  bool use_start_footpaths);
*/
std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  mc_raptor_state& state,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask,
                                                  bitvec const& transfer_mask);

}  // namespace nigiri::routing::para