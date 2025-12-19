#pragma once

#include <vector>
#include <string_view>

#include "nigiri/timetable.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/search.h"
#include "nigiri/common/interval.h"

namespace nigiri::routing::para {

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  std::string_view from,
                                                  std::string_view start_time,
                                                  std::string_view end_time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask);

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  std::string_view from,
                                                  interval<unixtime_t> time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask);

}  // namespace nigiri::routing::para