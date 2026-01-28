#pragma once

#include "nigiri/routing/start_times.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing::para {

void compress_start_times(timetable const& tt,
                          component_idx_t const cmpnt_idx,
                          std::vector<std::vector<cmpnt_dep_event>> const& source_dep_events,
                          std::vector<std::vector<cmpnt_dep_event>>& bins);

}