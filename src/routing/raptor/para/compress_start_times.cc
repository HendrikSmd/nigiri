#include "nigiri/routing/raptor/para/compress_start_times.h"

#include "nigiri/common/k_way_merge.h"
#include <ranges>

#include "utl/enumerate.h"

namespace nigiri::routing::para {

bool try_push_to_bin(std::vector<std::vector<duration_t>> const& fp_matrix,
                     std::vector<cmpnt_dep_event>& bin,
                     cmpnt_dep_event const& event) {
  for (const auto& emplaced_event : bin | std::ranges::views::reverse) {
    if (emplaced_event.dep_time_ + kMaxTravelTime < event.dep_time_) {
      break;
    }

    if (emplaced_event.dep_loc_ == event.dep_loc_) {
      continue;
    }

    if (fp_matrix[cista::to_idx(emplaced_event.dep_loc_)][cista::to_idx(event.dep_loc_)] == duration_t::max()) {
      return false;
    }

    if (event.dep_time_  -
        fp_matrix[cista::to_idx(emplaced_event.dep_loc_)][cista::to_idx(event.dep_loc_)] < emplaced_event.dep_time_) {
      return false;
    }
  }

  bin.push_back(event);
  return true;
}

void compress_start_times(timetable const& tt,
                          component_idx_t const cmpnt_idx,
                          std::vector<std::vector<cmpnt_dep_event>> const& source_dep_events,
                          std::vector<std::vector<cmpnt_dep_event>>& bins) {
  bins.clear();
  const auto& cmpnt_locs = tt.component_locations_[cmpnt_idx];

  std::vector fp_matrix(cmpnt_locs.size(), std::vector(cmpnt_locs.size(), duration_t::max()));
  std::unordered_map<location_idx_t, cmpnt_loc_idx_t> translator;
  for (const auto [cmpnt_loc_idx, loc_idx] : utl::enumerate(cmpnt_locs)) {
    translator.emplace(loc_idx, cmpnt_loc_idx_t{cmpnt_loc_idx});
    fp_matrix[cmpnt_loc_idx][cmpnt_loc_idx] = 0_minutes;
    for (const auto& out_fps : tt.locations_.footpaths_out_[kDefaultProfile][loc_idx]) {
      if (auto search = translator.find(out_fps.target()); search != translator.end()) {
        fp_matrix[cmpnt_loc_idx][cista::to_idx(search->second)] = out_fps.duration();
        fp_matrix[cista::to_idx(search->second)][cmpnt_loc_idx] = out_fps.duration();
      }
    }
  }

  k_way_merge merger(source_dep_events.begin(), source_dep_events.end(), std::less{});
  for (const auto& dep_event : merger) {
    bool bin_found = false;
    for (auto& bin : bins) {
      if (try_push_to_bin(fp_matrix, bin, dep_event)) {
        bin_found = true;
        break;
      }
    }
    if (!bin_found) {
      bins.emplace_back();
      bins.back().push_back(dep_event);
    }

  }
}


}
