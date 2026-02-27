#pragma once

#include "nigiri/routing/raptor/para/compress_start_times.h"

#include <vector>


namespace nigiri::routing::para {

struct relativized_component_departure_event {
  cmpnt_loc_idx_t dep_loc_;
  minutes_after_midnight_t dep_min_after_midnight_;
  bitfield active_days_;
};

using cell_cut_cmpnt_t = std::pair<cell_idx_t, component_idx_t>;
using bin_range_t = std::pair<size_t, size_t>;

struct start_times_registry {

  void populate(timetable const& tt,
                size_t n_of_cells,
                std_vecvec<cell_idx_t> const& cmpnt_to_cell_idxs,
                std::vector<bitvec> const& route_masks,
                bool compress_bins);

  void resize(size_t n_of_cells);

  void clear();

  std_vecvec<relativized_component_departure_event> cmpnt_dep_events_buffer_;
  std_vecvec<size_t> bin_start_idxs_;
  std_vecvec<bin_range_t> cell_cmpnt_search_bins_;
};

}  // namespace nigiri::routing::para
