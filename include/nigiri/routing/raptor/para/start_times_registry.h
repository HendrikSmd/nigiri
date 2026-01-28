#pragma once

#include "route_partition.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/routing/raptor/para/compress_start_times.h"

#include <vector>


namespace nigiri::routing::para {

struct start_times_registry {

  using cell_cut_cmpnt_t = std::pair<cell_idx_t, component_idx_t>;
  using bin_range_t = std::pair<size_t, size_t>;

  void populate(timetable const& tt,
                size_t n_of_cells,
                std::vector<std::vector<cell_idx_t>> const& cmpnt_to_cell_idxs,
                std::vector<bitvec> const& route_masks);

  void clear();

  std::vector<cmpnt_dep_event> dep_events_buffer_;
  std::vector<size_t> bin_start_idxs_;
  std::vector<bin_range_t> cell_cmpnt_search_bins_;
};

}  // namespace nigiri::routing::para
