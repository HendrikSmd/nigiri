#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct route_partition {

  static cell_idx_t get_parent_idx(cell_idx_t cell_idx);

  void from_hmetis_result(std::filesystem::path const&, timetable const& tt);

  void write(std::filesystem::path const&) const;

  static cista::wrapped<route_partition> read(std::filesystem::path const&);

  size_t get_num_of_cells_on_level(uint8_t level) const;

  cell_idx_t get_cell_of_route(route_idx_t const r_idx, uint8_t level) const;

  vector_map<route_idx_t, cell_idx_t> route_to_cell_idx_;
  cista::base_t<cell_idx_t> n_levels_{};
};

}
