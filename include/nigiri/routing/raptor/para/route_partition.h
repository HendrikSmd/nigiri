#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing::para {


struct route_partition {

  struct global_cell_idx {
    cell_idx_t cell_idx;
    std::uint16_t level;
  };

  static cell_idx_t get_parent_idx(cell_idx_t cell_idx, std::uint16_t levels = 1);

  void from_hmetis_result(std::filesystem::path const&, timetable const& tt);

  void write(std::filesystem::path const&) const;

  static cista::wrapped<route_partition> read(std::filesystem::path const&);

  cista::base_t<cell_idx_t> get_num_of_cells_on_level(cista::base_t<cell_idx_t> level) const;

  cell_idx_t get_cell_of_route(route_idx_t r_idx, uint16_t level) const;

  void assign_cells_to_components(timetable const& tt);


  vector_map<component_idx_t, global_cell_idx> cmpnt_to_cell_idx_;
  vector_map<route_idx_t, cell_idx_t> route_to_cell_idx_;
  cista::base_t<cell_idx_t> n_levels_{};
};

} // namespace nigiri::routing::para
