#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing {

struct route_partition {



  void from_hmetis_result(std::filesystem::path const&, timetable const& tt);

  void write(std::filesystem::path const&) const;

  static cista::wrapped<route_partition> read(std::filesystem::path const&);


  vector_map<route_idx_t, cell_idx_t> route_to_cell_idx_;
  cista::base_t<cell_idx_t> n_levels_{};
};

}
