#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::routing::para {

struct timetable_view {

  explicit timetable_view(timetable const& tt);

           timetable_view(timetable const& tt,
                          bitvec const& route_mask);




  void index(bitvec const& route_mask);
  location_idx_t::value_t get_n_locations() const;
  route_idx_t::value_t get_n_routes() const;
  timetable const& get_source_tt() const;
  bool is_in_view(location_idx_t location_idx) const;
  bool is_in_view(route_idx_t route_idx) const;

  location_idx_t get_source_idx(location_idx_view_t location_idx_view) const;
  route_idx_t get_source_idx(route_idx_view_t route_idx_view) const;
  location_idx_view_t get_view_idx(location_idx_t location_idx) const;
  route_idx_view_t get_view_idx(route_idx_t route_idx) const;

private:

  location_idx_t::value_t n_locations_in_view_{};
  route_idx_t::value_t n_routes_in_view_{};

  vector_map<location_idx_t, location_idx_view_t> location_to_internal_{};
  vector_map<location_idx_view_t, location_idx_t> internal_to_location_{};

  vector_map<route_idx_view_t, route_idx_t> internal_to_route_{};
  vector_map<route_idx_t, route_idx_view_t> route_to_internal_{};

  timetable const& tt_;
};

} // namespace nigiri::routing::para
