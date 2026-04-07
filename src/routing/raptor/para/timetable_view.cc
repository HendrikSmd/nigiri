#include "nigiri/routing/raptor/para/timetable_view.h"

#include "utl/enumerate.h"

namespace nigiri::routing::para {

timetable_view::timetable_view(timetable const& tt,
                               bitvec const& route_mask,
                               bitvec const& footpath_mask) : tt_(tt) {
  utl::verify(tt.n_routes() == route_mask.size(),
              "Route mask has illegal size");
  utl::verify(tt.locations_.footpaths_out_[kDefaultProfile].data_.size() ==
                  footpath_mask.size(),
              "Footpath mask has illegal size");
  index(route_mask, footpath_mask);
}

timetable_view::timetable_view(timetable const& tt)
    : timetable_view(
          tt,
          bitvec::max(tt.n_routes()),
          bitvec::max(
              tt.locations_.footpaths_out_[kDefaultProfile].data_.size())) {}


void timetable_view::index(bitvec const& route_mask, bitvec const& footpath_mask) {
  bitvec locations_in_view(tt_.n_locations());
  const auto n_routes_in_view = static_cast<route_idx_t::value_t>(route_mask.count());
  internal_to_route_.resize(n_routes_in_view);

  route_to_internal_.resize(tt_.n_routes());
  utl::fill(route_to_internal_, route_idx_view_t::invalid());

  auto current_route_view_idx = route_idx_view_t{0U};
  route_mask.for_each_set_bit([&](std::uint32_t const i) {
    const auto route_idx = route_idx_t{i};
    route_to_internal_[route_idx] = current_route_view_idx;
    internal_to_route_[current_route_view_idx] = route_idx;
    ++current_route_view_idx;

    const auto location_sequence =
      tt_.route_location_seq_[route_idx];
    for (const auto& loc : location_sequence) {
      const auto cmpnt_idx = tt_.location_component_[stop{loc}.location_idx()];
      for (const auto& cmpnt_loc : tt_.component_locations_[cmpnt_idx]) {
        locations_in_view.set(to_idx(cmpnt_loc), true);
      }
    }
  });

  for (auto loc = location_idx_t{0U}; loc < tt_.n_locations(); ++loc) {
    const auto out_fps = tt_.locations_.footpaths_out_[kDefaultProfile][loc];

    auto const fp_base_idx = static_cast<std::uint32_t>(std::distance(
        tt_.locations_.footpaths_out_[kDefaultProfile].data_.begin(),
        out_fps.begin()));

    for (const auto [i, fp] : utl::enumerate(out_fps)) {
      if (footpath_mask[fp_base_idx + static_cast<std::uint32_t>(i)]) {
        locations_in_view.set(to_idx(loc), true);
        locations_in_view.set(to_idx(fp.target()), true);
      }
    }
  }

  const auto n_locations_in_view = static_cast<location_idx_t::value_t>(locations_in_view.count());
  location_to_internal_.resize(tt_.n_locations());
  utl::fill(location_to_internal_, location_idx_view_t::invalid());
  internal_to_location_.resize(static_cast<location_idx_t::value_t>(n_locations_in_view));
  auto next_free_index = location_idx_view_t{0U};
  locations_in_view.for_each_set_bit([&](std::uint32_t const i) {
    const auto location_idx = location_idx_t{i};
    location_to_internal_[location_idx] = next_free_index;
    internal_to_location_[location_idx_view_t{next_free_index}] = location_idx;
    ++next_free_index;
  });

  n_routes_in_view_ = n_routes_in_view;
  n_locations_in_view_ = n_locations_in_view;
}

location_idx_t::value_t timetable_view::get_n_locations() const {
  return n_locations_in_view_;
}

route_idx_t::value_t timetable_view::get_n_routes() const {
  return n_routes_in_view_;
}

timetable const& timetable_view::get_source_tt() const {
  return tt_;
}

bool timetable_view::is_in_view(location_idx_t const location_idx) const {
  return location_to_internal_[location_idx] != location_idx_view_t::invalid();
}

bool timetable_view::is_in_view(route_idx_t const route_idx) const {
  return route_to_internal_[route_idx] != route_idx_view_t::invalid();
}

location_idx_t timetable_view::get_source_idx(
    location_idx_view_t const location_idx_view) const {
  return internal_to_location_[location_idx_view];
}

route_idx_t timetable_view::get_source_idx(
    route_idx_view_t const route_idx_view) const {
  return internal_to_route_[route_idx_view];
}

location_idx_view_t timetable_view::get_view_idx(
    location_idx_t const location_idx) const {
  return location_to_internal_[location_idx];
}

route_idx_view_t timetable_view::get_view_idx(
    route_idx_t const route_idx) const {
  return route_to_internal_[route_idx];
}

}  // namespace nigiri::routing::para
