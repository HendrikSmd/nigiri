#include "nigiri/routing/raptor/para/route_partition.h"

#include <fstream>

#include "boost/filesystem/path.hpp"

#include <ranges>

#include "cista/io.h"
#include "utl/enumerate.h"

namespace nigiri::routing::para {

cell_idx_t route_partition::get_parent_idx(cell_idx_t const cell_idx,
                                           std::uint8_t const levels) {
  return cell_idx >> levels;
}

void route_partition::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

cista::wrapped<route_partition> route_partition::read(std::filesystem::path const& path) {
  return cista::read<route_partition>(path);
}

void route_partition::from_hmetis_result(std::filesystem::path const& path,
                                         timetable const& tt) {
  auto const n_routes = tt.n_routes();
  auto const n_directed_footpaths =
      tt.locations_.footpaths_out_[kDefaultProfile].data_.size();
  route_to_cell_idx_.resize(n_routes, cell_idx_t::invalid());
  footpath_to_cell_idx_.resize(n_directed_footpaths, cell_idx_t::invalid());

  vector_map<footpath_idx_t, std::pair<location_idx_t, location_idx_t>>
      undirected_fp_edges;
  std::vector<std::vector<footpath_idx_t>> loc_incident_fps(tt.n_locations());
  for (auto loc = location_idx_t{0U}; loc < tt.n_locations(); ++loc) {
    auto const loc_fps_out = tt.locations_.footpaths_out_[kDefaultProfile][loc];
    for (auto const& fp : loc_fps_out) {
      if (loc < fp.target()) {
        auto const next_index = footpath_idx_t{undirected_fp_edges.size()};
        loc_incident_fps[to_idx(loc)].push_back(next_index);
        loc_incident_fps[to_idx(fp.target())].push_back(next_index);
        undirected_fp_edges.emplace_back(loc, fp.target());
      }
    }
  }

  std::ifstream ifs(path, std::ifstream::in);

  std::string line;
  std::uint32_t current_connection_idx{0U};
  std::uint32_t n_cells_{0U};
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      break;
    }

    cell_idx_t assigned_cell{std::stol(line)};
    if (current_connection_idx < n_routes) {
      route_to_cell_idx_.at(route_idx_t{current_connection_idx}) = assigned_cell;
    } else {
      auto const undirected_fp_idx =
          footpath_idx_t{current_connection_idx - n_routes};
      auto const fp_locations = undirected_fp_edges[undirected_fp_idx];
      assign_cell_to_undirected_footpath(tt, fp_locations.first,
                                         fp_locations.second, assigned_cell);
      assign_cell_to_undirected_footpath(tt, fp_locations.second,
                                   fp_locations.first, assigned_cell);
    }
    ++current_connection_idx;

    n_cells_ = std::max(n_cells_, static_cast<std::uint32_t>(assigned_cell.v_) + 1);
    utl::verify(n_cells_ <= std::numeric_limits<std::uint16_t>::max(), "Only support up to 2^16 - 1 many cells in lowest level");
  }

  utl::verify(current_connection_idx == n_routes + (n_directed_footpaths >> 1),
              "Unexpected number of lines received");

  if (n_cells_ == 0U) {
    throw utl::fail("partition is empty!");
  }

  if (!std::has_single_bit(n_cells_)) {
    throw utl::fail(
        "we only support partitions with number of cells being a power of 2! "
        "Got {} cells",
        n_cells_);
  }

  n_levels_ = static_cast<std::uint8_t>(std::bit_width(n_cells_) - 1);
  assign_cells_to_locations(tt);
}

void route_partition::assign_cells_to_locations(timetable const& tt) {
  auto const n_locations = tt.n_locations();
  location_to_cell_idx_.resize(
      n_locations, global_cell_idx{cell_idx_t::invalid(),
                                   std::numeric_limits<std::uint8_t>::max()});
  std::vector<std::vector<cell_idx_t>> location_cell_idxs;
  for (auto loc_idx = location_idx_t{0}; loc_idx < n_locations; ++loc_idx) {

    std::vector<cell_idx_t> cell_idxs;
    const auto routes_of_loc = tt.location_routes_[loc_idx];
    std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
      return route_to_cell_idx_[r];
    });

    auto const loc_fps =
    tt.locations_.footpaths_out_[kDefaultProfile][loc_idx];
    auto const fp_base_idx = std::distance(
        tt.locations_.footpaths_out_[kDefaultProfile].data_.begin(),
        loc_fps.begin());
    for (auto const [fp_offset, _] : utl::enumerate(loc_fps)) {
      auto const fp_idx =
          footpath_idx_t{static_cast<unsigned long>(fp_base_idx) + fp_offset};
      if (auto const cell_idx = footpath_to_cell_idx_[fp_idx];
          cell_idx != cell_idx_t::invalid()) {
        cell_idxs.push_back(cell_idx);
      } else {
        utl::fail("Footpath not assigned to cell");
      }
    }


    std::ranges::sort(cell_idxs);
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    cell_idxs.erase(last, cell_idxs.end());
    location_cell_idxs.emplace_back(cell_idxs);
  }

  for (std::uint16_t level = 0U; level <= static_cast<std::uint16_t>(n_levels_); ++level) {

    for (auto loc = location_idx_t{0}; loc < n_locations; ++loc) {
      if (auto& cell_idxs = location_cell_idxs[to_idx(loc)];
          cell_idxs.size() == 1) {
        location_to_cell_idx_[loc] = global_cell_idx{cell_idxs.front(), static_cast<std::uint8_t>(level)};
        cell_idxs.clear();
      }
    }

    for (auto& cell_idxs : location_cell_idxs) {
      std::ranges::transform(cell_idxs, cell_idxs.begin(), [](cell_idx_t const& cell_idx) {
        return get_parent_idx(cell_idx);
      });
      auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
      cell_idxs.erase(last, cell_idxs.end());
    }
  }
}

std::uint16_t route_partition::get_num_of_cells_on_level(std::uint8_t const level) const {
  assert(n_levels_ >= level);
  return static_cast<std::uint16_t>(1U << (n_levels_ - level));
}

cell_idx_t route_partition::get_cell_of_route(route_idx_t const r_idx, uint8_t const level) const {
  return route_to_cell_idx_.at(r_idx) >> level;
}

cell_idx_t route_partition::get_cell_of_footpath(footpath_idx_t const fp_idx, uint8_t const level) const {
  return footpath_to_cell_idx_.at(fp_idx) >> level;
}

void route_partition::print_cells_of_location(timetable const& tt, location_idx_t const loc) const {
  const auto routes_of_loc = tt.location_routes_[loc];
  for (const auto r : routes_of_loc) {
    std::cout << "Route " << r << ": " << route_to_cell_idx_[r] << std::endl;
  }

  const auto& out_fps = tt.locations_.footpaths_out_[kDefaultProfile];
  const auto fps_of_loc = out_fps[loc];
  const auto fp_base_idx = static_cast<std::uint32_t>(std::distance(out_fps.data_.begin(), fps_of_loc.begin()));
  for (const auto [off, fp] : utl::enumerate(fps_of_loc)) {
    std::cout << "Footpath to " << fp.target() << ": " << footpath_to_cell_idx_[footpath_idx_t{fp_base_idx + off}] << std::endl;
  }
}

void route_partition::assign_cell_to_undirected_footpath(
    timetable const& tt,
    location_idx_t const loc1,
    location_idx_t const loc2,
    cell_idx_t cell) {
  auto const& out_fps = tt.locations_.footpaths_out_[kDefaultProfile];

  auto const loc1_out_fps = out_fps[loc1];
  auto const fp1_base_idx = static_cast<size_t>(
      std::distance(out_fps.data_.begin(), loc1_out_fps.begin()));
  auto target_iter = std::ranges::find_if(
      loc1_out_fps, [loc2](auto const& fp) { return fp.target() == loc2; });

  utl::verify(target_iter != loc1_out_fps.end(),
              "Directed footpath {} to {} not found", loc1, loc2);

  auto const fp_idx =
      static_cast<size_t>(std::distance(loc1_out_fps.begin(), target_iter));
  footpath_to_cell_idx_[footpath_idx_t{fp1_base_idx + fp_idx}] = cell;
}


}