#include "nigiri/routing/raptor/para/route_partition.h"

#include <fstream>

#include "boost/filesystem/path.hpp"

#include <ranges>

#include "cista/io.h"

namespace nigiri::routing::para {

cell_idx_t route_partition::get_parent_idx(cell_idx_t const cell_idx,
                                           std::uint16_t const levels) {
  return cell_idx >> levels;
}

void route_partition::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

cista::wrapped<route_partition> route_partition::read(std::filesystem::path const& path) {
  return cista::read<route_partition>(path);
}

void route_partition::from_hmetis_result(std::filesystem::path const& path, timetable const& tt) {
  auto const n_routes = tt.n_routes();
  route_to_cell_idx_.resize(n_routes, cell_idx_t::invalid());

  std::ifstream ifs(path, std::ifstream::in);

  std::string line;
  route_idx_t current_idx{0U};
  cista::base_t<cell_idx_t> n_cells_{0U};
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      break;
    }

    cell_idx_t assigned_cell{std::stol(line)};
    route_to_cell_idx_.at(current_idx) = assigned_cell;
    ++current_idx;

    n_cells_ = std::max(n_cells_, static_cast<uint16_t>(assigned_cell.v_ + 1));
  }

  if (n_cells_ == 0U) {
    throw utl::fail("partition is empty!");
  }

  if (!std::has_single_bit(n_cells_)) {
    throw utl::fail("we only support partitions with number of cells being a power of 2! Got {} cells", n_cells_);
  }

  n_levels_ = static_cast<cista::base_t<cell_idx_t>>(std::bit_width(n_cells_) - 1);
  assign_cells_to_components(tt);
}

void route_partition::assign_cells_to_components(timetable const& tt) {
  auto n_components= tt.component_locations_.size();
  cmpnt_to_cell_idx_.resize(n_components,
                            global_cell_idx{
                              cell_idx_t::invalid(),
                              std::numeric_limits<std::uint8_t>::max()
                            });
  std::vector<std::vector<cell_idx_t>> component_cell_idxs;
  for (auto c = component_idx_t{0}; c < n_components; ++c) {

    std::vector<cell_idx_t> cell_idxs;
    for (const auto& loc : tt.component_locations_[c]) {
      const auto& routes_of_loc = tt.location_routes_[loc];

      if (routes_of_loc.empty()) {
        continue;
      }

      std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
        return route_to_cell_idx_[r];
      });
    }
    std::ranges::sort(cell_idxs);
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    cell_idxs.erase(last, cell_idxs.end());
    component_cell_idxs.emplace_back(cell_idxs);
  }

  for (std::uint16_t level = 0U; level <= n_levels_; ++level) {

    for (auto c = component_idx_t{0}; c < n_components; ++c) {
      if (auto& cell_idxs = component_cell_idxs[to_idx(c)];
          cell_idxs.size() == 1) {
        cmpnt_to_cell_idx_[c] = global_cell_idx{cell_idxs.front(), level};
        cell_idxs.clear();
      }
    }

    for (auto& cell_idxs : component_cell_idxs) {
      std::ranges::transform(cell_idxs, cell_idxs.begin(), [](cell_idx_t const& cell_idx) {
        return get_parent_idx(cell_idx);
      });
      auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
      cell_idxs.erase(last, cell_idxs.end());
    }
  }
}

cista::base_t<cell_idx_t> route_partition::get_num_of_cells_on_level(cista::base_t<cell_idx_t> const level) const {
  assert(n_levels_ >= level);
  return static_cast<cista::base_t<cell_idx_t>>(1U << (n_levels_ - level));
}

cell_idx_t route_partition::get_cell_of_route(route_idx_t const r_idx, uint16_t level) const {
  return route_to_cell_idx_.at(r_idx) >> level;
}


}