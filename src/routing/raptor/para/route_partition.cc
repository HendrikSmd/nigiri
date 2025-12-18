#include "nigiri/routing/raptor/para/route_partition.h"

#include <fstream>

#include "boost/filesystem/path.hpp"


#include "cista/io.h"

namespace nigiri::routing {

cell_idx_t route_partition::get_parent_idx(cell_idx_t cell_idx) {
  return cell_idx >> 1U;
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

  this->n_levels_ = static_cast<cista::base_t<cell_idx_t>>(std::bit_width(n_cells_) - 1);
}

size_t route_partition::get_num_of_cells_on_level(uint8_t const level) const {
  assert(n_levels_ >= level);
  return 1U << static_cast<size_t>(n_levels_ - level);
}

cell_idx_t route_partition::get_cell_of_route(route_idx_t const r_idx, uint8_t level) const {
  return route_to_cell_idx_.at(r_idx) >> level;
}


}