#include "nigiri/routing/raptor/para/route_partition.h"

#include <fstream>

#include "boost/filesystem/path.hpp"


#include "cista/io.h"

namespace nigiri::routing {

void route_partition::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

cista::wrapped<route_partition> route_partition::read(std::filesystem::path const& path) {
  return cista::read<route_partition>(path);
}

void route_partition::from_hmetis_result(std::filesystem::path const& path, timetable const& tt) {
  auto const n_routes = tt.n_routes();
  route_to_cell_idx_.resize(n_routes, cell_idx_t::invalid());

  this->n_cells_ = 0U;
  std::ifstream ifs(path, std::ifstream::in);

  std::string line;
  route_idx_t current_idx{0U};
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      break;
    }

    cell_idx_t assigned_cell{std::stol(line)};
    route_to_cell_idx_.at(current_idx) = assigned_cell;
    ++current_idx;

    this->n_cells_ = std::max(this->n_cells_, static_cast<uint16_t>(assigned_cell.v_ + 1));
  }
}


}