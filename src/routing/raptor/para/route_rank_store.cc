#include "nigiri/routing/raptor/para/route_rank_store.h"

namespace nigiri::routing::para {

auto route_rank_store::cista_members() {
  return std::tie(route_rank_start_idx_, route_ranks_, footpath_ranks_, partition_);
}

cista::wrapped<route_rank_store> route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<route_rank_store>(path);
}

void route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void route_rank_store::print_summary(std::ostream&) const {
  std::vector<size_t> route_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> dep_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> arr_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> footpath_rank_counts(partition_.n_levels_ + 1, 0ULL);

  auto const n_routes = route_rank_start_idx_.size();
  for (auto r = route_idx_t{0}; r < n_routes; ++r) {
    const auto r_from = route_rank_start_idx_[r];
    const auto r_to = route_rank_start_idx_[r + 1];
    route_rank_counts[to_idx(route_ranks_[r_from])]++;

    bool is_dep = true;
    for (auto r_rank_idx = r_from + 1; r_rank_idx < r_to; ++r_rank_idx) {
      if (is_dep) {
        dep_rank_counts[to_idx(route_ranks_[r_rank_idx])]++;
      } else {
        arr_rank_counts[to_idx(route_ranks_[r_rank_idx])]++;
      }

      is_dep = !is_dep;
    }
  }

  for (auto fp = footpath_idx_t{0U}; fp < footpath_ranks_.size(); ++fp) {
    footpath_rank_counts[to_idx(footpath_ranks_[fp])]++;
  }

  auto const n_sum_arr_dep_events =
      route_ranks_.size() - (route_rank_start_idx_.size() - 1);
  utl::verify(
      n_sum_arr_dep_events % 2 == 0,
      "There should be the same number of arrival and departure events");

  auto const n_arr_dep_events = n_sum_arr_dep_events / 2;

  std::cout << "Counts per rank: " << std::endl;
  for (size_t rank = 0U; rank <= partition_.n_levels_; ++rank) {
    std::cout << "  rank=" << std::left << std::setw(10) << rank << ": " << route_rank_counts[rank] << "/" << n_routes << " routes, "
    << dep_rank_counts[rank] << "/" << n_arr_dep_events << " dep route events, "
    << arr_rank_counts[rank] << "/" << n_arr_dep_events << " arr route events, "
    << footpath_rank_counts[rank] << "/" << footpath_ranks_.size() << " footpath ranks" << std::endl;
  }
}

} // namespace nigiri::routing::para
