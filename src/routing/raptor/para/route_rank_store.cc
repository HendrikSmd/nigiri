#include "nigiri/routing/raptor/para/route_rank_store.h"

namespace nigiri::routing::para {

auto route_rank_store::cista_members() {
  return std::tie(route_rank_start_idx_, ranks_, partition_);
}

cista::wrapped<route_rank_store> route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<route_rank_store>(path);
}

void route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void route_rank_store::print_summary(std::ostream&) const {
  std::vector<size_t> route_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> transfer_rank_counts(partition_.n_levels_ + 1, 0ULL);

  auto const n_routes = route_rank_start_idx_.size();
  for (auto r = route_idx_t{0}; r < n_routes; ++r) {
    const auto r_from = route_rank_start_idx_[r];
    const auto r_to = route_rank_start_idx_[r + 1];
    route_rank_counts[to_idx(ranks_[r_from])]++;

    for (auto r_rank_idx = r_from + 1; r_rank_idx < r_to; ++r_rank_idx) {
      transfer_rank_counts[to_idx(ranks_[r_rank_idx])]++;
    }
  }

  const auto n_transfers = ranks_.size() - (route_rank_start_idx_.size() - 1);

  std::cout << "Counts per rank: " << std::endl;
  for (size_t rank = 0U; rank <= partition_.n_levels_; ++rank) {
    std::cout << "  rank=" << std::left << std::setw(10) << rank << ": " << route_rank_counts[rank] << "/" << n_routes << " routes, "
    << transfer_rank_counts[rank] << "/" << n_transfers << " transfers" << std::endl;
  }
}

} // namespace nigiri::routing::para
