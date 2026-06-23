#include "nigiri/routing/raptor/para/route_rank_store.h"
#include "boost/json.hpp"

namespace nigiri::routing::para {

auto plain_route_rank_store::cista_members() {
  return std::tie(route_ranks_,
                  route_event_ranks_,
                  partition_);
}

cista::wrapped<plain_route_rank_store> plain_route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<plain_route_rank_store>(path);
}

void plain_route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void plain_route_rank_store::print_summary(std::ostream&, timetable const& tt) const {
  std::vector<size_t> route_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> route_departure_event_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> route_arrival_event_rank_counts(partition_.n_levels_ + 1, 0ULL);

  auto const n_routes = tt.n_routes();
  size_t n_dep_events = 0U;
  size_t n_arr_events = 0U;
  for (auto r = route_idx_t{0}; r < n_routes; ++r) {
    route_rank_counts[to_idx(route_ranks_[r])]++;
    const auto n_stops = tt.route_location_seq_[r].size();

    const auto route_event_ranks = route_event_ranks_[r];
    for (auto i = 0U; i < n_stops; ++i) {
      if (i == 0U) {
        route_departure_event_rank_counts[to_idx(route_event_ranks.front())]++;
        n_dep_events++;
        continue;
      } else if (i == n_stops - 1U) {
        n_arr_events++;
        route_arrival_event_rank_counts[to_idx(route_event_ranks.back())]++;
        continue;
      }

      route_departure_event_rank_counts[to_idx(route_event_ranks[i * 2])]++;
      route_arrival_event_rank_counts[to_idx(route_event_ranks[i * 2 - 1])]++;
      n_arr_events++;
      n_dep_events++;
    }
  }

  std::cout << "Counts per rank: " << std::endl;
  for (size_t rank = 0U; rank <= partition_.n_levels_; ++rank) {
    std::cout << "  rank=" << std::left << std::setw(10) << rank << ": " << route_rank_counts[rank] << "/" << n_routes << " routes, "
    << route_departure_event_rank_counts[rank] << "/" << n_dep_events << " departure events, "
    << route_arrival_event_rank_counts[rank] << "/" << n_arr_events << " arrival events" << std::endl;
  }
}

void plain_route_rank_store::digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks) {
  route_ranks_.clear();
  route_ranks_.resize(tt.n_routes(), rank_t{0U});

  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    auto max_rank = rank_t{0U};
    for (const auto event_rank : route_event_ranks[route_idx]) {
      max_rank = std::max(max_rank, event_rank);
    }
    route_ranks_[route_idx] = max_rank;
  }

  route_event_ranks_ = std::move(route_event_ranks);
  partition_ = std::move(partition);
}

auto skip_list_route_rank_store_lcl_packed::cista_members() {
  return std::tie(route_ranks_,
                  scan_stop_starts_,
                  scan_stops_,
                  partition_);
}

cista::wrapped<skip_list_route_rank_store_lcl_packed> skip_list_route_rank_store_lcl_packed::read(std::filesystem::path const& path) {
  return cista::read<skip_list_route_rank_store_lcl_packed>(path);
}

void skip_list_route_rank_store_lcl_packed::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void skip_list_route_rank_store_lcl_packed::digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks) {
  route_ranks_.clear();
  route_ranks_.resize(tt.n_routes(), rank_t{0U});
  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    auto max_rank = rank_t{0U};
    for (const auto event_rank : route_event_ranks[route_idx]) {
      max_rank = std::max(max_rank, event_rank);
    }
    route_ranks_[route_idx] = max_rank;
  }
  scan_stop_starts_.clear();
  scan_stops_.clear();
  const auto max_rank = partition.n_levels_;
  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    scan_stop_starts_.add_back_sized(max_rank);
    const auto stop_seq = tt.route_location_seq_[route_idx];
    const auto event_ranks = route_event_ranks[route_idx];
    for (auto min_lcl = rank_t{1U}; min_lcl <= max_rank; ++min_lcl) {
      scan_stop_starts_[route_idx][to_idx(min_lcl) - 1] = scan_stops_.size();
      for (stop_idx_t current_stop_idx = 0U; current_stop_idx < stop_seq.size(); ++current_stop_idx) {
        bool scan_arrival = false;
        bool scan_departure = false;
        if (current_stop_idx > 0) {
          const auto arr_rank = event_ranks[(current_stop_idx * 2) - 1];
          if (arr_rank >= min_lcl) {
            scan_arrival = true;
          }
        }

        if (current_stop_idx < stop_seq.size() - 1) {
          const auto dep_rank = event_ranks[(current_stop_idx * 2)];
          if (dep_rank >= min_lcl) {
            scan_departure = true;
          }
        }

        if (!scan_arrival && !scan_departure) {
          continue;
        }

        scan_stops_.emplace_back(
          static_cast<std::uint16_t>(current_stop_idx),
          static_cast<std::uint16_t>(scan_arrival ? 1 : 0),
          static_cast<std::uint16_t>(scan_departure ? 1 : 0)
        );
      }
      // Sentinel (Scan until here)
      scan_stops_.emplace_back(
        static_cast<std::uint16_t>(stop_seq.size()),
        static_cast<std::uint16_t>(0),
        static_cast<std::uint16_t>(0)
      );
    }
  }

  partition_ = std::move(partition);
}

auto skip_list_route_rank_store_route_packed::cista_members() {
  return std::tie(route_ranks_,
                  scan_stop_starts_,
                  scan_stops_,
                  partition_);
}

cista::wrapped<skip_list_route_rank_store_route_packed> skip_list_route_rank_store_route_packed::read(std::filesystem::path const& path) {
  return cista::read<skip_list_route_rank_store_route_packed>(path);
}

void skip_list_route_rank_store_route_packed::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void skip_list_route_rank_store_route_packed::digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks) {
  route_ranks_.clear();
  route_ranks_.resize(tt.n_routes(), rank_t{0U});
  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    auto max_rank = rank_t{0U};
    for (const auto event_rank : route_event_ranks[route_idx]) {
      max_rank = std::max(max_rank, event_rank);
    }
    route_ranks_[route_idx] = max_rank;
  }
  scan_stop_starts_.clear();
  scan_stops_.clear();
  const auto max_rank = partition.n_levels_;
  for (auto min_lcl = rank_t{1U}; min_lcl <= max_rank; ++min_lcl) {
    scan_stop_starts_.add_back_sized(tt.n_routes());
    for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
      const auto stop_seq = tt.route_location_seq_[route_idx];
      const auto event_ranks = route_event_ranks[route_idx];
      scan_stop_starts_[to_idx(min_lcl) - 1][to_idx(route_idx)] = scan_stops_.size();
      for (stop_idx_t current_stop_idx = 0U; current_stop_idx < stop_seq.size(); ++current_stop_idx) {
        bool scan_arrival = false;
        bool scan_departure = false;
        if (current_stop_idx > 0) {
          const auto arr_rank = event_ranks[(current_stop_idx * 2) - 1];
          if (arr_rank >= min_lcl) {
            scan_arrival = true;
          }
        }

        if (current_stop_idx < stop_seq.size() - 1) {
          const auto dep_rank = event_ranks[(current_stop_idx * 2)];
          if (dep_rank >= min_lcl) {
            scan_departure = true;
          }
        }

        if (!scan_arrival && !scan_departure) {
          continue;
        }

        scan_stops_.emplace_back(
          static_cast<std::uint16_t>(current_stop_idx),
          static_cast<std::uint16_t>(scan_arrival ? 1 : 0),
          static_cast<std::uint16_t>(scan_departure ? 1 : 0)
        );
      }
      // Sentinel (Scan until here)
      scan_stops_.emplace_back(
        static_cast<std::uint16_t>(stop_seq.size()),
        static_cast<std::uint16_t>(0),
        static_cast<std::uint16_t>(0)
      );
    }
  }

  partition_ = std::move(partition);
}

auto bitvec_route_rank_store::cista_members() {
  return std::tie(route_ranks_,
                  first_block_idx_,
                  blocks_,
                  partition_);
}

cista::wrapped<bitvec_route_rank_store> bitvec_route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<bitvec_route_rank_store>(path);
}

void bitvec_route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void bitvec_route_rank_store::digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks) {
  route_ranks_.clear();
  route_ranks_.resize(tt.n_routes(), rank_t{0U});
  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    auto max_rank = rank_t{0U};
    for (const auto event_rank : route_event_ranks[route_idx]) {
      max_rank = std::max(max_rank, event_rank);
    }
    route_ranks_[route_idx] = max_rank;
  }
  first_block_idx_.clear();
  blocks_.clear();
  const auto max_rank = partition.n_levels_;
  for (auto min_lcl = rank_t{1U}; min_lcl <= max_rank; ++min_lcl) {
    // + 1 because of sentinel
    first_block_idx_.add_back_sized(tt.n_routes() + 1);
    for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
      const auto stop_seq = tt.route_location_seq_[route_idx];
      const auto event_ranks = route_event_ranks[route_idx];
      first_block_idx_[to_idx(min_lcl) - 1][to_idx(route_idx)] = blocks_.size();

      const auto n_stops = stop_seq.size();
      bitvec bv(n_stops * 2);
      utl::fill(bv.blocks_, 0U);
      for (stop_idx_t current_stop_idx = 0U; current_stop_idx < n_stops; ++current_stop_idx) {
        bool scan_departure = false;
        bool scan_arrival = false;
        if (current_stop_idx > 0) {
          const auto arr_rank = event_ranks[(current_stop_idx * 2) - 1];
          if (arr_rank >= min_lcl) {
            scan_arrival = true;
          }
        }

        if (current_stop_idx < stop_seq.size() - 1) {
          const auto dep_rank = event_ranks[(current_stop_idx * 2)];
          if (dep_rank >= min_lcl) {
            scan_departure = true;
          }
        }

        if (!scan_arrival && !scan_departure) {
          continue;
        }

        stop_idx_t const dep_idx = current_stop_idx * 2;
        stop_idx_t const arr_idx = dep_idx + 1;

        if (scan_departure) {
          bv.set(dep_idx, true);
        }
        if (scan_arrival) {
          bv.set(arr_idx, true);
        }
      }
      blocks_.insert(blocks_.end(), bv.blocks_.begin(), bv.blocks_.end());
    }
    // Sentinel
    first_block_idx_[to_idx(min_lcl) - 1][tt.n_routes()] = blocks_.size();
  }

  partition_ = std::move(partition);
}

} // namespace nigiri::routing::para
