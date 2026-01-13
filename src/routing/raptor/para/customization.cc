#include "nigiri/routing/raptor/para/customization.h"

#include <thread>
#include <stop_token>

#include "boost/asio/post.hpp"
#include "boost/asio/thread_pool.hpp"

#include "utl/zip.h"

#include "nigiri/loader/gtfs/route.h"
#include "nigiri/routing/raptor/para/mc_raptor_search.h"

#include "utl/parallel_for.h"

namespace nigiri::routing::para {

customizer::customizer(timetable const& tt) :
  tt_(tt), finished_(false) {}

route_rank_store customizer::construct_route_rank_store(route_partition partition,
                                                        unsigned const n_threads) {
  log(log_lvl::info, "customization", "using {} threads on timetable from {} to {}",
    n_threads,
    tt_.external_interval().from_,
    tt_.external_interval().to_
  );

  initialize(partition);
  for (cista::base_t<cell_idx_t> level = 0U; level <= partition.n_levels_; ++level) {
    // Creating a new thread pool for any level may be
    // overkill but reusing the old one for a new level
    // does not work, since after pool.wait(), the pool
    // does not accept anymore tasks
    log(log_lvl::info, "customization", "starting to process {} cells on level {}", partition.get_num_of_cells_on_level(level), level);

    finished_.store(false);
    std::thread logger_thread([this] {
      while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5U));
        if (finished_) {
          return;
        }

        log_progress();
      }
    });

    std::vector<thread_task> tasks;
    for (auto cell_idx = cell_idx_t{0U}; cell_idx < partition.get_num_of_cells_on_level(level); ++cell_idx) {
      cell_cut_stops_[to_idx(cell_idx)].for_each_set_bit([&](uint64_t const idx) {
        tasks.emplace_back(cell_idx, location_idx_t{idx}, level);
      });
    }
    utl::parallel_for(tasks, [&](auto&& t) {
      this->cut_stop_routing_task(t);
    });
    finished_.store(true);
    logger_thread.join();
    prepare_next_level();
    if (std::ranges::all_of(cell_cut_stops_, [](const bitvec& bv){return !bv.any();})) {
      log(log_lvl::info, "customization", "No more cut stops. Terminating shortly");
      log_progress();
      break;
    }
  }

  std::vector<size_t> route_rank_counts(partition.n_levels_ + 1, 0U);
  for (auto r = route_idx_t{0U}; r < tt_.n_routes(); ++r) {
    route_rank_counts[to_idx(route_ranks_[r])]++;
  }

  std::cout << "Final route rank distribution:" << std::endl;
  for (cista::base_t<cell_idx_t> level = 0U; level <= partition.n_levels_; ++level) {
    std::cout << "Level " << level << ": " << route_rank_counts[level] << "/" << tt_.n_routes() << std::endl;
  }


  return route_rank_store(std::move(route_ranks_),
                          std::move(transport_ranks_),
                          std::move(partition));
}

void customizer::log_progress() const {
    std::cout << "Progress Update:\n";
    for (auto c_idx = cell_idx_t{0U}; c_idx < static_cast<cista::base_t<cell_idx_t>>(cell_cut_stops_.size()); ++c_idx) {
      std::cout << "Cell " << c_idx << ": " << cell_progress_[to_idx(c_idx)] << "/" << cell_cut_stops_[to_idx(c_idx)].count() << std::endl;
    }
    std::cout << std::endl;
}

void customizer::initialize(route_partition const& p) {
  auto n_of_cells_on_first_lvl = p.get_num_of_cells_on_level(0U);
  cell_mutexes_.clear();
  cell_mutexes_.resize(n_of_cells_on_first_lvl);

  cell_progress_.clear();
  cell_progress_.resize(n_of_cells_on_first_lvl, 0U);

  updated_routes_.clear();
  route_masks_.clear();
  transfer_masks_.clear();
  used_transfers_.clear();
  cell_cut_stops_.clear();
  cmpnt_to_current_lvl_cell_idxs_.clear();

  route_ranks_.clear();
  transport_ranks_.clear();

  updated_routes_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  route_masks_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  transfer_masks_.resize(n_of_cells_on_first_lvl, bitvec::max(tt_.n_locations()));
  used_transfers_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});
  cell_cut_stops_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});


  route_ranks_.resize(tt_.n_routes(), route_rank_t{0U});
  transport_ranks_.resize(tt_.transport_route_.size(), transport_rank_t{0U}); // this is a little bit dirty

  initialize_route_masks(p);
  append_cmpnt_cell_idxs(p, cmpnt_to_current_lvl_cell_idxs_);
  // The cut stops have to be initialized after
  // the (cmpnt -> cell_idxs) have been initialized!!!
  initialize_cut_stops();
  // must happen after cut stops have been initialized
  initialize_used_transfers();
}

void customizer::initialize_route_masks(route_partition const& p) {
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    auto const cell_idx_of_route = to_idx(p.route_to_cell_idx_[route_idx]);
    auto& bv = route_masks_[cell_idx_of_route];
    bv.set(to_idx(route_idx));
  }
}

void customizer::initialize_cut_stops() {
  for (auto c = component_idx_t{0}; c < tt_.component_locations_.size(); ++c) {
    const auto& cell_idxs = cmpnt_to_current_lvl_cell_idxs_[to_idx(c)];
    if (cell_idxs.size() <= 1) {
      // not a cut component -> cannot contain cut stop
      continue;
    }

    // cell_idxs.size() >= 2 ---> c is a cut component
    for (const auto& cell_idx : cell_idxs) {
      const auto& cmpnt_locs = tt_.component_locations_[c];
      auto& cut_stop_bv = cell_cut_stops_[to_idx(cell_idx)];
      for (location_idx_t l_idx : cmpnt_locs) {
        cut_stop_bv.set(to_idx(l_idx), true);
      }
    }
  }
}

void customizer::initialize_used_transfers() {
  for (auto [used_transfer_mask, cut_stop_mask] : utl::zip(used_transfers_, cell_cut_stops_)) {
    used_transfer_mask.zero_out();
    used_transfer_mask |= cut_stop_mask;
  }
}

void customizer::prepare_next_level() {
  assert(std::has_single_bit(route_masks_.size()));
  assert(route_masks_.size() == updated_routes_.size());

  auto n_cells_in_next_level = route_masks_.size() >> 1U;
  cell_mutexes_.resize(n_cells_in_next_level);

  // 1. update route masks
  // we only need to consider marked routes
  for (auto [route_mask, updated_routes] : utl::zip(route_masks_, updated_routes_)) {
    route_mask &= updated_routes;
  }
  // 2. clear route marks for next level
  updated_routes_.resize(n_cells_in_next_level);
  for (auto& bv : updated_routes_) {
    bv.zero_out();
  }
  // 3. unite route masks (parent routes = union of children routes)
  unite_route_masks();
  // 4. unite cut stop masks. A cut stop of cell C on level i + 1 is a
  // cut stop on level i for one (or both) of C's children
  unite_cut_stops();

  // 5. update incident cell indexes of
  // components for next level
  update_cmpnt_cell_idxs_next_level();

  unite_used_transfers();
  transfer_masks_.resize(n_cells_in_next_level);
  std::swap(transfer_masks_, used_transfers_);
  initialize_used_transfers();
  std::ranges::fill(cell_progress_, 0U);
}

inline void binary_or_reduce(std::vector<bitvec>& vec) {
  assert(vec.size() > 1U);
  assert(std::has_single_bit(vec.size()));
  // [m1, m2, m3, m4, ..., mn-1, mn] ->
  // [m1 OR m2, m3 OR m4, ..., mn-1 OR mn]
  auto new_size = vec.size() >> 1U;
  for (size_t i = 0U; i < new_size; ++i) {
    vec[i] = vec[i * 2] | vec[i * 2 + 1];
  }
  vec.resize(new_size);
}

void customizer::unite_route_masks() {
  binary_or_reduce(route_masks_);
}

void customizer::unite_cut_stops() {
  binary_or_reduce(cell_cut_stops_);
}

void customizer::unite_used_transfers() {
  binary_or_reduce(used_transfers_);
}

void customizer::cut_stop_routing_task(const thread_task& task) {
  const auto cut_stop_from = task.location_idx_;
  const auto cell = task.cell_idx_;
  const auto level = task.level_;
  const auto& res = mc_raptor_search(tt_,
                                     cut_stop_from,
                                     cell_cut_stops_[to_idx(cell)],
                                     route_masks_[to_idx(cell)],
                                     bitvec::max(tt_.n_locations()),
                                     true);
  std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);
  for (const auto& journeys : res) {
    for (const auto& j : journeys) {
      update_ranks_for(j, level, cell);
    }
  }
  cell_progress_[to_idx(cell)]++;
}

void customizer::append_cmpnt_cell_idxs(route_partition const& partition,
                                        std::vector<std::vector<cell_idx_t>>& cmpnt_to_cell_idxs) {
  auto const n_components= tt_.component_locations_.size();
  for (auto c_idx = component_idx_t{0}; c_idx < n_components; ++c_idx) {

    std::vector<cell_idx_t> cell_idxs;
    for (const auto& loc : tt_.component_locations_[c_idx]) {
      const auto& routes_of_loc = tt_.location_routes_[loc];

      if (routes_of_loc.empty()) {
        continue;
      }

      // map route indexes to their cell indexes, append them to cell_idxs
      std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
        return partition.route_to_cell_idx_[r];
      });
    }
    // we sort them to make sure that all
    // equal cell_idxs are adjacent to each other
    // this is needed for std::unique
    std::ranges::sort(cell_idxs);
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    cell_idxs.erase(last, cell_idxs.end());

    cmpnt_to_cell_idxs.emplace_back(cell_idxs);
  }
}


void customizer::update_cmpnt_cell_idxs_next_level() {
  auto n_components = tt_.component_locations_.size();
  for (auto c = component_idx_t{0}; c < n_components; ++c) {
    auto& cell_idxs = cmpnt_to_current_lvl_cell_idxs_[to_idx(c)];
    auto before_size = cell_idxs.size();
    std::ranges::transform(cell_idxs, cell_idxs.begin(), [](cell_idx_t const& cell_idx) {
      return route_partition::get_parent_idx(cell_idx, 1);
    });
    // above transform simple shifts every cell_idx by one
    // to the right, effectively dividing it by 2. This
    // gives us the cell_idx of the parent cell on the
    // next higher level.
    // In particular: The cell indexes stay sorted!
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    // ... and remove duplicates
    cell_idxs.erase(last, cell_idxs.end());

    // no longer a cut component
    if (cell_idxs.size() == 1 && before_size > 1) {
      auto& cell_bv = cell_cut_stops_[to_idx(cell_idxs.front())];
      for (const auto& loc : tt_.component_locations_[c]) {
        cell_bv.set(to_idx(loc), false);
      }
    }
  }
}

inline bool uses_transport(const journey::leg& l) {
  return holds_alternative<journey::run_enter_exit>(l.uses_);
}

void customizer::update_ranks_for(journey const& j,
                                  cista::base_t<cell_idx_t> const level,
                                  cell_idx_t const cell) {
  for (const auto& leg : j.legs_) {
    auto& used_transfers_bv = used_transfers_[to_idx(cell)];
    used_transfers_bv.set(to_idx(leg.from_), true);
    used_transfers_bv.set(to_idx(leg.to_), true);
    if (uses_transport(leg)) {
      const auto& run = std::get<journey::run_enter_exit>(leg.uses_);
      const auto transport_idx = run.r_.t_.t_idx_;
      const auto route_idx = tt_.transport_route_[transport_idx];

      // make sure our journey does not contain a leg that
      // uses a route we masked out
      assert(route_masks_[to_idx(cell)][to_idx(route_idx)]);

      route_ranks_[route_idx] = route_rank_t{level + 1};
      transport_ranks_[transport_idx] = transport_rank_t{level + 1};

      // mark route as updated for the next level
      updated_routes_[to_idx(cell)].set(to_idx(route_idx), true);
    }
  }
}





route_rank_store::route_rank_store(vector_map<route_idx_t, route_rank_t>&& route_ranks,
                                   vector_map<transport_idx_t, transport_rank_t>&& transport_ranks,
                                   route_partition&& p) :
  route_ranks_(std::move(route_ranks)),
  transport_ranks_(std::move(transport_ranks)),
  partition_(std::move(p)) {}

auto route_rank_store::cista_members() {
  return std::tie(route_ranks_, transport_ranks_, partition_);
}

void route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

void route_rank_store::print_summary(std::ostream&) const {
  std::vector<size_t> route_rank_counts(partition_.n_levels_ + 1, 0ULL);
  std::vector<size_t> transport_rank_counts(partition_.n_levels_ + 1, 0ULL);

  auto const n_routes = route_ranks_.size();
  for (auto r = route_idx_t{0}; r < n_routes; ++r) {
    route_rank_counts[to_idx(route_ranks_[r])]++;
  }

  auto const n_transports = transport_ranks_.size();
  for (auto t = transport_idx_t{0}; t < n_transports; ++t) {
    transport_rank_counts[to_idx(transport_ranks_[t])]++;
  }

  std::cout << "Counts per rank: " << std::endl;
  for (size_t rank = 0U; rank <= partition_.n_levels_; ++rank) {
    std::cout << "  rank=" << std::left << std::setw(10) << rank << ": " << route_rank_counts[rank] << "/" << n_routes << " routes, "
    << transport_rank_counts[rank] << "/" << n_transports << " transports" << std::endl;
  }
}

cista::wrapped<route_rank_store> route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<route_rank_store>(path);
}

} // namespace nigiri::routing::para
