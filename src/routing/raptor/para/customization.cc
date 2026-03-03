#include "nigiri/routing/raptor/para/customization.h"

#include <thread>
#include <stop_token>

#include "boost/asio/thread_pool.hpp"
#include "utl/zip.h"
#include "utl/parallel_for.h"
#include "nigiri/loader/gtfs/route.h"

#include "nigiri/routing/raptor/para/mc_raptor.h"
#include "nigiri/routing/raptor/para/mc_raptor_search.h"
#include "nigiri/routing/raptor/para/bmc_raptor.h"
#include "nigiri/routing/raptor/para/bmc_raptor_state.h"


namespace nigiri::routing::para {

customizer::customizer(timetable const& tt) :
  tt_(tt) {}

route_rank_store const& customizer::construct_route_rank_store(route_partition partition) {
  log(log_lvl::info, "customization", "on timetable from {} to {}",
    tt_.external_interval().from_,
    tt_.external_interval().to_
  );

  std::atomic_bool level_finished{false};
  initialize(partition);
  for (cista::base_t<cell_idx_t> level = 0U; level <= partition.n_levels_; ++level) {
    log(log_lvl::info, "customization", "starting to process {} cells on level {}", partition.get_num_of_cells_on_level(level), level);

    level_finished.store(false);
    std::thread logger_thread([&] {
      while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5U));
        if (level_finished) {
          return;
        }

        log_progress();
      }
    });

    std::vector<thread_task> tasks;
    for (auto cell_idx = cell_idx_t{0U}; cell_idx < partition.get_num_of_cells_on_level(level); ++cell_idx) {
      auto cut_cmpnt_bin_range_iter = start_times_registry_.cell_cmpnt_search_bins_[to_idx(cell_idx)].cbegin();
      cell_cut_cmpnts_[to_idx(cell_idx)].for_each_set_bit([&](uint64_t const idx) {
        tasks.emplace_back(cell_idx, component_idx_t{idx}, level, cut_cmpnt_bin_range_iter);
        ++cut_cmpnt_bin_range_iter;
      });
    }
    if constexpr (search_algo == search_algorithm::mc_raptor) {
      utl::parallel_for_run_threadlocal<mc_raptor_state>(tasks.size(), [&](auto& state, auto const t_idx) {
        this->cut_routing_task(tasks[t_idx], state);
      });
    } else {
      utl::parallel_for_run_threadlocal<bmc_raptor_state>(tasks.size(), [&](auto& state, auto const t_idx) {
        this->cut_routing_task(tasks[t_idx], state);
      });
    }
    level_finished.store(true);
    logger_thread.join();
    prepare_next_level();
    if (std::ranges::all_of(cell_cut_cmpnts_, [](const bitvec& bv){return !bv.any();})) {
      log(log_lvl::info, "customization", "No more cut components. Terminating shortly");
      log_progress();
      break;
    }
  }

  route_rank_store_.partition_ = std::move(partition);
  return route_rank_store_;
}

void customizer::log_progress() const {
    std::cout << "Progress Update:\n";
    for (auto cell_idx = cell_idx_t{0U}; cell_idx < static_cast<cista::base_t<cell_idx_t>>(cell_cut_cmpnts_.size()); ++cell_idx) {
      std::cout << "Cell " << cell_idx << ": " << cell_progress_[cista::to_idx(cell_idx)] << "/" << cell_cut_cmpnts_[cista::to_idx(cell_idx)].count() << std::endl;
    }
    std::cout << std::endl;
}

void customizer::initialize_ranks() {
  route_rank_store_.ranks_.clear();
  route_rank_store_.route_rank_ranges_.clear();

  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    const auto route_range_begin = route_rank_store_.ranks_.size();
    const auto n_stops = tt_.route_location_seq_[route_idx].size();
    const auto n_route_rank_entries = 1 + ((2 * n_stops) - 2);
    const auto route_range_end = route_range_begin + n_route_rank_entries;

    route_rank_store_.route_rank_ranges_.emplace_back(route_range_begin, route_range_end);
    route_rank_store_.ranks_.resize(route_rank_store_.ranks_.size() + n_route_rank_entries, rank_t{0U});
  }
}

void customizer::initialize(route_partition const& p) {
  auto const timer = scoped_timer("initializing customization process");

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
  cell_cut_cmpnts_.clear();
  cmpnt_to_current_lvl_cell_idxs_.clear();
  start_times_registry_.clear();

  updated_routes_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  route_masks_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  transfer_masks_.resize(n_of_cells_on_first_lvl, bitvec::max(tt_.n_locations()));
  used_transfers_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});
  cell_cut_stops_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});
  cell_cut_cmpnts_.resize(n_of_cells_on_first_lvl, bitvec{tt_.component_locations_.size()});


  initialize_ranks();

  initialize_route_masks(p);
  append_cmpnt_cell_idxs(p, cmpnt_to_current_lvl_cell_idxs_);
  // The cut stops have to be initialized after
  // the (cmpnt -> cell_idxs) have been initialized!!!
  initialize_cut_stops();
  // must happen after cut stops have been initialized
  initialize_used_transfers();
  start_times_registry_.populate(tt_, n_of_cells_on_first_lvl,
                                 cmpnt_to_current_lvl_cell_idxs_,
                                 route_masks_, !use_initial_footpaths);
}

void customizer::initialize_route_masks(route_partition const& p) {
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    auto const cell_idx_of_route = to_idx(p.route_to_cell_idx_[route_idx]);
    auto& bv = route_masks_[cell_idx_of_route];
    bv.set(to_idx(route_idx));
  }
}

void customizer::initialize_cut_stops() {
  for (auto cmpnt_idx = component_idx_t{0}; cmpnt_idx < tt_.component_locations_.size(); ++cmpnt_idx) {
    const auto& cell_idxs = cmpnt_to_current_lvl_cell_idxs_[to_idx(cmpnt_idx)];
    if (cell_idxs.size() <= 1) {
      // not a cut component -> cannot contain cut stop
      continue;
    }

    // cell_idxs.size() >= 2 ---> c is a cut component
    for (const auto& cell_idx : cell_idxs) {
      const auto& cmpnt_locs = tt_.component_locations_[cmpnt_idx];
      auto& cut_stop_bv = cell_cut_stops_[to_idx(cell_idx)];
      auto& cut_cmpnt_bv = cell_cut_cmpnts_[to_idx(cell_idx)];
      cut_cmpnt_bv.set(cista::to_idx(cmpnt_idx), true);
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
  auto const timer = scoped_timer{"preparing next level"};
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
  unite_cut_cmpnts();

  // 5. update incident cell indexes of
  // components for next level
  update_cmpnt_cell_idxs_next_level();

  unite_used_transfers();
  transfer_masks_.resize(n_cells_in_next_level);
  std::swap(transfer_masks_, used_transfers_);
  initialize_used_transfers();

  start_times_registry_.populate(tt_, n_cells_in_next_level,
                                 cmpnt_to_current_lvl_cell_idxs_,
                                 route_masks_, !use_initial_footpaths);

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

void customizer::unite_cut_cmpnts() {
  binary_or_reduce(cell_cut_cmpnts_);
}

void customizer::cut_routing_task(const thread_task& task, bmc_raptor_state& bmc_state) {
#ifdef NIGIRI_ENABLE_SIMD
  bmc_round_meta_data initial_md{
    .route_idx_ = 0U,
    .parent_bag_idx_ = 0U,
    .enter_stop_idx_ = 0U,
    .exit_stop_idx_ = 0U,
    .has_parent_ = 0,
    .is_footpath_ = 0
  };
#endif

  const auto cut_cmpnt_from = task.component_idx_;
  const auto cell = task.cell_idx_;
  const auto level = task.level_;

  const auto [bin_from, bin_to] = *task.iter_;

  const auto& cmpnt_locs = tt_.component_locations_[cut_cmpnt_from];
  for (auto bin_i = bin_from; bin_i < bin_to; ++bin_i) {
    const auto bin_begin_idx = start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_i];
    const auto bin_end_idx = start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_i + 1];
    bmc_raptor raptor{tt_,
                      bmc_state,
                      cell_cut_stops_[to_idx(cell)],
                      route_masks_[to_idx(cell)],
                      transfer_masks_[to_idx(cell)]};

    for (auto dep_event_i = bin_begin_idx; dep_event_i < bin_end_idx; ++dep_event_i) {
      const auto& rel_dep_event = start_times_registry_.cmpnt_dep_events_buffer_[to_idx(cell)][dep_event_i];
      const auto from_loc = cmpnt_locs[cista::to_idx(rel_dep_event.dep_loc_)];

      search_bitfield sbf;
      truncate_to(rel_dep_event.active_days_, sbf);
      const auto dep = static_cast<uint16_t>(rel_dep_event.dep_min_after_midnight_.count());
#ifdef NIGIRI_ENABLE_SIMD
      bool added = bmc_raptor::add_to_non_dest_round_bag(
        bmc_state.round_bags_[0U][to_idx(from_loc)],
        {dep, dep, dep},
        initial_md,
        sbf
      );
      if (added) {
        bmc_raptor::add_to_non_dest_round_bag(
          bmc_state.best_bags_[to_idx(from_loc)],
          {dep, dep, dep},
          initial_md,
          sbf
        );
        bmc_state.station_mark_.set(cista::to_idx(from_loc), true);
      }
#else
      bool added = bmc_raptor::add_to_non_dest_round_bag(
          bmc_state.round_bags_[0U][to_idx(from_loc)],
          {.route_idx_ = 0U,
           .enter_stop_idx_ = 0U,
           .exit_stop_idx_ = 0U,
           .arrival_ = dep,
           .parent_bag_idx_ = 0U,
           .arrival_with_transfer_ = dep,
           .departure_ = dep,
           .is_footpath_ = 0,
           .has_parent_ = 0},
          sbf);
      if (added) {
        bmc_raptor::add_to_non_dest_round_bag(
            bmc_state.best_bags_[to_idx(from_loc)],
            {.route_idx_ = 0U,
             .enter_stop_idx_ = 0U,
             .exit_stop_idx_ = 0U,
             .arrival_ = dep,
             .parent_bag_idx_ = 0U,
             .arrival_with_transfer_ = dep,
             .departure_ = dep,
             .is_footpath_ = 0,
             .has_parent_ = 0},
            sbf);
        bmc_state.station_mark_.set(cista::to_idx(from_loc), true);
      }
#endif
    }


    raptor.rounds();
    std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);

    std::vector<bmc_journey> bmc_journey_bag;
    cell_cut_stops_[to_idx(cell)].for_each_set_bit([&](size_t i) {
      raptor.emplace_relative_journeys_for(location_idx_t{i}, bmc_journey_bag);

      for (const auto& bmc_j : bmc_journey_bag) {
        backtrack_and_update_ranks(bmc_j.label_iter_,
                                   bmc_state,
                                   bmc_j.transfers_ + 1,
                                   location_idx_t{i},
                                   level,
                                   cell,
                                   cut_cmpnt_from);
      }


      bmc_journey_bag.clear();
    });

    bmc_state.reset();
  }
  std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);
  cell_progress_[to_idx(cell)]++;
}

void customizer::backtrack_and_update_ranks(bmc_raptor_bag_t::const_iterator root_label,
                                            bmc_raptor_state const& state,
                                            const unsigned k,
                                            location_idx_t const target,
                                            cista::base_t<cell_idx_t> const level,
                                            cell_idx_t const cell,
                                            component_idx_t) {
  auto& used_transfers_bv = used_transfers_[to_idx(cell)];
#ifdef NIGIRI_ENABLE_SIMD
  auto current_label = (*root_label).metadata_;
#else
  auto current_label = root_label->label_;
#endif
  auto current_k = k;
  auto target_loc = target;

  while (current_label.has_parent_ == 1U) {
    const auto route_idx = current_label.route_idx_;

    const auto& stop_sequence = tt_.route_location_seq_[route_idx_t{route_idx}];

    auto const enter_stop_idx = current_label.enter_stop_idx_;
    auto const exit_stop_idx = current_label.exit_stop_idx_;


    auto const enter_stp = stop{stop_sequence[enter_stop_idx]};
    auto const exit_stp = stop{stop_sequence[exit_stop_idx]};

    auto const enter_loc_idx = cista::to_idx(enter_stp.location_idx());
    auto const exit_loc_idx = cista::to_idx(exit_stp.location_idx());

    used_transfers_bv.set(enter_loc_idx, true);
    used_transfers_bv.set(exit_loc_idx, true);
    used_transfers_bv.set(to_idx(target_loc), true);

    if (route_idx != to_idx(route_idx_t::invalid())) {
      auto const [from, _] = route_rank_store_.route_rank_ranges_[route_idx_t{route_idx}];
      const unsigned dep_route_rank_off = 1 + (enter_stop_idx * 2);
      const unsigned arr_route_rank_off = (exit_stop_idx * 2);
      route_rank_store_.ranks_[from] = rank_t{level + 1};
      route_rank_store_.ranks_[from + dep_route_rank_off] = rank_t{level + 1};
      route_rank_store_.ranks_[from + arr_route_rank_off] = rank_t{level + 1};

      // mark route as updated for the next level
      updated_routes_[to_idx(cell)].set(route_idx, true);
    }
#ifdef NIGIRI_ENABLE_SIMD
    current_label = state.round_bags_[current_k - 1][enter_loc_idx].meta_data_[current_label.parent_bag_idx_];
#else
    current_label = state.round_bags_[current_k - 1][enter_loc_idx].labels_[current_label.parent_bag_idx_].label_;
#endif
    current_k--;
    target_loc = location_idx_t{enter_loc_idx};
  }
}

void customizer::cut_routing_task(const thread_task& task, mc_raptor_state& state) {
  const auto cut_cmpnt_from = task.component_idx_;
  const auto cell = task.cell_idx_;
  const auto level = task.level_;

  const auto [bin_from, bin_to] = *task.iter_;


  const auto& cmpnt_locs = tt_.component_locations_[cut_cmpnt_from];
  for (auto bin_i = bin_from; bin_i < bin_to; ++bin_i) {
    const auto bin_begin_idx = start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_i];
    const auto bin_end_idx = start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_i + 1];
    mc_raptor raptor{tt_,
                     state,
                     tt_.internal_interval(),
                     cell_cut_stops_[to_idx(cell)],
                     route_masks_[to_idx(cell)],
                     transfer_masks_[to_idx(cell)]};

    for (auto dep_event_i = bin_begin_idx; dep_event_i < bin_end_idx; ++dep_event_i) {
      const auto& rel_dep_event = start_times_registry_.cmpnt_dep_events_buffer_[to_idx(cell)][dep_event_i];
      rel_dep_event.active_days_.for_each_set_bit([&](uint16_t const day_idx) {
        const auto from_loc = cmpnt_locs[cista::to_idx(rel_dep_event.dep_loc_)];

        const routing_time dep_time = {day_idx_t{day_idx}, rel_dep_event.dep_min_after_midnight_};
        state.round_bags_[0U][cista::to_idx(from_loc)].add(mc_raptor_label(
          dep_time, 0_minutes, dep_time)
        );
        state.best_[cista::to_idx(from_loc)].add(mc_raptor_label(
          dep_time, 0_minutes, dep_time)
        );
        state.station_mark_.set(cista::to_idx(from_loc), true);
      });
    }

    raptor.route();
    std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);
    for (const auto& journeys : state.results_) {
      for (const auto& j : journeys) {
        update_ranks_for(j, level, cell);
      }
    }
    state.reset();
  }
  /*
  for (auto const from_loc : cmpnt_locs) {
    mc_raptor raptor{tt_,
                 state,
                 tt_.internal_interval(),
                 cell_cut_stops_[to_idx(cell)],
                 route_masks_[to_idx(cell)],
                 transfer_masks_[to_idx(cell)]};
    std::vector<start> starts;
    get_starts(direction::kForward, tt_, nullptr, tt_.internal_interval(), {{from_loc, 0_minutes, 0U}}, {}, {}, kMaxTravelTime, location_match_mode::kExact, true, starts, true, kDefaultProfile, {}, route_masks_[to_idx(cell)]);
    utl::equal_ranges_linear(
    starts,
    [](start const& a, start const& b) {
      return a.time_at_start_ == b.time_at_start_;
    },
    [&](auto&& from_it, auto&& to_it) {
      for (auto const& s : it_range{from_it, to_it}) {
        state.round_bags_[0U][to_idx(s.stop_)].add(mc_raptor_label(
            {tt_, s.time_at_stop_}, 0_minutes, {tt_, from_it->time_at_start_}));
        state.best_[to_idx(s.stop_)].add(mc_raptor_label(
            {tt_, s.time_at_stop_}, 0_minutes, {tt_, from_it->time_at_start_}));
        state.station_mark_.set(to_idx(s.stop_), true);
      }
    }
  );

    raptor.route();
    std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);
    for (const auto& journeys : state.results_) {
      for (const auto& j : journeys) {
        update_ranks_for(j, level, cell);
      }
    }
    state.reset();
  }
  */
  std::scoped_lock lock(cell_mutexes_[to_idx(cell)]);
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
  auto const n_components = tt_.component_locations_.size();
  for (auto cmpnt_idx = component_idx_t{0}; cmpnt_idx < n_components; ++cmpnt_idx) {
    auto& cell_idxs = cmpnt_to_current_lvl_cell_idxs_[to_idx(cmpnt_idx)];
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
      auto& cell_cmpnt_bv = cell_cut_cmpnts_[to_idx(cell_idxs.front())];
      cell_cmpnt_bv.set(cista::to_idx(cmpnt_idx), false);

      auto& cell_stop_bv = cell_cut_stops_[to_idx(cell_idxs.front())];
      for (const auto& loc : tt_.component_locations_[cmpnt_idx]) {
        cell_stop_bv.set(to_idx(loc), false);
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
  auto& used_transfers_bv = used_transfers_[to_idx(cell)];
  for (const auto& leg : j.legs_) {
    used_transfers_bv.set(to_idx(leg.from_), true);
    used_transfers_bv.set(to_idx(leg.to_), true);
    if (uses_transport(leg)) {
      const auto& run = std::get<journey::run_enter_exit>(leg.uses_);
      const auto transport_idx = run.r_.t_.t_idx_;
      const auto route_idx = tt_.transport_route_[transport_idx];
      // make sure our journey does not contain a leg that
      // uses a route we masked out
      assert(route_masks_[to_idx(cell)][to_idx(route_idx)]);
      auto const [from, _] = route_rank_store_.route_rank_ranges_[route_idx];
      auto const [from_stop, to_stop_exclusive] = run.stop_range_;
      const unsigned dep_route_rank_off = 1 + (from_stop * 2);
      const unsigned arr_route_rank_off = ((to_stop_exclusive-1) * 2);
      route_rank_store_.ranks_[from] = rank_t{level + 1};
      route_rank_store_.ranks_[from + dep_route_rank_off] = rank_t{level + 1};
      route_rank_store_.ranks_[from + arr_route_rank_off] = rank_t{level + 1};

      // mark route as updated for the next level
      updated_routes_[to_idx(cell)].set(to_idx(route_idx), true);
    }
  }
}

} // namespace nigiri::routing::para
