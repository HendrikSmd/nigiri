#include "nigiri/routing/raptor/para/customization.h"

#include <thread>
#include <stop_token>

#include "utl/zip.h"
#include "nigiri/common/parallel_for_with_args.h"
#include "nigiri/loader/gtfs/route.h"

#include "nigiri/routing/raptor/para/mc_raptor.h"
#include "nigiri/routing/raptor/para/mc_raptor_search.h"
#include "nigiri/routing/raptor/para/bmc_raptor.h"
#include "nigiri/routing/raptor/para/bmc_raptor_state.h"

#include <stack>

#include "absl/strings/internal/str_format/extension.h"

namespace nigiri::routing::para {

bool no_bits_set_in(std::vector<bitvec> const& bitvecs) {
  return std::ranges::all_of(bitvecs, [](const bitvec& bv) {
    return !bv.any();
  });
}

customizer::customizer(timetable const& tt) :
  tt_(tt) {}

void customizer::compute_ranks(route_partition const& partition, vecvec<route_idx_t, rank_t>& out_ranks) {
  log(log_lvl::info, "customization", "on timetable from {} to {}",
    tt_.external_interval().from_,
    tt_.external_interval().to_
  );

  std::atomic_bool level_finished{false};
  initialize(partition);

  atomic_ranks_t atomic_route_ranks(tt_.n_routes());
  atomic_ranks_t atomic_route_event_ranks(tt_.n_route_events());

  std::vector<std::atomic<size_t>> cell_progress(partition.get_num_of_cells_on_level(0U));
  for (std::uint16_t level = 0U; level <= static_cast<std::uint16_t>(partition.n_levels_); ++level) {
    log(log_lvl::info, "customization", "starting to process {} cells on level {}", partition.get_num_of_cells_on_level(static_cast<std::uint8_t>(level)), level);
    std::ranges::fill(cell_progress, 0U);
    level_finished.store(false);
    std::thread logger_thread([&] {
      while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5U));
        if (level_finished) {
          return;
        }

        log_progress(cell_progress);
      }
    });

    std::vector<thread_task> tasks;

    for (auto cell_idx = cell_idx_t{0U}; cell_idx < partition.get_num_of_cells_on_level(static_cast<std::uint8_t>(level)); ++cell_idx) {
      auto cut_cmpnt_bin_range_iter = start_times_registry_.cell_cmpnt_search_bins_[to_idx(cell_idx)].cbegin();
      cell_cut_components_[to_idx(cell_idx)].for_each_set_bit([&](uint64_t const idx) {
        const auto [bin_from, bin_to] = *cut_cmpnt_bin_range_iter;
        for (auto bin_i = bin_from; bin_i < bin_to; ++bin_i) {
          tasks.emplace_back(cell_idx,
                             component_idx_t{idx},
                             level,
                             bin_i,
                             atomic_route_ranks,
                             atomic_route_event_ranks);
        }
        ++cut_cmpnt_bin_range_iter;
      });
    }
    if constexpr (search_algo == search_algorithm::mc_raptor) {
      parallel_for_run_threadlocal<local_thread_context>(tasks.size(), [&](auto& state, auto const t_idx) {
        this->mc_cut_routing_task(tasks[t_idx], state, cell_progress);
      }, utl::noop_progress_update{}, utl::parallel_error_strategy::QUIT_EXEC, std::ref(tt_));
    } else {
      parallel_for_run_threadlocal<local_thread_context>(tasks.size(), [&](auto& context, auto const t_idx) {
        this->bmc_cut_routing_task(tasks[t_idx], context, cell_progress);
      }, utl::noop_progress_update{}, utl::parallel_error_strategy::QUIT_EXEC, std::ref(tt_));
    }
    level_finished.store(true);
    logger_thread.join();

    mark_routes_and_events_from_ranks(partition,
                                      static_cast<std::uint8_t>(level),
                                      atomic_route_ranks,
                                      atomic_route_event_ranks);
    prepare_next_level();
    if (no_bits_set_in(cell_cut_components_)) {
      log(log_lvl::info, "customization", "No more cut components. Terminating shortly");
      break;
    }
  }
  materialize_atomic_ranks(atomic_route_event_ranks, out_ranks);
}

void customizer::log_progress(std::vector<std::atomic<size_t>> const& cell_progress) const {
    std::cout << "Progress Update:\n";
    for (auto cell_idx = cell_idx_t{0U}; cell_idx < static_cast<cista::base_t<cell_idx_t>>( cell_cut_components_.size()); ++cell_idx) {
      const size_t progress = cell_progress[to_idx(cell_idx)];
      const auto limit = start_times_registry_.bin_start_idxs_[cista::to_idx(cell_idx)].size() - 1;
      if (progress == 0) {
        continue;
      }
      std::cout << "Cell " << cell_idx << ": " << progress << "/" << limit
      << ((progress == limit) ? " ✓" : "") <<std::endl;
    }
    std::cout << std::endl;
}

void customizer::initialize(route_partition const& p) {
  auto const timer = scoped_timer("initializing customization process");
  auto const n_of_cells_on_first_lvl = p.get_num_of_cells_on_level(0U);

  marked_routes_.resize(tt_.n_routes());
  marked_routes_.zero_out();

  marked_route_events_.resize(tt_.n_route_events());
  marked_route_events_.zero_out();
  compute_route_event_ranks_index();

  initialize_route_masks(p);
  compute_component_to_cells(p);
  // The cut stops have to be initialized after
  // the (cmpnt -> cell_idxs) have been initialized!!!
  initialize_cut_stops(p);

  start_times_registry_.clear();
  start_times_registry_.populate(tt_, n_of_cells_on_first_lvl,
                                 current_lvl_cells_of_components_,
                                 route_masks_, true);
}

void customizer::initialize_route_masks(route_partition const& p) {
  route_masks_.clear();
  route_masks_.resize(p.get_num_of_cells_on_level(0U), bitvec{tt_.n_routes()});

  auto n_route_events = 0U;
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    const auto n_stops = tt_.route_location_seq_[route_idx].size();
    n_route_events += (2 * n_stops) - 2;

    auto const cell_idx_of_route = to_idx(p.route_to_cell_idx_[route_idx]);
    auto& bv = route_masks_[cell_idx_of_route];
    bv.set(to_idx(route_idx));
  }

  route_event_mask_ = bitvec::max(n_route_events);
}

void customizer::initialize_cut_stops(route_partition const& p) {
  cell_cut_stops_.clear();
  cell_cut_stops_.resize(p.get_num_of_cells_on_level(0), bitvec{tt_.n_locations()});
  cell_cut_components_.clear();
  cell_cut_components_.resize(p.get_num_of_cells_on_level(0), bitvec{tt_.component_locations_.size()});
  const auto n_components = tt_.component_locations_.size();
  for (auto cmpnt_idx = component_idx_t{0}; cmpnt_idx < n_components; ++cmpnt_idx) {
    const auto& cell_idxs = current_lvl_cells_of_components_[to_idx(cmpnt_idx)];
    if (cell_idxs.size() <= 1) {
      // not a cut component -> cannot contain cut stop
      continue;
    }

    // cell_idxs.size() >= 2 ---> c is a cut component
    for (const auto& cell_idx : cell_idxs) {
      const auto& cmpnt_locs = tt_.component_locations_[cmpnt_idx];
      auto& cut_cmpnt_bv = cell_cut_components_[to_idx(cell_idx)];
      cut_cmpnt_bv.set(cista::to_idx(cmpnt_idx), true);

      auto& cut_stop_bv = cell_cut_stops_[to_idx(cell_idx)];
      for (location_idx_t l_idx : cmpnt_locs) {
        cut_stop_bv.set(to_idx(l_idx), true);
      }
    }
  }
}

void customizer::prepare_next_level() {
  auto const timer = scoped_timer{"preparing next level"};

  auto const n_cells_in_next_level = route_masks_.size() >> 1U;

  // 1. update route masks
  // we only need to consider marked routes
  for (auto route_mask : route_masks_) {
    route_mask &= marked_routes_;
  }
  // 2. Update route events
  route_event_mask_ &= marked_route_events_;

  // 3. unite route masks (parent routes = union of children routes)
  unite_route_masks();
  // 4. unite cut stop masks. A cut stop of cell C on level i + 1 is a
  // cut stop on level i for one (or both) of C's children
  unite_cut_stops();
  unite_cut_cmpnts();

  // 5. update incident cell indexes of
  // components for next level
  update_component_cell_idxs_for_next_level();

  start_times_registry_.populate(tt_, n_cells_in_next_level,
                                 current_lvl_cells_of_components_,
                                 route_masks_, true);
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

void customizer::unite_cut_cmpnts() {
  binary_or_reduce(cell_cut_components_);
}

void customizer::bmc_cut_routing_task(
    thread_task const& task,
    local_thread_context& context,
    std::vector<std::atomic<size_t>>& cell_progress) {

  const auto cut_cmpnt_from = task.component_idx_;
  auto const cell = task.cell_idx_;
  //auto const level = task.level_;
  auto const bin_index = task.bin_idx_;

  if (context.last_cell_idx_ != cell) {
    context.tt_view_.index(route_masks_[to_idx(cell)]);
  }

  auto const bin_begin_idx =
      start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_index];
  auto const bin_end_idx =
      start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_index + 1];

  bmc_raptor_state state;

  bmc_raptor raptor{context.tt_view_, state,
                    cell_cut_stops_[to_idx(cell)],
                    route_event_starts_index_,
                    route_event_mask_};

  auto const& cmpnt_locs = tt_.component_locations_[cut_cmpnt_from];
  for (auto dep_event_i = bin_begin_idx; dep_event_i < bin_end_idx;
       ++dep_event_i) {
    auto const& rel_dep_event =
        start_times_registry_
            .cmpnt_dep_events_buffer_[to_idx(cell)][dep_event_i];
    location_idx_t const from_loc_source_idx =
        cmpnt_locs[cista::to_idx(rel_dep_event.dep_loc_)];
    location_idx_view_t const from_loc_view_idx =
        context.tt_view_.get_view_idx(from_loc_source_idx);

    search_bitfield sbf;
    truncate_to(rel_dep_event.active_days_, sbf);
    auto const dep =
        static_cast<uint16_t>(rel_dep_event.dep_min_after_midnight_.count());
    bool added = bmc_raptor::add_to_non_dest_round_bag(
        state.round_bags_[0U][to_idx(from_loc_view_idx)],
        {
         .arrival_ = dep,
         .arrival_with_transfer_ = dep,
         .departure_ = dep
        },
        sbf);
    if (added) {
      state.station_mark_.set(cista::to_idx(from_loc_view_idx), true);
    }
  }

  raptor.rounds();

  raptor.reconstruct([&](stop_idx_t from, route_idx_t via, stop_idx_t to) {
    auto const rank_from = route_event_starts_index_[via];
    unsigned const dep_route_rank_off = (from * 2);
    unsigned const arr_route_rank_off = (to * 2) - 1;
    task.atomic_route_ranks_[to_idx(via)].store(task.level_ + 1);
    task.atomic_route_event_ranks_[rank_from + dep_route_rank_off].store(task.level_ + 1);
    task.atomic_route_event_ranks_[rank_from + arr_route_rank_off].store(task.level_ + 1);
  });

  context.last_cell_idx_ = cell;
  ++cell_progress[to_idx(cell)];
}

void customizer::mc_cut_routing_task(
    thread_task const& task,
    local_thread_context& context,
    std::vector<std::atomic<size_t>>& cell_progress) {

  const auto cut_cmpnt_from = task.component_idx_;
  auto const cell = task.cell_idx_;
  auto const level = task.level_;
  auto const bin_index = task.bin_idx_;

  if (context.last_cell_idx_ != cell) {
    context.tt_view_.index(route_masks_[to_idx(cell)]);
  }

  auto const bin_begin_idx =
      start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_index];
  auto const bin_end_idx =
      start_times_registry_.bin_start_idxs_[to_idx(cell)][bin_index + 1];

  mc_raptor_state state;

  mc_raptor raptor{context.tt_view_, state,
                    cell_cut_stops_[to_idx(cell)],
                    route_event_starts_index_,
                    route_event_mask_};

  auto const& cmpnt_locs = tt_.component_locations_[cut_cmpnt_from];
  for (auto dep_event_i = bin_begin_idx; dep_event_i < bin_end_idx;
       ++dep_event_i) {
    auto const& rel_dep_event =
        start_times_registry_
            .cmpnt_dep_events_buffer_[to_idx(cell)][dep_event_i];
    location_idx_t const from_loc_source_idx =
        cmpnt_locs[cista::to_idx(rel_dep_event.dep_loc_)];
    location_idx_view_t const from_loc_view_idx =
        context.tt_view_.get_view_idx(from_loc_source_idx);

    rel_dep_event.active_days_.for_each_set_bit([&](size_t const i) {
      routing_time const departure(static_cast<int>(i) * 1440 + rel_dep_event.dep_min_after_midnight_.count());
      bool const added = mc_raptor::add_to_non_dest_round_bag(
          state.round_bags_[0U][to_idx(from_loc_view_idx)],
          {.arrival_ = departure,
           .arrival_with_transfer_ = departure,
           .departure_ = departure,
           .route_idx_ = 0U,
           .enter_stop_idx_ = 0U,
           .exit_stop_idx_ = 0U,
           .parent_bag_idx_ = 0U,
           .is_footpath_ = false,
           .has_parent_ = false});
      if (added) {
        state.station_mark_.set(cista::to_idx(from_loc_view_idx), true);
      }
    });
  }

  raptor.route();

  std::vector<mc_journey> mc_journey_bag;
  cell_cut_stops_[to_idx(cell)].for_each_set_bit([&](size_t i) {
    location_idx_t const destination_loc_idx = location_idx_t{i};
    location_idx_view_t const destination_loc_view_idx =
        context.tt_view_.get_view_idx(destination_loc_idx);

    if (destination_loc_view_idx == location_idx_view_t::invalid()) {
      return;
    }

    raptor.emplace_relative_journeys_for(destination_loc_view_idx,
                                         mc_journey_bag);

    for (auto const& mc_j : mc_journey_bag) {
      mc_backtrack_and_update_ranks(
          mc_j.label_iter_, state, context, mc_j.transfers_ + 1,
          location_idx_t{i}, level, cell, cut_cmpnt_from,
          task.atomic_route_ranks_, task.atomic_route_event_ranks_);
    }

    mc_journey_bag.clear();
  });

  context.last_cell_idx_ = cell;
  ++cell_progress[to_idx(cell)];
}

// void customizer::bmc_backtrack_and_update_ranks(bmc_raptor_bag_t::const_iterator root_label,
//                                             bmc_raptor_state const& state,
//                                             local_thread_context const& context,
//                                             const unsigned k,
//                                             location_idx_t,
//                                             std::uint8_t const level,
//                                             cell_idx_t,
//                                             component_idx_t,
//                                             atomic_ranks_t& atomic_route_ranks,
//                                             atomic_ranks_t& atomic_route_event_ranks) {
//
//   auto current_label = root_label->label_;
//   auto current_k = k;
//
//   while (current_label.has_parent_ == 1U) {
//     const auto route_idx = current_label.route_idx_;
//
//     const auto& stop_sequence = tt_.route_location_seq_[route_idx_t{route_idx}];
//
//     auto const enter_stop_idx = current_label.enter_stop_idx_;
//     auto const exit_stop_idx = current_label.exit_stop_idx_;
//
//
//     auto const enter_stp = stop{stop_sequence[enter_stop_idx]};
//
//     auto const enter_loc_idx = enter_stp.location_idx();
//     auto const enter_loc_view_idx = context.tt_view_.get_view_idx(enter_loc_idx);
//     utl::verify(enter_loc_view_idx != location_idx_view_t::invalid(),
//                "Unmapped location while backtracking");
//
//
//
//     if (route_idx != to_idx(route_idx_t::invalid())) {
//       auto const from = route_event_starts_index_[route_idx_t{route_idx}];
//       const unsigned dep_route_rank_off = (enter_stop_idx * 2);
//       const unsigned arr_route_rank_off = (exit_stop_idx * 2) - 1;
//       atomic_route_ranks[route_idx].store(level + 1);
//       atomic_route_event_ranks[from + dep_route_rank_off].store(level + 1);
//       atomic_route_event_ranks[from + arr_route_rank_off].store(level + 1);
//     }
//
//     current_label =
//         state.round_bags_[current_k - 1][to_idx(enter_loc_view_idx)]
//             .labels_[current_label.parent_bag_idx_]
//             .label_;
//     current_k--;
//   }
// }

void customizer::mc_backtrack_and_update_ranks(pareto_set<mc_raptor_label>::const_iterator const root_label,
                                               mc_raptor_state const& state,
                                               local_thread_context const& context,
                                               const unsigned k,
                                               location_idx_t,
                                               std::uint8_t const level,
                                               cell_idx_t,
                                               component_idx_t,
                                               atomic_ranks_t& atomic_route_ranks,
                                               atomic_ranks_t& atomic_route_event_ranks) {

  auto current_label = *root_label;
  auto current_k = k;

  while (current_label.has_parent_) {
    const auto route_idx = current_label.route_idx_;

    const auto& stop_sequence = tt_.route_location_seq_[route_idx_t{route_idx}];

    auto const enter_stop_idx = current_label.enter_stop_idx_;
    auto const exit_stop_idx = current_label.exit_stop_idx_;


    auto const enter_stp = stop{stop_sequence[enter_stop_idx]};

    auto const enter_loc_idx = enter_stp.location_idx();
    auto const enter_loc_view_idx = context.tt_view_.get_view_idx(enter_loc_idx);
    utl::verify(enter_loc_view_idx != location_idx_view_t::invalid(),
               "Unmapped location while backtracking");



    if (route_idx != to_idx(route_idx_t::invalid())) {
      auto const from = route_event_starts_index_[route_idx_t{route_idx}];
      const unsigned dep_route_rank_off = (enter_stop_idx * 2);
      const unsigned arr_route_rank_off = (exit_stop_idx * 2) - 1;
      atomic_route_ranks[route_idx].store(level + 1);
      atomic_route_event_ranks[from + dep_route_rank_off].store(level + 1);
      atomic_route_event_ranks[from + arr_route_rank_off].store(level + 1);
    }

    current_label = state.round_bags_[current_k - 1][to_idx(enter_loc_view_idx)]
                        .els_[current_label.parent_bag_idx_];
    current_k--;
  }
}

void customizer::compute_component_to_cells(route_partition const& partition) {
  current_lvl_cells_of_components_.clear();
  auto const n_components = tt_.component_locations_.size();
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

    current_lvl_cells_of_components_.emplace_back(cell_idxs);
  }
}


void customizer::update_component_cell_idxs_for_next_level() {
  auto const n_components = tt_.component_locations_.size();
  for (auto cmpnt_idx = component_idx_t{0}; cmpnt_idx < n_components; ++cmpnt_idx) {
    auto& cell_idxs = current_lvl_cells_of_components_[to_idx(cmpnt_idx)];
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
      auto& cell_cmpnt_bv = cell_cut_components_[to_idx(cell_idxs.front())];
      cell_cmpnt_bv.set(cista::to_idx(cmpnt_idx), false);

      auto& cell_stop_bv = cell_cut_stops_[to_idx(cell_idxs.front())];
      // no longer cut stops
      for (const auto& loc : tt_.component_locations_[cmpnt_idx]) {
        cell_stop_bv.set(to_idx(loc), false);
      }
    }
  }
}

void customizer::mark_routes_and_events_from_ranks(
    route_partition const&,
    std::uint8_t const level,
    atomic_ranks_t const& atomic_route_ranks,
    atomic_ranks_t const& atomic_route_event_ranks) {
  auto timer = scoped_timer("marking routes and events that have an updated rank");

  // 1. Effectively clear all mark bits
  utl::fill(marked_routes_.blocks_, 0U);
  utl::fill(marked_route_events_.blocks_, 0U);

  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    if (atomic_route_ranks[to_idx(route_idx)] == level + 1) {
      // Route was used on current level
      marked_routes_.set(to_idx(route_idx), true);
    } else {
      // Route was not used, hence there is also
      // no route event that was used -> skip
      continue;
    }

    // check which route events have been used
    const auto route_event_from_idx = route_event_starts_index_[route_idx];
    const auto stop_seq = tt_.route_location_seq_[route_idx];
    const auto n_events = (stop_seq.size() * 2) - 2;
    for (auto i = route_event_from_idx; i < route_event_from_idx + n_events; ++i) {
      if (atomic_route_event_ranks[i] == level + 1) {
        marked_route_events_.set(i, true);
      }
    }
  }
}

void customizer::compute_route_event_ranks_index() {
  route_event_starts_index_.clear();
  std::uint32_t next_index = 0U;
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    route_event_starts_index_.emplace_back(next_index);
    const auto n_stops = tt_.route_location_seq_[route_idx].size();
    const auto n_route_events = (2 * n_stops) - 2;
    next_index += n_route_events;
  }
  route_event_starts_index_.emplace_back(next_index);
}

void customizer::materialize_atomic_ranks(atomic_ranks_t const& atomic_route_event_ranks,
                                          vecvec<route_idx_t, rank_t>& out_ranks) {
  auto timer = scoped_timer("materializing atomic ranks");
  utl::verify(route_event_starts_index_.size() == tt_.n_routes() + 1, "index has wrong dimensions");

  out_ranks.clear();
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    const auto from = route_event_starts_index_[route_idx];
    const auto to = route_event_starts_index_[route_idx + 1];
    const auto n_ranks = to - from;
    out_ranks.add_back_sized(n_ranks);
    for (auto i = 0U; i < n_ranks; ++i) {
      out_ranks.back()[i] = rank_t{atomic_route_event_ranks[from + i]};
    }
  }
}

} // namespace nigiri::routing::para
