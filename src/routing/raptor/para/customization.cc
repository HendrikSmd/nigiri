#include "nigiri/routing/raptor/para/customization.h"

#include <thread>
#include <stop_token>

#include "boost/asio/thread_pool.hpp"
#include "utl/zip.h"
#include "nigiri/common/parallel_for_with_args.h"
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
  std::vector<std::atomic<std::uint8_t>> atomic_route_ranks(route_rank_store_.route_ranks_.size());
  std::vector<std::atomic<std::uint8_t>> atomic_footpath_ranks(route_rank_store_.footpath_ranks_.size());

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
    for (auto cell_idx = cell_idx_t{0U};
         cell_idx <
         partition.get_num_of_cells_on_level(static_cast<std::uint8_t>(level));
         ++cell_idx) {
      cell_cut_stops_[to_idx(cell_idx)].for_each_set_bit(
          [&](uint64_t const idx) {
            tasks.emplace_back(cell_idx, location_idx_t{idx}, level,
                               atomic_route_ranks, atomic_footpath_ranks);
          });
    }
    parallel_for_run_threadlocal<local_thread_context>(
        tasks.size(),
        [&](auto& context, auto const t_idx) {
          this->cut_routing_task(tasks[t_idx], context, cell_progress);
        },
        utl::noop_progress_update{}, utl::parallel_error_strategy::QUIT_EXEC,
        std::ref(tt_));
    level_finished.store(true);
    logger_thread.join();

    mark_updated_routes_and_used_transfers(atomic_route_ranks, partition,
                                           static_cast<std::uint8_t>(level));
    mark_updated_footpaths(atomic_footpath_ranks, partition,
                           static_cast<std::uint8_t>(level));
    prepare_next_level();
    if (std::ranges::all_of(cell_cut_stops_, [](const bitvec& bv){return !bv.any();})) {
      log(log_lvl::info, "customization", "No more cut stops. Terminating shortly");
      break;
    }
  }
  materialize_atomic_ranks(atomic_route_ranks, atomic_footpath_ranks);
  route_rank_store_.partition_ = std::move(partition);
  return route_rank_store_;
}

void customizer::log_progress(std::vector<std::atomic<size_t>> const& cell_progress) const {
    std::cout << "Progress Update:\n";
    for (auto cell_idx = cell_idx_t{0U}; cell_idx < static_cast<cista::base_t<cell_idx_t>>(cell_cut_stops_.size()); ++cell_idx) {
      const size_t progress = cell_progress[to_idx(cell_idx)];
      const auto limit = cell_cut_stops_[cista::to_idx(cell_idx)].count();
      if (progress == 0) {
        continue;
      }
      std::cout << "Cell " << cell_idx << ": " << progress << "/" << limit
      << ((progress == limit) ? " ✓" : "") <<std::endl;
    }
    std::cout << std::endl;
}

void customizer::initialize_ranks() {
  route_rank_store_.route_ranks_.clear();
  route_rank_store_.footpath_ranks_.clear();
  route_rank_store_.route_rank_start_idx_.clear();

  const auto n_footpaths = tt_.locations_.footpaths_out_[kDefaultProfile].data_.size();
  route_rank_store_.footpath_ranks_.resize(n_footpaths, rank_t{0U});

  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    const auto route_range_begin = route_rank_store_.route_ranks_.size();
    const auto n_stops = tt_.route_location_seq_[route_idx].size();
    const auto n_route_rank_entries = 1 + ((2 * n_stops) - 2);

    route_rank_store_.route_rank_start_idx_.emplace_back(route_range_begin);
    route_rank_store_.route_ranks_.resize(route_rank_store_.route_ranks_.size() + n_route_rank_entries, rank_t{0U});
  }
  //Sentinel
  route_rank_store_.route_rank_start_idx_.emplace_back(route_rank_store_.route_ranks_.size());
}

void customizer::initialize(route_partition const& p) {
  auto const timer = scoped_timer("initializing customization process");
  auto n_of_cells_on_first_lvl = p.get_num_of_cells_on_level(0U);

  updated_routes_.clear();
  updated_footpaths_.clear();
  route_masks_.clear();
  footpath_masks_.clear();
  transfer_masks_.clear();
  used_transfers_.clear();
  cell_cut_stops_.clear();
  location_to_current_level_cell_idxs_.clear();

  updated_routes_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  updated_footpaths_.resize(
      n_of_cells_on_first_lvl,
      bitvec{tt_.locations_.footpaths_out_[kDefaultProfile].data_.size()});
  footpath_masks_.resize(
    n_of_cells_on_first_lvl,
    bitvec{tt_.locations_.footpaths_out_[kDefaultProfile].data_.size()});
  route_masks_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_routes()});
  transfer_masks_.resize(n_of_cells_on_first_lvl, bitvec::max(tt_.n_locations()));
  used_transfers_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});
  cell_cut_stops_.resize(n_of_cells_on_first_lvl, bitvec{tt_.n_locations()});

  initialize_ranks();

  initialize_route_masks(p);
  initialize_footpath_masks(p);
  append_location_cell_idxs(p, location_to_current_level_cell_idxs_);
  // The cut stops have to be initialized after
  // the (location -> cell_idxs) have been initialized!!!
  initialize_cut_stops();
  // must happen after cut stops have been initialized
  initialize_used_transfers();
}

void customizer::initialize_route_masks(route_partition const& p) {
  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    auto const cell_idx_of_route = to_idx(p.route_to_cell_idx_[route_idx]);
    auto& bv = route_masks_[cell_idx_of_route];
    bv.set(to_idx(route_idx), true);
  }
}

void customizer::initialize_footpath_masks(route_partition const& p) {
  for (auto fp_idx = footpath_idx_t{0U};
       fp_idx < tt_.locations_.footpaths_out_[kDefaultProfile].data_.size();
       ++fp_idx) {
    auto const cell_idx_of_fp = to_idx(p.footpath_to_cell_idx_[fp_idx]);
    auto& bv = footpath_masks_[cell_idx_of_fp];
    bv.set(to_idx(fp_idx), true);
  }
}

void customizer::initialize_cut_stops() {
  for (auto loc = location_idx_t{0}; loc < tt_.n_locations(); ++loc) {
    const auto& cell_idxs = location_to_current_level_cell_idxs_[to_idx(loc)];
    if (cell_idxs.size() <= 1) {
      // not a cut stop
      continue;
    }

    // cell_idxs.size() >= 2 ---> c is a cut location
    for (const auto cell_idx : cell_idxs) {
      auto& cut_stop_bv = cell_cut_stops_[to_idx(cell_idx)];
      cut_stop_bv.set(to_idx(loc), true);
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

  auto const n_cells_in_next_level = route_masks_.size() >> 1U;

  // 1. update route and footpath masks
  // we only need to consider marked routes and footpaths
  for (auto [route_mask, updated_routes] : utl::zip(route_masks_, updated_routes_)) {
    route_mask &= updated_routes;
  }
  for (auto [footpath_mask, updated_footpaths] : utl::zip(footpath_masks_, updated_footpaths_)) {
    footpath_mask &= updated_footpaths;
  }
  // 2. clear route and footpath marks for next level
  updated_routes_.resize(n_cells_in_next_level);
  updated_footpaths_.resize(n_cells_in_next_level);
  for (auto [route_bv, footpath_bv] : utl::zip(updated_routes_, updated_footpaths_)) {
    route_bv.zero_out();
    footpath_bv.zero_out();
  }

  // 3. unite route and footpath masks (parent routes/footpaths = union of children routes/footpaths)
  unite_route_masks();
  unite_footpath_masks();
  // 4. unite cut stop masks. A cut stop of cell C on level i + 1 is a
  // cut stop on level i for one (or both) of C's children
  unite_cut_stops();

  // 5. update incident cell indexes of
  // components for next level
  update_location_cell_idxs();

  unite_used_transfers();
  transfer_masks_.resize(n_cells_in_next_level);
  std::swap(transfer_masks_, used_transfers_);
  initialize_used_transfers();
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

void customizer::unite_footpath_masks() {
  binary_or_reduce(footpath_masks_);
}

void customizer::unite_cut_stops() {
  binary_or_reduce(cell_cut_stops_);
}

void customizer::unite_used_transfers() {
  binary_or_reduce(used_transfers_);
}

void customizer::cut_routing_task(const thread_task& task,
                                  local_thread_context& context,
                                  std::vector<std::atomic<size_t>>& cell_progress) {
  const auto cell = task.cell_idx_;
  const auto level = task.level_;

  if (context.last_cell_idx_ != cell) {
    context.tt_view_.index(route_masks_[to_idx(cell)],
                           footpath_masks_[to_idx(cell)]);
  }
  bmc_raptor raptor{context.tt_view_, context.state_,
                    cell_cut_stops_[to_idx(cell)],
                    transfer_masks_[to_idx(cell)],
                    footpath_masks_[to_idx(cell)]};

  location_idx_view_t start_view_location_idx =
      context.tt_view_.get_view_idx(task.start_cut_location_);

  if (start_view_location_idx != location_idx_view_t::invalid()) {
    for (int x = 0; x < 2; ++x) {
      raptor.init_starts(start_view_location_idx, x == 1);
      raptor.rounds();

      std::vector<bmc_journey> bmc_journey_bag;
      cell_cut_stops_[to_idx(cell)].for_each_set_bit([&](size_t i) {
        auto const destination_loc_idx = location_idx_t{i};
        if (destination_loc_idx == task.start_cut_location_) {
          return;
        }

        location_idx_view_t const destination_loc_view_idx =
            context.tt_view_.get_view_idx(destination_loc_idx);

        if (destination_loc_view_idx == location_idx_view_t::invalid()) {
          return;
        }

        if (tt_.location_component_[task.start_cut_location_] == tt_.location_component_[destination_loc_idx]) {
          const auto& out_fps = tt_.locations_.footpaths_out_[kDefaultProfile];
          const auto start_out_fps = out_fps[task.start_cut_location_];
          size_t fp_base_idx = static_cast<size_t>(std::distance(out_fps.data_.begin(), start_out_fps.begin()));
          for (const auto [fp_off, fp] : utl::enumerate(start_out_fps)) {
            if (fp.target() == destination_loc_idx) {
              const auto fp_idx = footpath_idx_t{fp_base_idx + fp_off};
              task.atomic_footpath_ranks_[to_idx(fp_idx)].store(level + 1);
              break;
            }
          }
        }


        raptor.emplace_relative_journeys_for(destination_loc_view_idx,
                                             bmc_journey_bag);

        for (auto const& bmc_j : bmc_journey_bag) {
          backtrack_and_update_ranks(
              bmc_j.label_iter_, context, bmc_j.transfers_ + 1,
              task.start_cut_location_, destination_loc_idx, level,
              task.atomic_route_ranks_, task.atomic_footpath_ranks_);
        }

        bmc_journey_bag.clear();
      });

      context.state_.reset();
    }

  }
  context.last_cell_idx_ = cell;
  ++cell_progress[to_idx(cell)];
}

void customizer::backtrack_and_update_ranks(
    bmc_raptor_bag_t::const_iterator root_label,
    local_thread_context const& context,
    unsigned const k,
    location_idx_t start,
    location_idx_t destination,
    std::uint8_t const level,
    std::vector<std::atomic<std::uint8_t>>& atomic_route_ranks,
    std::vector<std::atomic<std::uint8_t>>& atomic_footpath_ranks) {
#ifdef NIGIRI_ENABLE_SIMD
  auto current_label = (*root_label).metadata_;
#else
  auto current_label = root_label->label_;
#endif
  auto current_k = k;

  while (current_label.has_parent_ == 1U) {
    auto const route_idx = current_label.route_idx_;

    auto const& stop_sequence = tt_.route_location_seq_[route_idx_t{route_idx}];

    auto const enter_stop_idx = current_label.enter_stop_idx_;
    auto const exit_stop_idx = current_label.exit_stop_idx_;

    auto const enter_stp = stop{stop_sequence[enter_stop_idx]};
    auto const exit_stp = stop{stop_sequence[exit_stop_idx]};

    auto const enter_loc_idx = enter_stp.location_idx();
    auto const exit_loc_idx = exit_stp.location_idx();

    if (current_label.is_footpath_ == 1) {
      const auto out_fps = tt_.locations_.footpaths_out_[kDefaultProfile][exit_loc_idx];
      auto const base_idx = std::distance(
          tt_.locations_.footpaths_out_[kDefaultProfile].data_.begin(),
          out_fps.begin());

      for (const auto [i, fp] : utl::enumerate(out_fps)) {
        if (fp.target() == destination) {
          const auto fp_idx = footpath_idx_t{static_cast<std::uint32_t>(base_idx) + i};
          atomic_footpath_ranks[to_idx(fp_idx)].store(level + 1);
          break;
        }
      }

    }

    auto const enter_loc_view_idx = context.tt_view_.get_view_idx(enter_loc_idx);
    utl::verify(enter_loc_view_idx != location_idx_view_t::invalid(),
               "Unmapped location while backtracking");



    if (route_idx != to_idx(route_idx_t::invalid())) {
      auto const from = route_rank_store_.route_rank_start_idx_[route_idx_t{route_idx}];
      const unsigned dep_route_rank_off = 1 + (enter_stop_idx * 2);
      const unsigned arr_route_rank_off = (exit_stop_idx * 2);
      atomic_route_ranks[from].store(level + 1);
      atomic_route_ranks[from + dep_route_rank_off].store(level + 1);
      atomic_route_ranks[from + arr_route_rank_off].store(level + 1);
    }
#ifdef NIGIRI_ENABLE_SIMD
    current_label =
        context.state_.round_bags_[current_k - 1][to_idx(enter_loc_view_idx)]
            .meta_data_[current_label.parent_bag_idx_];
#else
    current_label =
        context.state_.round_bags_[current_k - 1][to_idx(enter_loc_view_idx)]
            .labels_[current_label.parent_bag_idx_]
            .label_;
#endif

    current_k--;
    destination = enter_loc_idx;
  }

  if (current_label.is_footpath_ == 1) {
    const auto out_fps = tt_.locations_.footpaths_out_[kDefaultProfile][start];
    auto const base_idx = std::distance(
        tt_.locations_.footpaths_out_[kDefaultProfile].data_.begin(),
        out_fps.begin());

    for (const auto [i, fp] : utl::enumerate(out_fps)) {
      if (fp.target() == destination) {
        const auto fp_idx = footpath_idx_t{static_cast<std::uint32_t>(base_idx) + i};
        atomic_footpath_ranks[to_idx(fp_idx)].store(level + 1);
        break;
      }
    }
  }
}

void customizer::append_location_cell_idxs(route_partition const& partition,
                                        std::vector<std::vector<cell_idx_t>>& location_to_cell_idxs) {
  auto const n_locations= tt_.n_locations();
  for (auto loc = location_idx_t{0}; loc < n_locations; ++loc) {

    std::vector<cell_idx_t> cell_idxs;
    const auto routes_of_loc = tt_.location_routes_[loc];

    // map route indexes to their cell indexes, append them to cell_idxs
    std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
      return partition.route_to_cell_idx_[r];
    });

    auto const loc_fps = tt_.locations_.footpaths_out_[kDefaultProfile][loc];
    auto const fp_base_idx = std::distance(
        tt_.locations_.footpaths_out_[kDefaultProfile].data_.begin(),
        loc_fps.begin());
    for (auto const [fp_offset, _] : utl::enumerate(loc_fps)) {
      auto const fp_idx =
          footpath_idx_t{static_cast<unsigned long>(fp_base_idx) + fp_offset};
      auto const cell_idx = partition.footpath_to_cell_idx_[fp_idx];
      utl::verify(cell_idx != cell_idx_t::invalid(), "footpath has no cell");
      cell_idxs.push_back(cell_idx);
    }

    // we sort them to make sure that all
    // equal cell_idxs are adjacent to each other
    // this is needed for std::unique
    std::ranges::sort(cell_idxs);
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    cell_idxs.erase(last, cell_idxs.end());
    location_to_cell_idxs.emplace_back(cell_idxs);
  }
}


void customizer::update_location_cell_idxs() {
  auto const n_locations = tt_.n_locations();
  for (auto loc = location_idx_t{0}; loc < n_locations; ++loc) {
    auto& cell_idxs = location_to_current_level_cell_idxs_[to_idx(loc)];
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
      auto& cell_stop_bv = cell_cut_stops_[to_idx(cell_idxs.front())];
      cell_stop_bv.set(to_idx(loc), false);
    }
  }
}

inline bool uses_transport(const journey::leg& l) {
  return holds_alternative<journey::run_enter_exit>(l.uses_);
}

void customizer::mark_updated_routes_and_used_transfers(
    std::vector<std::atomic<std::uint8_t>> const& atomic_route_ranks,
    route_partition const& partition,
    std::uint8_t const level) {
  auto timer = scoped_timer("marking updated routes and used transfers");

  for (auto route_idx = route_idx_t{0U}; route_idx < tt_.n_routes(); ++route_idx) {
    const auto cell_idx = partition.get_cell_of_route(route_idx, level);
    const auto from_idx = route_rank_store_.route_rank_start_idx_[route_idx];
    if (atomic_route_ranks[from_idx] == level + 1) {
      updated_routes_[to_idx(cell_idx)].set(to_idx(route_idx), true);
    } else {
      continue;
    }

    auto base = from_idx + 1;
    const auto stop_seq = tt_.route_location_seq_[route_idx];
    for (auto i = 0U; i < stop_seq.size(); ++i) {
      location_idx_t stop_loc = stop{stop_seq[i]}.location_idx();
      if (i == 0 || i == stop_seq.size() - 1) {
        if (atomic_route_ranks[base] == level + 1) {
          used_transfers_[to_idx(cell_idx)].set(to_idx(stop_loc), true);
        }
        ++base;
        continue;
      }

      if (atomic_route_ranks[base] == level + 1 ||
          atomic_route_ranks[base + 1] == level + 1) {
        used_transfers_[to_idx(cell_idx)].set(to_idx(stop_loc), true);
      }

      base += 2;
    }
  }
}

void customizer::mark_updated_footpaths(
    std::vector<std::atomic<std::uint8_t>> const& atomic_footpath_ranks,
    route_partition const& partition,
    std::uint8_t level) {
  auto timer = scoped_timer("marking updated footpaths and used transfers");
  auto const n_footpaths =
      tt_.locations_.footpaths_out_[kDefaultProfile].data_.size();

  for (auto fp_idx = footpath_idx_t{0U}; fp_idx < n_footpaths; ++fp_idx) {
    if (atomic_footpath_ranks[to_idx(fp_idx)] != level + 1) {
      continue;
    }

    const auto cell_idx_of_fp = partition.get_cell_of_footpath(fp_idx, level);
    updated_footpaths_[to_idx(cell_idx_of_fp)].set(to_idx(fp_idx), true);
  }
}

void customizer::materialize_atomic_ranks(
    std::vector<std::atomic<std::uint8_t>> const& atomic_route_ranks,
    std::vector<std::atomic<std::uint8_t>> const& atomic_footpath_ranks) {
  auto timer = scoped_timer("materializing atomic ranks");
  utl::verify(
      atomic_route_ranks.size() == route_rank_store_.route_ranks_.size(),
      "atomic route ranks do not have the same size as route rank store ranks");
  utl::verify(
      atomic_footpath_ranks.size() == route_rank_store_.footpath_ranks_.size(),
      "atomic footpath ranks do not have the same size as route rank store "
      "ranks");

  for (auto i = 0U; i < atomic_route_ranks.size(); ++i) {
    route_rank_store_.route_ranks_[i] = rank_t{atomic_route_ranks[i]};
  }
  for (auto i = 0U; i < atomic_footpath_ranks.size(); ++i) {
    route_rank_store_.footpath_ranks_[footpath_idx_t{i}] =
        rank_t{atomic_footpath_ranks[i]};
  }
}

} // namespace nigiri::routing::para
