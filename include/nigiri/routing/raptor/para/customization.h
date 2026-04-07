#pragma once

#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/routing/start_times.h"

#include <deque>

#include "bmc_raptor_state.h"
#include "mc_raptor_state.h"
#include "route_rank_store.h"
#include "start_times_registry.h"
#include "timetable_view.h"

namespace nigiri::routing::para {

struct customizer {

  struct thread_task {
    thread_task(cell_idx_t const cell_idx,
                location_idx_t const start_cut_location,
                std::uint8_t const level,
                std::vector<std::atomic<std::uint8_t>>& atomic_route_ranks,
                std::vector<std::atomic<std::uint8_t>>& atomic_footpath_ranks) :
    cell_idx_(cell_idx),
    start_cut_location_(start_cut_location),
    level_{level},
    atomic_route_ranks_(atomic_route_ranks),
    atomic_footpath_ranks_(atomic_footpath_ranks){}

    cell_idx_t cell_idx_;
    location_idx_t start_cut_location_;
    std::uint8_t level_;
    std::vector<std::atomic<std::uint8_t>>& atomic_route_ranks_;
    std::vector<std::atomic<std::uint8_t>>& atomic_footpath_ranks_;
  };

  struct local_thread_context {

    local_thread_context(timetable const& tt) :
      last_cell_idx_(cell_idx_t::invalid()),
      tt_view_(tt) {}

    cell_idx_t last_cell_idx_;
    timetable_view tt_view_;
    bmc_raptor_state state_{};
  };

  customizer(timetable const& tt);

  route_rank_store const& construct_route_rank_store(route_partition partition);
  void initialize(route_partition const& p);
  void initialize_route_masks(route_partition const& p);
  void initialize_footpath_masks(route_partition const& p);
  void initialize_cut_stops();
  void initialize_ranks();
  void prepare_next_level();
  void initialize_used_transfers();
  void unite_route_masks();
  void unite_footpath_masks();
  void unite_cut_stops();
  void unite_used_transfers();
  void append_location_cell_idxs(route_partition const& partition,
                                 std::vector<std::vector<cell_idx_t>>& location_to_cell_idxs);
  void update_location_cell_idxs();

  void cut_routing_task(thread_task const& task, local_thread_context& state,
                        std::vector<std::atomic<size_t>>& cell_progress);

  void backtrack_and_update_ranks(bmc_raptor_bag_t::const_iterator root_label,
                                  local_thread_context const& context,
                                  unsigned k,
                                  location_idx_t start,
                                  location_idx_t dest,
                                  std::uint8_t level,
                                  std::vector<std::atomic<std::uint8_t>>& atomic_route_ranks,
                                  std::vector<std::atomic<std::uint8_t>>& atomic_footpath_ranks);
  void log_progress(std::vector<std::atomic<size_t>> const& cell_progress) const;

  void mark_updated_routes_and_used_transfers(
      std::vector<std::atomic<std::uint8_t>> const& atomic_route_ranks,
      route_partition const& partition, std::uint8_t level);

  void mark_updated_footpaths(
      std::vector<std::atomic<std::uint8_t>> const& atomic_footpath_ranks,
      route_partition const& partition, std::uint8_t level);

  void materialize_atomic_ranks(
      std::vector<std::atomic<std::uint8_t>> const& atomic_route_ranks,
      std::vector<std::atomic<std::uint8_t>> const& atomic_footpath_ranks);

  const timetable& tt_;

  // operational data
  std::vector<bitvec> updated_routes_;
  std::vector<bitvec> updated_footpaths_;

  std::vector<bitvec> route_masks_;
  std::vector<bitvec> footpath_masks_;

  std::vector<bitvec> transfer_masks_;
  std::vector<bitvec> used_transfers_;

  std::vector<bitvec> cell_cut_stops_;
  std::vector<std::vector<cell_idx_t>> location_to_current_level_cell_idxs_;

  // Results
  route_rank_store route_rank_store_;
};

} // namespace nigiri::routing::para