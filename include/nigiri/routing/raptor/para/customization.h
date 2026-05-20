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

  enum class search_algorithm { mc_raptor, bmc_raptor };

  constexpr static search_algorithm search_algo = search_algorithm::bmc_raptor;
  constexpr static bool use_initial_footpaths = false;

  struct thread_task {
    thread_task(cell_idx_t const cell_idx,
                component_idx_t const component_idx,
                std::uint8_t const level,
                size_t const bin_idx,
                std::vector<std::atomic<std::uint8_t>>& atomic_ranks) :
    cell_idx_(cell_idx),
    component_idx_(component_idx),
    level_{level},
    bin_idx_(bin_idx),
    atomic_ranks_(atomic_ranks) {}

    cell_idx_t cell_idx_;
    component_idx_t component_idx_;
    std::uint8_t level_;
    size_t bin_idx_;
    std::vector<std::atomic<std::uint8_t>>& atomic_ranks_;
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
  void initialize_cut_stops();
  void initialize_ranks();
  void prepare_next_level();
  void initialize_used_transfers();
  void unite_route_masks();
  void unite_cut_stops();
  void unite_cut_cmpnts();
  void unite_used_transfers();
  void append_cmpnt_cell_idxs(route_partition const& partition,
                              std::vector<std::vector<cell_idx_t>>& cmpnt_to_cell_idxs);
  void update_cmpnt_cell_idxs_next_level();

  void cut_routing_task(thread_task const& task, local_thread_context& state,
                        std::vector<std::atomic<size_t>>& cell_progress);
  void cut_routing_task(thread_task const& task, mc_raptor_state& state,
                        std::vector<std::atomic<size_t>>& cell_progress);
  void update_ranks_for(journey const& j,
                        std::uint8_t level,
                        cell_idx_t cell);

  void backtrack_and_update_ranks(bmc_raptor_bag_t::const_iterator root_label,
                                  local_thread_context const& context,
                                  unsigned k,
                                  location_idx_t target,
                                  std::uint8_t level,
                                  cell_idx_t cell,
                                  component_idx_t component_idx,
                                  std::vector<std::atomic<std::uint8_t>>& atomic_ranks);
  void log_progress(std::vector<std::atomic<size_t>> const& cell_progress) const;

  void mark_updated_routes_and_used_transfers(
    std::vector<std::atomic<std::uint8_t>> const& atomic_ranks,
    route_partition const& partition,
    std::uint8_t level);

  void materialize_atomic_ranks(std::vector<std::atomic<std::uint8_t>> const& atomic_ranks);

  const timetable& tt_;

  // operational data
  std::vector<bitvec> updated_routes_;
  std::vector<bitvec> route_masks_;
  std::vector<bitvec> transfer_masks_;
  std::vector<bitvec> used_transfers_;
  std::vector<bitvec> cell_cut_cmpnts_;
  std::vector<bitvec> cell_cut_stops_;
  std::vector<std::vector<cell_idx_t>> cmpnt_to_current_lvl_cell_idxs_;

  // Start Times
  start_times_registry start_times_registry_;

  // Results
  route_rank_store route_rank_store_;
};

} // namespace nigiri::routing::para