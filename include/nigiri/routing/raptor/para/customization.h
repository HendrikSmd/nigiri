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

  using atomic_ranks_t = std::vector<std::atomic<std::uint8_t>>;

  enum class search_algorithm { mc_raptor, bmc_raptor };

  constexpr static search_algorithm search_algo = search_algorithm::bmc_raptor;

  struct thread_task {
    thread_task(cell_idx_t const cell_idx,
                component_idx_t const component_idx,
                std::uint8_t const level,
                size_t const bin_idx,
                atomic_ranks_t& atomic_route_ranks,
                atomic_ranks_t& atomic_route_event_ranks) :
    cell_idx_(cell_idx),
    component_idx_(component_idx),
    level_{level},
    bin_idx_(bin_idx),
    atomic_route_ranks_(atomic_route_ranks),
    atomic_route_event_ranks_(atomic_route_event_ranks) {}

    cell_idx_t cell_idx_;
    component_idx_t component_idx_;
    std::uint8_t level_;
    size_t bin_idx_;
    atomic_ranks_t& atomic_route_ranks_;
    atomic_ranks_t& atomic_route_event_ranks_;
  };

  struct local_thread_context {

    local_thread_context(timetable const& tt) :
      last_cell_idx_(cell_idx_t::invalid()),
      tt_view_(tt) {}

    cell_idx_t last_cell_idx_;
    timetable_view tt_view_;
  };

  customizer(timetable const& tt);

  void compute_ranks(route_partition const& partition, vecvec<route_idx_t, rank_t>& out_ranks);
  void initialize(route_partition const& p);
  void initialize_route_masks(route_partition const& p);
  void initialize_cut_stops(route_partition const& p);
  void prepare_next_level();
  void unite_route_masks();
  void unite_cut_stops();
  void unite_cut_cmpnts();
  void compute_component_to_cells(route_partition const& partition);
  void update_component_cell_idxs_for_next_level();

  void bmc_cut_routing_task(thread_task const& task,
                            local_thread_context& state,
                            std::vector<std::atomic<size_t>>& cell_progress);
  void mc_cut_routing_task(thread_task const& task,
                            local_thread_context& state,
                            std::vector<std::atomic<size_t>>& cell_progress);

  void bmc_backtrack_and_update_ranks(
      bmc_raptor_bag_t::const_iterator root_label,
      bmc_raptor_state const& state, local_thread_context const& context,
      unsigned k, location_idx_t target, std::uint8_t level, cell_idx_t cell,
      component_idx_t component_idx, atomic_ranks_t& atomic_route_ranks,
      atomic_ranks_t& atomic_route_event_ranks);

  void mc_backtrack_and_update_ranks(
    pareto_set<mc_raptor_label>::const_iterator root_label,
    mc_raptor_state const& state, local_thread_context const& context,
    unsigned k, location_idx_t target, std::uint8_t level, cell_idx_t cell,
    component_idx_t component_idx, atomic_ranks_t& atomic_route_ranks,
    atomic_ranks_t& atomic_route_event_ranks);

  void log_progress(std::vector<std::atomic<size_t>> const& cell_progress) const;

  void mark_routes_and_events_from_ranks(route_partition const& partition,
                                         std::uint8_t level,
                                         atomic_ranks_t const& atomic_route_ranks,
                                         atomic_ranks_t const& atomic_route_event_ranks);

  void materialize_atomic_ranks(atomic_ranks_t const& atomic_route_event_ranks,
                                vecvec<route_idx_t, rank_t>& out_ranks);

  void compute_route_event_ranks_index();

  const timetable& tt_;

  /*
   * route_masks_ contains for every cell of the current
   * level a bitvec of size tt.n_routes(). Bit i is set
   * in bitvec of cell j, if the route i is contained in
   * cell j on the current level and it was marked in the
   * last level.
   */
  std::vector<bitvec> route_masks_;

  /*
   * route_event_mask_ has a bit for every departure/arrival
   * event of any route at every of its stops. The structure
   * is as follows
   * Route 0:
   *   stop-0-dep, stop-1-arr, stop-1-dep, ..., stop-N_0-arr
   * Route 1:
   *   stop-0-dep, stop-1-arr, stop-1-dep, ..., stop-N_1-arr
   * ...
   * For the current level an event bit is set iff the event
   * was marked in the last round. This bitvec is initially
   * an all-ones mask. Gradually bits are cleared.
   */
  bitvec route_event_mask_;

  bitvec marked_routes_;
  vector_map<route_idx_t, std::uint32_t> route_event_starts_index_;
  bitvec marked_route_events_;

  std::vector<bitvec> cell_cut_components_;
  std::vector<bitvec> cell_cut_stops_;
  std::vector<std::vector<cell_idx_t>> current_lvl_cells_of_components_;

  // Start Times
  start_times_registry start_times_registry_;
};

} // namespace nigiri::routing::para