#pragma once

#include "boost/thread/pthread/mutex.hpp"

#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/routing/start_times.h"

#include "mc_raptor_state.h"
#include "start_times_registry.h"

namespace nigiri::routing::para {

struct route_rank_store;

struct customizer {

  struct thread_task {
    thread_task(cell_idx_t const cell_idx,
                component_idx_t const component_idx,
                cista::base_t<cell_idx_t> const level,
                std::vector<start_times_registry::bin_range_t>::const_iterator const iter) :
    cell_idx_(cell_idx),
    component_idx_(component_idx),
    level_{level},
    iter_(iter) {}

    cell_idx_t cell_idx_;
    component_idx_t component_idx_;
    cista::base_t<cell_idx_t> level_;
    std::vector<start_times_registry::bin_range_t>::const_iterator iter_;
  };

  customizer(timetable const& tt);

  route_rank_store construct_route_rank_store(route_partition partition);
private:
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
  void cut_routing_task(const thread_task& task, mc_raptor_state& state);
  void update_ranks_for(journey const& j,
                        cista::base_t<cell_idx_t> level,
                        cell_idx_t cell);
  void log_progress() const;

  const timetable& tt_;

  // operational data
  std::vector<bitvec> updated_routes_;
  std::vector<bitvec> route_masks_;
  std::vector<bitvec> transfer_masks_;
  std::vector<bitvec> used_transfers_;
  std::vector<bitvec> cell_cut_cmpnts_;
  std::vector<bitvec> cell_cut_stops_;
  std::vector<std::vector<cell_idx_t>> cmpnt_to_current_lvl_cell_idxs_;

  start_times_registry start_times_registry_;


  // Results
  vector_map<route_idx_t, interval<std::uint32_t>> route_rank_ranges_;
  vector<rank_t> ranks_;


  // Threads
  std::deque<std::mutex> cell_mutexes_;

  // Progress
  std::vector<size_t> cell_progress_;
  std::atomic_bool finished_;
};


struct route_rank_store {
  route_rank_store() = default;
  explicit route_rank_store(vector_map<route_idx_t, interval<std::uint32_t>>&& route_rank_ranges,
                            vector<rank_t>&& ranks,
                            route_partition&& p);

  auto                                          cista_members();

  static cista::wrapped<route_rank_store>       read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;
  void                                          print_summary(std::ostream&) const;

  vector_map<route_idx_t, interval<std::uint32_t>> route_rank_ranges_;
  vector<rank_t> ranks_;
  route_partition partition_;
};



} // namespace nigiri::routing::para