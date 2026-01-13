#pragma once

#include "boost/thread/pthread/mutex.hpp"

#include "nigiri/routing/raptor/para/route_partition.h"

namespace nigiri::routing::para {

struct route_rank_store;

struct customizer {

  struct thread_task {
    thread_task(cell_idx_t const cell_idx,
                location_idx_t const location_idx,
                cista::base_t<cell_idx_t> const level) :
    cell_idx_(cell_idx),
    location_idx_(location_idx),
    level_{level}{}

    cell_idx_t cell_idx_;
    location_idx_t location_idx_;
    cista::base_t<cell_idx_t> level_;
  };

  customizer(timetable const& tt);

  route_rank_store construct_route_rank_store(route_partition partition,
                                              unsigned n_threads);
private:
  void initialize(route_partition const& p);
  void initialize_route_masks(route_partition const& p);
  void initialize_cut_stops();
  void prepare_next_level();
  void unite_route_masks();
  void initialize_used_transfers();
  void unite_cut_stops();
  void unite_used_transfers();
  void append_cmpnt_cell_idxs(route_partition const& partition,
                              std::vector<std::vector<cell_idx_t>>& cmpnt_to_cell_idxs);
  void update_cmpnt_cell_idxs_next_level();
  void cut_stop_routing_task(const thread_task& task);
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
  std::vector<bitvec> cell_cut_stops_;
  std::vector<std::vector<cell_idx_t>> cmpnt_to_current_lvl_cell_idxs_;


  // Results
  vector_map<route_idx_t, route_rank_t> route_ranks_;
  vector_map<transport_idx_t, transport_rank_t> transport_ranks_;

  // Threads
  std::deque<std::mutex> cell_mutexes_;

  // Progress
  std::vector<size_t> cell_progress_;
  std::atomic_bool finished_;
};


struct route_rank_store {
  route_rank_store() = default;
  explicit route_rank_store(vector_map<route_idx_t, route_rank_t>&& route_ranks,
                            vector_map<transport_idx_t, transport_rank_t>&& transport_ranks,
                            route_partition&& p);

  auto                                          cista_members();

  static cista::wrapped<route_rank_store>       read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;
  void                                          print_summary(std::ostream&) const;

  vector_map<route_idx_t, route_rank_t> route_ranks_;
  vector_map<transport_idx_t, transport_rank_t> transport_ranks_;
  route_partition partition_;
};



} // namespace nigiri::routing::para