#pragma once

#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/para/mc_raptor_state.h"
#include "nigiri/routing/query.h"

namespace nigiri::routing::para {

struct mc_raptor_stats {
    std::uint64_t n_routing_time_{0ULL};
    std::uint64_t n_footpaths_visited_{0ULL};
    std::uint64_t n_routes_visited_{0ULL};
    std::uint64_t n_earliest_trip_calls_{0ULL};
    std::uint64_t n_earliest_arrival_updated_by_route_{0ULL};
    std::uint64_t n_earliest_arrival_updated_by_footpath_{0ULL};
    std::uint64_t fp_update_prevented_by_lower_bound_{0ULL};
    std::uint64_t route_update_prevented_by_lower_bound_{0ULL};
};

struct mc_raptor {

    static constexpr bool kVerifyReconstruction = false;

    mc_raptor(timetable const& tt,
              mc_raptor_state& state,
              interval<unixtime_t> search_interval,
              location_match_mode start_match_mode,
              std::vector<offset> const& start,
              bitvec const& reconstruct_mask,
              bitvec const& route_mask);

    void route();
    mc_raptor_stats const& get_stats() const;

private:

  static unsigned             end_k();
  static bool                 is_better(auto a, auto b);
  static bool                 is_better_or_eq(auto a, auto b);

  void                        rounds();
  void                        reconstruct() const;
  bool                        update_route(unsigned k, route_idx_t route);
  transport                   get_earliest_transport(const mc_raptor_label& current,
                                                     route_idx_t r,
                                                     stop_idx_t stop_idx);
  void                        update_footpaths(unsigned k) const;
  day_idx_t                   start_day_offset() const;
  bool                        is_journey_start(location_idx_t l) const;
  std::optional<journey::leg> find_start_footpath(location_idx_t leg_start_location,
                                                   routing_time leg_start_time,
                                                  routing_time journey_start_time) const;
  interval<stop_idx_t>        find_enter_exit(transport via,
                                              location_idx_t enter,
                                              routing_time enter_after,
                                              location_idx_t exit) const;
  duration_t                  get_fastest_start_dest_overlap(location_idx_t dest) const;
  duration_t                  get_fastest_direct_with_foot(location_idx_t dest) const;
  duration_t                  get_fastest_direct(location_idx_t dest) const;

  timetable const& tt_;
  int n_tt_days_;
  mc_raptor_state& state_;
  interval<unixtime_t> const search_interval_;
  mc_raptor_stats stats_;
  std::uint32_t n_locations_, n_routes_;
  std::vector<offset> const start_;
  location_match_mode start_match_mode_;
  bitvec const& reconstruct_mask_;
  bitvec const& route_mask_;
};

} // namespace nigiri::routing::para