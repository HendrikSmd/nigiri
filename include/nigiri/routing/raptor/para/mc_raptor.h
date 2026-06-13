#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/routing/raptor/para/mc_raptor_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/timetable.h"

#include "timetable_view.h"

namespace nigiri::routing::para {

struct mc_journey {

  static bool dominates(mc_journey const& j1, mc_journey const& j2);

  routing_time arrival_;
  routing_time departure_;
  std::uint16_t transfers_;
  pareto_set<mc_raptor_label>::const_iterator label_iter_;
};

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

    mc_raptor(timetable_view const& tt_view,
              mc_raptor_state& state,
              bitvec const& destination_mask,
              vector_map<route_idx_t, std::uint32_t> const& route_events_from,
              bitvec const& route_event_mask);

    void route();
    mc_raptor_stats const& get_stats() const;

    void emplace_relative_journeys_for(location_idx_view_t loc_idx,
                                       std::vector<mc_journey>& bag) const;

  static bool                 dominates_destination(mc_raptor_label const& l1,
                                                    mc_raptor_label const& l2);

  static bool                 dominates_non_destination(mc_raptor_label const& l1,
                                                        mc_raptor_label const& l2);

  static bool                 dominates_destination_skip_fps(mc_raptor_label const& l1,
                                                             mc_raptor_label const& l2);

  static bool                 dominates_non_destination_skip_fps(mc_raptor_label const& l1,
                                                                 mc_raptor_label const& l2);

  static bool                 add_to_non_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                                        mc_raptor_label const& label);

  static bool                 add_to_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                                    mc_raptor_label const& label);

  template <bool skip_bag_arrivals_with_fps>
  static bool is_dominated_by_non_dest_bag(
      pareto_set<mc_raptor_label> const& bag, mc_raptor_label const& label) {
    if constexpr (skip_bag_arrivals_with_fps) {
      return bag.is_dominated<&dominates_non_destination_skip_fps>(label);
    } else {
      return bag.is_dominated<&dominates_non_destination>(label);
    }
  }

  template <bool skip_bag_arrivals_with_fps>
  static bool is_dominated_by_dest_bag(pareto_set<mc_raptor_label> const& bag,
                                       mc_raptor_label const& label) {
    if constexpr (skip_bag_arrivals_with_fps) {
      return bag.is_dominated<&dominates_destination_skip_fps>(label);
    } else {
      return bag.is_dominated<&dominates_destination>(label);
    }
  }
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

  timetable_view const& tt_view_;
  mc_raptor_state& state_;
  mc_raptor_stats stats_;
  int n_tt_days_;
  std::uint32_t n_locations_, n_routes_;
  bitvec const& destination_mask_;

  vector_map<route_idx_t, std::uint32_t> route_events_from_;
  bitvec const& route_event_mask_;
};

} // namespace nigiri::routing::para