#pragma once

#include <vector>

#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/raptor/para/routing_time.h"
#include "nigiri/common/standard_flat_matrix.h"
#include "nigiri/types.h"


namespace nigiri::routing::para {

struct mc_raptor_label {

  bool        dominates_non_destination(mc_raptor_label const& o) const;
  static bool dominates_non_destination(mc_raptor_label const& l1, mc_raptor_label const& l2);

  bool        dominates_destination(mc_raptor_label const& o) const;
  static bool dominates_destination(mc_raptor_label const& l1, mc_raptor_label const& l2);

  routing_time arrival_;
  routing_time arrival_with_transfer_;
  routing_time departure_;
  std::uint32_t route_idx_;
  std::uint16_t enter_stop_idx_;
  std::uint16_t exit_stop_idx_;
  std::uint32_t parent_bag_idx_;
  bool is_footpath_;
  bool has_parent_;
};

struct mc_raptor_route_label {

  bool        dominates(mc_raptor_route_label const& other) const;
  static bool dominates(mc_raptor_route_label const& l1, mc_raptor_route_label const& l2);

  transport transport_;
  routing_time departure_;

  std::uint32_t parent_bag_idx_;
  std::uint16_t enter_stop_idx_;
};

using journey_with_label = std::pair<journey, mc_raptor_label>;

struct mc_raptor_state {
  mc_raptor_state() = default;
  mc_raptor_state(mc_raptor_state const&) = delete;
  mc_raptor_state& operator=(mc_raptor_state const&) = delete;
  mc_raptor_state(mc_raptor_state&&) = default;
  mc_raptor_state& operator=(mc_raptor_state&&) = default;
  ~mc_raptor_state() = default;

  void resize(unsigned n_locations,
              unsigned n_routes,
              unsigned n_destinations);

  void reset();

  std::vector<pareto_set<mc_raptor_label>> best_;
  simple_flat_matrix<pareto_set<mc_raptor_label>> round_bags_;
  bitvec station_mark_;
  bitvec prev_station_mark_;
  bitvec route_mark_;
  std::vector<pareto_set<journey>> results_;
};

}  // namespace nigiri::routing::para