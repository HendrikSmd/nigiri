#pragma once

#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/common/standard_flat_matrix.h"

#include "bmc_raptor_bag.h"
#include "bmc_raptor_label.h"
#include "bmc_raptor_route_label.h"

namespace nigiri::routing::para {

using bmc_raptor_bag_t = bmc_raptor_bag<bmc_raptor_label>;
using bmc_raptor_route_bag_t = bmc_raptor_bag<bmc_raptor_route_label>;

struct bmc_raptor_state {

  void resize(unsigned n_locations,
              unsigned n_routes,
              unsigned n_destinations);

  void reset();

  std::vector<bmc_raptor_bag_t> best_bags_;
  simple_flat_matrix<bmc_raptor_bag_t> round_bags_;

  std::vector<pareto_set<journey>> results_;
  bitvec fp_label_added_;
  bitvec station_mark_;
  bitvec prev_station_mark_;
  bitvec route_mark_;
};

} // nigiri::routing::para
