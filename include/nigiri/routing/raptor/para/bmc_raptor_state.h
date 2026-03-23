#pragma once

#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/common/standard_flat_matrix.h"

#include "bmc_raptor_bag.h"
#include "bmc_raptor_label.h"
#include "bmc_raptor_route_label.h"
#include "simd_pareto_set.h"

//#define NIGIRI_ENABLE_SIMD

namespace nigiri::routing::para {

#ifdef NIGIRI_ENABLE_SIMD
using bmc_raptor_bag_t = simd_pareto_bag<std::uint16_t, kMaxSearchDays, 3, bmc_round_meta_data>;
using buffered_fp_label = std::tuple<std::array<std::uint16_t, 3>, bmc_round_meta_data, search_bitfield>;
#else
using bmc_raptor_bag_t = bmc_raptor_bag<bmc_raptor_label>;
using buffered_fp_label = std::pair<bmc_raptor_label, search_bitfield>;
#endif
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
