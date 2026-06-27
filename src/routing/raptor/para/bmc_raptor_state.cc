#include "nigiri/routing/raptor/para/bmc_raptor_state.h"

#include "nigiri/common/clear_all.h"
#include "nigiri/routing/limits.h"

namespace nigiri::routing::para {

void bmc_raptor_state::resize(unsigned const n_locations,
                              unsigned const n_routes,
                              unsigned const n_destinations) {
  station_mark_.resize(n_locations);
  prev_station_mark_.resize(n_locations);
  route_mark_.resize(n_routes);
  best_bags_.resize(n_locations);
  tmp_bags_.resize(n_locations);
  round_bags_.resize(kMaxTransfers + 1U, n_locations);
  results_.resize(n_destinations);
}

void bmc_raptor_state::reset() {
  station_mark_.zero_out();
  prev_station_mark_.zero_out();
  route_mark_.zero_out();

  clear_all(best_bags_.begin(), best_bags_.end());
  clear_all(round_bags_.entries_.begin(), round_bags_.entries_.end());
  clear_all(results_.begin(), results_.end());
  clear_all(tmp_bags_.begin(), tmp_bags_.end());
}

} // nigiri::routing::raptor::para