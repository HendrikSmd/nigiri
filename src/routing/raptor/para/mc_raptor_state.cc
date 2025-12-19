#include "nigiri/routing/raptor/para/mc_raptor_state.h"

#include "nigiri/routing/limits.h"

namespace nigiri::routing::para {

void mc_raptor_state::resize(size_t const n_locations,
                             size_t const n_routes,
                             size_t const n_destinations) {
  station_mark_.resize(n_locations);
  prev_station_mark_.resize(n_locations);
  route_mark_.resize(n_routes);
  best_.resize(n_locations);
  round_bags_.resize(kMaxTransfers + 1U, n_locations);
  results_.resize(n_destinations);
}

}  // namespace nigiri::routing::para