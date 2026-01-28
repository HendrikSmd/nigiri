#include "nigiri/routing/raptor/para/mc_raptor_state.h"

#include "nigiri/routing/limits.h"

#include <concepts>
#include <iterator>

namespace nigiri::routing::para {

template<typename T>
concept Clearable = requires(T a) {
  a.clear();
};

/**
 * Iterates through a range and calls .clear() on every element.
 */
template<std::input_iterator It>
void clear_all(It first, It last) {
  for (; first != last; ++first) {
    if constexpr (Clearable<std::iter_value_t<It>>) {
      first->clear();
    }
  }
}

void mc_raptor_state::resize(unsigned const n_locations,
                             unsigned const n_routes,
                             unsigned const n_destinations) {
  station_mark_.resize(n_locations);
  prev_station_mark_.resize(n_locations);
  route_mark_.resize(n_routes);
  best_.resize(n_locations);
  round_bags_.resize(kMaxTransfers + 1U, n_locations);
  results_.resize(n_destinations);
}

void mc_raptor_state::reset() {
  station_mark_.zero_out();
  prev_station_mark_.zero_out();
  route_mark_.zero_out();

  clear_all(best_.begin(), best_.end());
  clear_all(round_bags_.entries_.begin(), round_bags_.entries_.end());
  clear_all(results_.begin(), results_.end());
}

}  // namespace nigiri::routing::para