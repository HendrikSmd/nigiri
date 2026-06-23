#include "nigiri/routing/raptor/para/mc_raptor_state.h"

#include "nigiri/common/clear_all.h"
#include "nigiri/routing/limits.h"

namespace nigiri::routing::para {

bool mc_raptor_label::dominates_non_destination(mc_raptor_label const& o) const {
  return departure_ >= o.departure_ &&
         arrival_with_transfer_ <= o.arrival_with_transfer_;
}

bool mc_raptor_label::dominates_non_destination(mc_raptor_label const& l1,
                                                mc_raptor_label const& l2) {
  return l1.dominates_non_destination(l2);
}

bool mc_raptor_label::dominates_destination(mc_raptor_label const& o) const {
  return departure_ >= o.departure_ &&
         arrival_with_transfer_ <= o.arrival_;
}

bool mc_raptor_label::dominates_destination(mc_raptor_label const& l1,
                                            mc_raptor_label const& l2) {
  return l1.dominates_destination(l2);
}

bool mc_raptor_route_label::dominates(mc_raptor_route_label const& other) const {
  return departure_ >= other.departure_ &&
         (transport_.day_ < other.transport_.day_ ||
            (transport_.day_ == other.transport_.day_ && transport_.t_idx_ <= other.transport_.t_idx_)
         );
}

bool mc_raptor_route_label::dominates(mc_raptor_route_label const& l1,
                                      mc_raptor_route_label const& l2) {
  return l1.dominates(l2);
}

void mc_raptor_state::resize(unsigned const n_locations,
                             unsigned const n_routes,
                             unsigned const n_destinations) {
  station_mark_.resize(n_locations);
  prev_station_mark_.resize(n_locations);
  fp_label_added_.resize(n_locations);
  route_mark_.resize(n_routes);
  best_.resize(n_locations);
  round_bags_.resize(kMaxTransfers + 1U, n_locations);
  results_.resize(n_destinations);
}

void mc_raptor_state::reset() {
  station_mark_.zero_out();
  prev_station_mark_.zero_out();
  route_mark_.zero_out();
  fp_label_added_.zero_out();

  clear_all(best_.begin(), best_.end());
  clear_all(round_bags_.entries_.begin(), round_bags_.entries_.end());
  clear_all(results_.begin(), results_.end());
}

}  // namespace nigiri::routing::para