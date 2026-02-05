#include "nigiri/routing/raptor/para/bmc_raptor_route_label.h"

namespace nigiri::routing::para {

bool bmc_raptor_route_label::dominates(bmc_raptor_route_label const& other) const {
  return departure_ >= other.departure_ &&
         (transport_day_offset_ < other.transport_day_offset_ ||
            (transport_day_offset_ == other.transport_day_offset_ && transport_idx_ <= other.transport_idx_)
         );
}

bool bmc_raptor_route_label::dominates(bmc_raptor_route_label const& l1,
                                       bmc_raptor_route_label const& l2) {
  return l1.dominates(l2);
}

} // nigiri::routing::raptor::para
