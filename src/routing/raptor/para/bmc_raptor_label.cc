#include "nigiri/routing/raptor/para/bmc_raptor_label.h"

namespace nigiri::routing::para {

bool bmc_raptor_label::dominates_non_destination(bmc_raptor_label const& o) const {
  return departure_ >= o.departure_ &&
         arrival_with_transfer_ <= o.arrival_with_transfer_;
}

bool bmc_raptor_label::dominates_non_destination(bmc_raptor_label const& l1,
                                                 bmc_raptor_label const& l2) {
  return l1.dominates_non_destination(l2);
}

bool bmc_raptor_label::dominates_destination(bmc_raptor_label const& o) const {
  return departure_ >= o.departure_ &&
         arrival_with_transfer_ <= o.arrival_;
}

bool bmc_raptor_label::dominates_destination(bmc_raptor_label const& l1,
                                             bmc_raptor_label const& l2) {
  return l1.dominates_destination(l2);
}

bool bmc_raptor_best_label::dominates_non_destination(bmc_raptor_best_label const& l1, bmc_raptor_best_label const& l2) {
  return l1.departure_ >= l2.departure_ &&
         l1.arrival_with_transfer_ <= l2.arrival_with_transfer_;
}

bool bmc_raptor_best_label::dominates_destination(bmc_raptor_best_label const& l1, bmc_raptor_best_label const& l2) {
  return l1.departure_ >= l2.departure_ &&
       l1.arrival_with_transfer_ <= l2.arrival_;
}

bool bmc_raptor_best_label::equals(bmc_raptor_best_label const& l1, bmc_raptor_best_label const& l2) {
  return l1.departure_ == l2.departure_ &&
         l1.arrival_ == l2.arrival_ &&
         l1.arrival_with_transfer_ == l2.arrival_with_transfer_;
}

} // nigiri::routing::raptor::para
