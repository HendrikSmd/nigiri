#pragma once

#include <vector>
#include "routing_time.h"
#include "nigiri/routing/raptor/para/bmc_raptor_bag.h"
#include "nigiri/routing/raptor/para/bmc_raptor_label.h"

namespace nigiri::routing::para {


struct reconstruction_label {
  routing_time arrival_;
  routing_time departure_;
};

struct reconstruction_bag {

  void move_up_until_departure(routing_time target);

  bool is_initialized() const;
  bool no_more_candidates() const;
  void clear();

  void decompress(bmc_raptor_bag<bmc_raptor_label> const& round_bag);

  std::vector<reconstruction_label> labels_{};
  std::vector<reconstruction_label>::const_iterator scan_begin_{};
  bool initialized_ = false;
};

}