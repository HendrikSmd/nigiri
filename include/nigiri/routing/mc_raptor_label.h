#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"

#include <cinttypes>

namespace nigiri::routing {

struct mc_raptor_label {
  mc_raptor_label()
      : arrival_(routing_time::min()),
        departure_(routing_time::max()),
        walking_time_(minutes_after_midnight_t::max())
  {}

  mc_raptor_label(routing_time arrival,
                  routing_time departure,
                  minutes_after_midnight_t walking_time)
      : arrival_(arrival),
        departure_(departure),
        walking_time_(walking_time)
  {}

  inline bool equals(const mc_raptor_label& other) const noexcept {
    return  arrival_ == other.arrival_ &&
            departure_ == other.departure_ &&
            walking_time_ == other.walking_time_;
  }

  inline bool dominates(const mc_raptor_label& other) const noexcept {
    return  arrival_ <= other.arrival_ &&
            departure_ >= other.departure_ &&
            walking_time_ <= other.walking_time_;
  }

  routing_time arrival_;
  routing_time departure_;
  minutes_after_midnight_t walking_time_;
};

}