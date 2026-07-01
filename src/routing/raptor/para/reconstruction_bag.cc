#include "nigiri/routing/raptor/para/reconstruction_bag.h"

namespace nigiri::routing::para {

void reconstruction_bag::move_up_until_departure(routing_time const target) {
  while (scan_begin_ != labels_.end()) {
    if (scan_begin_->departure_ >= target) {
      break;
    }
    ++scan_begin_;
  }
}

bool reconstruction_bag::is_initialized() const {
  return initialized_;
}

bool reconstruction_bag::no_more_candidates() const {
  return scan_begin_ == labels_.end();
}

void reconstruction_bag::clear() {
  labels_.clear();
  scan_begin_ = labels_.end();
  initialized_ = false;
}

void reconstruction_bag::decompress(bmc_raptor_bag<bmc_raptor_label> const& round_bag) {

  for (const auto& label : round_bag) {
    auto const& active_days = label.tdb_;
    active_days.for_each_set_bit([&](size_t const i) {
      labels_.emplace_back(
        routing_time{static_cast<int>(i * 1440 + label.label_.arrival_)},
        routing_time{static_cast<int>(i * 1440 + label.label_.departure_)}
      );
    });
  }

  std::sort(labels_.begin(), labels_.end(),
    [](const reconstruction_label& a, const reconstruction_label& b) {
        if (a.departure_ != b.departure_) {
            return a.departure_ < b.departure_;
        }
        return a.arrival_ < b.arrival_;
    });

  auto it = std::unique(labels_.begin(), labels_.end(),
    [](const reconstruction_label& a, const reconstruction_label& b) {
        return a.departure_ == b.departure_;
    });

  labels_.erase(it, labels_.end());
  scan_begin_ = labels_.begin();
  initialized_ = true;
}

}  // namespace nigiri::routing::para
