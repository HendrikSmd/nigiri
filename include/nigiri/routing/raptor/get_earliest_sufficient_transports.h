#pragma once

#include <cstdint>

#include "cista/cuda_check.h"
#include "nigiri/common/linear_lower_bound.h"
#include "nigiri/routing/label.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

#include "cista/containers/bitset.h"
#include "utl/enumerate.h"

namespace nigiri::routing {

template <std::size_t Size_1, std::size_t Size_2>
CISTA_CUDA_COMPAT void truncate_to(const cista::bitset<Size_1>& bitset_1, cista::bitset<Size_2>& bitset_2) {
  for (auto i = 0U; i<bitset_2.blocks_.size(); ++i) {
    bitset_2.blocks_[i] = bitset_1.blocks_[i];
  }
}

template <size_t Size, typename Fun, typename TimetableType, typename LabelType>
CISTA_CUDA_COMPAT void get_earliest_sufficient_transports(TimetableType const& tt,
                                        LabelType const l,
                                        route_idx_t route_idx,
                                        unsigned short stop_idx,
                                        Fun&& consume) {
  auto const dep_event_times =
      tt.event_times_at_stop(route_idx, stop_idx, event_type::kDep);

  constexpr auto n_days_to_iterate = kMaxTravelTime.count() / 1440 + 1U;

  delta const arr_as_delta(l.arrival_with_transfer_);
  auto const arr_days_after_dep =
      static_cast<std::uint16_t>(arr_as_delta.days());

  auto const seek_first_day = [&]() {
    return linear_lb(
        dep_event_times.begin(), dep_event_times.end(), arr_as_delta.mam(),
        [&](delta const a, int16_t const b) { return a.mam() < b; });
  };

  auto to_serve_tdb = l.active_days_;
  for (auto days_after_dep = arr_days_after_dep;
       days_after_dep < n_days_to_iterate; ++days_after_dep) {
    if (to_serve_tdb.none()) {
      return;
    }

    auto const time_range_to_scan =
        it_range{days_after_dep == arr_days_after_dep ? seek_first_day()
                                                      : dep_event_times.begin(),
                 dep_event_times.end()};

    if (time_range_to_scan.empty()) {
      continue;
    }

    auto const base = static_cast<unsigned>(&*time_range_to_scan.begin_ -
                                            dep_event_times.data());
    for (auto i = 0U; i < time_range_to_scan.size(); ++i) {
      const auto event_time = time_range_to_scan[i];
      if (to_serve_tdb.none()) {
        return;
      }

      auto const travel_time_lb =
          event_time.mam() + 1440 * days_after_dep - l.departure_;
      if (travel_time_lb > kMaxTravelTime.count()) {
        return;
      }

      auto const event_day_offset = event_time.days();
      
      // We resolve the route transport range's base index
#ifdef __CUDA_ARCH__
      auto const route_transports = tt.route_transport_ranges_[route_idx];
      auto const transport = transport_idx_t{route_transports.from_ + base + i};
#else
      auto const transport =
          tt.route_transport_ranges_[route_idx][base + i];
#endif

      int const net_shift_right = days_after_dep - event_day_offset;
      auto const bitfield_idx = tt.transport_traffic_days_[transport];
      auto const& transport_tdb = tt.bitfields_[bitfield_idx];
      auto const aligned_transport_tdb =
          (net_shift_right >= 0)
              ? (transport_tdb >> static_cast<size_t>(net_shift_right))
              : (transport_tdb << static_cast<size_t>(-net_shift_right));


      cista::bitset<Size> truncated_aligned_transport_tdb;
      truncate_to(aligned_transport_tdb, truncated_aligned_transport_tdb);

      if (truncated_aligned_transport_tdb.none()) {
        continue;
      }

      auto const matches = to_serve_tdb & truncated_aligned_transport_tdb;
      if (matches.any()) {
        consume({l.arrival_, l.arrival_with_transfer_, l.departure_, matches});
        to_serve_tdb &= ~matches;
      }
    }
  }
}

}