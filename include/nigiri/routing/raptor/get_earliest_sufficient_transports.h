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

template <std::size_t Size>
CISTA_CUDA_COMPAT void bitset_sanitize(cista::bitset<Size>& bs) noexcept {
  constexpr auto const bits_per_block = sizeof(typename cista::bitset<Size>::block_t) * 8U;
  constexpr auto const num_blocks = cista::bitset<Size>::num_blocks;
  if constexpr ((Size % bits_per_block) != 0U) {
    using block_t = typename cista::bitset<Size>::block_t;
    bs.blocks_[num_blocks - 1U] &= ~((~block_t{0U}) << (Size % bits_per_block));
  }
}

template <std::size_t Size>
CISTA_CUDA_COMPAT bool bitset_none(cista::bitset<Size> const& bs) noexcept {
  for (std::size_t i = 0; i < bs.blocks_.size(); ++i) {
    if (bs.blocks_[i] != 0U) {
      return false;
    }
  }
  return true;
}

template <std::size_t Size>
CISTA_CUDA_COMPAT bool bitset_any(cista::bitset<Size> const& bs) noexcept {
  return !bitset_none(bs);
}

template <std::size_t Size>
CISTA_CUDA_COMPAT cista::bitset<Size> bitset_and(cista::bitset<Size> const& a, cista::bitset<Size> const& b) noexcept {
  cista::bitset<Size> ret;
  for (std::size_t i = 0; i < a.blocks_.size(); ++i) {
    ret.blocks_[i] = a.blocks_[i] & b.blocks_[i];
  }
  return ret;
}

template <std::size_t Size>
CISTA_CUDA_COMPAT void bitset_and_not(cista::bitset<Size>& a, cista::bitset<Size> const& b) noexcept {
  for (std::size_t i = 0; i < a.blocks_.size(); ++i) {
    a.blocks_[i] &= ~b.blocks_[i];
  }
}

template <std::size_t Size>
CISTA_CUDA_COMPAT cista::bitset<Size> bitset_shift_right(cista::bitset<Size> const& bs, std::size_t const shift) noexcept {
  cista::bitset<Size> ret = bs;
  bitset_sanitize(ret);
  if (shift >= Size) {
    ret.blocks_ = {};
    return ret;
  }
  constexpr auto const bits_per_block = sizeof(typename cista::bitset<Size>::block_t) * 8U;
  constexpr auto const num_blocks = cista::bitset<Size>::num_blocks;
  
  if constexpr (num_blocks == 1U) {
    ret.blocks_[0U] >>= shift;
    return ret;
  } else {
    if (shift == 0U) {
      return ret;
    }
    auto const shift_blocks = shift / bits_per_block;
    auto const shift_bits = shift % bits_per_block;
    auto const border = num_blocks - shift_blocks - 1U;

    if (shift_bits == 0U) {
      for (std::size_t i = 0U; i <= border; ++i) {
        ret.blocks_[i] = ret.blocks_[i + shift_blocks];
      }
    } else {
      for (std::size_t i = 0U; i < border; ++i) {
        ret.blocks_[i] =
            (ret.blocks_[i + shift_blocks] >> shift_bits) |
            (ret.blocks_[i + shift_blocks + 1] << (bits_per_block - shift_bits));
      }
      ret.blocks_[border] = (ret.blocks_[num_blocks - 1] >> shift_bits);
    }
    for (auto i = border + 1U; i != num_blocks; ++i) {
      ret.blocks_[i] = 0U;
    }
    return ret;
  }
}

template <std::size_t Size>
CISTA_CUDA_COMPAT cista::bitset<Size> bitset_shift_left(cista::bitset<Size> const& bs, std::size_t const shift) noexcept {
  cista::bitset<Size> ret = bs;
  if (shift >= Size) {
    ret.blocks_ = {};
    return ret;
  }
  constexpr auto const bits_per_block = sizeof(typename cista::bitset<Size>::block_t) * 8U;
  constexpr auto const num_blocks = cista::bitset<Size>::num_blocks;

  if constexpr (num_blocks == 1U) {
    ret.blocks_[0U] <<= shift;
    bitset_sanitize(ret);
    return ret;
  } else {
    if (shift == 0U) {
      return ret;
    }
    auto const shift_blocks = shift / bits_per_block;
    auto const shift_bits = shift % bits_per_block;

    if (shift_bits == 0U) {
      for (auto i = std::size_t{num_blocks - 1}; i >= shift_blocks; --i) {
        ret.blocks_[i] = ret.blocks_[i - shift_blocks];
      }
    } else {
      for (auto i = std::size_t{num_blocks - 1}; i != shift_blocks; --i) {
        ret.blocks_[i] =
            (ret.blocks_[i - shift_blocks] << shift_bits) |
            (ret.blocks_[i - shift_blocks - 1U] >> (bits_per_block - shift_bits));
      }
      ret.blocks_[shift_blocks] = (ret.blocks_[0U] << shift_bits);
    }
    for (std::size_t i = 0; i < shift_blocks; ++i) {
      ret.blocks_[i] = 0U;
    }
    bitset_sanitize(ret);
    return ret;
  }
}

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
  //printf("Start get earliest sufficient transport departure=%u, arr_with_transfer=%u\n", l.departure_, l.arrival_with_transfer_);
  auto const dep_event_times =
      tt.event_times_at_stop(route_idx, stop_idx, event_type::kDep);

  constexpr auto n_days_to_iterate = kMaxTravelTime.count() / 1440 + 1U;

  delta const arr_as_delta(l.arrival_with_transfer_);
  auto const arr_days_after_dep =
      static_cast<std::uint16_t>(arr_as_delta.days());

  //printf("arr_days_after_dep=%u\n", arr_days_after_dep);

  auto const seek_first_day = [&]() {
    return linear_lb(
        dep_event_times.begin(), dep_event_times.end(), arr_as_delta.mam(),
        [&](delta const a, int16_t const b) { return a.mam() < b; });
  };

  auto to_serve_tdb = l.active_days_;
  //printf("To serve %s\n\n", to_serve_tdb.to_string().c_str());
  for (auto days_after_dep = arr_days_after_dep;
       days_after_dep < n_days_to_iterate; ++days_after_dep) {
    //printf("days_after_dep=%u\n", days_after_dep);
    if (bitset_none(to_serve_tdb)) {
      //printf("to serve does not have any bits -> nothing to do");
      return;
    }

    auto begin_it = dep_event_times.begin();
    if (days_after_dep == arr_days_after_dep) {
      begin_it = seek_first_day();
    }

    if (begin_it == dep_event_times.end()) {
      //printf("time range empty -> next day");
      continue;
    }

    auto const base = static_cast<unsigned>(&*begin_it -
                                            dep_event_times.data());
    // printf("time range to scan: %lu\n", time_range_to_scan.size());
    for (auto it = begin_it; it != dep_event_times.end(); ++it) {
      const auto event_time = *it;
      //printf("Event time %u\n", event_time.mam());
      if (bitset_none(to_serve_tdb)) {
        //printf("to serve does not have any bits -> nothing to do");
        return;
      }
      auto const travel_time_lb =
          event_time.mam() + 1440 * days_after_dep - l.departure_;
#ifdef __CUDA_ARCH__
      if (travel_time_lb > kMaxCudaTravelTime) {
        return;
      }
#else
      if (travel_time_lb > kMaxTravelTime.count()) {
        //printf("Travel time will be higer than maxTravelTime -> stop search \n");
        return;
      }
#endif

      auto const event_day_offset = event_time.days();
      
      // We resolve the route transport range's base index
#ifdef __CUDA_ARCH__
      auto const route_transports = tt.route_transport_ranges_[route_idx];
      auto const transport = transport_idx_t{route_transports.from_ + base + std::distance(begin_it, it)};
#else
      auto const transport =
          tt.route_transport_ranges_[route_idx][base + std::distance(begin_it, it)];
#endif

      int const net_shift_right = days_after_dep - event_day_offset;
      auto const bitfield_idx = tt.transport_traffic_days_[transport];
      auto const& transport_tdb = tt.bitfields_[bitfield_idx];
      auto const aligned_transport_tdb =
          (net_shift_right >= 0)
              ? bitset_shift_right(transport_tdb, static_cast<size_t>(net_shift_right))
              : bitset_shift_left(transport_tdb, static_cast<size_t>(-net_shift_right));


      cista::bitset<Size> truncated_aligned_transport_tdb;
      truncate_to(aligned_transport_tdb, truncated_aligned_transport_tdb);
      //printf("Aligned event bitfield %s\n", truncated_aligned_transport_tdb.to_string().c_str());
      if (bitset_none(truncated_aligned_transport_tdb)) {
        continue;
      }

      auto const matches = bitset_and(to_serve_tdb, truncated_aligned_transport_tdb);
      if (bitset_any(matches)) {
        //printf("Found match\n");
        auto const label = route_label<64>{
          l.departure_,
          static_cast<int16_t>(net_shift_right),
          to_idx(transport),
          matches
        };
        consume(label);
        bitset_and_not(to_serve_tdb, matches);
        //printf("It remains to serve: %s\n", to_serve_tdb.to_string().c_str());
      }
    }
  }
}

}