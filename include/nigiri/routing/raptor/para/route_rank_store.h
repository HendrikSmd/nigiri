#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/raptor/para/route_partition.h"

namespace nigiri::routing::para {

struct scan_stop {
  stop_idx_t stop_idx_ : 14;
  stop_idx_t scan_arrive_ : 1;
  stop_idx_t scan_depart_ : 1;
};

template <std::size_t NMaxTypes>
constexpr auto static_type_hash(scan_stop const*,
                                cista::hash_data<NMaxTypes> h) noexcept {
  return h.combine(cista::hash("nigiri::routing::para::scan_stop"));
}

template <typename Ctx>
inline void serialize(Ctx&, scan_stop const*, cista::offset_t const) {}

template <typename Ctx>
inline void deserialize(Ctx const&, scan_stop*) {}

struct empty_route_rank_store {};

struct plain_route_rank_store {
  auto                                          cista_members();
  static cista::wrapped<plain_route_rank_store> read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;
  void                                          print_summary(std::ostream& out, timetable const& tt) const;
  void                                          digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks);

  vector_map<route_idx_t, rank_t> route_ranks_;
  vecvec<route_idx_t, rank_t> route_event_ranks_;
  route_partition partition_;
};

struct skip_list_route_rank_store_lcl_packed {
  auto                                                         cista_members();
  static cista::wrapped<skip_list_route_rank_store_lcl_packed> read(std::filesystem::path const&);
  void                                                         write(std::filesystem::path const&) const;
  void                                                         digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks);

  vector_map<route_idx_t, rank_t> route_ranks_;
  vecvec<route_idx_t, size_t> scan_stop_starts_;
  /*
   * Route 0:
   *   minLCL = 1: [stops to scan...]
   *   minLCL = 2: [stops to scan...]
   * Route 1:
   */
  vector<scan_stop> scan_stops_;
  route_partition partition_;
};

struct skip_list_route_rank_store_route_packed {
  auto                                                           cista_members();
  static cista::wrapped<skip_list_route_rank_store_route_packed> read(std::filesystem::path const&);
  void                                                           write(std::filesystem::path const&) const;
  void                                                           digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks);

  vector_map<route_idx_t, rank_t> route_ranks_;
  vecvec<std::uint32_t, size_t> scan_stop_starts_;
  /*
   * minLCL = 1:
   *   Route 0: [stops to scan...]
   *   Route 1: [stops to scan...]
   * minLCL = 2:
   */
  vector<scan_stop> scan_stops_;
  route_partition partition_;
};

struct bitvec_route_rank_store {
  using block_t = std::uint64_t;

  auto                                           cista_members();
  static cista::wrapped<bitvec_route_rank_store> read(std::filesystem::path const&);
  void                                           write(std::filesystem::path const&) const;
  void                                           digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks);

  template <bool enable_bmi = true, typename Fn>
  void for_each_stop_to_scan(route_idx_t const r, rank_t const min_lcl, Fn&& f) const {
    const auto from_block = first_block_idx_[to_idx(min_lcl) - 1][to_idx(r)];
    const auto to_block = first_block_idx_[to_idx(min_lcl) - 1][to_idx(r) + 1];
    if (to_block == from_block) {
      return;
    }
    auto const check_block = [&](stop_idx_t const i, block_t block) {
      if (block == 0U) {
        return false;
      }
      if constexpr (enable_bmi) {
        while (block != 0U) {
          // 1. Instantly isolate the lowest active 2-bit stop combo
          // e.g., if block is 00110000, lowest_stop_bits becomes 00110000
          // We use a known trick: block & (~block + 1) or a quick bitwise manipulation
          // To handle 2-bit steps perfectly, we look at trailing zeros:
          int const trailing_zeros = std::countr_zero(block);
          int const stop = trailing_zeros / 2;

          // 2. Align to the lowest bits using a fixed mask rather than a variable shift
          // By shifting the 3U mask *up* instead of shifting the block *down*,
          // we reduce dependencies on the block register.
          auto const mask = block_t{3U} << (stop * 2);
          auto const active_bits = (block & mask) >> (stop * 2);

          bool const early_stop =
              f(stop_idx_t{static_cast<unsigned short int>(i * 32U + stop)},
                (active_bits & block_t{1U}) != 0U,
                (active_bits & block_t{2U}) != 0U);

          if (early_stop) {
            return true;
          }

          // 3. Clear the processed stop instantly
          block &= ~mask;
        }

        return false;
      } else {
        for (auto stop = stop_idx_t{0U}; stop != 32U; ++stop) {
          if ((block & block_t{3U}) != 0U) {
            bool const early_stop =
                f(stop_idx_t{static_cast<unsigned short int>(i * 32U + stop)},
                  (block & block_t{1U}) != 0U, (block & block_t{2U}) != 0U);
            if (early_stop) {
              return true;
            }
          }
          block >>= 2U;
        }

        return false;
      }
    };
    for (auto j = from_block; j != to_block; ++j) {
      const bool finished = check_block(j - from_block, blocks_[j]);
      if (finished) {
        break;
      }
    }
  }

  vector_map<route_idx_t, rank_t> route_ranks_;
  vecvec<std::uint32_t, size_t> first_block_idx_;
  /*
   * minLCL = 1:
   *   Route 0: [...]
   *   Route 1: [...]
   * minLCL = 2:
   */
  vector<block_t> blocks_;
  route_partition partition_;
};

template <typename T>
concept para_rank_store = std::disjunction_v<
    std::is_same<T, plain_route_rank_store>,
    std::is_same<T, skip_list_route_rank_store_lcl_packed>,
    std::is_same<T, skip_list_route_rank_store_route_packed>,
    std::is_same<T, bitvec_route_rank_store>
>;

} // namespace nigiri::routing::para