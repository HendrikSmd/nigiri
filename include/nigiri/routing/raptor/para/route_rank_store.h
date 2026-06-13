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

struct skip_list_route_rank_store {
  auto                                              cista_members();
  static cista::wrapped<skip_list_route_rank_store> read(std::filesystem::path const&);
  void                                              write(std::filesystem::path const&) const;
  void                                              digest(timetable const& tt, route_partition partition, vecvec<route_idx_t, rank_t> route_event_ranks);

  vector_map<route_idx_t, rank_t> route_ranks_;
  vecvec<route_idx_t, size_t> scan_stop_starts_;
  /*
   * Route 0:
   *   minLCL = 1: [offsets]
   *   minLCL = 2: [offsets]
   * Route 1:
   */
  vector<scan_stop> scan_stops_;
  route_partition partition_;
};

template <typename T>
concept para_rank_store = std::disjunction_v<
    std::is_same<T, plain_route_rank_store>,
    std::is_same<T, skip_list_route_rank_store>
>;

} // namespace nigiri::routing::para