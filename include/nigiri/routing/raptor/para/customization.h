#pragma once

#include "nigiri/routing/raptor/para/route_partition.h"

namespace nigiri::routing::para {

struct route_rank_store;

struct customizer {
  customizer(timetable const& tt);

  route_rank_store construct_route_rank_store(interval<unixtime_t> ranks_for,
                                              route_partition&& partition);

  const timetable& tt_;
};


struct route_rank_store {
  route_rank_store() = default;
  explicit route_rank_store(interval<unixtime_t> ranks_for,
                            vector_map<route_idx_t, route_rank_t>&& ranks,
                            route_partition&& p);

  auto                                          cista_members();

  static cista::wrapped<route_rank_store>       read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;

  interval<unixtime_t> ranks_valid_for_;
  vector_map<route_idx_t, route_rank_t> ranks_;
  route_partition partition_;
};



} // namespace nigiri::routing::para