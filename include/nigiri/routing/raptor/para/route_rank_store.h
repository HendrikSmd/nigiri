#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/raptor/para/route_partition.h"

namespace nigiri::routing::para {

struct route_rank_store {
  auto                                          cista_members();
  static cista::wrapped<route_rank_store>       read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;
  void                                          print_summary(std::ostream&) const;

  vector_map<route_idx_t, interval<std::uint32_t>> route_rank_ranges_;
  vector<rank_t> ranks_;
  route_partition partition_;
};

} // namespace nigiri::routing::para