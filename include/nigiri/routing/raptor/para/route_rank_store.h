#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/raptor/para/route_partition.h"

namespace nigiri::routing::para {

struct route_rank_store {
  auto                                          cista_members();
  static cista::wrapped<route_rank_store>       read(std::filesystem::path const&);
  void                                          write(std::filesystem::path const&) const;
  void                                          print_summary(std::ostream&) const;

  vector_map<route_idx_t, std::uint32_t> route_rank_start_idx_;
  vector<rank_t> route_ranks_;
  vector_map<footpath_idx_t, rank_t> footpath_ranks_;

  route_partition partition_;
};

} // namespace nigiri::routing::para