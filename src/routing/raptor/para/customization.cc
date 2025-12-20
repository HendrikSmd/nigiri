#include "nigiri/routing/raptor/para/customization.h"

namespace nigiri::routing::para {

customizer::customizer(timetable const& tt) :
  tt_(tt) {}

route_rank_store customizer::construct_route_rank_store(interval<unixtime_t> ranks_for,
                                                        route_partition&& partition) {
  //TODO
  return route_rank_store(ranks_for, {}, std::move(partition));
}

route_rank_store::route_rank_store(interval<unixtime_t> ranks_for,
                                   vector_map<route_idx_t, route_rank_t>&& ranks,
                                   route_partition&& p) :
  ranks_valid_for_(std::move(ranks_for)),
  ranks_(std::move(ranks)),
  partition_(std::move(p)) {}

auto route_rank_store::cista_members() {
  return std::tie(ranks_valid_for_, ranks_, partition_);
}

void route_rank_store::write(std::filesystem::path const& path) const {
  return cista::write(path, *this);
}

cista::wrapped<route_rank_store> route_rank_store::read(std::filesystem::path const& path) {
  return cista::read<route_rank_store>(path);
}

} // namespace nigiri::routing::para
