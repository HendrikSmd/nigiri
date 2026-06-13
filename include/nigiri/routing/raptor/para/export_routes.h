#pragma once

#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/para/route_rank_store.h"

#include <ostream>


namespace nigiri::routing::para {

void export_routes(timetable const& tt, std::ostream& out, plain_route_rank_store const& store);

} // namespace nigiri::routing::para