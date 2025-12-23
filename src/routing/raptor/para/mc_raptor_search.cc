#include "nigiri/routing/raptor/para/mc_raptor_search.h"

#include "nigiri/common/parse_time.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/search.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/para/mc_raptor_state.h"
#include "nigiri/routing/raptor/para/mc_raptor.h"

namespace nigiri::routing::para {

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  location_idx_t const from,
                                                  std::string_view const start_time,
                                                  std::string_view const end_time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask) {
  mc_raptor_state state{};

  interval<unixtime_t> inter;
  inter.from_ = parse_time(start_time, "%Y-%m-%d %H:%M %Z");
  inter.to_ = parse_time(end_time, "%Y-%m-%d %H:%M %Z");
  mc_raptor raptor{tt,
                   state,
                   inter,
                   location_match_mode::kExact,
                   {
                     {from, 0_minutes, 0U}
                   },
                   reconstruct_mask,
                   route_mask};
  raptor.route();
  return state.results_;
}

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  location_idx_t const from,
                                                  interval<unixtime_t> const time,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask) {
  mc_raptor_state state{};
  mc_raptor raptor{tt,
                   state,
                   time,
                   location_match_mode::kExact,
                   {
                       {from, 0_minutes, 0U}
                   },
                   reconstruct_mask,
                   route_mask};
  raptor.route();
  return state.results_;
}

std::vector<pareto_set<journey>> mc_raptor_search(timetable const& tt,
                                                  location_idx_t const from,
                                                  bitvec const& reconstruct_mask,
                                                  bitvec const& route_mask) {
  // Caveat: This searches the whole timetable!
  mc_raptor_state state{};
  mc_raptor raptor{tt,
                   state,
                   tt.internal_interval(),
                   location_match_mode::kExact,
                   {
                         {from, 0_minutes, 0U}
                   },
                   reconstruct_mask,
                   route_mask};
  raptor.route();
  return state.results_;
}

}  // namespace nigiri::routing::para