#include "nigiri/routing/raptor/raptor_search.h"

#include "nigiri/common/parse_time.h"
#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/search.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/mc_raptor_state.h"
#include "nigiri/routing/raptor/mc_raptor.h"
#include "fmt/core.h"

namespace nigiri::routing {

template <direction SearchDir>
routing_result<raptor_stats> raptor_search(timetable const& tt,
                                           rt_timetable const* rtt,
                                           routing::query q,
                                           reach_mode mode) {
  using algo_state_t = routing::raptor_state;
  static auto search_state = routing::search_state{};
  static auto algo_state = algo_state_t{};

  if (rtt == nullptr) {
    using algo_t = routing::raptor<SearchDir, false>;
    return routing::search<SearchDir, algo_t>{tt, rtt, search_state,
                                                algo_state, std::move(q), mode}
                 .execute();
  } else {
    using algo_t = routing::raptor<SearchDir, true>;
    return routing::search<SearchDir, algo_t>{tt, rtt, search_state,
                                                algo_state, std::move(q), mode}
                 .execute();
  }
}

routing_result<raptor_stats> raptor_search(timetable const& tt,
                                           rt_timetable const* rtt,
                                           routing::query q,
                                           reach_mode mode,
                                           direction const search_dir) {
  if (search_dir == direction::kForward) {
    return raptor_search<direction::kForward>(tt, rtt, std::move(q), mode);
  } else {
    return raptor_search<direction::kBackward>(tt, rtt, std::move(q), mode);
  }
}

routing_result<raptor_stats> raptor_search(timetable const& tt,
                                           rt_timetable const* rtt,
                                           std::string_view from,
                                           std::string_view to,
                                           routing::start_time_t time,
                                           reach_mode mode,
                                           direction const search_dir) {
  auto const src = source_idx_t{0};
  auto q = routing::query{
      .start_time_ = time,
      .start_ = {{tt.locations_.location_id_to_idx_.at({from, src}), 0_minutes,
                  0U}},
      .destination_ = {
          {tt.locations_.location_id_to_idx_.at({to, src}), 0_minutes, 0U}}};
  return raptor_search(tt, rtt, std::move(q), mode, search_dir);
}

routing_result<raptor_stats> raptor_search(timetable const& tt,
                                           rt_timetable const* rtt,
                                           std::string_view from,
                                           std::string_view to,
                                           std::string_view time,
                                           reach_mode mode,
                                           direction const search_dir) {
  return raptor_search(tt, rtt, from, to, parse_time(time, "%Y-%m-%d %H:%M %Z"),
                       mode, search_dir);
}

routing_result<raptor_stats> raptor_search(timetable const& tt,
                                           rt_timetable const* rtt,
                                           std::string_view from,
                                           std::string_view to,
                                           std::string_view start_time,
                                           std::string_view end_time,
                                           reach_mode mode,
                                           direction const search_dir) {
  interval<unixtime_t> inter;
  inter.from_ = parse_time(start_time, "%Y-%m-%d %H:%M %Z");
  inter.to_ = parse_time(end_time, "%Y-%m-%d %H:%M %Z");
  return raptor_search(tt, rtt, from, to, inter,
                       mode, search_dir);
}

std::vector<pareto_set<routing::journey>> mc_raptor_search(timetable const& tt,
                                                           std::string_view from,
                                                           std::string_view start_time,
                                                           std::string_view end_time) {
  auto const src = source_idx_t{0};
  mc_raptor_state state{};

  interval<unixtime_t> inter;
  inter.from_ = parse_time(start_time, "%Y-%m-%d %H:%M %Z");
  inter.to_ = parse_time(end_time, "%Y-%m-%d %H:%M %Z");
  routing::mc_raptor raptor{tt, state, inter, location_match_mode::kExact, {{tt.locations_.location_id_to_idx_.at({from, src}), 0_minutes,
                                                                    0U}}};
  raptor.route();
  return state.results_;
}

std::vector<pareto_set<routing::journey>> mc_raptor_search(timetable const& tt,
                                                           std::string_view from,
                                                           interval<unixtime_t> time) {
  auto const src = source_idx_t{0};
  mc_raptor_state state{};
  routing::mc_raptor raptor{tt, state, time, location_match_mode::kExact, {{tt.locations_.location_id_to_idx_.at({from, src}), 0_minutes,
                                                                             0U}}};
  raptor.route();
  return state.results_;
}

routing_result<raptor_stats> raptor_intermodal_search(
                          timetable const& tt,
                          rt_timetable const* rtt,
                          std::vector<routing::offset> start,
                          std::vector<routing::offset> destination,
                          routing::start_time_t const interval,
                          direction const search_dir,
                          std::uint8_t const min_connection_count,
                          bool const extend_interval_earlier,
                          bool const extend_interval_later) {
  auto q = routing::query{
      .start_time_ = interval,
      .start_match_mode_ = routing::location_match_mode::kIntermodal,
      .dest_match_mode_ = routing::location_match_mode::kIntermodal,
      .start_ = std::move(start),
      .destination_ = std::move(destination),
      .min_connection_count_ = min_connection_count,
      .extend_interval_earlier_ = extend_interval_earlier,
      .extend_interval_later_ = extend_interval_later};
  return raptor_search(tt, rtt, std::move(q), reach_mode::kNoReach, search_dir);
}

}  // namespace nigiri::test