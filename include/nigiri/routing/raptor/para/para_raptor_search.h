#pragma once

#include "nigiri/routing/raptor/raptor.h"
#include "nigiri/routing/raptor/para/para_search.h"
#include "nigiri/routing/raptor/para/route_rank_store.h"
#include "nigiri/location_match_mode.h"

namespace nigiri::routing::para {

template <para_rank_store store_t>
routing_result para_raptor_search(
    timetable const& tt,
    store_t const& rrs,
    search_state& s_state,
    raptor_state& r_state,
    query q,
    std::optional<std::chrono::seconds> const timeout = std::nullopt) {
  auto span = get_otel_tracer()->StartSpan("para_raptor_search");
  auto scope = opentelemetry::trace::Scope{span};
  if (span->IsRecording()) {
    std::visit(utl::overloaded{
                   [&](interval<unixtime_t> const& interval) {
                     span->SetAttribute("nigiri.query.start_time_interval.from",
                                        date::format("%FT%RZ", interval.from_));
                     span->SetAttribute("nigiri.query.start_time_interval.to",
                                        date::format("%FT%RZ", interval.to_));
                   },
                   [&](unixtime_t const& t) {
                     span->SetAttribute("nigiri.query.start_time",
                                        date::format("%FT%RZ", t));
                   }},
               q.start_time_);
    span->SetAttribute("nigiri.query.start_match_mode",
                       location_match_mode_str(q.start_match_mode_));
    span->SetAttribute("nigiri.query.destination_match_mode",
                       location_match_mode_str(q.dest_match_mode_));
    span->SetAttribute("nigiri.query.use_start_footpaths",
                       q.use_start_footpaths_);
    span->SetAttribute("nigiri.query.start_count", q.start_.size());
    span->SetAttribute("nigiri.query.destination_count", q.destination_.size());
    span->SetAttribute("nigiri.query.td_start_count", q.td_start_.size());
    span->SetAttribute("nigiri.query.td_destination_count", q.td_dest_.size());
    span->SetAttribute("nigiri.query.max_start_offset",
                       q.max_start_offset_.count());
    span->SetAttribute("nigiri.query.max_transfers", q.max_transfers_);
    span->SetAttribute("nigiri.query.min_connection_count",
                       q.min_connection_count_);
    span->SetAttribute("nigiri.query.extend_interval_earlier",
                       q.extend_interval_earlier_);
    span->SetAttribute("nigiri.query.extend_interval_later",
                       q.extend_interval_later_);
    span->SetAttribute("nigiri.query.prf_idx", q.prf_idx_);
    span->SetAttribute("nigiri.query.allowed_classes", q.allowed_claszes_);
    span->SetAttribute("nigiri.query.require_bike_transport",
                       q.require_bike_transport_);
    span->SetAttribute("nigiri.query.transfer_time_settings.default",
                       q.transfer_time_settings_.default_);
    span->SetAttribute("nigiri.query.via_stops_count", q.via_stops_.size());
    span->SetAttribute(
        "nigiri.query.search_direction","forward");
    if (timeout) {
      span->SetAttribute("nigiri.query.timeout", timeout.value().count());
    }
  }
  q.sanitize(tt);
  return para_search{tt, rrs, s_state, r_state, std::move(q), timeout}
  .execute();
}




} // namespace nigiri::routing::para
