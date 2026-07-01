#include "nigiri/routing/raptor/para/bmc_raptor.h"

#include "boost/iostreams/seek.hpp"
#include "boost/program_options/detail/convert.hpp"

#include "nigiri/loader/register.h"

#include "nigiri/common/linear_lower_bound.h"
#include "nigiri/routing/raptor/para/reconstruction_bag.h"
#include "nigiri/routing/raptor/para/routing_time.h"
#include "nigiri/stop.h"
#include "nigiri/types.h"

#include "nigiri/routing/raptor/para/timetable_view.h"
#include "utl/enumerate.h"

namespace nigiri::routing::para {

bool bmc_journey::dominates(bmc_journey const& j1, bmc_journey const& j2) {
  return j1.departure_ >= j2.departure_ && j1.arrival_ <= j2.arrival_ &&
         j1.used_transports_ <= j2.used_transports_;
}

bmc_raptor::bmc_raptor(timetable_view const& tt_view,
                       bmc_raptor_state& state,
                       bitvec const& destination_mask,
                       vector_map<route_idx_t, std::uint32_t> const& route_events_from,
                       bitvec const& route_event_mask)
    : tt_view_(tt_view),
      state_(state),
      destination_mask_(destination_mask),
      route_event_mask_(route_event_mask),
      route_events_from_(route_events_from),
      tt_day_mask_(get_tt_day_mask(tt_view.get_source_tt())) {
  state_.resize(tt_view.get_n_locations(), tt_view.get_n_routes());
  };

bitset<kMaxDays> bmc_raptor::get_tt_day_mask(timetable const& tt) {
  auto const n_days_in_tt =
      tt.day_idx_mam(tt.internal_interval().to_ - 1_minutes).first -
      tt.day_idx_mam(tt.internal_interval().from_).first + 1;
  return bitset<kMaxDays>{std::string(to_idx(n_days_in_tt), '1')};
}

bool bmc_raptor::dominates_destination(bmc_raptor_label const& l1,
                                       bmc_raptor_label const& l2) {
  return l1.dominates_destination(l2);
}

bool bmc_raptor::dominates_non_destination(bmc_raptor_label const& l1,
                                           bmc_raptor_label const& l2) {
  return l1.dominates_non_destination(l2);
}

bool bmc_raptor::add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                           bmc_raptor_label const& label,
                                           search_bitfield const bf) {
  return bag.add<&dominates_non_destination, bmc_raptor_label::equals>(label, bf);
}

bool bmc_raptor::add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                       bmc_raptor_label const& label,
                                       search_bitfield const bf) {
  return bag.add<&dominates_destination, bmc_raptor_label::equals>(label, bf);
}

bool bmc_raptor::add_to_route_bag(bmc_raptor_route_bag_t& bag,
                                  bmc_raptor_route_label label,
                                  search_bitfield bf) {
  constexpr auto dom = [](bmc_raptor_route_label const& l1,
                          bmc_raptor_route_label const& l2) {
    return l1.dominates(l2);
  };
  return bag.add<dom, bmc_raptor_route_label::equals>(label, bf);
}

void bmc_raptor::init_location_with_offset(location_idx_view_t const location_idx_view,
                                           duration_t const minutes_to_arrive) {
  if (minutes_to_arrive > kMaxTravelTime) {
    return;
  }

  const auto& tt = tt_view_.get_source_tt();
  location_idx_t const source_location_idx = tt_view_.get_source_idx(location_idx_view);
  utl::verify(source_location_idx != location_idx_t::invalid(),
              "Unmapped location found");
  for (auto const r : tt.location_routes_[source_location_idx]) {
    auto const location_seq = tt.route_location_seq_.at(r);
    for (auto const [i, s] : utl::enumerate(location_seq)) {
      if (stop{s}.location_idx() != source_location_idx) {
        continue;
      }

      auto const& transport_range = tt.route_transport_ranges_[r];
      for (auto transport_idx = transport_range.from_;
           transport_idx != transport_range.to_; ++transport_idx) {

        auto const stop_time = tt.event_mam(
            transport_idx, static_cast<stop_idx_t>(i), event_type::kDep);

        auto const start_time = stop_time - delta{minutes_to_arrive};

        int16_t const start_time_day_offset = start_time.days();
        int16_t const start_time_mam = start_time.mam();

        int16_t extra_days = start_time_mam / 1440;
        int16_t normalized_start_time_mam = start_time_mam % 1440;

        if (normalized_start_time_mam < 0) {
          normalized_start_time_mam += 1440;
          --extra_days;
        }

        int total_day_offset = start_time_day_offset + extra_days;
        auto trip_tdb =
            tt.bitfields_[tt.transport_traffic_days_[transport_idx]];
        if (total_day_offset > 0) {
          trip_tdb <<= static_cast<size_t>(total_day_offset);
        } else {
          trip_tdb >>= static_cast<size_t>(-total_day_offset);
        }

        bitset<kMaxSearchDays> label_bf;
        for (auto block_i = 0U; block_i < bitset<kMaxSearchDays>::num_blocks;
             ++block_i) {
          label_bf.blocks_[block_i] = trip_tdb.blocks_[block_i];
        }

        const auto arrival_t = static_cast<uint16_t>(normalized_start_time_mam + minutes_to_arrive.count());

        if (label_bf.any()) {
          bool const added = add_to_non_dest_round_bag(
              state_.round_bags_[0U][to_idx(location_idx_view)],
              {
               .arrival_ = arrival_t,
               .arrival_with_transfer_ = arrival_t,
               .departure_ = static_cast<uint16_t>(normalized_start_time_mam),
              },
              label_bf);
          if (added) {
            state_.station_mark_.set(to_idx(location_idx_view), true);
          }
        }
      }
    }
  }
}

void bmc_raptor::init_starts(location_idx_view_t const location_idx,
                             bool const use_initial_fp) {
  location_idx_t source_location_idx = tt_view_.get_source_idx(location_idx);
  utl::verify(source_location_idx != location_idx_t::invalid(),
              "Unmapped location");
  init_location_with_offset(location_idx, 0_minutes);
  if (use_initial_fp) {
    auto const& fps_out =
        tt_view_.get_source_tt().locations_.footpaths_out_[kDefaultProfile][source_location_idx];
    for (auto const& fp : fps_out) {
      init_location_with_offset(tt_view_.get_view_idx(fp.target()), fp.duration());
    }
  }
}

void bmc_raptor::get_earliest_sufficient_transports(
    std::uint32_t,
    std::uint16_t departure,
    std::uint16_t arrival_with_transfer,
    search_bitfield const& label_tdb,
    route_idx_t route_idx,
    unsigned short stop_idx,
    bmc_raptor_route_bag_t& route_bag) {
  const auto& tt = tt_view_.get_source_tt();
  auto const dep_event_times =
      tt.event_times_at_stop(route_idx, stop_idx, event_type::kDep);

  constexpr auto n_days_to_iterate = kMaxTravelTime.count() / 1440 + 1U;

  delta const arr_as_delta(arrival_with_transfer);
  auto const arr_days_after_dep =
      static_cast<std::uint16_t>(arr_as_delta.days());

  auto const seek_first_day = [&]() {
    return linear_lb(
        dep_event_times.begin(), dep_event_times.end(), arr_as_delta.mam(),
        [&](delta const a, int16_t const b) { return a.mam() < b; });
  };

  search_bitfield to_serve_tdb = label_tdb;
  for (auto days_after_dep = arr_days_after_dep;
       days_after_dep < n_days_to_iterate; ++days_after_dep) {
    if (to_serve_tdb.none()) {
      return;
    }

    auto const time_range_to_scan =
        it_range{days_after_dep == arr_days_after_dep ? seek_first_day()
                                                      : dep_event_times.begin(),
                 dep_event_times.end()};

    if (time_range_to_scan.empty()) {
      continue;
    }

    auto const base = static_cast<unsigned>(&*time_range_to_scan.begin_ -
                                            dep_event_times.data());
    for (auto const [t_offset, event_time] :
         utl::enumerate(time_range_to_scan)) {
      if (to_serve_tdb.none()) {
        return;
      }

      auto const travel_time_lb =
          event_time.mam() + 1440 * days_after_dep - departure;
      if (travel_time_lb > kMaxTravelTime.count()) {
        return;
      }

      auto const event_day_offset = event_time.days();
      auto const transport =
          tt.route_transport_ranges_[route_idx][base + t_offset];
      int const net_shift_right = days_after_dep - event_day_offset;
      auto const& transport_tdb =
          tt.bitfields_[tt.transport_traffic_days_[transport]];
      auto const aligned_transport_tdb =
          (net_shift_right >= 0)
              ? (transport_tdb >> static_cast<size_t>(net_shift_right))
              : (transport_tdb << static_cast<size_t>(-net_shift_right));

      search_bitfield truncated_aligned_transport_tdb;
      truncate_to(aligned_transport_tdb, truncated_aligned_transport_tdb);

      if (truncated_aligned_transport_tdb.none()) {
        continue;
      }

      auto const matches = to_serve_tdb & truncated_aligned_transport_tdb;
      if (matches.any()) {
        add_to_route_bag(
            route_bag,
            {
                .transport_idx_ = transport.v_,
                .transport_day_offset_ = static_cast<int16_t>(net_shift_right),
                .departure_ = departure,
            },
            matches);
        to_serve_tdb &= ~matches;
      }
    }
  }
}

void bmc_raptor::update_footpaths(unsigned const k) {
  const auto& tt = tt_view_.get_source_tt();

  state_.station_mark_.for_each_set_bit([&](std::uint32_t const i) {
    location_idx_view_t const location_view_idx{i};
    location_idx_t const source_location_idx =
        tt_view_.get_source_idx(location_view_idx);

    const auto is_transitive = tt.is_location_transitive_.test(to_idx(source_location_idx));

    auto const& scan_bag = is_transitive ? state_.round_bags_[k][i] :
                                           state_.tmp_bags_[i];

    auto const fps = tt.locations_.footpaths_out_[kDefaultProfile][source_location_idx];
    for (auto const label : scan_bag) {
      auto const base_arr = label.label_.arrival_;
      auto const dep = label.label_.departure_;
      if (!is_transitive) {
        bool added = false;
        if (destination_mask_.test(to_idx(source_location_idx))) {
          added = add_to_dest_round_bag(
              state_.round_bags_[k][i], label.label_, label.tdb_);
        } else {
          added = add_to_non_dest_round_bag(
              state_.round_bags_[k][i], label.label_, label.tdb_);
        }
        if (added) {
          state_.station_mark_.set(i, true);
        }
      }

      for (auto const& fp : fps) {
        auto const target = fp.target();

        if (target == source_location_idx) {
          continue;
        }
        std::uint16_t const arr_with_foot =
            base_arr + static_cast<uint16_t>(fp.duration().count());

        if (arr_with_foot - dep > kMaxTravelTime.count()) {
          continue;
        }
        const bmc_raptor_label label_with_foot{
            .arrival_ = arr_with_foot,
            .arrival_with_transfer_ = arr_with_foot,
            .departure_ = dep,
        };
        auto tdb = label.tdb_;

        location_idx_view_t const target_view_idx =
            tt_view_.get_view_idx(target);
        utl::verify(target_view_idx != location_idx_view_t::invalid(),
                    "Unmapped location");

        bool const is_target_destination = destination_mask_[to_idx(target)];

        if (is_target_destination) {
          filter_by_dest_bag(state_.best_bags_[to_idx(target_view_idx)],
                                    label_with_foot, tdb);
        } else {
          filter_by_non_dest_bag(state_.best_bags_[to_idx(target_view_idx)],
                                        label_with_foot, tdb);
        }

        if (tdb.none()) {
          continue;
        }


        bool added = false;
        if (is_target_destination) {
          added = add_to_dest_round_bag(
              state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot,
              tdb);
        } else {
          added = add_to_non_dest_round_bag(
              state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot,
              tdb);
        }
        if (added) {
          state_.station_mark_.set(to_idx(target_view_idx), true);
        }
      }
    }
  });
}

bool bmc_raptor::update_route(unsigned const k, route_idx_t const route_idx) {
  const auto& tt = tt_view_.get_source_tt();
  bool any_marked = false;
  auto const& stop_sequence = tt.route_location_seq_[route_idx];

  const auto first_route_event_idx = route_events_from_[route_idx];

  bmc_raptor_route_bag_t route_bag{};
  for (stop_idx_t stop_idx = 0U; stop_idx != stop_sequence.size(); ++stop_idx) {
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const source_location_idx = stp.location_idx();
    auto const view_location_idx = tt_view_.get_view_idx(source_location_idx);

    if (stop_idx == 0 ||
        route_event_mask_.test(first_route_event_idx + (stop_idx * 2) - 1)) {
      auto const transfer_time_offset =
          tt.locations_.transfer_time_[location_idx_t{source_location_idx}]
              .count();
      auto const is_destination =
          destination_mask_[to_idx(source_location_idx)];
      for (auto const& active_label : route_bag) {
        if (active_label.label_.transport_idx_ ==
            to_idx(transport_idx_t::invalid())) {
          continue;
        }
        auto const new_arr_delta = tt.event_mam(
            route_idx, transport_idx_t{active_label.label_.transport_idx_},
            stop_idx, event_type::kArr);

        auto const new_arr_day =
            new_arr_delta.days() + active_label.label_.transport_day_offset_;
        auto const new_arr_mam = new_arr_delta.mam();

        int32_t const new_arr = new_arr_day * 1440 + new_arr_mam;

        if (!stp.out_allowed() ||
            new_arr - active_label.label_.departure_ > kMaxTravelTime.count()) {
          continue;
        }

        auto const candidate_lbl = bmc_raptor_label{
            .arrival_ = static_cast<uint16_t>(new_arr),
            .arrival_with_transfer_ =
                static_cast<uint16_t>(new_arr + transfer_time_offset),
            .departure_ = static_cast<uint16_t>(active_label.label_.departure_)
        };

        auto candidate_tdb = active_label.tdb_;
        const auto& best_bag = state_.best_bags_[to_idx(view_location_idx)];
        const auto is_transitive = tt.is_location_transitive_.test(to_idx(source_location_idx));
        if (is_transitive) {
          if (is_destination) {
            filter_by_dest_bag(best_bag, candidate_lbl, candidate_tdb);
          } else {
            filter_by_non_dest_bag(best_bag, candidate_lbl, candidate_tdb);
          }
        } else {
          // Todo
        }

        if (candidate_tdb.none()) {
          continue;
        }

        bool added = false;
        auto& insert_in_bag = is_transitive ? state_.round_bags_[k][to_idx(view_location_idx)] : state_.tmp_bags_[to_idx(view_location_idx)];
        if (is_destination) {
          added = add_to_dest_round_bag(
              insert_in_bag, candidate_lbl,
              candidate_tdb);
        } else {
          added = add_to_non_dest_round_bag(
              insert_in_bag, candidate_lbl,
              candidate_tdb);
        }
        if (added) {
          state_.station_mark_.set(to_idx(view_location_idx), true);
          any_marked = true;
        }
      }
    }

    if (stop_idx == stop_sequence.size() - 1) {
      return any_marked;
    }

    if (stp.in_allowed() && state_.prev_station_mark_[to_idx(view_location_idx)] && route_event_mask_.test(first_route_event_idx + (stop_idx * 2))) {
      const auto& scan_bag = state_.round_bags_[k - 1][to_idx(view_location_idx)];
      for (const auto [i, l] : utl::enumerate(scan_bag)) {
        get_earliest_sufficient_transports(
            static_cast<std::uint32_t>(i), l.label_.departure_,
            l.label_.arrival_with_transfer_, l.tdb_, route_idx, stop_idx,
            route_bag);
      }
    }
  }
  return any_marked;
}

void bmc_raptor::rounds() {
  const auto& tt = tt_view_.get_source_tt();
  for (auto k = 1U; k != end_k(); ++k) {
    // Round k
    auto any_marked = false;
    for (auto location_view_idx = location_idx_view_t{0U}; location_view_idx != tt_view_.get_n_locations();
         ++location_view_idx) {

      location_idx_t const source_location_idx = tt_view_.get_source_idx(location_view_idx);

      bool is_destination = destination_mask_[to_idx(source_location_idx)];
      if (state_.station_mark_[to_idx(location_view_idx)]) {
        if (k > 1) {
          for (auto const& l : state_.round_bags_[k - 1][to_idx(location_view_idx)]) {
            auto& best_bag = state_.best_bags_[to_idx(location_view_idx)];
            if (is_destination) {
              add_to_dest_round_bag(best_bag, l.label_, l.tdb_);
            } else {
              add_to_non_dest_round_bag(best_bag, l.label_, l.tdb_);
            }
          }
        }
        any_marked = true;
        for (auto const r : tt.location_routes_[source_location_idx]) {
          route_idx_view_t const view_route_idx = tt_view_.get_view_idx(r);
          if (view_route_idx == route_idx_view_t::invalid()) {
            continue;
          }
          state_.route_mark_.set(to_idx(view_route_idx), true);
        }
      }
    }

    std::swap(state_.prev_station_mark_, state_.station_mark_);
    state_.station_mark_.zero_out();

    if (!any_marked) {
      return;
    }

    any_marked = false;
    for (auto route_view_idx = route_idx_view_t{0U}; route_view_idx != tt_view_.get_n_routes(); ++route_view_idx) {
      if (!state_.route_mark_[to_idx(route_view_idx)]) {
        continue;
      }

      any_marked |= update_route(k, tt_view_.get_source_idx(route_view_idx));
    }

    state_.route_mark_.zero_out();
    if (!any_marked) {
      return;
    }
    update_footpaths(k);
  }
}

void bmc_raptor::gather_journeys(std::vector<pareto_set<journey>>& buffer) {
  buffer.resize(destination_mask_.count());
  const auto& tt = tt_view_.get_source_tt();
  auto write_idx = 0U;
  destination_mask_.for_each_set_bit([&](uint32_t source_location_idx) {
    location_idx_view_t const location_view_idx =
        tt_view_.get_view_idx(location_idx_t{source_location_idx});

    if (location_view_idx == location_idx_view_t::invalid()) {
      ++write_idx;
      return;
    }

    for (auto k = 0U; k != end_k(); ++k) {
      auto const& round_bag = state_.round_bags_[k][to_idx(location_view_idx)];
      if (round_bag.size() == 0) {
        continue;
      }

      constexpr auto fastest_direct = duration_t::max();
      for (auto label_it = round_bag.begin(); label_it != round_bag.end();
           ++label_it) {

        auto const& label = label_it->label_;
        auto const& lbf = label_it->tdb_;

        auto const departure = label.departure_;
        auto const arrival = label.departure_;


        if (duration_t const travel_time{arrival - departure};
            travel_time >= fastest_direct) {
          continue;
        }

        lbf.for_each_set_bit([&](uint16_t tt_day_offset) {
          auto const day_offset = day_idx_t{tt_day_offset};
          routing_time const dep{
              day_offset,
              duration_t{static_cast<std::int16_t>(departure)}};

          auto const arrival_day_offset =
              day_idx_t{arrival / 1440} + day_offset;
          auto const arrival_mam =
              static_cast<std::int16_t>(arrival % 1440);

          routing_time const arr{arrival_day_offset, duration_t{arrival_mam}};

          buffer[write_idx].add(
              journey{.legs_ = {},
                      .start_time_ = dep.to_unixtime(tt),
                      .dest_time_ = arr.to_unixtime(tt),
                      .dest_ = location_idx_t{source_location_idx},
                      .transfers_ = static_cast<std::uint8_t>(k - 1)});
        });
      }
    }
    ++write_idx;
  });
}

unsigned bmc_raptor::end_k() { return kMaxTransfers + 1U; }

void bmc_raptor::emplace_relative_journeys_for(location_idx_view_t const loc_idx,
                                               std::vector<bmc_journey>& bag) const {
  for (auto k = 1U; k != end_k(); ++k) {
    auto const& round_bag = state_.round_bags_[k][to_idx(loc_idx)];
    if (round_bag.size() == 0) {
      continue;
    }

    for (auto label_it = round_bag.begin(); label_it != round_bag.end();
         ++label_it) {
      auto const label_view = *label_it;
      auto const& tdb = label_view.tdb_;
      tdb.for_each_set_bit([&](size_t const i) {
        pareto_utils<bmc_journey>::pareto_add(
            bag,
            {
             .arrival_ = routing_time{static_cast<int>(i * 1440 + label_view.label_.arrival_)},
             .departure_ = routing_time{static_cast<int>(i * 1440 + label_view.label_.departure_)},
             .used_transports_ = static_cast<std::uint16_t>(k),
             .target_location_idx_ = loc_idx
            },
            bmc_journey::dominates);
      });
    }
  }
}

}  // namespace nigiri::routing::para
