#include "nigiri/routing/raptor/para/mc_raptor.h"

#include "utl/equal_ranges_linear.h"
#include "utl/erase_if.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"

#include "nigiri/common/linear_lower_bound.h"
#include "nigiri/for_each_meta.h"
#include "nigiri/logging.h"
#include "nigiri/routing/raptor/para/bmc_raptor_state.h"
#include "nigiri/routing/raptor/para/mc_raptor_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/stop.h"
#include "nigiri/timetable.h"

namespace nigiri::routing::para {

bool mc_journey::dominates(mc_journey const& j1, mc_journey const& j2) {
  return j1.departure_ >= j2.departure_ && j1.arrival_ <= j2.arrival_ &&
         j1.transfers_ <= j2.transfers_;
}

bool mc_raptor::is_better(auto a, auto b) { return a < b; }

bool mc_raptor::is_better_or_eq(auto a, auto b) { return a <= b; }

bool mc_raptor::dominates_destination(mc_raptor_label const& l1,
                                      mc_raptor_label const& l2) {
  return l1.dominates_destination(l2);
}

bool mc_raptor::dominates_non_destination(mc_raptor_label const& l1,
                                          mc_raptor_label const& l2) {
  return l1.dominates_non_destination(l2);
}

bool mc_raptor::dominates_destination_skip_fps(mc_raptor_label const& l1,
                                               mc_raptor_label const& l2) {
  return !l1.is_footpath_ && l1.dominates_destination(l2);
}

bool mc_raptor::dominates_non_destination_skip_fps(mc_raptor_label const& l1,
                                                   mc_raptor_label const& l2) {
  return !l1.is_footpath_ && l1.dominates_non_destination(l2);
}

bool mc_raptor::add_to_non_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                          mc_raptor_label const& label) {
  return std::get<0>(bag.add<&dominates_non_destination>(label));
}

bool mc_raptor::add_to_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                      mc_raptor_label const& label) {
  return std::get<0>(bag.add<&dominates_destination>(label));
}

bool mc_raptor::add_careful_to_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                              mc_raptor_label const& label) {
  return std::get<0>(bag.add_careful<&dominates_destination>(label));
}

bool mc_raptor::add_careful_to_non_dest_round_bag(pareto_set<mc_raptor_label>& bag,
                                                  mc_raptor_label const& label) {
  return std::get<0>(bag.add_careful<&dominates_non_destination>(label));
}

mc_raptor::mc_raptor(timetable_view const& tt_view,
                     mc_raptor_state& state,
                     bitvec const& destination_mask,
                     vector_map<route_idx_t, std::uint32_t> const& route_events_from,
                     bitvec const& route_event_mask)
    : tt_view_{tt_view},
      state_{state},
      n_tt_days_{tt_view.get_source_tt().internal_interval_days().size().count()},
      n_locations_{tt_view_.get_n_locations()},
      n_routes_{tt_view.get_n_routes()},
      destination_mask_(destination_mask),
      route_events_from_(route_events_from),
      route_event_mask_(route_event_mask){
  state_.resize(n_locations_,
                n_routes_,
                static_cast<unsigned int>(destination_mask_.count()));
  }

mc_raptor_stats const& mc_raptor::get_stats() const { return stats_; }

unsigned mc_raptor::end_k() { return kMaxTransfers + 1U; }

void mc_raptor::route() {
  rounds();
}

void mc_raptor::rounds() {
  const auto& tt = tt_view_.get_source_tt();
  for (auto k = 1U; k != end_k(); ++k) {
    // Round k
    auto any_marked = false;
    for (auto location_view_idx = location_idx_view_t{0U};
         location_view_idx != n_locations_; ++location_view_idx) {

      location_idx_t const source_location_idx = tt_view_.get_source_idx(location_view_idx);

      bool is_destination = destination_mask_[to_idx(source_location_idx)];
      if (state_.station_mark_[to_idx(location_view_idx)]) {
        if (k > 1) {
          for (auto const& l : state_.round_bags_[k - 1][to_idx(location_view_idx)]) {
            auto& best_bag = state_.best_[to_idx(location_view_idx)];
            if (is_destination) {
              add_to_dest_round_bag(best_bag, l);
            } else {
              add_to_non_dest_round_bag(best_bag, l);
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

bool mc_raptor::update_route(unsigned const k, route_idx_t const route) {
  const auto& tt = tt_view_.get_source_tt();
  auto any_marked = false;
  auto const stop_sequence = tt.route_location_seq_[route];

  const auto first_route_event_idx = route_events_from_[route];

  pareto_set<mc_raptor_route_label> route_bag{};
  for (stop_idx_t stop_idx = 0U; stop_idx != stop_sequence.size(); ++stop_idx) {
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const source_location_idx = stp.location_idx();
    auto const view_location_idx = tt_view_.get_view_idx(source_location_idx);

    if (stop_idx == 0 ||
        route_event_mask_.test(first_route_event_idx + (stop_idx * 2) - 1)) {
      auto const transfer_time_offset =
          tt.locations_.transfer_time_[location_idx_t{source_location_idx}];
      auto const is_destination =
          destination_mask_[to_idx(source_location_idx)];
      for (auto const& active_label : route_bag) {
        if (active_label.transport_.t_idx_ == transport_idx_t::invalid()) {
          continue;
        }
        auto const& trip = active_label.transport_;

        routing_time const new_arr(
            trip.day_,
            tt.event_mam(route, trip.t_idx_, stop_idx, event_type::kArr)
                .as_duration());

        if (!stp.out_allowed() ||
            (new_arr -
             active_label.departure_) > kMaxTravelTime) {
          continue;
        }

        auto const candidate_lbl = mc_raptor_label {
          .arrival_ = new_arr,
          .arrival_with_transfer_ = new_arr + transfer_time_offset,
          .departure_ = active_label.departure_,
          .route_idx_ = to_idx(route),
          .enter_stop_idx_ = active_label.enter_stop_idx_,
          .exit_stop_idx_ = stop_idx,
          .parent_bag_idx_ = active_label.parent_bag_idx_,
          .is_footpath_ = false,
          .has_parent_ = true
        };

        bool dominated = false;
        const auto& best_bag = state_.best_[to_idx(view_location_idx)];
        if (tt.is_location_transitive_.test(to_idx(source_location_idx))) {
          if (is_destination) {
            dominated = is_dominated_by_dest_bag<false>(best_bag, candidate_lbl);
          } else {
            dominated = is_dominated_by_non_dest_bag<false>(best_bag, candidate_lbl);
          }
        } else {
          if (is_destination) {
            dominated = is_dominated_by_dest_bag<true>(best_bag, candidate_lbl);
          } else {
            dominated = is_dominated_by_non_dest_bag<true>(best_bag, candidate_lbl);
          }
        }

        if (dominated) {
          continue;
        }


        bool added = false;
        if (is_destination) {
          added = add_to_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], candidate_lbl);
        } else {
          added = add_to_non_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], candidate_lbl);
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
      const auto& scan_bag = state_.round_bags_[k - 1][cista::to_idx(view_location_idx)];
      for (const auto [i, l] : utl::enumerate(scan_bag)) {
        if (auto const new_et = get_earliest_transport(l, route, stop_idx);
            new_et.is_valid()) {
          route_bag.add(mc_raptor_route_label{
            .transport_ = new_et,
            .departure_ = l.departure_,
            .parent_bag_idx_ = static_cast<std::uint32_t>(i),
            .enter_stop_idx_ = stop_idx,
          });
        }
      }
    }
  }
  return any_marked;
}

void mc_raptor::update_footpaths(unsigned const k) const {
  const auto& tt = tt_view_.get_source_tt();
  std::vector buffered_labels{n_locations_, std::vector<mc_raptor_label>()};

  state_.station_mark_.for_each_set_bit([&](std::uint32_t const i) {
    auto const& round_bag = state_.round_bags_[k][i];
    location_idx_view_t const location_view_idx{i};
    location_idx_t const source_location_idx =
        tt_view_.get_source_idx(location_view_idx);

    auto const fps = tt.locations_.footpaths_out_[kDefaultProfile][source_location_idx];
    for (const auto& rl : round_bag) {
      if (rl.is_footpath_) {
        break;
      }

      auto const base_arr = rl.arrival_;
      auto const dep = rl.departure_;
      for (auto const& fp : fps) {
        auto const target = fp.target();

        if (target == source_location_idx) {
          continue;
        }

        routing_time const arr_with_foot = base_arr + fp.duration();

        if (arr_with_foot - dep > kMaxTravelTime) {
          continue;
        }

        const mc_raptor_label label_with_foot {
          .arrival_ = arr_with_foot,
          .arrival_with_transfer_ = arr_with_foot,
          .departure_ = dep,
          .route_idx_ = rl.route_idx_,
          .enter_stop_idx_ = rl.enter_stop_idx_,
          .exit_stop_idx_ = rl.exit_stop_idx_,
          .parent_bag_idx_ = rl.parent_bag_idx_,
          .is_footpath_ = true,
          .has_parent_ = rl.has_parent_,
        };

        location_idx_view_t const target_view_idx =
            tt_view_.get_view_idx(target);
        utl::verify(target_view_idx != location_idx_view_t::invalid(),
                    "Unmapped location");

        bool const is_target_destination = destination_mask_[to_idx(target)];

        bool dominated = false;
        if (is_target_destination) {
          dominated = is_dominated_by_dest_bag<false>(
              state_.best_[to_idx(target_view_idx)], label_with_foot);
        } else {
          dominated = is_dominated_by_non_dest_bag<false>(
              state_.best_[to_idx(target_view_idx)], label_with_foot);
        }
        if (dominated) {
          continue;
        }

        if (to_idx(target_view_idx) < i || tt.is_location_transitive_[to_idx(target)]) {
          bool added = false;
          if (is_target_destination) {
            added = add_to_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot);
          } else {
            added = add_to_non_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot);
          }
          if (added) {
            state_.station_mark_.set(to_idx(target_view_idx), true);
          }
        } else {
          bool added = false;
          if (is_target_destination) {
            added = add_careful_to_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot);
          } else {
            added = add_careful_to_non_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot);
          }
          if (added) {
            state_.fp_label_added_.set(to_idx(target_view_idx), true);
            state_.station_mark_.set(to_idx(target_view_idx), true);
          }
        }
      }
    }
  });
}

transport mc_raptor::get_earliest_transport(const mc_raptor_label& current,
                                            route_idx_t const r,
                                            stop_idx_t const stop_idx) {
  const auto& tt = tt_view_.get_source_tt();
  auto const time = current.arrival_with_transfer_;

  if (time == kInvalidTime<direction::kForward>) {
    return {transport_idx_t::invalid(), day_idx_t::invalid()};
  }

  auto const [day_at_stop, mam_at_stop] = time.day_idx_mam();

  auto const n_days_to_iterate =
      std::min(kMaxTravelTime.count() / 1440 + 1,
               n_tt_days_ - static_cast<int>(day_at_stop.v_) + 1);

  auto const event_times =
      tt.event_times_at_stop(r, stop_idx, event_type::kDep);

  auto const seek_first_day = [&]() {
    return linear_lb(event_times.begin(), event_times.end(),
                     mam_at_stop,
                     [&](delta const a, minutes_after_midnight_t const b) {
                       return is_better(a.mam(), b.count());
                     });
  };

  for (auto i = day_idx_t::value_t{0U}; i != n_days_to_iterate; ++i) {
    auto const day = day_at_stop + i;
    auto const ev_time_range = it_range{
        i == 0U ? seek_first_day() : event_times.begin(), event_times.end()};
    if (ev_time_range.empty()) {
      continue;
    }
    auto const base =
        static_cast<unsigned>(&*ev_time_range.begin_ - event_times.data());
    for (auto const [t_offset, ev] : utl::enumerate(ev_time_range)) {
      auto const ev_mam = minutes_after_midnight_t{
          ev.count() < 1440 ? ev.count() : ev.count() % 1440};

      auto const t = tt.route_transport_ranges_[r][base + t_offset];
      if (day == day_at_stop && !is_better_or_eq(mam_at_stop, ev_mam)) {
        continue;
      }

      auto const ev_day_offset = static_cast<day_idx_t::value_t>(
          ev.count() < 1440
              ? 0
              : static_cast<cista::base_t<day_idx_t>>(ev.count() / 1440));
      if (!tt.bitfields_[tt.transport_traffic_days_[t]].test(
              static_cast<std::size_t>(to_idx(day) - ev_day_offset))) {
        continue;
      }
      return {t, (day - ev_day_offset)};
    }
  }
  return {transport_idx_t::invalid(), day_idx_t::invalid()};
}

void mc_raptor::emplace_relative_journeys_for(location_idx_view_t const loc_idx,
                                              std::vector<mc_journey>& bag) const {
  constexpr auto dom = [](mc_journey const& l1, mc_journey const& l2) {
    return mc_journey::dominates(l1, l2);
  };

  for (auto k = 0U; k != end_k(); ++k) {
    auto const& round_bag = state_.round_bags_[k][to_idx(loc_idx)];
    if (round_bag.size() == 0) {
      continue;
    }

    for (auto label_it = round_bag.begin(); label_it != round_bag.end();
       ++label_it) {
      const auto& label = *label_it;
      pareto_utils<mc_journey>::pareto_add(
          bag,
          {.arrival_ = label.arrival_,
           .departure_ = label.departure_,
           .transfers_ = static_cast<std::uint16_t>(k > 0 ? k - 1 : 0U),
           .label_iter_ = label_it},
          dom);
    }
  }
}

// void mc_raptor::reconstruct() const {
//   size_t w_idx = 0U;
//   for (auto location_idx = 0U; location_idx != n_locations_; ++location_idx) {
//     if (!destination_mask_[location_idx]) {
//       continue;
//     }
//
//     // const auto fastest_direct =
//     // get_fastest_direct(location_idx_t{location_idx});
//     constexpr auto fastest_direct = duration_t::max();
//     for (auto k = 1U; k != end_k(); ++k) {
//       auto const& round_bag = state_.round_bags_[k][location_idx];
//       if (round_bag.size() == 0) {
//         continue;
//       }
//       for (auto journey_it = round_bag.begin(); journey_it != round_bag.end();
//            ++journey_it) {
//
//         if (duration_t const travel_time{
//                 std::abs((get<0>(journey_it->arrival_) - journey_it->departure_)
//                              .count())};
//             travel_time >= fastest_direct) {
//           continue;
//         }
//
//
//         auto const [optimal, it, _] = state_.results_[w_idx].add(
//             journey{.legs_ = {},
//                     .start_time_ = journey_it->departure_.to_unixtime(tt_),
//                     .dest_time_ = get<0>(journey_it->arrival_).to_unixtime(tt_),
//                     .dest_ = location_idx_t{location_idx},
//                     .transfers_ = static_cast<std::uint8_t>(k - 1)});
//         if (!optimal) {
//           continue;
//         }
//
//         auto current_label = journey_it;
//
//         for (auto i = k; i > 0; i--) {
//
//           const auto& curr_opt_transfer = current_label->transfer_;
//           const auto& curr_opt_transport = current_label->with_;
//
//           if (!curr_opt_transfer.has_value() ||
//               !curr_opt_transport.has_value()) {
//             throw utl::fail(
//                 "No Transfer or Transport leg given for the current label!\n");
//           }
//
//           const auto target_time = get<0>(current_label->arrival_);
//
//           const auto& curr_transfer_leg = *curr_opt_transfer;
//           const auto& curr_transport_leg = *curr_opt_transport;
//
//           const auto& prev_label = current_label->prev_;
//
//           if (curr_transport_leg.exit_ == curr_transfer_leg.target_ && i == k) {
//
//             it->add(journey::leg{direction::kForward, curr_transfer_leg.target_,
//                 curr_transfer_leg.target_,
//                 target_time.to_unixtime(tt_), target_time.to_unixtime(tt_),
//                  footpath{curr_transfer_leg.target_, 0_minutes}});
//
//           } else {
//
//             if (kVerifyReconstruction) {
//               const auto& fps = tt_.locations_.footpaths_out_[kDefaultProfile][curr_transport_leg.exit_];
//               bool hit = false;
//               for (const auto& fp : fps) {
//                 if (fp.target() == curr_transfer_leg.target_ && fp.duration() == curr_transfer_leg.duration_) {
//                   hit = true;
//                   break;
//                 }
//               }
//               if (curr_transport_leg.exit_ == curr_transfer_leg.target_ &&
//                   tt_.locations_.transfer_time_[curr_transfer_leg.target_] == curr_transfer_leg.duration_) {
//                 hit = true;
//               }
//               if (!hit) {
//                 nigiri::log(log_lvl::error,
//                             "mc_raptor.reconstruction",
//                             "Footpath {} -- {} --> {}, used in routing does not exist!",
//                             loc{tt_, curr_transport_leg.exit_},
//                             curr_transfer_leg.duration_,
//                             loc{tt_, curr_transfer_leg.target_});
//               }
//             }
//
//             it->add(journey::leg{direction::kForward, curr_transport_leg.exit_,
//                                  curr_transfer_leg.target_,
//                                  (target_time +
//                                     (curr_transport_leg.exit_ == curr_transfer_leg.target_ ? 0 : -1)
//                                     * curr_transfer_leg.duration_).to_unixtime(tt_),
//                                  (target_time +
//                                   (curr_transport_leg.exit_ == curr_transfer_leg.target_ ? 1 : 0)
//                                       * curr_transfer_leg.duration_).to_unixtime(tt_),
//                                  footpath{curr_transfer_leg.target_,
//                   curr_transfer_leg.duration_}});
//           }
//
//           rt::run r;
//           r.t_ = current_label->with_.value().via_;
//           r.stop_range_ = interval<stop_idx_t>{0, 0};
//           auto const [from_, to_] = find_enter_exit(
//               curr_transport_leg.via_, curr_transport_leg.enter_,
//               get<1>(prev_label->arrival_), curr_transport_leg.exit_);
//
//           if (kVerifyReconstruction) {
//             if (auto const& stop_sequence =
//                     tt_.route_location_seq_[tt_.transport_route_[curr_transport_leg.via_.t_idx_]];
//                 from_ >= to_ || from_ == stop_sequence.size() || to_ == stop_sequence.size()) {
//               log(log_lvl::error,
//                   "mc_raptor.reconstruction",
//                   "Invalid transport ride used while routing. Not possible to enter transport {} at {} and exit at {}",
//                   curr_transport_leg.via_,
//                   loc{tt_, curr_transport_leg.enter_},
//                   loc{tt_, curr_transport_leg.exit_});
//             }
//           }
//
//           const auto trans_dep_time =
//               tt_.event_time(r.t_, from_, event_type::kDep);
//
//           const auto trans_arr_time =
//               tt_.event_time(r.t_, to_, event_type::kArr);
//
//           if (kVerifyReconstruction) {
//             if (get<1>(prev_label->arrival_).to_unixtime(tt_) > trans_dep_time) {
//               nigiri::log(log_lvl::error,
//                           "mc_raptor.reconstruction",
//                           "Not possible to enter transport {} (Route {}) at location {}, when arriving not earlier than {}",
//                           curr_transport_leg.via_,
//                           tt_.transport_route_[curr_transport_leg.via_.t_idx_],
//                           curr_transport_leg.enter_,
//                           get<1>(prev_label->arrival_).to_unixtime(tt_));
//             }
//           }
//
//           it->add(journey::leg{
//               direction::kForward, curr_transport_leg.enter_,
//               curr_transport_leg.exit_, trans_dep_time,
//               trans_arr_time,
//               journey::run_enter_exit{r, from_, to_}});
//
//           if (i == 1) {
//             break;
//           }
//           current_label = prev_label;
//         }
//         //const auto target_after_init_transfer =
//         //   k == 0 ? location_idx_t{location_idx} : current_label->with_->enter_;
//
//         if (k > 0) {
//           current_label = current_label->prev_;
//         }
//         /*
//         if (!is_journey_start(target_after_init_transfer)) {
//           auto init_fp = find_start_footpath(target_after_init_transfer,
//                                              get<1>(current_label->arrival_),
//                                              current_label->departure_);
//           if (!init_fp.has_value()) {
//             throw utl::fail("No initial footpath found!\n");
//           }
//
//           it->add(std::move(*init_fp));
//         }
//         */
//         std::ranges::reverse(it->legs_);
//       }
//     }
//     w_idx++;
//   }
// }
//
// interval<stop_idx_t> mc_raptor::find_enter_exit(transport const via,
//                                                 location_idx_t const enter,
//                                                 routing_time const enter_after,
//                                                 location_idx_t const exit) const {
//
//   auto const stop_sequence = tt_.route_location_seq_[tt_.transport_route_[via.t_idx_]];
//   interval enter_exit{
//       static_cast<unsigned short>(stop_sequence.size()),
//       static_cast<unsigned short>(stop_sequence.size())};
//
//   bool entered = false;
//   for (auto i = 0U; i != stop_sequence.size(); ++i) {
//     auto const stop_idx = static_cast<stop_idx_t>(i);
//     auto const stp = stop{stop_sequence[stop_idx]};
//
//     if (!entered &&
//         stp.location_idx() == enter && stp.in_allowed() &&
//         enter_after.to_unixtime(tt_) <= tt_.event_time(via, stop_idx, event_type::kDep)) {
//       enter_exit.from_ = stop_idx;
//       entered = true;
//     }
//
//     if (stp.location_idx() == exit && entered && stp.out_allowed()) {
//       enter_exit.to_ = stop_idx;
//       break;
//     }
//   }
//   return enter_exit;
// }

/*

bool mc_raptor::is_journey_start(location_idx_t const l) const {
  return utl::any_of(start_offsets_, [&](offset const& o) {
    return matches(tt_, start_match_mode_, o.target(), l);
  });
}

std::optional<journey::leg> mc_raptor::find_start_footpath(
    location_idx_t const leg_start_location,
    routing_time const leg_start_time,
    routing_time const journey_start_time) const {

  auto const start_matches = [&](routing_time const a, routing_time const b) {
    return a == b;
  };

  if (is_journey_start(leg_start_location) &&
      is_better_or_eq(journey_start_time, leg_start_time)) {
    return std::nullopt;
  }

  for (auto const& footpaths =
           tt_.locations_.footpaths_in_[kDefaultProfile][leg_start_location];
       auto const& fp : footpaths) {
    if (is_journey_start(fp.target()) &&
        leg_start_time != routing_time::max() &&
        start_matches(journey_start_time + fp.duration(), leg_start_time)) {
      return journey::leg{direction::kForward,
                          fp.target(),
                          leg_start_location,
                          journey_start_time.to_unixtime(tt_),
                          leg_start_time.to_unixtime(tt_),
                          fp};
    }
  }

  return std::nullopt;
}

duration_t mc_raptor::get_fastest_start_dest_overlap(location_idx_t const dest) const {
  auto min = duration_t{std::numeric_limits<duration_t::rep>::max()};
  for (auto const& s : start_offsets_) {
    for_each_meta(tt_, start_match_mode_, s.target_,
                  [&](location_idx_t const start) {
                      if (start == dest) {
                        min = std::min(min, s.duration_);
                      }
                  });
  }
  return min;
}

duration_t mc_raptor::get_fastest_direct_with_foot(location_idx_t const dest) const {
  auto min = duration_t{std::numeric_limits<duration_t::rep>::max()};
  for (auto const& start : start_offsets_) {
    for (auto const& footpaths = tt_.locations_.footpaths_out_[kDefaultProfile];
         auto const& fp : footpaths[start.target_]) {
        if (dest == fp.target()) {
          min = std::min(min, start.duration_ + fp.duration());
        }
    }
  }
  return min;
}

duration_t mc_raptor::get_fastest_direct(location_idx_t const dest) const {
  return std::min(get_fastest_direct_with_foot(dest),
                  get_fastest_start_dest_overlap(dest));
}

*/


} // namespace nigiri::routing::para