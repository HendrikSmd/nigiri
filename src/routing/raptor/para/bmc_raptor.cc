#include "nigiri/routing/raptor/para/bmc_raptor.h"

#include "boost/iostreams/seek.hpp"

#include "nigiri/common/linear_lower_bound.h"
#include "nigiri/routing/raptor/para/routing_time.h"
#include "nigiri/stop.h"
#include "nigiri/types.h"

#include "utl/enumerate.h"

namespace nigiri::routing::para {

bool bmc_journey::dominates(bmc_journey const& j1,
                            bmc_journey const& j2) {
  return j1.departure_ >= j2.departure_ && j1.arrival_ <= j2.arrival_ && j1.transfers_ <= j2.transfers_;
}

bmc_raptor::bmc_raptor(timetable const& tt,
                       bmc_raptor_state& state,
                       bitvec const& destination_mask,
                       bitvec const& route_mask,
                       bitvec const& transfer_mask)
  : tt_(tt),
    state_(state),
    destination_mask_(destination_mask),
    route_mask_(route_mask),
    transfer_mask_(transfer_mask),
    tt_day_mask_(get_tt_day_mask(tt)) {
    state_.resize(tt.n_locations(),
                  tt.n_routes(),
                  static_cast<unsigned int>(destination_mask_.count()));
    };

bitset<kMaxDays> bmc_raptor::get_tt_day_mask(timetable const& tt) {
  const auto n_days_in_tt = tt.day_idx_mam(tt.internal_interval().to_ - 1_minutes).first
                          - tt.day_idx_mam(tt.internal_interval().from_).first
                          + 1;
  return bitset<kMaxDays>{std::string(to_idx(n_days_in_tt), '1')};
}

bool bmc_raptor::add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                           bmc_raptor_label label,
                                           search_bitfield bf) {
  constexpr auto dom = [](bmc_raptor_label const& l1, bmc_raptor_label const& l2) {
    return l1.dominates_non_destination(l2);
  };
  return bag.add<dom>(label, bf);
}

bool bmc_raptor::add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                       bmc_raptor_label const label,
                                       search_bitfield const bf) {
  constexpr auto dom = [](bmc_raptor_label const& l1, bmc_raptor_label const& l2) {
    return l1.dominates_destination(l2);
  };
  return bag.add<dom>(label, bf);
}

void bmc_raptor::filter_by_non_dest_bag(bmc_raptor_bag_t const& bag, bmc_raptor_label const& label, search_bitfield& sbf) {
  constexpr auto dom = [](bmc_raptor_label const& l1, bmc_raptor_label const& l2) {
    return l1.dominates_non_destination(l2);
  };
  bag.filter_dominated<dom>(label, sbf);
}

void bmc_raptor::filter_by_dest_bag(bmc_raptor_bag_t const& bag, bmc_raptor_label const& label, search_bitfield& sbf) {
  constexpr auto dom = [](bmc_raptor_label const& l1, bmc_raptor_label const& l2) {
    return l1.dominates_destination(l2);
  };
  bag.filter_dominated<dom>(label, sbf);
}

bool bmc_raptor::add_to_route_bag(bmc_raptor_route_bag_t& bag,
                                  bmc_raptor_route_label label,
                                  search_bitfield bf) {
  constexpr auto dom = [](bmc_raptor_route_label const& l1, bmc_raptor_route_label const& l2) {
    return l1.dominates(l2);
  };
  return bag.add<dom>(label, bf);
}

void bmc_raptor::init_location_with_offset(location_idx_t const location_idx,
                                           duration_t const minutes_to_arrive) {
  if (minutes_to_arrive > kMaxTravelTime) {
    return;
  }

  for (auto const& r : tt_.location_routes_.at(location_idx)) {
    if (!route_mask_[to_idx(r)]) {
      continue;
    }

    auto const location_seq = tt_.route_location_seq_.at(r);
    for (auto const [i, s] : utl::enumerate(location_seq)) {
      if (stop{s}.location_idx() != location_idx) {
        continue;
      }

      auto const& transport_range = tt_.route_transport_ranges_[r];
      for (auto transport_idx = transport_range.from_;
           transport_idx != transport_range.to_; ++transport_idx) {

        auto const stop_time =
            tt_.event_mam(transport_idx,
                         static_cast<stop_idx_t>(i),
                         event_type::kDep);

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
        auto trip_tdb = tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]];
        if (total_day_offset > 0) {
          trip_tdb <<= static_cast<size_t>(total_day_offset);
        } else {
          trip_tdb >>= static_cast<size_t>(-total_day_offset);
        }

        bitset<kMaxSearchDays> label_bf;
        for (auto block_i = 0U; block_i < bitset<kMaxSearchDays>::num_blocks; ++block_i) {
          label_bf.blocks_[block_i] = trip_tdb.blocks_[block_i];
        }

        if (label_bf.any()) {
          bool added = add_to_non_dest_round_bag(
            state_.round_bags_[0U][to_idx(location_idx)],
            {
              .route_idx_ = 0U,
              .enter_stop_idx_ = 0U,
              .exit_stop_idx_ = 0U,
              .arrival_ = static_cast<uint16_t>(normalized_start_time_mam + minutes_to_arrive.count()),
              .parent_bag_idx_ = 0U,
              .arrival_with_transfer_ = static_cast<uint16_t>(normalized_start_time_mam + minutes_to_arrive.count()),
              .departure_ = static_cast<uint16_t>(normalized_start_time_mam),
              .is_footpath_ = static_cast<uint16_t>((minutes_to_arrive > 0_minutes) ? 1 : 0),
              .has_parent_ = 0
            },
            label_bf);
          if (added) {
            state_.station_mark_.set(to_idx(location_idx), true);
          }
        }
      }
    }
  }
}

void bmc_raptor::init_starts(location_idx_t const location_idx,
                             bool const use_initial_fp) {
  init_location_with_offset(location_idx, 0_minutes);
  if (use_initial_fp) {
    auto const& fps_out = tt_.locations_.footpaths_out_[kDefaultProfile][location_idx];
    for (const auto& fp : fps_out) {
      init_location_with_offset(fp.target(), fp.duration());
    }
  }
}

void bmc_raptor::get_earliest_sufficient_transports(std::uint32_t const bag_idx,
                                                    bmc_raptor_label const& label,
                                                    search_bitfield const& label_tdb,
                                                    route_idx_t route_idx,
                                                    unsigned short stop_idx,
                                                    bmc_raptor_route_bag_t& route_bag) {
  auto const& dep_event_times = tt_.event_times_at_stop(
    route_idx, stop_idx, event_type::kDep);

  constexpr auto n_days_to_iterate = kMaxTravelTime.count() / 1440 + 1U;

  const delta arr_as_delta(label.arrival_with_transfer_);
  const auto arr_days_after_dep = static_cast<std::uint16_t>(arr_as_delta.days());

  auto const seek_first_day = [&]() {
    return linear_lb(dep_event_times.begin(), dep_event_times.end(),
                     arr_as_delta.mam(),
                     [&](delta const a, int16_t const b) {
                       return a.mam() < b;
                     });
  };

  search_bitfield to_serve_tdb = label_tdb;
  for (auto days_after_dep = arr_days_after_dep;
            days_after_dep < n_days_to_iterate;
            ++days_after_dep) {
    if (to_serve_tdb.none()) {
      return;
    }

    auto const time_range_to_scan = it_range{
      days_after_dep == arr_days_after_dep ? seek_first_day() : dep_event_times.begin(),
      dep_event_times.end()};

    if (time_range_to_scan.empty()) {
      continue;
    }

    auto const base = static_cast<unsigned>(&*time_range_to_scan.begin_ - dep_event_times.data());
    for (auto const [t_offset, event_time] : utl::enumerate(time_range_to_scan)) {
      if (to_serve_tdb.none()) {
        return;
      }

      const auto travel_time_lb = event_time.mam() + 1440 * days_after_dep - label.departure_;
      if (travel_time_lb > kMaxTravelTime.count()) {
        return;
      }

      const auto event_day_offset = event_time.days();
      const auto transport = tt_.route_transport_ranges_[route_idx][base + t_offset];
      const int net_shift_right = days_after_dep - event_day_offset;
      auto const& transport_tdb = tt_.bitfields_[tt_.transport_traffic_days_[transport]];
      auto const aligned_transport_tdb = (net_shift_right >= 0) ?
        (transport_tdb >> static_cast<size_t>(net_shift_right)) :
        (transport_tdb << static_cast<size_t>(-net_shift_right));

      search_bitfield truncated_aligned_transport_tdb;
      truncate_to(aligned_transport_tdb, truncated_aligned_transport_tdb);

      if (truncated_aligned_transport_tdb.none()) {
        continue;
      }


      const auto matches = to_serve_tdb & truncated_aligned_transport_tdb;
      if (matches.any()) {
        add_to_route_bag(
          route_bag,
          {
            .transport_idx_ = transport.v_,
            .enter_stop_idx_ = stop_idx,
            .transport_day_offset_ = static_cast<int16_t>(net_shift_right),
            .parent_bag_idx_ = bag_idx,
            .departure_ = label.departure_,
          },
          matches
        );
        to_serve_tdb &= ~matches;
      }
    }
  }
}

void bmc_raptor::update_footpaths(unsigned const k) {
  state_.station_mark_.for_each_set_bit([&](std::uint32_t const i) {
    const auto& round_bag = state_.round_bags_[k][i];
    const location_idx_t l_idx{i};
    const auto& fps = tt_.locations_.footpaths_out_[kDefaultProfile][l_idx];
    for (const auto& rl : round_bag) {
      if (rl.label_.is_footpath_ == 1) {
        continue;
      }

      const auto base_arr = rl.label_.arrival_;
      const auto dep = rl.label_.departure_;

      for (auto const& fp : fps) {
        auto const target = fp.target();

        if (target == l_idx) {
          continue;
        }

        const std::uint16_t arr_with_foot = base_arr
          + static_cast<uint16_t>(fp.duration().count());

        if (arr_with_foot - dep > kMaxTravelTime.count()) {
          continue;
        }

        const bmc_raptor_label label_with_foot {
          .route_idx_ = rl.label_.route_idx_,
          .enter_stop_idx_ = rl.label_.enter_stop_idx_,
          .exit_stop_idx_ = rl.label_.exit_stop_idx_,
          .arrival_ = arr_with_foot,
          .parent_bag_idx_ = rl.label_.parent_bag_idx_,
          .arrival_with_transfer_ = arr_with_foot,
          .departure_ = dep,
          .is_footpath_ = 1,
          .has_parent_ = rl.label_.has_parent_,
        };

        auto tdb = rl.tdb_;

        if (destination_mask_[to_idx(target)]) {
          filter_by_dest_bag(state_.best_bags_[to_idx(target)], label_with_foot, tdb);
        } else {
          filter_by_non_dest_bag(state_.best_bags_[to_idx(target)], label_with_foot, tdb);
        }

        if (tdb.none()) {
          continue;
        }

        state_.fps_buffers_[to_idx(target)].emplace_back(
          label_with_foot,
          tdb
        );
      }
    }
  });

  for (auto l_idx = 0U; l_idx != tt_.n_locations(); ++l_idx) {
    const auto is_destination = destination_mask_[l_idx];
    for (auto const& [lbl, tdb] : state_.fps_buffers_[l_idx]) {
      bool added = false;
      if (is_destination) {
        added = add_to_dest_round_bag(state_.round_bags_[k][l_idx], lbl, tdb);
      } else {
        added = add_to_non_dest_round_bag(state_.round_bags_[k][l_idx], lbl, tdb);
      }
      if (added) {
        state_.station_mark_.set(l_idx, true);
      }
    }
    state_.fps_buffers_[l_idx].clear();
  }
}

bool bmc_raptor::update_route(unsigned const k, route_idx_t const route_idx) {
  bool any_marked = false;
  const auto& stop_sequence = tt_.route_location_seq_[route_idx];

  bmc_raptor_route_bag_t route_bag{};
  for (stop_idx_t stop_idx = 0U; stop_idx != stop_sequence.size(); ++stop_idx) {
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stp.location_idx());

    if (!transfer_mask_[l_idx]) {
      continue;
    }

    auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}].count();
    auto const is_destination = destination_mask_[l_idx];
    for (const auto& active_label : route_bag) {
      if (active_label.label_.transport_idx_ == transport_idx_t::invalid().v_) {
        continue;
      }
      const auto new_arr_delta = tt_.event_mam(route_idx,
                                               transport_idx_t{active_label.label_.transport_idx_},
                                               stop_idx,
                                               event_type::kArr);

      const auto new_arr_day = new_arr_delta.days() + active_label.label_.transport_day_offset_;
      const auto new_arr_mam = new_arr_delta.mam();

      const int32_t new_arr = new_arr_day * 1440 + new_arr_mam;

      if (!stp.out_allowed() || new_arr - active_label.label_.departure_ > kMaxTravelTime.count()) {
        continue;
      }

      const auto candidate_lbl = bmc_raptor_label{
        .route_idx_ = to_idx(route_idx),
        .enter_stop_idx_ = active_label.label_.enter_stop_idx_,
        .exit_stop_idx_ = stop_idx,
        .arrival_ = static_cast<uint16_t>(new_arr),
        .parent_bag_idx_ = active_label.label_.parent_bag_idx_,
        .arrival_with_transfer_ = static_cast<uint16_t>(new_arr + transfer_time_offset),
        .departure_ = static_cast<uint16_t>(active_label.label_.departure_),
        .is_footpath_ = 0,
        .has_parent_ = 1
      };

      auto candidate_tdb = active_label.tdb_;
      if (is_destination) {
        filter_by_dest_bag(state_.best_bags_[l_idx], candidate_lbl, candidate_tdb);
      } else {
        filter_by_non_dest_bag(state_.best_bags_[l_idx], candidate_lbl, candidate_tdb);
      }

      if (candidate_tdb.none()) {
        continue;
      }

      bool added = false;
      if (is_destination) {
        added = add_to_dest_round_bag(state_.round_bags_[k][l_idx], candidate_lbl, candidate_tdb);
      } else {
        added = add_to_non_dest_round_bag(state_.round_bags_[k][l_idx], candidate_lbl, candidate_tdb);
      }
      if (added) {
        state_.station_mark_.set(l_idx, true);
        any_marked = true;
      }
    }

    if (stop_idx == stop_sequence.size() - 1) {
      return any_marked;
    }

    if (stp.in_allowed() && state_.prev_station_mark_[l_idx]) {
      for (const auto [i, l] : utl::enumerate(state_.round_bags_[k - 1][l_idx])) {
        get_earliest_sufficient_transports(static_cast<std::uint32_t>(i),
                                           l.label_,
                                           l.tdb_,
                                           route_idx,
                                           stop_idx,
                                           route_bag);
      }
    }
  }
  return any_marked;
}

void bmc_raptor::rounds() {
  for (auto k = 1U; k != end_k(); ++k) {
    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != state_.station_mark_.size(); ++l_idx) {

      bool is_destination = destination_mask_[to_idx(l_idx)];
      if (state_.station_mark_[to_idx(l_idx)]) {
        for (const auto& l : state_.round_bags_[k - 1][to_idx(l_idx)]) {
            if (is_destination) {
              add_to_dest_round_bag(state_.best_bags_[to_idx(l_idx)], l.label_, l.tdb_);
            } else {
              add_to_non_dest_round_bag(state_.best_bags_[to_idx(l_idx)], l.label_, l.tdb_);
            }
        }
        any_marked = true;
        for (auto const r : tt_.location_routes_[l_idx]) {
          if (!route_mask_[to_idx(r)]) {
            continue;
          }
          state_.route_mark_.set(to_idx(r), true);
        }
      }
    }

    std::swap(state_.prev_station_mark_, state_.station_mark_);
    state_.station_mark_.zero_out();

    if (!any_marked) {
      return;
    }

    any_marked = false;
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      any_marked |= update_route(k, route_idx_t{r_id});
    }


    state_.route_mark_.zero_out();
    if (!any_marked) {
      return;
    }
    update_footpaths(k);
  }
}

void bmc_raptor::gather_journeys() {
  auto write_idx = 0U;
  destination_mask_.for_each_set_bit([&](uint32_t location_idx) {
    for (auto k = 0U; k != end_k(); ++k) {
      auto const& round_bag = state_.round_bags_[k][location_idx];
      if (round_bag.size() == 0) {
        continue;
      }

      constexpr auto fastest_direct = duration_t::max();
      for (auto label_it = round_bag.begin(); label_it != round_bag.end(); ++label_it) {

        const auto& label = label_it->label_;
        const auto& lbf = label_it->tdb_;

        if (duration_t const travel_time{label.arrival_ - label.departure_};
            travel_time >= fastest_direct) {
          continue;
        }

        lbf.for_each_set_bit([&](uint16_t tt_day_offset) {
          const auto day_offset = day_idx_t{tt_day_offset};
          const routing_time dep{day_offset, duration_t{static_cast<std::int16_t>(label.departure_)}};

          const auto arrival_day_offset = day_idx_t{label.arrival_ / 1440} + day_offset;
          const auto arrival_mam = static_cast<std::int16_t>(label.arrival_ % 1440);

          const routing_time arr{arrival_day_offset, duration_t{arrival_mam}};


          state_.results_[write_idx].add(
              journey{.legs_ = {},
                      .start_time_ = dep.to_unixtime(tt_),
                      .dest_time_ = arr.to_unixtime(tt_),
                      .dest_ = location_idx_t{location_idx},
                      .transfers_ = static_cast<std::uint8_t>(k - 1)});
        });
      }
    }
  });
}

unsigned bmc_raptor::end_k() { return kMaxTransfers + 1U; }

void bmc_raptor::emplace_relative_journeys_for(location_idx_t loc_idx, std::vector<bmc_journey>& bag) {
  constexpr auto dom = [](bmc_journey const& l1, bmc_journey const& l2) {
    return bmc_journey::dominates(l1, l2);
  };

  for (auto k = 0U; k != end_k(); ++k) {
    auto const& round_bag = state_.round_bags_[k][to_idx(loc_idx)];
    if (round_bag.size() == 0) {
      continue;
    }

    for (auto label_it = round_bag.begin(); label_it != round_bag.end(); ++label_it) {
      const auto& label = label_it->label_;
      const auto& tdb = label_it->tdb_;
      tdb.for_each_set_bit([&](size_t const i) {
        pareto_utils<bmc_journey>::pareto_add(
          bag,
          {
            .arrival_ = routing_time{static_cast<int>(i * 1440 + label.arrival_)},
            .departure_ = routing_time{static_cast<int>(i * 1440 + label.departure_)},
            .transfers_ = static_cast<std::uint16_t>(k > 0 ? k-1 : 0U),
            .label_iter_ = label_it
          },
          dom);
      });
    }
  }
}

} // nigiri::routing::raptor::para
