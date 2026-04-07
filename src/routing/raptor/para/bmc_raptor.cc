#include "nigiri/routing/raptor/para/bmc_raptor.h"

#include "boost/iostreams/seek.hpp"

#include "nigiri/loader/register.h"

#include "nigiri/common/linear_lower_bound.h"
#include "nigiri/routing/raptor/para/routing_time.h"
#include "nigiri/routing/raptor/para/simd_dominance_relations.h"
#include "nigiri/stop.h"
#include "nigiri/types.h"

#include "utl/enumerate.h"
#include "nigiri/routing/raptor/para/timetable_view.h"

namespace nigiri::routing::para {

bool bmc_journey::dominates(bmc_journey const& j1, bmc_journey const& j2) {
  return j1.departure_ >= j2.departure_ && j1.arrival_ <= j2.arrival_ &&
         j1.transfers_ <= j2.transfers_;
}

bmc_raptor::bmc_raptor(timetable_view const& tt_view,
                       bmc_raptor_state& state,
                       bitvec const& destination_mask,
                       bitvec const& transfer_mask,
                       bitvec const& footpath_mask)
    : tt_view_(tt_view),
      state_(state),
      destination_mask_(destination_mask),
      transfer_mask_(transfer_mask),
      footpath_mask_(footpath_mask),
      tt_day_mask_(get_tt_day_mask(tt_view.get_source_tt())) {
  state_.resize(tt_view.get_n_locations(), tt_view.get_n_routes(),
                static_cast<unsigned int>(destination_mask_.count()));
  };

bitset<kMaxDays> bmc_raptor::get_tt_day_mask(timetable const& tt) {
  auto const n_days_in_tt =
      tt.day_idx_mam(tt.internal_interval().to_ - 1_minutes).first -
      tt.day_idx_mam(tt.internal_interval().from_).first + 1;
  return bitset<kMaxDays>{std::string(to_idx(n_days_in_tt), '1')};
}

#ifdef NIGIRI_ENABLE_SIMD

bool bmc_raptor::add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                           std::array<std::uint16_t, 3> const& label,
                                           bmc_round_meta_data const& meta_data,
                                           search_bitfield sbf) {
  return bag.add<bicriteria_dominance_16>(label, sbf, meta_data);
}

bool bmc_raptor::add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                  std::array<std::uint16_t, 3> const& label,
                                  bmc_round_meta_data const& meta_data,
                                  search_bitfield sbf) {
  return bag.add<bicriteria_conservative_dominance_16>(label, sbf, meta_data);
}

void bmc_raptor::filter_by_non_dest_bag(bmc_raptor_bag_t const& bag,
                                        std::array<std::uint16_t, 3> const& label,
                                        search_bitfield& sbf) {
  bag.filter_dominated<bicriteria_dominance_16>(label, sbf);
}

void bmc_raptor::filter_by_dest_bag(bmc_raptor_bag_t const& bag,
                                    std::array<std::uint16_t, 3> const& label,
                                    search_bitfield& sbf) {
  bag.filter_dominated<bicriteria_conservative_dominance_16>(label, sbf);
}
#else

bool bmc_raptor::dominates_destination(bmc_raptor_label const& l1,
                                       bmc_raptor_label const& l2) {
  return l1.dominates_destination(l2);
}

bool bmc_raptor::dominates_non_destination(bmc_raptor_label const& l1,
                                           bmc_raptor_label const& l2) {
  return l1.dominates_non_destination(l2);
}

void bmc_raptor::cleanup_after_footpaths_at_dest(bmc_raptor_bag_t& bag) {
  cleanup_after_footpaths_added<&dominates_destination>(bag);
}

void bmc_raptor::cleanup_after_footpaths_at_non_dest(bmc_raptor_bag_t& bag) {
  cleanup_after_footpaths_added<&dominates_non_destination>(bag);
}

bool bmc_raptor::add_carefully_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                                 const bmc_raptor_label& label,
                                                 search_bitfield sbf) {
  return bag.add_careful<&dominates_destination>(label, sbf);
}

bool bmc_raptor::add_carefully_to_non_dest_round_bag(
    bmc_raptor_bag_t& bag, bmc_raptor_label const& label, search_bitfield sbf) {
  return bag.add_careful<&dominates_non_destination>(label, sbf);
}

bool bmc_raptor::add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                           bmc_raptor_label const& label,
                                           search_bitfield const bf) {
  return bag.add<&dominates_non_destination>(label, bf);
}

bool bmc_raptor::add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                       bmc_raptor_label const& label,
                                       search_bitfield const bf) {
  return bag.add<&dominates_destination>(label, bf);
}

void bmc_raptor::filter_by_non_dest_bag(bmc_raptor_bag_t const& bag,
                                        bmc_raptor_label const& label,
                                        search_bitfield& sbf) {
  bag.filter_dominated<&dominates_non_destination>(label, sbf);
}

void bmc_raptor::filter_by_dest_bag(bmc_raptor_bag_t const& bag,
                                    bmc_raptor_label const& label,
                                    search_bitfield& sbf) {
  bag.filter_dominated<&dominates_destination>(label, sbf);
}
#endif

bool bmc_raptor::add_to_route_bag(bmc_raptor_route_bag_t& bag,
                                  bmc_raptor_route_label label,
                                  search_bitfield bf) {
  constexpr auto dom = [](bmc_raptor_route_label const& l1,
                          bmc_raptor_route_label const& l2) {
    return l1.dominates(l2);
  };
  return bag.add<dom>(label, bf);
}

void bmc_raptor::init_location_with_offset(location_idx_view_t const location_idx_view,
                                           duration_t const minutes_to_arrive) {
  if (minutes_to_arrive > kMaxTravelTime) {
    return;
  }

  std::array<search_bitfield, 1440> departures_for_minute;

#ifdef NIGIRI_ENABLE_SIMD
  bmc_round_meta_data initial_md{
    .route_idx_ = 0U,
    .parent_bag_idx_ = 0U,
    .enter_stop_idx_ = 0U,
    .exit_stop_idx_ = 0U,
    .has_parent_ = 0,
    .is_footpath_ = static_cast<uint16_t>((minutes_to_arrive > 0_minutes) ? 1 : 0)
  };
#endif

  const auto& tt = tt_view_.get_source_tt();
  location_idx_t const source_location_idx = tt_view_.get_source_idx(location_idx_view);
  utl::verify(source_location_idx != location_idx_t::invalid(),
              "Unmapped location found");
  for (auto const r : tt.location_routes_[source_location_idx]) {
    route_idx_view_t const view_route_idx = tt_view_.get_view_idx(r);
    if (view_route_idx == route_idx_view_t::invalid()) {
      continue;
    }

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

        search_bitfield label_bf;
        truncate_to(trip_tdb, label_bf);

        departures_for_minute[static_cast<std::uint16_t>(
            normalized_start_time_mam)] |= label_bf;
      }
    }
  }

  for (std::uint16_t i = 0U; i < 1440; ++i) {
    auto const& minute_bf = departures_for_minute[i];
    if (minute_bf.any()) {
#ifdef NIGIRI_ENABLE_SIMD
      bool const added = add_to_non_dest_round_bag(
          state_.round_bags_[0U][to_idx(location_idx_view)],
          {i, static_cast<std::uint16_t>(i + minutes_to_arrive.count()),
           static_cast<std::uint16_t>(i + minutes_to_arrive.count())},
          initial_md, minute_bf);
#else
      bool const added = add_to_non_dest_round_bag(
          state_.round_bags_[0U][to_idx(location_idx_view)],
          {.route_idx_ = 0U,
           .enter_stop_idx_ = 0U,
           .exit_stop_idx_ = 0U,
           .arrival_ =
               static_cast<std::uint16_t>(i + minutes_to_arrive.count()),
           .parent_bag_idx_ = 0U,
           .arrival_with_transfer_ =
               static_cast<std::uint16_t>(i + minutes_to_arrive.count()),
           .departure_ = i,
           .is_footpath_ =
               static_cast<uint16_t>((minutes_to_arrive > 0_minutes) ? 1 : 0),
           .has_parent_ = 0},
          minute_bf);
#endif
      if (added) {
        state_.station_mark_.set(to_idx(location_idx_view), true);
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
    auto const& out_fps = tt_view_.get_source_tt().locations_.footpaths_out_[kDefaultProfile];
    auto const loc_out_fps = out_fps[source_location_idx];
    const auto fp_base_idx = static_cast<std::uint32_t>(std::distance(out_fps.data_.begin(), loc_out_fps.begin()));
    for (auto const [i, fp] : utl::enumerate(loc_out_fps)) {
      auto fp_idx = footpath_idx_t{fp_base_idx + i};
      if (! footpath_mask_.test(to_idx(fp_idx))) {
        continue;
      }

      init_location_with_offset(tt_view_.get_view_idx(fp.target()),
                                fp.duration());
    }
  }
}

void bmc_raptor::get_earliest_sufficient_transports(
    std::uint32_t const bag_idx,
    std::uint16_t departure,
    std::uint16_t arrival_with_transfer,
    search_bitfield const& label_tdb,
    route_idx_t route_idx,
    unsigned short stop_idx,
    bmc_raptor_route_bag_t& route_bag) {
  const auto& tt = tt_view_.get_source_tt();
  auto const& dep_event_times =
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
                .enter_stop_idx_ = stop_idx,
                .transport_day_offset_ = static_cast<int16_t>(net_shift_right),
                .parent_bag_idx_ = bag_idx,
                .departure_ = departure,
            },
            matches);
        to_serve_tdb &= ~matches;
      }
    }
  }
}

void bmc_raptor::update_footpaths(unsigned const k) {
#ifdef NIGIRI_ENABLE_SIMD
  std::array<std::uint16_t, 3> new_fields_buff = {0,0,0};
#endif

  state_.station_mark_.for_each_set_bit([&](std::uint32_t const i) {
    auto const& round_bag = state_.round_bags_[k][i];
    location_idx_view_t const location_view_idx{i};
    location_idx_t const source_location_idx =
        tt_view_.get_source_idx(location_view_idx);

    auto const& out_fps =
        tt_view_.get_source_tt().locations_.footpaths_out_[kDefaultProfile];
    auto const loc_out_fps = out_fps[source_location_idx];
    auto const fp_base_idx = static_cast<std::uint32_t>(
        std::distance(out_fps.data_.begin(), loc_out_fps.begin()));

    for (auto rl_view : round_bag) {
#ifdef NIGIRI_ENABLE_SIMD
      if (rl_view.metadata_.is_footpath_ == 1) {
        continue;
      }
      auto const base_arr = rl_view.fields_[1];
      auto const dep = rl_view.fields_[0];
#else
      if (rl_view.label_.is_footpath_ == 1) {
        // All remaining labels are footpaths
        break;
      }

      auto const base_arr = rl_view.label_.arrival_;
      auto const dep = rl_view.label_.departure_;
#endif

      for (auto const [j, fp] : utl::enumerate(loc_out_fps)) {
        auto const fp_idx = footpath_idx_t{fp_base_idx + j};
        if (!footpath_mask_.test(to_idx(fp_idx))) {
          continue;
        }

        auto const target = fp.target();

        if (target == source_location_idx) {
          continue;
        }

        std::uint16_t const arr_with_foot =
            base_arr + static_cast<uint16_t>(fp.duration().count());

        if (arr_with_foot - dep > kMaxTravelTime.count()) {
          continue;
        }

#ifdef NIGIRI_ENABLE_SIMD
        new_fields_buff[0] = dep;
        new_fields_buff[1] = arr_with_foot;
        new_fields_buff[2] = arr_with_foot;
        auto tdb = rl_view.bitset_;
#else
        const bmc_raptor_label label_with_foot{
            .route_idx_ = rl_view.label_.route_idx_,
            .enter_stop_idx_ = rl_view.label_.enter_stop_idx_,
            .exit_stop_idx_ = rl_view.label_.exit_stop_idx_,
            .arrival_ = arr_with_foot,
            .parent_bag_idx_ = rl_view.label_.parent_bag_idx_,
            .arrival_with_transfer_ = arr_with_foot,
            .departure_ = dep,
            .is_footpath_ = 1,
            .has_parent_ = rl_view.label_.has_parent_,
        };
        auto tdb = rl_view.tdb_;

        location_idx_view_t const target_view_idx =
            tt_view_.get_view_idx(target);
        utl::verify(target_view_idx != location_idx_view_t::invalid(),
                    "Unmapped location");

        bool const is_target_destination = destination_mask_[to_idx(target)];

#endif
        if (is_target_destination) {
#ifdef NIGIRI_ENABLE_SIMD
          filter_by_dest_bag(state_.best_bags_[to_idx(target_view_idx)], new_fields_buff, tdb);
#else
          filter_by_dest_bag(state_.best_bags_[to_idx(target_view_idx)], label_with_foot, tdb);
#endif
        } else {
#ifdef NIGIRI_ENABLE_SIMD
          filter_by_non_dest_bag(state_.best_bags_[to_idx(target_view_idx)], new_fields_buff, tdb);
#else
          filter_by_non_dest_bag(state_.best_bags_[to_idx(target_view_idx)], label_with_foot, tdb);
#endif
        }

        if (tdb.none()) {
          continue;
        }

#ifdef NIGIRI_ENABLE_SIMD
        bmc_round_meta_data const with_foot_md{
          .route_idx_ = rl_view.metadata_.route_idx_,
          .parent_bag_idx_ = rl_view.metadata_.parent_bag_idx_,
          .enter_stop_idx_ = rl_view.metadata_.enter_stop_idx_,
          .exit_stop_idx_ = rl_view.metadata_.exit_stop_idx_,
          .has_parent_ = rl_view.metadata_.has_parent_,
          .is_footpath_ = 1,
        };
        state_.fps_buffers_[to_idx(target_view_idx)].emplace_back(new_fields_buff, with_foot_md, tdb);
#else
        if (to_idx(target_view_idx) < i) {
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
        } else {
          bool added = false;
          if (is_target_destination) {
            added = add_carefully_to_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot,
                tdb);
          } else {
            added = add_carefully_to_non_dest_round_bag(
                state_.round_bags_[k][to_idx(target_view_idx)], label_with_foot,
                tdb);
          }
          if (added) {
            state_.fp_label_added_.set(to_idx(target_view_idx), true);
            state_.station_mark_.set(to_idx(target_view_idx), true);
          }
        }
#endif
      }
    }
  });

  state_.fp_label_added_.for_each_set_bit([&](std::uint32_t const j) {
    location_idx_view_t view_j(j);
    location_idx_t source_j = tt_view_.get_source_idx(view_j);
    if (destination_mask_[to_idx(source_j)]) {
      cleanup_after_footpaths_at_dest(state_.round_bags_[k][to_idx(view_j)]);
    } else {
      cleanup_after_footpaths_at_non_dest(state_.round_bags_[k][to_idx(view_j)]);
    }
  });
  utl::fill(state_.fp_label_added_.blocks_, 0U);
}

bool bmc_raptor::update_route(unsigned const k, route_idx_t const route_idx) {
  const auto& tt = tt_view_.get_source_tt();
  bool any_marked = false;
  auto const& stop_sequence = tt.route_location_seq_[route_idx];
#ifdef NIGIRI_ENABLE_SIMD
  std::array<std::uint16_t, 3> new_fields_buff = {0,0,0};
#endif

  bmc_raptor_route_bag_t route_bag{};
  for (stop_idx_t stop_idx = 0U; stop_idx != stop_sequence.size(); ++stop_idx) {
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const source_location_idx = stp.location_idx();
    auto const view_location_idx = tt_view_.get_view_idx(source_location_idx);

    if (!transfer_mask_[to_idx(source_location_idx)]) {
      continue;
    }

    auto const transfer_time_offset =
        tt.locations_.transfer_time_[location_idx_t{source_location_idx}].count();
    auto const is_destination = destination_mask_[to_idx(source_location_idx)];
    for (auto const& active_label : route_bag) {
      if (active_label.label_.transport_idx_ == transport_idx_t::invalid().v_) {
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

#ifdef NIGIRI_ENABLE_SIMD
      new_fields_buff[0] = static_cast<uint16_t>(active_label.label_.departure_);
      new_fields_buff[1] = static_cast<uint16_t>(new_arr);
      new_fields_buff[2] = static_cast<uint16_t>(new_arr + transfer_time_offset);
#else
      const auto candidate_lbl = bmc_raptor_label{
        .route_idx_ = to_idx(route_idx),
        .enter_stop_idx_ = active_label.label_.enter_stop_idx_,
        .exit_stop_idx_ = stop_idx,
        .arrival_ = static_cast<uint16_t>(new_arr),
        .parent_bag_idx_ = active_label.label_.parent_bag_idx_,
        .arrival_with_transfer_ =
            static_cast<uint16_t>(new_arr + transfer_time_offset),
        .departure_ = static_cast<uint16_t>(active_label.label_.departure_),
        .is_footpath_ = 0,
        .has_parent_ = 1
      };
#endif

      auto candidate_tdb = active_label.tdb_;

      if (is_destination) {
#ifdef NIGIRI_ENABLE_SIMD
        filter_by_dest_bag(state_.best_bags_[to_idx(view_location_idx)], new_fields_buff, candidate_tdb);
#else
        filter_by_dest_bag(state_.best_bags_[to_idx(view_location_idx)], candidate_lbl, candidate_tdb);
#endif
      } else {
#ifdef NIGIRI_ENABLE_SIMD
        filter_by_non_dest_bag(state_.best_bags_[to_idx(view_location_idx)], new_fields_buff, candidate_tdb);
#else
        filter_by_non_dest_bag(state_.best_bags_[to_idx(view_location_idx)], candidate_lbl, candidate_tdb);
#endif
      }

      if (candidate_tdb.none()) {
        continue;
      }

#ifdef NIGIRI_ENABLE_SIMD
      bmc_round_meta_data const new_md {
        .route_idx_ = to_idx(route_idx),
        .parent_bag_idx_ = active_label.label_.parent_bag_idx_,
        .enter_stop_idx_ = active_label.label_.enter_stop_idx_,
        .exit_stop_idx_ = stop_idx,
        .has_parent_ = 1,
        .is_footpath_ = 0
      };
#endif

      bool added = false;
      if (is_destination) {
#ifdef NIGIRI_ENABLE_SIMD
        added = add_to_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], new_fields_buff, new_md, candidate_tdb);
#else
        added = add_to_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], candidate_lbl, candidate_tdb);
#endif
      } else {
#ifdef NIGIRI_ENABLE_SIMD
        added = add_to_non_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], new_fields_buff, new_md, candidate_tdb);
#else
        added = add_to_non_dest_round_bag(state_.round_bags_[k][to_idx(view_location_idx)], candidate_lbl, candidate_tdb);
#endif
      }
      if (added) {
        state_.station_mark_.set(to_idx(view_location_idx), true);
        any_marked = true;
      }
    }

    if (stop_idx == stop_sequence.size() - 1) {
      return any_marked;
    }

    if (stp.in_allowed() && state_.prev_station_mark_[to_idx(view_location_idx)]) {
      const auto& scan_bag = state_.round_bags_[k - 1][to_idx(view_location_idx)];
#ifdef NIGIRI_ENABLE_SIMD
      for (auto bag_iter = scan_bag.begin(); bag_iter != scan_bag.end();
           ++bag_iter) {
        auto const iter_view = *bag_iter;
        get_earliest_sufficient_transports(
            static_cast<std::uint32_t>(bag_iter.index_), iter_view.fields_[0],
            iter_view.fields_[2], iter_view.bitset_, route_idx, stop_idx,
            route_bag);
      }
#else
      for (const auto [i, l] : utl::enumerate(scan_bag)) {
        get_earliest_sufficient_transports(
            static_cast<std::uint32_t>(i), l.label_.departure_,
            l.label_.arrival_with_transfer_, l.tdb_, route_idx, stop_idx,
            route_bag);
      }
#endif
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
        for (auto const l : state_.round_bags_[k - 1][to_idx(location_view_idx)]) {
          if (is_destination) {
#ifdef NIGIRI_ENABLE_SIMD
            add_to_dest_round_bag(state_.best_bags_[to_idx(location_view_idx)], l.fields_, l.metadata_, l.bitset_);
#else
            add_to_dest_round_bag(state_.best_bags_[to_idx(location_view_idx)], l.label_, l.tdb_);
#endif
          } else {
#ifdef NIGIRI_ENABLE_SIMD
            add_to_non_dest_round_bag(state_.best_bags_[to_idx(location_view_idx)],l.fields_, l.metadata_, l.bitset_);
#else
            add_to_non_dest_round_bag(state_.best_bags_[to_idx(location_view_idx)],l.label_, l.tdb_);
#endif
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

void bmc_raptor::gather_journeys() {
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

#ifdef NIGIRI_ENABLE_SIMD
        auto const label_view = *label_it;
        auto const label_fields = label_view.fields_;
        auto const& lbf = label_view.bitset_;

        const auto departure = label_fields[0];
        const auto arrival = label_fields[1];
#else
        const auto& label = label_it->label_;
        const auto& lbf = label_it->tdb_;

        const auto departure = label.departure_;
        const auto arrival = label.departure_;
#endif

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

          state_.results_[write_idx].add(
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
                                               std::vector<bmc_journey>& bag) {
  constexpr auto dom = [](bmc_journey const& l1, bmc_journey const& l2) {
    return bmc_journey::dominates(l1, l2);
  };

  for (auto k = 0U; k != end_k(); ++k) {
    auto const& round_bag = state_.round_bags_[k][to_idx(loc_idx)];
    if (round_bag.size() == 0) {
      continue;
    }

    for (auto label_it = round_bag.begin(); label_it != round_bag.end();
         ++label_it) {
      auto const label_view = *label_it;
#ifdef NIGIRI_ENABLE_SIMD
      auto const& tdb = label_view.bitset_;
      tdb.for_each_set_bit([&](size_t const i) {
        pareto_utils<bmc_journey>::pareto_add(
            bag,
            {.arrival_ = routing_time{static_cast<int>(i * 1440 +
                                                       label_view.fields_[1])},
             .departure_ = routing_time{static_cast<int>(
                 i * 1440 + label_view.fields_[0])},
             .transfers_ = static_cast<std::uint16_t>(k > 0 ? k - 1 : 0U),
             .label_iter_ = label_it},
            dom);
      });
#else
      auto const& tdb = label_view.tdb_;
      tdb.for_each_set_bit([&](size_t const i) {
        pareto_utils<bmc_journey>::pareto_add(
            bag,
            {.arrival_ = routing_time{static_cast<int>(i * 1440 +
                                                       label_view.label_.arrival_)},
             .departure_ = routing_time{static_cast<int>(
                 i * 1440 + label_view.label_.departure_)},
             .transfers_ = static_cast<std::uint16_t>(k > 0 ? k - 1 : 0U),
             .label_iter_ = label_it},
            dom);
      });
#endif
    }
  }
}

}  // namespace nigiri::routing::para
