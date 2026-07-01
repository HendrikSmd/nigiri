#pragma once

#include "nigiri/timetable.h"

#include "bmc_raptor_state.h"
#include "routing_time.h"
#include "timetable_view.h"
#include "utl/enumerate.h"

namespace nigiri::routing::para {

struct bmc_journey {

  static bool dominates(bmc_journey const& j1, bmc_journey const& j2);

  routing_time arrival_;
  routing_time departure_;
  std::uint16_t used_transports_;
  location_idx_view_t target_location_idx_;
};

struct bmc_raptor {

  bmc_raptor(timetable_view const& tt_view, bmc_raptor_state& state,
             bitvec const& destination_mask,
             vector_map<route_idx_t, std::uint32_t> const& route_events_from,
             bitvec const& route_event_mask);

  static bool dominates_destination(bmc_raptor_label const& l1,
                                    bmc_raptor_label const& l2);

  static bool dominates_non_destination(bmc_raptor_label const& l1,
                                        bmc_raptor_label const& l2);

  static bool add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                        const bmc_raptor_label& label,
                                        search_bitfield sbf);

  static bool add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                    bmc_raptor_label const& label,
                                    search_bitfield sbf);

  static void filter_by_non_dest_bag(bmc_raptor_bag_t const& bag,
                                     bmc_raptor_label const& label,
                                     search_bitfield& sbf) {
      bag.filter_dominated<&dominates_non_destination>(label, sbf);
  }

  static void filter_by_dest_bag(bmc_raptor_bag_t const& bag,
                                 bmc_raptor_label const& label,
                                 search_bitfield& sbf) {
      bag.filter_dominated<&dominates_destination>(label, sbf);
  }

  static bool add_to_route_bag(bmc_raptor_route_bag_t& bag,
                               bmc_raptor_route_label label,
                               search_bitfield sbf);

  static bitset<kMaxDays> get_tt_day_mask(timetable const& tt);

  void init_starts(location_idx_view_t location_idx,
                   bool use_initial_fp);

  void init_location_with_offset(location_idx_view_t location_idx_view,
                                 duration_t minutes_to_arrive);

  void get_earliest_sufficient_transports(
      std::uint32_t bag_idx, std::uint16_t departure, std::uint16_t arr_with_transfer,
      search_bitfield const& td_bitfield, route_idx_t route_idx,
      unsigned short stop_idx, bmc_raptor_route_bag_t& bag);

  void update_footpaths(unsigned k);
  bool update_route(unsigned k, route_idx_t r);
  void rounds();
  void gather_journeys(std::vector<pareto_set<journey>>& buffer);
  static unsigned end_k();

  void emplace_relative_journeys_for(location_idx_view_t location_view_idx,
                                     std::vector<bmc_journey>& bag) const;
  template <typename Fun>
  void reconstruct(Fun&& consume) {
    std::vector<bmc_journey> tmp;
    std::vector<bmc_journey> to_reconstruct;
    destination_mask_.for_each_set_bit([&](size_t i) {
      location_idx_t const destination_loc_idx = location_idx_t{i};
      location_idx_view_t const destination_loc_view_idx =
          tt_view_.get_view_idx(destination_loc_idx);

      if (destination_loc_view_idx == location_idx_view_t::invalid()) {
        return;
      }

      emplace_relative_journeys_for(destination_loc_view_idx, tmp);

      to_reconstruct.insert(to_reconstruct.end(), tmp.begin(), tmp.end());
      tmp.clear();
    });

    std::ranges::sort(to_reconstruct,
                      [](bmc_journey const& j1, bmc_journey const& j2) {
                        return j1.departure_ < j2.departure_;
                      });

    auto const find_entry_in_prev_round =
        [&](unsigned const k, transport const& tr, route_idx_t const route,
            stop_idx_t const from_stop_idx, routing_time const departure)
        -> std::pair<location_idx_view_t, stop_idx_t> {
      auto const& tt = tt_view_.get_source_tt();
      auto const stop_seq = tt.route_location_seq_[route];
      auto const n_stops = from_stop_idx + 1U;
      for (auto i = 1U; i != n_stops; ++i) {
        auto const stop_idx = static_cast<stop_idx_t>(from_stop_idx - i);
        auto const stp = stop{stop_seq[stop_idx]};
        auto const l = stp.location_idx();

        if (!stp.in_allowed()) {
          continue;
        }

        auto ev_time = routing_time{
            tr.day_,
            tt.event_mam(tr.t_idx_, stop_idx, event_type::kDep).as_duration()};
        auto const l_view = tt_view_.get_view_idx(l);

        auto& reconstruction_bag =
            state_.reconstruction_bags_[k - 1][to_idx(l_view)];
        if (!reconstruction_bag.initialized_) {
          auto const& round_bag = state_.round_bags_[k - 1][to_idx(l_view)];
          reconstruction_bag.decompress(round_bag);
        }
        reconstruction_bag.move_up_until_departure(departure);
        if (reconstruction_bag.no_more_candidates()) {
          continue;
        }

        auto const candidate = reconstruction_bag.scan_begin_;
        if (candidate->departure_ > departure) {
          continue;
        }
        auto const possible_dep_time =
            candidate->arrival_ + tt.locations_.transfer_time_[l];
        if (possible_dep_time <= ev_time) {
          return {l_view, stop_idx};
        }
      }

      return {location_idx_view_t::invalid(), stop_seq.size()};
    };

    auto const is_transport_active = [&](transport_idx_t const t,
                                         std::size_t const day) {
      auto const& tt = tt_view_.get_source_tt();
      return tt.bitfields_[tt.transport_traffic_days_[t]].test(day);
    };

    auto const get_route_transport =
        [&](unsigned const k, routing_time const departure_time,
            routing_time const curr_arrival_time, route_idx_t const r,
            stop_idx_t const stop_idx)
        -> std::pair<location_idx_view_t, stop_idx_t> {
      auto const& tt = tt_view_.get_source_tt();
      auto const [day, mam] = curr_arrival_time.day_idx_mam();

      for (auto const t : tt.route_transport_ranges_[r]) {
        auto const event_mam = tt.event_mam(t, stop_idx, event_type::kArr);

        if (minutes_after_midnight_t{event_mam.count() % 1440} != mam) {
          continue;
        }

        auto const traffic_day = to_idx(day) - event_mam.count() / 1440;
        if (!is_transport_active(t, static_cast<std::size_t>(traffic_day))) {
          continue;
        }

        auto tr = transport{t, day_idx_t{traffic_day}};

        auto const [from_loc, s] =
            find_entry_in_prev_round(k, tr, r, stop_idx, departure_time);
        if (from_loc != location_idx_view_t::invalid()) {
          return {from_loc, s};
        }
      }

      return {location_idx_view_t::invalid(), tt.route_location_seq_[r].size()};
    };

    auto const get_transport = [&](unsigned const k,
                                   location_idx_view_t const loc_view,
                                   routing_time const departure_time,
                                   routing_time const curr_arrival_time)
        -> std::tuple<location_idx_view_t, stop_idx_t, route_idx_t,
                      stop_idx_t> {
      auto const& tt = tt_view_.get_source_tt();
      auto const loc = tt_view_.get_source_idx(loc_view);
      for (auto const& r : tt.location_routes_[loc]) {

        auto const location_seq = tt.route_location_seq_[r];
        for (auto const [i, s] : utl::enumerate(location_seq)) {
          auto const stp = stop{s};
          if (stp.location_idx() != loc || i == 0U || !stp.out_allowed()) {
            continue;
          }

          auto [from_loc_view, from_stop] =
              get_route_transport(k, departure_time, curr_arrival_time, r,
                                  static_cast<stop_idx_t>(i));
          if (from_loc_view != location_idx_view_t::invalid()) {
            return {from_loc_view, from_stop, r, static_cast<stop_idx_t>(i)};
          }
        }
      }
      return {location_idx_view_t::invalid(), 0, route_idx_t::invalid(), 0};
    };

    auto const get_legs =
        [&](unsigned const k, location_idx_view_t const loc_idx,
            routing_time const departure_time,
            routing_time const curr_target_arrival) -> location_idx_view_t {
      auto const& tt = tt_view_.get_source_tt();

      auto const src_loc_idx = tt_view_.get_source_idx(loc_idx);

      auto const [from_loc_view_idx, stop_from, route, stop_to] =
          get_transport(k, loc_idx, departure_time, curr_target_arrival);
      if (from_loc_view_idx != location_idx_view_t::invalid()) {
        std::forward<Fun>(consume)(stop_from, route, stop_to);
        return from_loc_view_idx;
      }

      auto const footpaths =
          tt.locations_.footpaths_in_[kDefaultProfile][src_loc_idx];
      for (auto const& fp : footpaths) {
        auto const fp_from = fp.target();
        location_idx_view_t fp_view_from = tt_view_.get_view_idx(fp_from);
        auto const [from_loc_view_idx_fp, stop_from_fp, route_fp, stop_to_fp] =
            get_transport(k, fp_view_from, departure_time,
                          curr_target_arrival - fp.duration());
        if (from_loc_view_idx_fp != location_idx_view_t::invalid()) {
          std::forward<Fun>(consume)(stop_from, route, stop_to);
          return from_loc_view_idx_fp;
        }
      }

      throw utl::fail("reconstruction failed");
    };

    for (auto const& j : to_reconstruct) {
      location_idx_view_t from_loc_view_idx = location_idx_view_t::invalid();
      for (auto i = 0U; i < j.used_transports_; ++i) {
        auto const k = j.used_transports_ - i;
        if (k == j.used_transports_) {
          from_loc_view_idx =
              get_legs(k, j.target_location_idx_, j.departure_, j.arrival_);
        } else {
          auto const label_iter =
              state_.reconstruction_bags_[k][to_idx(from_loc_view_idx)]
                  .scan_begin_;
          from_loc_view_idx =
              get_legs(k, from_loc_view_idx, label_iter->departure_,
                       label_iter->arrival_);
        }
      }
    }
  }

  timetable_view const& tt_view_;
  bmc_raptor_state& state_;
  bitvec const& destination_mask_; // Indexed by source location_idx_t
  bitvec const& route_event_mask_;    // Indexed by source route_idx_t
  vector_map<route_idx_t, std::uint32_t> const& route_events_from_;
  bitset<kMaxDays> const tt_day_mask_;
};

}  // namespace nigiri::routing::para
