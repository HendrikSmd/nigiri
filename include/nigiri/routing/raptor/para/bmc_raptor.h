#pragma once

#include "nigiri/timetable.h"

#include "bmc_raptor_state.h"
#include "routing_time.h"
#include "timetable_view.h"

namespace nigiri::routing::para {

struct bmc_journey {

  static bool dominates(bmc_journey const& j1, bmc_journey const& j2);

  routing_time arrival_;
  routing_time departure_;
  std::uint16_t transfers_;
  bmc_raptor_bag_t::const_iterator label_iter_;
};

struct bmc_raptor {

  bmc_raptor(timetable_view const& tt_view, bmc_raptor_state& state,
             bitvec const& destination_mask,
             vector_map<route_idx_t, std::uint32_t> const& route_events_from,
             bitvec const& route_event_mask);

  template <auto dominates>
  static void cleanup_after_footpaths_added(bmc_raptor_bag_t& bag) {
    for (size_t fp_i = bag.labels_.size() - 1; fp_i > 0; --fp_i) {
      const auto& fp_label = bag.labels_[fp_i];

      if (fp_label.label_.is_footpath_ != 1) {
        break;
      }

      if (fp_label.tdb_.none()) {
        continue;
      }

      for (size_t round_i = fp_i;
           round_i > 0; --round_i) {

        auto& round_label = bag.labels_[round_i - 1];
        if (round_label.tdb_.none()) {
          continue;
        }

        if (dominates(fp_label.label_, round_label.label_)) {
          round_label.tdb_ &= ~fp_label.tdb_;
        }

      }
    }

    std::erase_if(bag.labels_, [](const auto& label){ return label.tdb_.none(); });
  }

  static bool dominates_destination(bmc_raptor_label const& l1,
                                    bmc_raptor_label const& l2);

  static bool dominates_non_destination(bmc_raptor_label const& l1,
                                        bmc_raptor_label const& l2);

  static bool dominates_destination_skip_fps(bmc_raptor_label const& l1,
                                             bmc_raptor_label const& l2);

  static bool dominates_non_destination_skip_fps(bmc_raptor_label const& l1,
                                                 bmc_raptor_label const& l2);

  static void cleanup_after_footpaths_at_dest(bmc_raptor_bag_t& bag);

  static void cleanup_after_footpaths_at_non_dest(bmc_raptor_bag_t& bag);

  static bool add_carefully_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                              const bmc_raptor_label& label,
                                              search_bitfield const& sbf);

  static bool add_carefully_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                                  bmc_raptor_label const& label,
                                                  search_bitfield const& sbf);

  static bool add_to_non_dest_round_bag(bmc_raptor_bag_t& bag,
                                        const bmc_raptor_label& label,
                                        search_bitfield const& sbf);

  static bool add_to_dest_round_bag(bmc_raptor_bag_t& bag,
                                    bmc_raptor_label const& label,
                                    search_bitfield const& sbf);

  static bool add_to_non_dest_best_bag(bmc_raptor_best_bag_t& bag,
                                       bmc_raptor_label const& label,
                                       search_bitfield const& sbf);

  static bool add_to_dest_best_bag(bmc_raptor_best_bag_t& bag,
                                   bmc_raptor_label const& label,
                                   search_bitfield const& sbf);

  static void filter_by_non_dest_bag(bmc_raptor_best_bag_t const& bag,
                                     bmc_raptor_label const& label,
                                     search_bitfield& sbf) {
      bag.filter_dominated<bmc_raptor_best_label::dominates_non_destination>(
        {label.arrival_, label.arrival_with_transfer_, label.departure_}, sbf);
  }

  static void filter_by_dest_bag(bmc_raptor_best_bag_t const& bag,
                                 bmc_raptor_label const& label,
                                 search_bitfield& sbf) {
      bag.filter_dominated<bmc_raptor_best_label::dominates_destination>(
        {label.arrival_, label.arrival_with_transfer_, label.departure_}, sbf);
  }

  static bool add_to_route_bag(bmc_raptor_route_bag_t& bag,
                               bmc_raptor_route_label const& label,
                               search_bitfield const& sbf);

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
  void gather_journeys();
  static unsigned end_k();

  void emplace_relative_journeys_for(location_idx_view_t location_view_idx,
                                     std::vector<bmc_journey>& bag) const;

  timetable_view const& tt_view_;
  bmc_raptor_state& state_;
  bitvec const& destination_mask_; // Indexed by source location_idx_t
  bitvec const& route_event_mask_;    // Indexed by source route_idx_t
  vector_map<route_idx_t, std::uint32_t> const& route_events_from_;
  bitset<kMaxDays> const tt_day_mask_;
};

}  // namespace nigiri::routing::para
