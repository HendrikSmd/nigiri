#pragma once

#include "nigiri/timetable.h"

#include "bmc_raptor_state.h"
#include "routing_time.h"

namespace nigiri::routing::para {

struct bmc_journey {

  static bool dominates(bmc_journey const& j1, bmc_journey const& j2);

  routing_time arrival_;
  routing_time departure_;
  std::uint16_t transfers_;
  bmc_raptor_bag_t::const_iterator label_iter_;
};

struct bmc_raptor {

  bmc_raptor(timetable const& tt,
             bmc_raptor_state& state,
             bitvec const& destination_mask,
             bitvec const& route_mask,
             bitvec const& transfer_mask);


  static bool add_to_non_dest_round_bag(bmc_raptor_bag_t& bag, bmc_raptor_label label, search_bitfield sbf);
  static bool add_to_dest_round_bag(bmc_raptor_bag_t& bag, bmc_raptor_label label, search_bitfield sbf);

  static void filter_by_non_dest_bag(bmc_raptor_bag_t const& bag, bmc_raptor_label const& label, search_bitfield& sbf);
  static void filter_by_dest_bag(bmc_raptor_bag_t const& bag, bmc_raptor_label const& label, search_bitfield& sbf);

  static bool add_to_route_bag(bmc_raptor_route_bag_t& bag, bmc_raptor_route_label label, search_bitfield sbf);
  static bitset<kMaxDays> get_tt_day_mask(timetable const& tt);

  void init_starts(location_idx_t location_idx,
                   bool use_initial_fp);
  void init_location_with_offset(location_idx_t location_idx,
                                 duration_t minutes_to_arrive);

  void get_earliest_sufficient_transports(std::uint32_t bag_idx,
                                          bmc_raptor_label const& label,
                                          search_bitfield const& td_bitfield,
                                          route_idx_t route_idx,
                                          unsigned short stop_idx,
                                          bmc_raptor_route_bag_t& bag);

  void update_footpaths(unsigned k);
  bool update_route(unsigned k, route_idx_t r);
  void rounds();
  void gather_journeys();
  static unsigned end_k();


  void emplace_relative_journeys_for(location_idx_t location_idx, std::vector<bmc_journey>& bag);



  timetable const& tt_;
  bmc_raptor_state& state_;
  bitvec const& destination_mask_;
  bitvec const& route_mask_;
  bitvec const& transfer_mask_;
  bitset<kMaxDays> const tt_day_mask_;
};

} // nigiri::routing::raptor::para
