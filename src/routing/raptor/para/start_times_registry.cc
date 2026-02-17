#include "nigiri/routing/raptor/para/start_times_registry.h"

#include "nigiri/common/clear_all.h"
#include "nigiri/routing/raptor/para/routing_time.h"

namespace nigiri::routing::para {

  void relativize_bin_and_push(std::vector<cmpnt_dep_event> const& dep_events,
                               timetable const& tt,
                               std::vector<size_t>& bin_start_idxs,
                               std::vector<relativized_cmpnt_dep_event>& rel_dep_events) {

    std::unordered_map<cmpnt_loc_idx_t, std::array<bitfield, 1440>> dep_loc_map;
    for (const auto& dep_event : dep_events) {
      auto& bfs = dep_loc_map[dep_event.dep_loc_];

      routing_time as_routing_time(tt, dep_event.dep_time_);

      const auto day_off = as_routing_time.day();
      const auto mam = static_cast<std::uint16_t>(as_routing_time.mam().count());

      bfs[mam].set(day_off.v_, true);
    }

    bin_start_idxs.push_back(rel_dep_events.size());
    for (const auto& [cmpnt_loc_idx, mam_arr] : dep_loc_map) {
      for (auto i = 0U; i < 1440; ++i) {
        if (mam_arr[i].none()) continue;

        rel_dep_events.emplace_back(
          minutes_after_midnight_t{i},
          0_minutes,
          cmpnt_loc_idx,
          cmpnt_loc_idx,
          mam_arr[i]
        );
      }
    }
  }

void push_bins(std_vecvec<cmpnt_dep_event> const& bins,
               timetable const& tt,
               std::vector<size_t>& bin_start_idxs,
               std::vector<bin_range_t>& cell_cmpnt_search_bins,
               std::vector<relativized_cmpnt_dep_event>& rel_dep_events) {
    const auto next_bin_start_idx = bin_start_idxs.size();
    size_t n_non_empty = 0U;
    for (const auto& bin : bins) {
      if (bin.empty()) continue;

      relativize_bin_and_push(bin, tt, bin_start_idxs, rel_dep_events);
      n_non_empty++;
    }
    cell_cmpnt_search_bins.emplace_back(next_bin_start_idx, next_bin_start_idx + n_non_empty);
  }

  void populate_start_times_for(bin_grouping_strategy const strategy,
                                component_idx_t cut_cmpnt_idx,
                                timetable const& tt,
                                bitvec const& route_mask,
                                std::vector<relativized_cmpnt_dep_event>& dep_events_buffer,
                                std::vector<size_t>& bin_start_idxs,
                                std::vector<bin_range_t>& cell_cmpnt_search_bins) {
    const auto& cmpnt_locations = tt.component_locations_[cut_cmpnt_idx];
    std_vecvec<cmpnt_dep_event> imm_dep_events(cmpnt_locations.size());
    get_cmpnt_dep_events(tt, cut_cmpnt_idx, route_mask,
                         imm_dep_events);

    for (auto& inner : imm_dep_events) {
      std::ranges::sort(inner, std::less{}, &cmpnt_dep_event::dep_time_);
      auto last = std::unique(inner.begin(), inner.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.dep_time_ == rhs.dep_time_;
      });
      inner.erase(last, inner.end());
    }

    if (strategy == bin_grouping_strategy::DIRECT ||
        strategy == bin_grouping_strategy::INITIAL_FOOTPATHS) {
          push_bins(imm_dep_events, tt, bin_start_idxs, cell_cmpnt_search_bins, dep_events_buffer);
    } else {
      // strategy == bin_grouping_strategy::SQUASHED
      std::vector<std::vector<cmpnt_dep_event>> bins;
      compress_start_times(tt, cut_cmpnt_idx, imm_dep_events, bins);
      push_bins(bins, tt, bin_start_idxs, cell_cmpnt_search_bins, dep_events_buffer);
    }
  }

  void populate_start_times_for_cell(bin_grouping_strategy const strategy,
                                     timetable const& tt,
                                     std::vector<component_idx_t> const& cut_cmpnts,
                                     bitvec const& route_mask,
                                     std::vector<relativized_cmpnt_dep_event>& dep_events_buffer,
                                     std::vector<size_t>& bin_start_idxs,
                                     std::vector<bin_range_t>& cmpnt_search_bins) {
    if (cut_cmpnts.empty()) {
      return;
    }

    for (const auto cut_cmpnt_idx : cut_cmpnts) {
      populate_start_times_for(strategy, cut_cmpnt_idx, tt, route_mask,
                               dep_events_buffer, bin_start_idxs, cmpnt_search_bins);
    }
    // sentinel
    bin_start_idxs.push_back(dep_events_buffer.size());
  }

  std::vector<std::vector<component_idx_t>> collect_cell_cut_cmpnts(timetable const& tt,
                                                                    size_t const n_of_cells,
                                                                    std_vecvec<cell_idx_t> const& cmpnt_to_cell_idxs) {
    std::vector<std::vector<component_idx_t>> cell_cut_cmpnts(n_of_cells);
    const auto n_of_components = tt.component_locations_.size();

    for (auto cmpnt_idx = component_idx_t{0U}; cmpnt_idx < n_of_components; ++cmpnt_idx) {
      const auto& cmpnt_cell_idxs = cmpnt_to_cell_idxs[cista::to_idx(cmpnt_idx)];
      if (cmpnt_cell_idxs.size() <= 1U) {
        continue;
      }
      for (const auto& cell_idx : cmpnt_cell_idxs) {
        cell_cut_cmpnts[to_idx(cell_idx)].push_back(cmpnt_idx);
      }
    }

    return cell_cut_cmpnts;
  }

  void start_times_registry::populate(bin_grouping_strategy const strategy,
                                      timetable const& tt,
                                      size_t const n_of_cells,
                                      std_vecvec<cell_idx_t> const& cmpnt_to_cell_idxs,
                                      std::vector<bitvec> const& route_masks) {
    auto const timer = scoped_timer("Populating departure events");
    resize(n_of_cells);
    clear();

    const auto cell_cut_cmpnts = collect_cell_cut_cmpnts(tt, n_of_cells, cmpnt_to_cell_idxs);
    for (auto cell_idx = 0U; cell_idx < n_of_cells; ++cell_idx) {
      populate_start_times_for_cell(strategy, tt, cell_cut_cmpnts[cell_idx], route_masks[cell_idx],
                                    rel_dep_events_buffer_[cell_idx], bin_start_idxs_[cell_idx],
                                    cell_cmpnt_search_bins_[cell_idx]);
    }
  }

  void start_times_registry::resize(size_t const n_of_cells) {
    rel_dep_events_buffer_.resize(n_of_cells);
    bin_start_idxs_.resize(n_of_cells);
    cell_cmpnt_search_bins_.resize(n_of_cells);
  }

  void start_times_registry::clear() {
    clear_all(rel_dep_events_buffer_.begin(), rel_dep_events_buffer_.end());
    clear_all(bin_start_idxs_.begin(), bin_start_idxs_.end());
    clear_all(cell_cmpnt_search_bins_.begin(), cell_cmpnt_search_bins_.end());
  }
} // namespace nigiri::routing::para