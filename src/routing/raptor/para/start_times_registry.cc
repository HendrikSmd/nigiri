#include "nigiri/routing/raptor/para/start_times_registry.h"

namespace nigiri::routing::para {

  void populate_start_times_for(component_idx_t cut_cmpnt_idx,
                                timetable const& tt,
                                bitvec const& route_mask,
                                std::vector<cmpnt_dep_event>& dep_events_buffer,
                                std::vector<size_t>& bin_start_idxs,
                                std::vector<std::pair<size_t, size_t>>& cell_cmpnt_search_bins) {
    const auto& cmpnt_locations = tt.component_locations_[cut_cmpnt_idx];
    std::vector<std::vector<cmpnt_dep_event>> imm_dep_events(cmpnt_locations.size());
    get_cmpnt_dep_events(tt, cut_cmpnt_idx,
                         route_mask, imm_dep_events);

    size_t total_n_of_events = 0U;
    for (auto& inner : imm_dep_events) {
      std::ranges::sort(inner, std::less{}, &cmpnt_dep_event::dep_time_);
      auto last = std::unique(inner.begin(), inner.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.dep_time_ == rhs.dep_time_;
      });
      inner.erase(last, inner.end());
      total_n_of_events += inner.size();
    }

    dep_events_buffer.reserve(dep_events_buffer.size() + total_n_of_events);

    std::vector<std::vector<cmpnt_dep_event>> bins;
    compress_start_times(tt, cut_cmpnt_idx, imm_dep_events, bins);

    const auto next_bin_start_idx = bin_start_idxs.size();
    cell_cmpnt_search_bins.emplace_back(next_bin_start_idx, next_bin_start_idx + bins.size());
    for (const auto& bin : bins) {
      bin_start_idxs.push_back(dep_events_buffer.size());
      dep_events_buffer.insert(dep_events_buffer.end(), bin.begin(), bin.end());
    }
  }

  std::vector<std::vector<component_idx_t>> collect_cell_cut_cmpnts(timetable const& tt,
                                                                    size_t const n_of_cells,
                                                                    std::vector<std::vector<cell_idx_t>> const& cmpnt_to_cell_idxs) {
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

  void start_times_registry::populate(timetable const& tt,
                                      size_t const n_of_cells,
                                      std::vector<std::vector<cell_idx_t>> const& cmpnt_to_cell_idxs,
                                      std::vector<bitvec> const& route_masks) {
    auto const timer = scoped_timer("populating departure events");

    const auto cell_cut_cmpnts = collect_cell_cut_cmpnts(tt, n_of_cells, cmpnt_to_cell_idxs);
    for (auto cell_idx = 0U; cell_idx < n_of_cells; ++cell_idx) {
      const auto& cut_cmpnts = cell_cut_cmpnts[cell_idx];
      for (const auto cut_cmpnt_idx : cut_cmpnts) {
        populate_start_times_for(cut_cmpnt_idx, tt, route_masks[cista::to_idx(cell_idx)],
                                 dep_events_buffer_, bin_start_idxs_, cell_cmpnt_search_bins_);
      }
    }
    // sentinel
    bin_start_idxs_.push_back(dep_events_buffer_.size());
  }

  void start_times_registry::clear() {
    dep_events_buffer_.clear();
    bin_start_idxs_.clear();
    cell_cmpnt_search_bins_.clear();
  }



}