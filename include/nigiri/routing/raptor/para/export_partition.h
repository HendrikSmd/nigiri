#pragma once

#include "boost/json.hpp"

#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/timetable.h"

#include <string>

#include "utl/enumerate.h"

namespace nigiri::routing::para {

inline boost::json::array to_array(geo::latlng const& coord) { return {coord.lng(), coord.lat()}; }

inline bool has_distinct(const std::vector<cell_idx_t>& cells) {
  if (cells.size() < 2) {
    return false;
  }

  const auto& first = cells.front();
  for (size_t i = 1U; i < cells.size(); ++i) {
    if (cells[i] != first) {
      return true;
    }
  }

  return false;
}

inline std::optional<boost::json::object> location_to_feature(timetable const& tt,
                                               route_partition const& rtp,
                                               location_idx_t const l_idx) {
  auto cmpnt_idx = tt.location_component_[l_idx];
  auto const& global_cell = rtp.cmpnt_to_cell_idx_[cmpnt_idx];
  if (global_cell == route_partition::global_cell_idx::invalid()) {
    return std::nullopt;
  }

  boost::json::array cell_idx_on_levels(rtp.n_levels_ + 1, boost::json::value{-1});
  auto current_lvl = global_cell.level_;
  auto current_cell_idx = global_cell.cell_idx_;
  while (current_lvl <= rtp.n_levels_) {
    cell_idx_on_levels[current_lvl] = boost::json::value{to_idx(current_cell_idx)};
    current_cell_idx = route_partition::get_parent_idx(current_cell_idx);
    current_lvl++;
  }

  return boost::json::object{
    {"type", "Feature"},
    {"geometry",
      boost::json::object{
        {"type", "Point"},
        {"coordinates", to_array(tt.locations_.coordinates_[l_idx])}
      }
    },
    {"properties",
      boost::json::object{
        {"cell_idxs", cell_idx_on_levels}
      }
    }
  };
}

inline void emplace_features(timetable const& tt,
                            route_partition const& rtp,
                            boost::json::array& features) {
  for (auto l_idx = location_idx_t{0}; l_idx < tt.n_locations(); ++l_idx) {
    if (auto const opt_feature = location_to_feature(tt, rtp, l_idx);
        opt_feature.has_value()) {
      features.emplace_back(opt_feature.value());
    }
  }
}

inline std::string to_featurecollection(timetable const& tt, route_partition const& rtp) {
  boost::json::array features;
  emplace_features(tt, rtp, features);
  return boost::json::serialize(boost::json::object{
    {"type", "FeatureCollection"},
    {"features", features}
  });
}

inline size_t estimate_chars(size_t const n) {
  if (n == 0) return 1;
  return static_cast<size_t>(std::log10(n)) + 1;
}

inline void append_links(route_partition const& rtp, std::string& out) {
  size_t const n_of_cells = 1U << rtp.n_levels_;
  constexpr std::string_view link_template = "C{}_{} -> C{}_{};\n";
  for (size_t level = rtp.n_levels_; level > 0; --level) {
    size_t const n_of_cells_in_level = n_of_cells >> level;
    for (size_t cell_idx = 0U; cell_idx < n_of_cells_in_level; ++cell_idx) {
      size_t child_idx = cell_idx << 1U;
      out.append(fmt::format(link_template, cell_idx, level, child_idx, level - 1));
      out.append(fmt::format(link_template, cell_idx, level, child_idx + 1, level - 1));
    }
  }
}

std::vector<size_t> count_routes_on_level(timetable const& tt,
                                          route_partition const& rtp,
                                          uint8_t const level) {
  std::vector<size_t> n_of_routes_in_cells(rtp.get_num_of_cells_on_level(level), 0U);
  for (auto r_idx = route_idx_t{0}; r_idx < tt.n_routes(); ++r_idx) {
    n_of_routes_in_cells[to_idx(rtp.get_cell_of_route(r_idx, level))]++;
  }
  return n_of_routes_in_cells;
}

std::string to_graphviz(timetable const& tt, route_partition const& rtp) {
  size_t n_of_cells = rtp.get_num_of_cells_on_level(0);
  std::string graph_repr("digraph BinaryTree {\nnode [shape=record];\nedge [arrowsize=0.8];\n");
  append_links(rtp, graph_repr);

  std::string nodes_repr;
  constexpr std::string_view node_template =
      "C{} [label=\"<f0> #routes: {} |<f1> #cmpnts: {} |<f2> #cut cmpnts: {} "
      "|<f2> #cut locations: {} \"]\n";


  auto n_components = tt.component_locations_.size();
  std::vector<std::vector<cell_idx_t>> component_cell_idxs;
  for (auto c = component_idx_t{0}; c < n_components; ++c) {
    std::vector<cell_idx_t> cell_idxs;
    for (const auto& l : tt.component_locations_[c]) {
      const auto& routes_of_loc = tt.location_routes_[l];

      std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
        return rtp.route_to_cell_idx_[r];
      });
    }
    std::ranges::sort(cell_idxs);
    auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
    cell_idxs.erase(last, cell_idxs.end());
    component_cell_idxs.emplace_back(cell_idxs);
  }


  vector_map<cell_idx_t, size_t> internal_cmpnt_cell_counts(n_of_cells, 0U);
  vector_map<cell_idx_t, size_t> cut_cmpnt_cell_counts(n_of_cells, 0U);
  vector_map<cell_idx_t, size_t> cut_location_cell_counts(n_of_cells, 0U);
  for (size_t level = 0U; level <= rtp.n_levels_; ++level) {

    const auto n_routes_per_cell = count_routes_on_level(tt, rtp, level);
    for (const auto [cmpnt_idx, cell_idxs] : utl::enumerate(component_cell_idxs)) {
      if (cell_idxs.size() == 1) {
        internal_cmpnt_cell_counts[cell_idxs.front()]++;
        continue;
      }

      auto const cmpnt_locations =
          tt.component_locations_[component_idx_t{cmpnt_idx}];

      for (const auto& cell_idx : cell_idxs) {
        cut_cmpnt_cell_counts[cell_idx]++;
        cut_location_cell_counts[cell_idx] += cmpnt_locations.size();
      }
    }

    for (auto cell_idx = cell_idx_t{0}; cell_idx < n_of_cells; ++cell_idx) {
      std::string ident = fmt::format("{}_{}", cell_idx, level);
      nodes_repr.append(fmt::format(node_template,
        ident,
        n_routes_per_cell[to_idx(cell_idx)],
        internal_cmpnt_cell_counts[cell_idx],
        cut_cmpnt_cell_counts[cell_idx],
        cut_location_cell_counts[cell_idx])
      );
    }

    n_of_cells >>= 1U;
    internal_cmpnt_cell_counts.resize(n_of_cells);
    std::ranges::fill(internal_cmpnt_cell_counts, 0U);
    cut_cmpnt_cell_counts.resize(n_of_cells);
    std::ranges::fill(cut_cmpnt_cell_counts, 0U);
    cut_location_cell_counts.resize(n_of_cells);
    std::ranges::fill(cut_location_cell_counts, 0U);
    for (auto& cell_idxs : component_cell_idxs) {
      std::ranges::transform(cell_idxs, cell_idxs.begin(), [](cell_idx_t const& cell_idx) {
        return route_partition::get_parent_idx(cell_idx);
      });
      auto last = std::unique(cell_idxs.begin(), cell_idxs.end());
      cell_idxs.erase(last, cell_idxs.end());
    }
  }

  graph_repr.append(nodes_repr);
  graph_repr.append("}\n");
  return graph_repr;
}



}