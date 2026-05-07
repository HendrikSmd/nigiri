#pragma once

#include "nigiri/common/apply_permutation.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

#include <algorithm>
#include <bit>
#include <fstream>
#include <vector>

#include "utl/timer.h"
#include "utl/verify.h"

namespace nigiri::routing {

enum class hedge_weighting : std::uint8_t { kNumRoutes, kNumEvents };

enum class hedge_normalization : std::uint8_t {
  kNone,
  kLog,
  kCmpntSize,
  kLogCmpntSize
};

struct route_hyper_graph {

  using node_value_t = size_t;
  using hedge_value_t = size_t;
  using hedge_t = std::vector<route_idx_t>;

  void sort_hedges_lexicographically() {
    utl::verify(hyper_edges.size() == hedge_weights.size(),
                "hyper edges must have 1:1 correspondence to its weights");

    std::vector<size_t> permutation(hyper_edges.size());
    std::iota(permutation.begin(), permutation.end(), 0);

    std::sort(permutation.begin(), permutation.end(),
              [&](size_t const idx1, size_t const idx2) {
                return hyper_edges[idx1] < hyper_edges[idx2];
              });

    apply_permutation(hyper_edges, permutation);
    apply_permutation(hedge_weights, permutation);
  }

  void merge_duplicate_hedges() {
    size_t w_idx = 0U;
    size_t n_hedges = hyper_edges.size();
    for (size_t r_idx = 1U; r_idx < n_hedges; ++r_idx) {

      if (hyper_edges[r_idx] == hyper_edges[w_idx]) {

        // merge weights of duplicate hyper edge
        hedge_weights[w_idx] += hedge_weights[r_idx];
      } else {
        w_idx++;

        if (w_idx != r_idx) {
          hyper_edges[w_idx] = std::move(hyper_edges[r_idx]);
          hedge_weights[w_idx] = hedge_weights[r_idx];
        }
      }
    }

    size_t new_size = w_idx + 1;
    hyper_edges.erase(hyper_edges.begin() + new_size, hyper_edges.end());
    hedge_weights.erase(hedge_weights.begin() + new_size, hedge_weights.end());
  }

  void insert_route_into_hedge(route_idx_t const r_idx, hedge_t& hedge) {
    if (auto const insert_it = std::ranges::lower_bound(hedge, r_idx);
        insert_it == hedge.end() || *insert_it != r_idx) {
      hedge.insert(insert_it, r_idx);
    }
  }

  void from(timetable const& tt, hedge_weighting const weighting_scheme,
            hedge_normalization const normalization) {
    hyper_edges.clear();
    node_weights.clear();
    hedge_weights.clear();

    auto const timer = scoped_timer{"build hyper graph from timetable"};
    // ==========================
    // write hyper edges
    // --------------------------
    auto const n_components = tt.component_locations_.size();
    for (auto component_idx = component_idx_t{0}; component_idx < n_components;
         ++component_idx) {

      auto const& locs_in_component = tt.component_locations_[component_idx];
      hedge_t reachable_routes;
      for (auto const& loc : locs_in_component) {
        auto const& loc_routes = tt.location_routes_[loc];
        for (auto const& r_idx : loc_routes) {
          insert_route_into_hedge(r_idx, reachable_routes);
        }
      }

      hyper_edges.emplace_back(std::move(reachable_routes));
    }

    // ==========================
    // write node weights
    // --------------------------
    auto const n_routes = tt.n_routes();
    n_nodes = n_routes;
    node_weights.resize(n_routes, 0);
    for (auto route_idx = route_idx_t{0}; route_idx < n_routes; ++route_idx) {
      node_weights[route_idx.v_] = tt.n_events_for_route(route_idx);
    }

    // ==========================
    // write hyper edge weights
    // --------------------------
    hedge_weights.resize(n_components, 0);
    for (auto component_idx = component_idx_t{0}; component_idx < n_components;
         ++component_idx) {
      auto const& locs_in_cmpnt = tt.component_locations_[component_idx];
      size_t total_weight = 0;
      for (auto const loc : locs_in_cmpnt) {
        if (weighting_scheme == hedge_weighting::kNumEvents) {
          total_weight += tt.n_events_at_location(loc);
        } else if (weighting_scheme == hedge_weighting::kNumRoutes) {
          total_weight += tt.n_routes_at_location(loc);
        }
      }
      if (normalization == hedge_normalization::kCmpntSize ||
          normalization == hedge_normalization::kLogCmpntSize) {
        hedge_weights[to_idx(component_idx)] =
            total_weight / locs_in_cmpnt.size();
      } else {
        hedge_weights[to_idx(component_idx)] = total_weight;
      }
    }

    // ==========================
    // merge hyper edges that relate
    // to the same routes
    // --------------------------
    sort_hedges_lexicographically();
    merge_duplicate_hedges();
    // normalizing must happen after merging duplicate hedges
    // to avoid wrong weight scaling (log(x) + log(y) = log(x * y))
    if (normalization == hedge_normalization::kLog ||
        normalization == hedge_normalization::kLogCmpntSize) {
      log_normalize_hedge_weights();
    }
    if (!hyper_edges.empty() && hyper_edges[0].empty()) {
      hyper_edges.erase(hyper_edges.begin());
      hedge_weights.erase(hedge_weights.begin());
    }
  }

  void log_normalize_hedge_weights() {
    for (auto& hedge_weight : hedge_weights) {
      // The base 2 logarithm of an unsigned is essentially its highest set bit
      // position
      hedge_weight =
          std::max(static_cast<size_t>(std::bit_width<size_t>(hedge_weight)),
                   static_cast<size_t>(1));
    }
  }

  void export_as_hmetis(std::filesystem::path const& out,
                        bool export_node_weights,
                        bool export_hedge_weights) const {
    std::ofstream out_file(out, std::ofstream::out);

    auto const timer = scoped_timer{"write hyper graph to output file"};
    // ==========================
    // write hmetis header [#edges #nodes 11]
    // the 11 signals both: edge and node weights
    // --------------------------
    out_file << hyper_edges.size() << " " << n_nodes;
    if (export_node_weights && export_hedge_weights) {
      out_file << " 11";
    } else if (export_node_weights) {
      out_file << " 10";
    } else if (export_hedge_weights) {
      out_file << " 1";
    }
    out_file << std::endl;

    // ==========================
    // write one line per hyper edge
    //   [#weight {node_idx}*]
    // --------------------------
    // out_file << "% Hyper Edges:" << std::endl;
    for (size_t hedge_idx = 0U; hedge_idx < hyper_edges.size(); ++hedge_idx) {
      if (export_hedge_weights) {
        out_file << hedge_weights[hedge_idx] << " ";
      }
      auto& nodes = hyper_edges[hedge_idx];
      std::stringstream out_line;
      for (auto const& node : nodes) {
        // hmetis starts node indexing at 1
        out_line << node + 1 << " ";
      }
      out_line.seekp(-1, std::ios::cur);
      out_line << std::endl;
      out_file << out_line.str();
    }

    // ==========================
    // write one line for node weight
    // --------------------------
    // out_file << "% Node weights:" << std::endl;
    if (export_node_weights) {
      for (size_t node_idx = 0U; node_idx < n_nodes; ++node_idx) {
        out_file << node_weights[node_idx] << std::endl;
      }
    }
    out_file.close();
  }

  size_t n_nodes;
  std::vector<hedge_t> hyper_edges;
  std::vector<node_value_t> node_weights;
  std::vector<hedge_value_t> hedge_weights;
};

}  // namespace nigiri::routing