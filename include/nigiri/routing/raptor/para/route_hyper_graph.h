#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"
#include "utl/verify.h"
#include "utl/timer.h"

#include <vector>
#include <bit>
#include <algorithm>
#include <fstream>

namespace nigiri::routing {

struct route_hyper_graph {

  using node_value_t = size_t;
  using hedge_value_t = size_t;
  using hedge_t = std::vector<route_idx_t>;

  template<typename T>
  void apply_permutation(std::vector<T>& data, const std::vector<size_t>& permutation) {
    std::vector<T> shuffled(data.size());
    for (size_t i = 0U; i < permutation.size(); ++i) {
      shuffled[i] = std::move(data[permutation[i]]);
    }

    data = std::move(shuffled);
  }

  void sort_hedges_lexicographically() {
    utl::verify(hyper_edges.size() == hedge_weights.size(),
      "hyper edges must have 1:1 correspondence to its weights"
    );

    std::vector<size_t> permutation(hyper_edges.size());
    std::iota(permutation.begin(), permutation.end(), 0);

    std::sort(permutation.begin(), permutation.end(), [&](size_t const idx1, size_t const idx2) {
      return hyper_edges[idx1] < hyper_edges[idx2];
    });

    apply_permutation(hyper_edges, permutation);
    apply_permutation(hedge_weights, permutation);
  }

  void merge_duplicates() {
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

  void insert_route_into_hedge(hedge_t& hedge, route_idx_t const r_idx) {
    if (auto const insert_it = std::ranges::lower_bound(hedge, r_idx);
        insert_it == hedge.end() || *insert_it != r_idx) {
      hedge.insert(insert_it, r_idx);
    }
  }


  void from(const timetable& tt) {
    hyper_edges.clear();
    node_weights.clear();
    hedge_weights.clear();

    auto const timer = scoped_timer{"build hyper graph from timetable"};
    // ==========================
    // write hyper edges
    // --------------------------
    const auto n_components = tt.component_locations_.size();
    for (auto component_idx = component_idx_t{0}; component_idx < n_components; ++component_idx) {

      const auto& locs_in_component = tt.component_locations_[component_idx];
      hedge_t reachable_routes;
      for (const auto& loc : locs_in_component) {
        const auto& loc_routes = tt.location_routes_[loc];
        for (const auto& r_idx : loc_routes) {
          insert_route_into_hedge(reachable_routes, r_idx);
        }
      }

      hyper_edges.emplace_back(std::move(reachable_routes));
    }

    // ==========================
    // write node weights
    // --------------------------
    const auto n_routes = tt.n_routes();
    n_nodes = n_routes;
    node_weights.resize(n_routes, 0);
    for (auto route_idx = route_idx_t{0}; route_idx < n_routes; ++route_idx) {
      node_weights[route_idx.v_] = tt.n_events_for_route(route_idx);
    }

    // ==========================
    // write hyper edge weights
    // --------------------------
    hedge_weights.resize(n_components, 0);
    for (auto component_idx = component_idx_t{0}; component_idx < n_components; ++component_idx) {
      auto const& locs_in_cmpnt = tt.component_locations_[component_idx];
      size_t events_in_component = 0;
      for (auto const& loc : locs_in_cmpnt) {
        events_in_component += tt.n_events_at_location(loc);
      }
      // The base 2 logarithm of an unsigned is essentially its highest set bit position
      hedge_weights[component_idx.v_] = std::max(std::bit_width<size_t>(events_in_component / locs_in_cmpnt.size()), 1) - 1;
    }

    // ==========================
    // merge hyper edges that relate
    // to the same routes
    // --------------------------
    sort_hedges_lexicographically();
    merge_duplicates();
    if (!hyper_edges.empty() && hyper_edges[0].empty()) {
      hyper_edges.erase(hyper_edges.begin());
      hedge_weights.erase(hedge_weights.begin());
    }
  }

  void export_as_hmetis(const std::filesystem::path& out) {
    std::ofstream out_file(out, std::ofstream::out);

    auto const timer = scoped_timer{"write hyper graph to output file"};
    // ==========================
    // write hmetis header [#edges #nodes 11]
    // the 11 signals both: edge and node weights
    // --------------------------
    out_file << hyper_edges.size() << " " << n_nodes << " 11" << std::endl;

    // ==========================
    // write one line per hyper edge
    //   [#weight {node_idx}*]
    // --------------------------
    out_file << "% Hyper Edges:" << std::endl;
    for (size_t hedge_idx = 0U; hedge_idx < hyper_edges.size(); ++hedge_idx) {
      out_file << hedge_weights[hedge_idx];
      const auto& nodes = hyper_edges[hedge_idx];
      for (const auto& node : nodes) {
        out_file << " " << node;
      }
      out_file << std::endl;
    }

    // ==========================
    // write one line for node weight
    // --------------------------
    out_file << "% Node weights:" << std::endl;
    for (size_t node_idx = 0U; node_idx < n_nodes; ++node_idx) {
      out_file << node_weights[node_idx] << std::endl;
    }
    out_file.close();
  }


  size_t n_nodes;
  std::vector<hedge_t> hyper_edges;
  std::vector<node_value_t> node_weights;
  std::vector<hedge_value_t> hedge_weights;
};


}