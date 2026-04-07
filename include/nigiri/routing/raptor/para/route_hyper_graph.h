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
  using hedge_t = std::vector<hyper_graph_node_idx_t>;

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
    hyper_edges.resize(new_size);
    hedge_weights.resize(new_size);
  }

  void insert_node_into_hedge(hedge_t& hedge,
                              hyper_graph_node_idx_t const node_idx) {
    if (auto const insert_it = std::ranges::lower_bound(hedge, node_idx);
        insert_it == hedge.end() || *insert_it != node_idx) {
      hedge.insert(insert_it, node_idx);
    }
  }

  hyper_graph_node_idx_t to_node_idx(route_idx_t route_idx) {
    return hyper_graph_node_idx_t{to_idx(route_idx)};
  }

  hyper_graph_node_idx_t to_node_idx(footpath_idx_t fp_idx,
                                     timetable const& tt) {
    return hyper_graph_node_idx_t{tt.n_routes() + to_idx(fp_idx)};
  }


  void from(const timetable& tt) {
    hyper_edges.clear();
    node_weights.clear();
    hedge_weights.clear();

    auto const timer = scoped_timer{"build hyper graph from timetable"};

    vector_map<footpath_idx_t, std::pair<location_idx_t, location_idx_t>> undirected_fp_edges;
    std::vector<std::vector<footpath_idx_t>> loc_incident_fps(tt.n_locations());
    for (auto loc = location_idx_t{0U}; loc < tt.n_locations(); ++loc) {
      const auto loc_fps_out = tt.locations_.footpaths_out_[kDefaultProfile][loc];
      for (const auto& fp : loc_fps_out) {
        if (loc < fp.target()) {
          const auto next_index = footpath_idx_t{undirected_fp_edges.size()};
          loc_incident_fps[to_idx(loc)].push_back(next_index);
          loc_incident_fps[to_idx(fp.target())].push_back(next_index);
          undirected_fp_edges.emplace_back(loc, fp.target());
        }
      }
    }

    // ==========================
    // build hyper edges
    // --------------------------
    build_hyper_edges_uncontracted(tt, loc_incident_fps);


    // ==========================
    // write node weights
    // --------------------------
    auto const n_nodes =
        tt.n_routes() + undirected_fp_edges.size();
    node_weights.resize(n_nodes, 1U);
    for (auto route_idx = route_idx_t{0}; route_idx < tt.n_routes();
         ++route_idx) {
      node_weights[route_idx.v_] = tt.n_events_for_route(route_idx);
    }
    // This leaves nodes representing footpaths with
    // a unary weight


    // ==========================
    // build hyper edge weights
    // --------------------------
    build_hyper_edge_weights_uncontracted(tt, loc_incident_fps);


    // ==========================
    // merge hyper edges that relate
    // to the same connections
    // --------------------------
    sort_hedges_lexicographically();
    merge_duplicates();
    // normalizing must happen after merging duplicate hedges
    // to avoid wrong weight scaling (log(x) + log(y) = log(x * y))
    normalize_hedge_weights();
    if (!hyper_edges.empty() && hyper_edges[0].empty()) {
      hyper_edges.erase(hyper_edges.begin());
      hedge_weights.erase(hedge_weights.begin());
    }
  }

  void normalize_hedge_weights() {
    for (auto& hedge_weight : hedge_weights) {
      // The base 2 logarithm of an unsigned is essentially its highest set bit position
      hedge_weight = std::max(static_cast<size_t>(std::bit_width<size_t>(hedge_weight)), static_cast<size_t>(1)) - 1;
    }
  }

  void export_as_hmetis(const std::filesystem::path& out,
                        bool export_node_weights,
                        bool export_hedge_weights) const {
    std::ofstream out_file(out, std::ofstream::out);

    auto const timer = scoped_timer{"write hyper graph to output file"};
    // ==========================
    // write hmetis header [#edges #nodes 11]
    // the 11 signals both: edge and node weights
    // --------------------------
    out_file << hyper_edges.size() << " " << node_weights.size();
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
    //out_file << "% Hyper Edges:" << std::endl;
    for (size_t hedge_idx = 0U; hedge_idx < hyper_edges.size(); ++hedge_idx) {
      if (export_hedge_weights) {
        out_file << hedge_weights[hedge_idx] << " ";
      }
      auto& nodes = hyper_edges[hedge_idx];
      std::stringstream out_line;
      for (const auto& node : nodes) {
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
    //out_file << "% Node weights:" << std::endl;
    if (export_node_weights) {
      for (unsigned long node_weight : node_weights) {
        out_file << node_weight << std::endl;
      }
    }
    out_file.close();
  }

private:

  void build_hyper_edges_uncontracted(
      timetable const& tt,
      std::vector<std::vector<footpath_idx_t>> const& loc_incident_fps) {
    auto const n_locations = tt.n_locations();
    utl::verify(loc_incident_fps.size() == n_locations, "Unexpected dimension");

    for (auto location_idx = location_idx_t{0U}; location_idx < n_locations;
         ++location_idx) {

      hedge_t reachable_connections;

      auto const loc_routes = tt.location_routes_[location_idx];
      for (auto const route_idx : loc_routes) {
        insert_node_into_hedge(reachable_connections, to_node_idx(route_idx));
      }

      for (auto const incident_fp_idx :
           loc_incident_fps[to_idx(location_idx)]) {
        insert_node_into_hedge(reachable_connections,
                               to_node_idx(incident_fp_idx, tt));
      }
      hyper_edges.emplace_back(std::move(reachable_connections));
    }
  }

  void build_hyper_edge_weights_uncontracted(
      timetable const& tt,
      std::vector<std::vector<footpath_idx_t>> const& loc_incident_fps) {
    auto const n_locations = tt.n_locations();
    hedge_weights.resize(n_locations, 0);
    for (auto location_idx = location_idx_t{0}; location_idx < n_locations;
         ++location_idx) {
      size_t const events_at_location = tt.n_events_at_location(location_idx);
      hedge_weights[to_idx(location_idx)] =
          events_at_location +
          loc_incident_fps[to_idx(location_idx)].size();
    }
  }



  std::vector<hedge_t> hyper_edges;
  std::vector<node_value_t> node_weights;
  std::vector<hedge_value_t> hedge_weights;
};


}