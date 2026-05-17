#include "nigiri/common/clique.h"
#include "nigiri/timetable.h"

#include <future>

#include "utl/enumerate.h"
#include "utl/verify.h"

namespace nigiri {

namespace {

struct component_clique_cover {
  component_idx_t component_;
  std::vector<bitvec> cliques_;
};

std::vector<cmpnt_loc_idx_t> get_location_to_component_idx_lookup(
    timetable const& timetable) {
  std::vector res(timetable.n_locations(), cmpnt_loc_idx_t::invalid());
  const auto n_components = timetable.component_locations_.size();

  for (auto c = component_idx_t{0U}; c < n_components; ++c) {
    const auto& c_locs = timetable.component_locations_[c];
    for (const auto [idx, loc] : utl::enumerate(c_locs)) {
      res[to_idx(loc)] = cmpnt_loc_idx_t{idx};
    }
  }
  return res;
}

adjacency_matrix build_component_footgraph(
    timetable const& tt,
    component_idx_t const idx,
    std::vector<cmpnt_loc_idx_t> const& lookup) {
  auto const& component_locations = tt.component_locations_[idx];
  auto const n_locations_in_component = component_locations.size();

  adjacency_matrix matrix(n_locations_in_component,
                          bitvec{n_locations_in_component});

  for (auto const [i, loc] : utl::enumerate(component_locations)) {
    for (auto const& fp : tt.locations_.footpaths_out_[kDefaultProfile][loc]) {
      matrix[i].set(to_idx(lookup[to_idx(fp.target())]), true);
    }
  }

  return matrix;
}

size_t select_pivot(bitvec const& P,
                    bitvec const& X,
                    adjacency_matrix const& g) {
  size_t pivot = 0;
  size_t max_neighbors = 0;
  bool assigned = false;
  (P | X).for_each_set_bit([&](size_t const i) {
    size_t const n_neighbors = (P & g[i]).count();
    if (!assigned || n_neighbors > max_neighbors) {
      max_neighbors = n_neighbors;
      pivot = i;
      assigned = true;
    }
  });
  return pivot;
}

void tomita_worker(adjacency_matrix const& g,
                   bitvec& R, bitvec& P, bitvec& X,
                   bitvec& max_clique, size_t& max_clique_size) {

  // Base Case: If both P and X are empty, R is a maximal clique
  if (P.none() && X.none()) {
    size_t current_size = R.count();
    if (current_size > max_clique_size) {
      max_clique_size = current_size;
      max_clique = R;
    }
    return;
  }

  // Pruning Bound: Can this branch even beat our current max?
  if (R.count() + P.count() <= max_clique_size) {
    return;
  }

  // 1. Choose the pivot using Tomita's heuristic
  size_t const u = select_pivot(P, X, g);

  // 2. Generate candidates to explore: P \ N(u)
  bitvec remaining = P & ~g[u];

  // 3. Iterate over the remaining candidate vertices
  remaining.for_each_set_bit([&](unsigned int const v) {
    // Make the recursive call step
    R.set(v);
    bitvec next_P = P & g[v];
    bitvec next_X = X & g[v];

    tomita_worker(g, R, next_P, next_X, max_clique, max_clique_size);

    // Backtrack steps
    R.set(v, false);  // Remove v from current clique
    P.set(v, false);  // Move v from P...
    X.set(v, true);  // ...to X
  });
}

}

bitvec tomita(adjacency_matrix const& g, bitvec const& vertices) {
  utl::verify(g.size() == vertices.size(), "Not the same number of vertices");
  auto const n_vertices = static_cast<std::uint32_t>(g.size());

  if (vertices.none()) {
    return bitvec(n_vertices);
  }

  bitvec max_clique(n_vertices);
  size_t max_clique_size = 0;

  bitvec R(n_vertices);
  bitvec P(vertices);
  bitvec X(n_vertices);

  tomita_worker(g, R, P, X, max_clique, max_clique_size);

  return max_clique;
}

std::vector<bitvec> clique_cover(adjacency_matrix const& matrix) {
  std::vector<bitvec> res;

  bitvec not_covered = bitvec::max(matrix.size());
  while (not_covered.any()) {
    bitvec max_clique = tomita(matrix, not_covered);

    // Safety fallback (e.g., isolated node with no footpaths)
    if (max_clique.none()) {
      // Grab the first remaining vertex and make it its own standalone hub
      max_clique.set(not_covered.next_set_bit(0U).value(), true);
    }

    // Save the clique
    res.push_back(max_clique);

    // 3. Mask out the covered vertices so they aren't considered in the next run
    not_covered &= ~max_clique;
  }

  return res;
}

vecvec<clique_idx_t, location_idx_t> clique_cover(timetable const& tt) {
  const auto lookup = get_location_to_component_idx_lookup(tt);
  const auto n_components = tt.component_locations_.size();

  std::vector<component_clique_cover> collected;
  collected.reserve(n_components);

  for (auto c = component_idx_t{0U}; c < n_components; ++c) {
      auto const matrix = build_component_footgraph(tt, c, lookup);
      auto cliques = clique_cover(matrix);
      collected.emplace_back(c, std::move(cliques));
  }

  std::ranges::sort(collected, std::less{},
                    &component_clique_cover::component_);

  vecvec<clique_idx_t, location_idx_t> res;
  std::vector<location_idx_t> enumerated_clique_buffer;
  for (auto c = component_idx_t{0U}; c < n_components; ++c) {

    const auto& component_locations = tt.component_locations_[c];

    const auto& component_cliques = collected[to_idx(c)];
    for (const auto& clique : component_cliques.cliques_) {
      enumerated_clique_buffer.resize(clique.count(),
                                      location_idx_t::invalid());
      auto idx = 0U;
      clique.for_each_set_bit(
          [&enumerated_clique_buffer, &component_locations, &idx](size_t const i) {
            enumerated_clique_buffer[idx++] = component_locations[i];
          });

      res.emplace_back(enumerated_clique_buffer);
      enumerated_clique_buffer.resize(0U);
    }
  }
  return res;
}

}
