#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri {

using adjacency_matrix = std::vector<bitvec>;

bitvec tomita(adjacency_matrix const& g, bitvec const& vertices);

std::vector<bitvec> clique_cover(adjacency_matrix const& matrix);

vecvec<clique_idx_t, location_idx_t> clique_cover(timetable const& tt);

} // namespace nigiri
