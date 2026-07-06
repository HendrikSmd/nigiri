#pragma once

#include <immintrin.h>

#include "nigiri/types.h"
#include "route_partition.h"

namespace nigiri::routing::para {

inline rank_t LCL(cell_idx_t const route_cell, para::route_partition::global_cell_idx const g_cell) {
  return rank_t{g_cell.level_ + static_cast<uint8_t>(std::bit_width<uint16_t>(g_cell.cell_idx_.v_ ^ (route_cell.v_ >> g_cell.level_)))};
}

// AVX512 implementation (processes 32 elements at a time)
inline void compute_min_lcls_avx512(
    std::vector<rank_t>& min_lcls,
    vector_map<route_idx_t, cell_idx_t> const& route_to_cell_idx,
    route_partition::global_cell_idx const g_cell_start,
    route_partition::global_cell_idx const g_cell_dest,
    route_idx_t::value_t const n_routes
) {
    min_lcls.resize(n_routes);
    auto r_idx = route_idx_t{0U};

#if defined(__AVX512F__) && defined(__AVX512VL__)

    const __m512i zero = _mm512_setzero_si512();
    const __m512i cell_start_vec = _mm512_set1_epi16(to_idx(g_cell_start.cell_idx_));
    const __m512i cell_dest_vec = _mm512_set1_epi16(to_idx(g_cell_dest.cell_idx_));
    const __m512i level_start_16 = _mm512_set1_epi16(g_cell_start.level_);
    const __m512i level_dest_16 = _mm512_set1_epi16(g_cell_dest.level_);

    const __m512i sixteen_vec = _mm512_set1_epi16(16);

    for (; r_idx + 31 < n_routes; r_idx += 32) {
        // Load 32 route cells
        __m512i route_cells = _mm512_loadu_si512((const __m512i*)&route_to_cell_idx[r_idx]);

        // Compute for g_cell_start
        __m512i shifted_start = _mm512_srlv_epi16(route_cells, level_start_16);
        __m512i xor_start = _mm512_xor_si512(shifted_start, cell_start_vec);

        // Compute for g_cell_dest
        __m512i shifted_dest = _mm512_srlv_epi16(route_cells, level_dest_16);
        __m512i xor_dest = _mm512_xor_si512(shifted_dest, cell_dest_vec);

        // Compute bit_width = 16 - lzcnt for 16-bit values
        __m512i lzcnt_start = _mm512_lzcnt_epi16(xor_start);
        __m512i lzcnt_dest = _mm512_lzcnt_epi16(xor_dest);

        __m512i bit_width_start = _mm512_sub_epi16(sixteen_vec, lzcnt_start);
        __m512i bit_width_dest = _mm512_sub_epi16(sixteen_vec, lzcnt_dest);

        // Add level (maintain in 16-bit for saturation)
        __m512i lcl_start_16 = _mm512_add_epi16(level_start_16, bit_width_start);
        __m512i lcl_dest_16 = _mm512_add_epi16(level_dest_16, bit_width_dest);

        // Compute min in 16-bit
        __m512i min_vals_16 = _mm512_min_epu16(lcl_start_16, lcl_dest_16);

        // Pack to bytes (saturating)
        __m512i min_vals = _mm512_packus_epi16(min_vals_16, zero);

        // Store results (only lower 32 bytes)
        _mm512_storeu_si512((__m512i*)&min_lcls[to_idx(r_idx)], min_vals);
    }
#endif

    // Handle remainder with scalar code
    for (; r_idx < n_routes; ++r_idx) {
        min_lcls[to_idx(r_idx)] = std::min(
            LCL(route_to_cell_idx[r_idx], g_cell_start),
            LCL(route_to_cell_idx[r_idx], g_cell_dest)
        );
    }
}


} // namespace nigiri::routing::para