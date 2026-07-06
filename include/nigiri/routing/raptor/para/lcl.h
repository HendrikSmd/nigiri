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
    auto r_idx = 0U;

#if defined(__AVX512F__) && defined(__AVX512VL__)
    uint8_t* out_ptr = reinterpret_cast<uint8_t*>(min_lcls.data());

    const __m512i cell_start_32 = _mm512_set1_epi32(static_cast<short>(to_idx(g_cell_start.cell_idx_)));
    const __m512i cell_dest_32 = _mm512_set1_epi32(static_cast<short>(to_idx(g_cell_dest.cell_idx_)));
    const __m512i level_start_32 = _mm512_set1_epi32(g_cell_start.level_);
    const __m512i level_dest_32 = _mm512_set1_epi32(g_cell_dest.level_);

    const __m512i thirtytwo_v = _mm512_set1_epi32(32);

    for (; r_idx + 31 < n_routes; r_idx += 32) {
        // Load 32 route cells
        const void* raw_route_ptr = static_cast<const void*>(&route_to_cell_idx[route_idx_t{r_idx}]);
        __m512i route_cells = _mm512_loadu_si512(reinterpret_cast<const __m512i*>(raw_route_ptr));


        __m512i routes_lo_32 = _mm512_cvtepu16_epi32(_mm512_castsi512_si256(route_cells));
        __m512i routes_hi_32 = _mm512_cvtepu16_epi32(_mm512_extracti64x4_epi64(route_cells, 1));

        auto compute_lcl_512 = [&](__m512i v_route, __m512i v_level, __m512i v_idx) {
            __m512i shift   = _mm512_srlv_epi32(v_route, v_level);
            __m512i xor_res = _mm512_xor_si512(v_idx, shift);
            __m512i lzcnt   = _mm512_lzcnt_epi32(xor_res);
            __m512i bw      = _mm512_sub_epi32(thirtytwo_v, lzcnt);
            return _mm512_add_epi32(v_level, bw);
        };
        __m512i min_lo = _mm512_min_epu32(
            compute_lcl_512(routes_lo_32, level_start_32, cell_start_32),
            compute_lcl_512(routes_lo_32, level_dest_32, cell_dest_32)
        );
        __m512i min_hi = _mm512_min_epu32(
            compute_lcl_512(routes_hi_32, level_start_32, cell_start_32),
            compute_lcl_512(routes_hi_32, level_dest_32, cell_dest_32)
        );

        void* raw_out_ptr_lo = static_cast<void*>(&out_ptr[r_idx]);
        void* raw_out_ptr_hi = static_cast<void*>(&out_ptr[r_idx + 16]);
        _mm_storeu_si128(reinterpret_cast<__m128i*>(raw_out_ptr_lo), _mm512_cvtepi32_epi8(min_lo));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(raw_out_ptr_hi), _mm512_cvtepi32_epi8(min_hi));
    }
#endif
    // Handle remainder with scalar code
    for (; r_idx < n_routes; ++r_idx) {
        min_lcls[r_idx] = std::min(
            LCL(route_to_cell_idx[route_idx_t{r_idx}], g_cell_start),
            LCL(route_to_cell_idx[route_idx_t{r_idx}], g_cell_dest)
        );
    }
}


} // namespace nigiri::routing::para