#pragma once

#include <immintrin.h>

#include <cstdint>

namespace nigiri {

/*
 * Takes two 256bit wide registers
 * a = [ a0 | a1 | a2 | a3 | a4 | a5 | a6 | a7 ]
 * b = [ b0 | b1 | b2 | b3 | b4 | b5 | b6 | b7 ]
 * where ai and bi are each 32bit. and returns c as
 * c = [ a0 <= b0 | a1 <= b1 | a2 <= b2 | a3 <= b3 | a4 <= b4 | a5 <= b5 | a6 <= b6 | a7 <= b7 ]
 * where true = 0xFFFFFFFF and false = 0x00000000
 */
inline __m256i cmp_le_epu32(const __m256i a, const __m256i b) {
  // If min(a, b) == a, then a <= b
  return _mm256_cmpeq_epi32(_mm256_min_epu32(a, b), a);
}

inline __m256i cmp_le_epi32(const __m256i a, const __m256i b) {
  // If min(a, b) == a, then a <= b
  return _mm256_cmpeq_epi32(_mm256_min_epi32(a, b), a);
}

/*
 * Takes two 256bit wide registers
 * a = [ a0 | a1 | a2 | a3 | a4 | a5 | a6 | a7 ]
 * b = [ b0 | b1 | b2 | b3 | b4 | b5 | b6 | b7 ]
 * where ai and bi are each 32bit. and returns c as
 * c = [ a0 >= b0 | a1 >= b1 | a2 >= b2 | a3 >= b3 | a4 >= b4 | a5 >= b5 | a6 >= b6 | a7 >= b7 ]
 * where true = 0xFFFFFFFF and false = 0x00000000
 */
inline __m256i cmp_ge_epu32(const __m256i a, const __m256i b) {
  return cmp_le_epu32(b, a);
}

inline __m256i cmp_ge_epi32(const __m256i a, const __m256i b) {
  return cmp_le_epi32(b, a);
}

inline __m256i cmp_lt_epi32(const __m256i a, __m256i b) {
  __m256i const all_ones = _mm256_set1_epi32(-1);
  return _mm256_andnot_si256(cmp_ge_epi32(a,b), all_ones);
}

/*
 * Takes two 256bit wide registers
 * a = [ a0 | a1 | ... | a14 | a15 ]
 * b = [ b0 | b1 | ... | b14 | b15 ]
 * where ai and bi are each 16bit. and returns c as
 * c = [ a0 <= b0 | a1 <= b1 | ... | a14 <= b14 | a15 <= b15 ]
 * where true = 0xFFFFFFFF and false = 0x00000000
 */
inline __m256i cmp_le_epu16(const __m256i a, const __m256i b) {
  // If min(a, b) == a, then a <= b
  return _mm256_cmpeq_epi16(_mm256_min_epu16(a, b), a);
}

/*
 * Takes two 256bit wide registers
 * a = [ a0 | a1 | ... | a14 | a15 ]
 * b = [ b0 | b1 | ... | b14 | b15 ]
 * where ai and bi are each 16bit. and returns c as
 * c = [ a0 >= b0 | a1 >= b1 | ... | a14 >= b14 | a15 <= b15 ]
 * where true = 0xFFFFFFFF and false = 0x00000000
 */
inline __m256i cmp_ge_epu16(const __m256i a, const __m256i b) {
  return cmp_le_epu16(b, a);
}

inline std::uint16_t movemask_epi16(const __m256i a) {
#if defined(__AVX512BW__) && defined(__AVX512VL__)
  // TARGET: Zen 4
  // Compiles ONLY if -march=znver4 or -mavx512bw is set.
  return static_cast<std::uint16_t>(_mm256_movepi16_mask(a));
#else
  // 1. Get 32 bits (2 bits per 16-bit lane)
  int const mask32 = _mm256_movemask_epi8(a);
  // 2. Extract every 2nd bit to compress to 16 bits
  // Caveat: This is extremely slow on zen 2 and lower
  return static_cast<std::uint16_t>(_pext_u32(static_cast<uint32_t>(mask32), 0xAAAAAAAA));
#endif
}

inline std::uint8_t movemask_epi32(const __m256i a) {
#if defined(__AVX512BW__) && defined(__AVX512VL__)
  // TARGET: Zen 4
  // Compiles ONLY if -march=znver4 or -mavx512bw is set.
  return static_cast<std::uint8_t>(_mm256_movepi32_mask(a));
#else
  // For every 32bit block: Take the msb and store it in the first
  // 8 bits of mask_dominates_new. The cast is necessary because this
  // operation assumes single-precision floating point numbers.
  int const mask_dominates_candidate = _mm256_movemask_ps(_mm256_castsi256_ps(a));
  return static_cast<std::uint8_t>(mask_dominates_candidate);
#endif
}

} // namespace nigiri
