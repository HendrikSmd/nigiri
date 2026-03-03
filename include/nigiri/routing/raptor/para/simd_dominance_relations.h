#pragma once

#include <immintrin.h>

#include <cstddef>
#include <cstdint>

#include "nigiri/common/simd_utility.h"

#include <array>

namespace nigiri::routing::para {

// field 0: Departure
// field 1: Arrival
// field 2: Arrival with Transfer time
struct bicriteria_conservative_dominance_16 {
  using field_type = std::uint16_t;
  using mask_t = std::uint16_t;
  static constexpr std::size_t lane_count = 256 / (sizeof(field_type) * 8);

  static mask_t dominates_candidate(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_candidate = _mm256_and_si256(
        cmp_ge_epu16(existing_broadcasted[0], candidate_duplicated[0]),
        cmp_le_epu16(existing_broadcasted[2], candidate_duplicated[1]));
    return static_cast<mask_t>(movemask_epi16(dominates_candidate));
  }

  static bool scalar_dominates_candidate(
      std::array<field_type, 3> const& candidate,
      std::array<field_type, 3> const& existing) {
    return existing[0] >= candidate[0] && existing[2] <= candidate[1];
  }

  static mask_t dominates_existing(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_existing = _mm256_and_si256(
        cmp_ge_epu16(candidate_duplicated[0], existing_broadcasted[0]),
        cmp_le_epu16(candidate_duplicated[2], existing_broadcasted[1]));
    return static_cast<mask_t>(movemask_epi16(dominates_existing));
  }

  static bool scalar_dominates_existing(
      std::array<field_type, 3> const& candidate,
      std::array<field_type, 3> const& existing) {
    return candidate[0] >= existing[0] && candidate[2] <= existing[1];
  }
};

// field 0: Departure
// field 1: Arrival
// field 2: Arrival with Transfer time
struct bicriteria_dominance_16 {
  using field_type = std::uint16_t;
  using mask_t = std::uint16_t;
  static constexpr std::size_t lane_count = 256 / (sizeof(field_type) * 8);

  static mask_t dominates_candidate(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_candidate = _mm256_and_si256(
        cmp_ge_epu16(existing_broadcasted[0], candidate_duplicated[0]),
        cmp_le_epu16(existing_broadcasted[2], candidate_duplicated[2]));
    return static_cast<mask_t>(movemask_epi16(dominates_candidate));
  }

  static bool scalar_dominates_candidate(
      std::array<field_type, 3> const& candidate,
      std::array<field_type, 3> const& existing) {
    return existing[0] >= candidate[0] && existing[2] <= candidate[2];
  }

  static mask_t dominates_existing(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_existing = _mm256_and_si256(
        cmp_ge_epu16(candidate_duplicated[0], existing_broadcasted[0]),
        cmp_le_epu16(candidate_duplicated[2], existing_broadcasted[2]));
    return static_cast<mask_t>(movemask_epi16(dominates_existing));
  }

  static bool scalar_dominates_existing(
      std::array<field_type, 3> const& candidate,
      std::array<field_type, 3> const& existing) {
    return candidate[0] >= existing[0] && candidate[2] <= existing[2];
  }
};

// field 0: Departure
// field 1: transport_idx
// field 2: transport_day_offset (treat uint32_t as int32_t)
struct boarded_transport_dominance_32 {
  using field_type = std::uint32_t;
  using mask_t = std::uint8_t;
  static constexpr std::size_t lane_count = 256 / (sizeof(field_type) * 8);

  static mask_t dominates_candidate(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_candidate = _mm256_and_si256(
        cmp_ge_epu32(existing_broadcasted[0], candidate_duplicated[0]),
        _mm256_or_si256(
          cmp_lt_epi32(existing_broadcasted[2], candidate_duplicated[2]),
          _mm256_and_si256(
            _mm256_cmpeq_epi32(existing_broadcasted[2], candidate_duplicated[2]),
            cmp_le_epu32(existing_broadcasted[1], candidate_duplicated[1])
          )
        )
    );
    return static_cast<mask_t>(movemask_epi32(dominates_candidate));
  }

  static bool scalar_dominates_candidate(
      std::array<std::uint32_t, 3> const& candidate,
      std::array<std::uint32_t, 3> const& existing) {
    return existing[0] >= candidate[0] &&
          (static_cast<std::int32_t>(existing[2]) < static_cast<std::int32_t>(candidate[2]) ||
           (existing[2] == candidate[2] && existing[1] <= candidate[1]));
  }

  static mask_t dominates_existing(
      std::array<__m256i, 3> const& candidate_duplicated,
      std::array<__m256i, 3> const& existing_broadcasted) {
    __m256i const dominates_existing = _mm256_and_si256(
        cmp_ge_epu32(candidate_duplicated[0], existing_broadcasted[0]),
        _mm256_or_si256(
          cmp_lt_epi32(candidate_duplicated[2], existing_broadcasted[2]),
          _mm256_and_si256(
            _mm256_cmpeq_epi32(candidate_duplicated[2], existing_broadcasted[2]),
            cmp_le_epu32(candidate_duplicated[1], existing_broadcasted[1])
          )
        )
    );
    return static_cast<mask_t>(movemask_epi32(dominates_existing));
  }

  static bool scalar_dominates_existing(
      std::array<std::uint32_t, 3> const& candidate,
      std::array<std::uint32_t, 3> const& existing) {
    return candidate[0] >= existing[0] &&
          (static_cast<std::int32_t>(candidate[2]) < static_cast<std::int32_t>(existing[2]) ||
           (candidate[2] == existing[2] && candidate[1] <= existing[1]));
  }
};

}  // namespace nigiri::routing::para