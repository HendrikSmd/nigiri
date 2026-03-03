#pragma once

#include <immintrin.h>

#include <concepts>

#include "nigiri/types.h"

#include <array>
#include <vector>

namespace nigiri::routing::para {

template <typename T>
concept fixed_width_ints =
    std::same_as<T, std::int8_t> || std::same_as<T, std::uint8_t> ||
    std::same_as<T, std::int16_t> || std::same_as<T, std::uint16_t> ||
    std::same_as<T, std::int32_t> || std::same_as<T, std::uint32_t> ||
    std::same_as<T, std::int64_t> || std::same_as<T, std::uint64_t>;

template <fixed_width_ints F, size_t B, size_t N, typename M>
struct pareto_element_view {
  std::array<F, N> fields_;
  bitset<B> const& bitset_;
  M const& metadata_;
};

template <fixed_width_ints F, size_t B, size_t N, typename M>
struct simd_pareto_bag {
  using field_type = F;
  using field_vector = std::vector<field_type>;
  using mask_t = std::conditional_t<
      sizeof(field_type) == 1, std::uint32_t,
      std::conditional_t<sizeof(field_type) == 2, std::uint16_t, std::uint8_t>>;

  static constexpr size_t lane_count = 256 / (sizeof(field_type) * 8);
  static constexpr size_t lane_count_minus_one = lane_count - 1;
  static constexpr float compaction_threshold_ = 0.0F;

  size_t dead_count_ = 0;

  std::array<field_vector, N> fields_;
  std::vector<bitset<B>> bitsets_;

  // Meta data
  std::vector<M> meta_data_;

  struct const_iterator {
    using iterator_category = std::forward_iterator_tag;
    using value_type = pareto_element_view<F, B, N, M>;
    using difference_type = std::ptrdiff_t;
    using pointer = void;
    using reference = value_type;

    simd_pareto_bag const* bag_;
    size_t index_;

    const_iterator() = default;
    const_iterator(simd_pareto_bag const* bag, size_t index)
        : bag_{bag}, index_{index} {}

    // Dereference: Assembles the SoA data into a view
    reference operator*() const {
      std::array<F, N> label;
      for (size_t i = 0; i < N; ++i) {
        label[i] = bag_->fields_[i][index_];
      }
      return {label, bag_->bitsets_[index_], bag_->meta_data_[index_]};
    }

    const_iterator& operator++() {
      index_++;
      // Skip "dead" elements that haven't been compacted yet
      while (index_ < bag_->bitsets_.size() && bag_->bitsets_[index_].none()) {
        index_++;
      }
      return *this;
    }

    bool operator==(const_iterator const& other) const {
      return index_ == other.index_;
    }
    bool operator!=(const_iterator const& other) const { return !(*this == other); }

    const_iterator(const_iterator const&) = default;
    const_iterator& operator=(const_iterator const& other) = default;
  };

  const_iterator begin() const {
    size_t i = 0;
    while (i < bitsets_.size() && bitsets_[i].none()) i++;
    return {this, i};
  }

  const_iterator end() const { return {this, bitsets_.size()}; }

  template <typename D>
  bool add(std::array<field_type, N> const& candidate,
           bitset<B> new_bits,
           M meta_data) {
    using dominance_relation = D;

    size_t i = 0U;
    size_t const len = bitsets_.size();

    while (i < len) {
      if (i + len < len) {
        std::array<__m256i, N> candidate_broadcasted{};
        std::array<__m256i, N> existing_loaded{};
        for (size_t field_idx = 0; field_idx < N; field_idx++) {
          candidate_broadcasted[field_idx] = broadcast(candidate[field_idx]);
          existing_loaded[field_idx] = load(&fields_[field_idx][i]);
        }

        // 1. Check which existing labels dominate the new label
        mask_t mask_dominates_candidate = dominance_relation::dominates_candidate(
            candidate_broadcasted, existing_loaded);

        if (mask_dominates_candidate != 0) {
          // at least one stored label dominates the new one
          for (size_t j = 0; j < lane_count; j++) {
            if (mask_dominates_candidate & (1 << j)) {
              new_bits &= ~bitsets_[i + j];
            }
            if (new_bits.none()) {
              return false;
            }
          }
        }

        // 2. Check which existing labels are dominated by the new label
        mask_t mask_dominates_existing = dominance_relation::dominates_existing(
            candidate_broadcasted, existing_loaded);
        if (mask_dominates_existing != 0) {
          // at least one stored label  is dominated by the new one
          for (size_t j = 0; j < lane_count; j++) {
            if (bitsets_[i + j].none()) {
              continue;
            }
            if (mask_dominates_existing & (1 << j)) {
              bitsets_[i + j] &= ~new_bits;
            }
            if (bitsets_[i + j].none()) {
              dead_count_++;
            }
          }
        }

        i += lane_count;
      } else {
        for (; i < len; ++i) {
          if (bitsets_[i].none()) {
            continue;
          }
          if (dominance_relation::scalar_dominates_candidate(candidate, get_label_at(i))) {
            new_bits &= ~bitsets_[i];
            if (new_bits.none()) {
              return false;
            }
          } else if (dominance_relation::scalar_dominates_existing(candidate, get_label_at(i))) {
            bitsets_[i] &= ~new_bits;
            if (bitsets_[i].none()) {
              dead_count_++;
            }
          }
        }
      }
    }

    meta_data_.push_back(meta_data);
    bitsets_.push_back(new_bits);
    for (size_t field_idx = 0U; field_idx < N; field_idx++) {
      fields_[field_idx].push_back(candidate[field_idx]);
    }

    // Trigger compaction if threshold reached

    if constexpr (compaction_threshold_ == 0.0) {
      compact();
    } else {
      if (bitsets_.size() > 0 && static_cast<double>(dead_count_) / static_cast<double>(bitsets_.size()) >
          compaction_threshold_) {
        compact();
    }
    }
    return true;
  }

  template <typename D>
  void filter_dominated(std::array<field_type, N> candidate,
                        bitset<B>& candidate_bits) const {
    using dominance_relation = D;
    size_t const len = bitsets_.size();
    size_t i = 0U;

    while (i < len) {
      if (i + len < len) {
        std::array<__m256i, N> existing_loaded{};
        std::array<__m256i, N> candidate_broadcasted{};
        for (size_t field_idx = 0; field_idx < N; field_idx++) {
          candidate_broadcasted[field_idx] = broadcast(candidate[field_idx]);
          existing_loaded[field_idx] = load(&fields_[field_idx][i]);
        }

        // 1. Check which existing labels dominate the new label
        mask_t mask_dominates_candidate =
            dominance_relation::dominates_candidate(candidate_broadcasted,existing_loaded);
        if (mask_dominates_candidate != 0) {
          // at least one stored label dominates the new one
          for (size_t j = 0; j < lane_count; j++) {
            if (mask_dominates_candidate & (1 << j)) {
              candidate_bits &= ~bitsets_[i + j];
            }
            if (candidate_bits.none()) {
              return;
            }
          }
        }
        i += lane_count;
      } else {
        for (; i < len; ++i) {
          if (candidate_bits.none()) {
            return;
          }
          if (dominance_relation::scalar_dominates_candidate(candidate, get_label_at(i))) {
            candidate_bits &= ~bitsets_[i];
          }
        }
      }
    }
  }

  size_t size() const { return bitsets_.size(); }

  void clear() {
    clear_fields();
    meta_data_.clear();
    bitsets_.clear();

    dead_count_ = 0U;
  }

private:
  template <fixed_width_ints T>
  static inline __m256i broadcast(T val) {
    if constexpr (sizeof(T) == 1)
      return _mm256_set1_epi8(static_cast<char>(val));
    else if constexpr (sizeof(T) == 2)
      return _mm256_set1_epi16(static_cast<short>(val));
    else if constexpr (sizeof(T) == 4)
      return _mm256_set1_epi32(static_cast<int>(val));
    else if constexpr (sizeof(T) == 8)
      return _mm256_set1_epi64x(static_cast<long long>(val));
  }

  template <fixed_width_ints T>
  static inline __m256i load(T const* ptr) {
    return _mm256_loadu_si256(reinterpret_cast<__m256i const*>(ptr));
  }

  void clear_fields() {
    for (size_t i = 0U; i < N; i++) {
      fields_[i].clear();
    }
  }

  inline std::array<field_type, N> get_label_at(size_t i) const {
    std::array<field_type, N> label;
    for (size_t field_idx = 0; field_idx < N; ++field_idx) {
      label[field_idx] = fields_[field_idx][i];
    }
    return label;
  }

  inline void move_fields(size_t const write_idx, size_t const read_idx) {
    for (size_t field_idx = 0; field_idx < N; ++field_idx) {
      fields_[field_idx][write_idx] = fields_[field_idx][read_idx];
    }
  }

  inline void resize_fields(size_t new_size) {
    for (size_t field_idx = 0; field_idx < N; ++field_idx) {
      fields_[field_idx].resize(new_size);
    }
  }

  inline void set_fields(size_t const i,
                         std::array<field_type, N> const& values) {
    for (size_t field_idx = 0; field_idx < N; ++field_idx) {
      fields_[field_idx][i] = values[field_idx];
    }
  }

  void compact() {
    size_t write_idx = 0U;
    for (size_t read_idx = 0U; read_idx < size(); ++read_idx) {
      if (bitsets_[read_idx].any()) {
        if (read_idx != write_idx) {
          move_fields(write_idx, read_idx);
          meta_data_[write_idx] = meta_data_[read_idx];
          bitsets_[write_idx] = bitsets_[read_idx];
        }
        write_idx++;
      }
    }

    resize_fields(write_idx);
    bitsets_.resize(write_idx);
    meta_data_.resize(write_idx);
    dead_count_ = 0;
  }

};

}  // namespace nigiri::routing::para
