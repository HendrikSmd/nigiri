#pragma once

#include <cassert>

#include <vector>

namespace nigiri {

template <typename T>
struct simple_flat_matrix {
  using value_type = T;
  using size_type = std::size_t;

  simple_flat_matrix() = default;

  simple_flat_matrix(size_type const n_rows, size_type const n_columns)
      : n_rows_(n_rows), n_columns_(n_columns), entries_(n_rows * n_columns) {}

  // Mimic the Cista "row" helper for matrix[i][j] access
  struct row {
    row(simple_flat_matrix& matrix, size_type const i)
        : matrix_(matrix), i_(i) {}

    T& operator[](size_type const j) {
      assert(j < matrix_.n_columns_);
      return matrix_.entries_[i_ * matrix_.n_columns_ + j];
    }

    auto begin() {
      return matrix_.entries_.begin() + (i_ * matrix_.n_columns_);
    }
    auto end() {
      return matrix_.entries_.begin() + ((i_ + 1) * matrix_.n_columns_);
    }

    simple_flat_matrix& matrix_;
    size_type i_;
  };

  struct const_row {
    const_row(simple_flat_matrix const& matrix, size_type const i)
        : matrix_(matrix), i_(i) {}

    T const& operator[](size_type const j) const {
      assert(j < matrix_.n_columns_);
      return matrix_.entries_[i_ * matrix_.n_columns_ + j];
    }

    auto begin() const {
      return matrix_.entries_.begin() + (i_ * matrix_.n_columns_);
    }
    auto end() const {
      return matrix_.entries_.begin() + ((i_ + 1) * matrix_.n_columns_);
    }

    simple_flat_matrix const& matrix_;
    size_type i_;
  };

  row operator[](size_type const i) {
    assert(i < n_rows_);
    return {*this, i};
  }

  const_row operator[](size_type const i) const {
    assert(i < n_rows_);
    return {*this, i};
  }

  void resize(size_type n_rows, size_type n_columns) {
    n_rows_ = n_rows;
    n_columns_ = n_columns;
    entries_.assign(n_rows * n_columns, T{});  // assign ensures clean state
  }

  // Required for the loop in bmc_raptor_state::reset()
  auto begin() { return entries_.begin(); }
  auto end() { return entries_.end(); }
  auto begin() const { return entries_.begin(); }
  auto end() const { return entries_.end(); }

  size_type n_rows_{0};
  size_type n_columns_{0};
  std::vector<T> entries_;
};

}  // namespace nigiri