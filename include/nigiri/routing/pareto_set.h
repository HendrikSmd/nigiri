#pragma once

#include <cinttypes>
#include <tuple>
#include <vector>
#include "utl/helpers/algorithm.h"

namespace nigiri {

template<typename T>
struct pareto_utils {
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  template<typename Compare>
  static std::tuple<bool, iterator, iterator> pareto_add(std::vector<T>& set, T el, Compare dominates) {
    auto n_removed = std::size_t{0};
    for (auto i = 0U; i < set.size(); ++i) {
      if (dominates(set[i], el)) {
        return {false, set.end(), std::next(set.begin(), i)};
      }
      if (dominates(el, set[i])) {
        n_removed++;
        continue;
      }
      if (n_removed > 0U) {
        set[i - n_removed] = set[i];
      }
    }
    set.resize(set.size() - n_removed + 1);
    set.back() = std::move(el);
    return {true, std::next(set.begin(), static_cast<unsigned>(set.size() - 1)),
            set.end()};
  }
};

template <typename T>
struct pareto_set {
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  size_t size() const { return els_.size(); }
  bool empty() const { return els_.empty(); }

  bool is_dominated(T const& el) const {
    return utl::any_of(els_, [&](T const& x) { return x.dominates(el); });
  }

  template <auto dominates>
  bool is_dominated(T const& el) const {
    return utl::any_of(els_, [&](T const& x) { return dominates(x, el); });
  }

  std::tuple<bool, iterator, iterator> add(T el) {
    return pareto_utils<T>::pareto_add(els_, std::move(el), [](const T& a, const T& b) { return T::dominates(a, b); });
  }

  template <auto dominates>
  std::tuple<bool, iterator, iterator> add(T el) {
    return pareto_utils<T>::pareto_add(els_, std::move(el), [](const T& a, const T& b) { return dominates(a, b); });
  }

  void add_not_optimal(T j) { els_.emplace_back(std::move(j)); }

  friend const_iterator begin(pareto_set const& s) { return s.begin(); }
  friend const_iterator end(pareto_set const& s) { return s.end(); }
  friend iterator begin(pareto_set& s) { return s.begin(); }
  friend iterator end(pareto_set& s) { return s.end(); }
  iterator begin() { return els_.begin(); }
  iterator end() { return els_.end(); }
  const_iterator begin() const { return els_.begin(); }
  const_iterator end() const { return els_.end(); }
  iterator erase(iterator const& it) { return els_.erase(it); }
  iterator erase(iterator const& from, iterator const& to) {
    return els_.erase(from, to);
  }
  void clear() { els_.clear(); }

  std::vector<T> els_;
};

}  // namespace nigiri
