#pragma once

#include "nigiri/types.h"

namespace nigiri::routing::para {

template <typename T>
struct bmc_raptor_bag {

  struct indexed_label {
    indexed_label() = default;
    indexed_label(T label, search_bitfield const tdb)
        : label_(label),
          tdb_(tdb) {}

    T label_;
    search_bitfield tdb_;
  };

  using iterator = typename std::vector<indexed_label>::iterator;
  using const_iterator = typename std::vector<indexed_label>::const_iterator;

  template <auto dominates>
  bool add(T label, search_bitfield tdb) {
    auto n_removed = std::size_t{0};
    for (size_t i = 0U; i < labels_.size(); ++i) {
      auto& el = labels_[i];
      if (dominates(el.label_, label)) {
        tdb &= ~el.tdb_;
        if (!tdb.any()) {
          return false;
        }
      }
      else if (dominates(label, el.label_)) {
        el.tdb_ &= ~tdb;
        if (!el.tdb_.any()) {
          n_removed++;
          continue;
        }
      }
      labels_[i - n_removed] = labels_[i];
    }
    labels_.resize(labels_.size() - n_removed + 1);
    labels_.back() = indexed_label{label, tdb};
    return true;
  }

  template <auto dominates>
  bool add_careful(T label, search_bitfield tdb) {
    for (size_t i = 0U; i < labels_.size(); ++i) {
      const auto& el = labels_[i];
      if (dominates(el.label_, label)) {
        tdb &= ~el.tdb_;
        if (!tdb.any()) {
          return false;
        }
      }
    }
    labels_.resize(labels_.size() + 1);
    labels_.back() = indexed_label{label, tdb};
    return true;
  }

  template <auto dominates>
  void filter_dominated(const T& label, search_bitfield& tdb) const {
    for (const auto& l : labels_) {
      if (tdb.none()) {
        return;
      }
      if (dominates(l.label_, label)) {
        tdb &= ~l.tdb_;
      }
    }
  }

  friend const_iterator begin(bmc_raptor_bag const& s) { return s.begin(); }
  friend const_iterator end(bmc_raptor_bag const& s) { return s.end(); }
  friend iterator begin(bmc_raptor_bag& s) { return s.begin(); }
  friend iterator end(bmc_raptor_bag& s) { return s.end(); }
  iterator begin() { return labels_.begin(); }
  iterator end() { return labels_.end(); }
  const_iterator begin() const { return labels_.begin(); }
  const_iterator end() const { return labels_.end(); }
  iterator erase(iterator const& it) { return labels_.erase(it); }
  iterator erase(iterator const& from, iterator const& to) {return labels_.erase(from, to);}
  void clear() { labels_.clear(); }
  std::size_t size() const { return labels_.size(); }

  std::vector<indexed_label> labels_;
};


} // nigiri::routing::raptor::para