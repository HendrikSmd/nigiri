#pragma once

#include <cstddef>

#include <optional>
#include <queue>
#include <vector>

#include "absl/strings/str_format.h"
#include "utl/init_from.h"

namespace nigiri {

template <typename iter_t,
          typename compare_t = std::greater<typename iter_t::value_type::value_type>>
class k_way_merge {
  using container_t = iter_t::value_type;
  using T = container_t::value_type;
  using T_iter = container_t::const_iterator;
private:

  struct element_t {
    T_iter val_it_;
    T_iter end_it_;
    const compare_t* comp_;

    bool operator>(const element_t& other) const {
      return (*comp_)(*other.val_it_, *val_it_);
    }
  };

  iter_t begin_range_, end_range_;
  compare_t comparator_;

public:
  k_way_merge(iter_t begin, iter_t end, compare_t comp)
    : begin_range_(begin),
      end_range_(end),
      comparator_(comp) {}

  struct iterator {

    using reference = const T&;
    using pointer   = const T*;

    std::priority_queue<element_t, std::vector<element_t>, std::greater<element_t>> min_heap_;

    iterator(iter_t begin, iter_t end, const compare_t& comp) {
        for (auto it = begin; it != end; ++it) {
          if (!it->empty()) {
            min_heap_.push(element_t{it->begin(), it->end(), &comp});
          }
        }
    }

    iterator() = default;

    reference operator*() const {
      return *(min_heap_.top().val_it_);
    }

    iterator& operator++() {
      element_t top = min_heap_.top();
      min_heap_.pop();

      ++top.val_it_;
      if (top.val_it_ != top.end_it_) {
        min_heap_.push(top);
      }
      return *this;
    }

    bool operator!=(const iterator& other) const {
      return this->min_heap_.empty() != other.min_heap_.empty();
    }
  };

  iterator begin() { return iterator(begin_range_, end_range_, comparator_); }
  iterator end() { return iterator(); }

};

}