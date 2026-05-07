#pragma once

#include <vector>

#include "utl/verify.h"

namespace nigiri {

template <typename T>
void apply_permutation(std::vector<T>& data,
                       std::vector<size_t> const& permutation) {
  utl::verify(data.size() == permutation.size(), "vector size mismatch");
  std::vector<T> shuffled(data.size());
  for (size_t i = 0U; i < permutation.size(); ++i) {
    shuffled[i] = std::move(data[permutation[i]]);
  }

  data = std::move(shuffled);
}

}
