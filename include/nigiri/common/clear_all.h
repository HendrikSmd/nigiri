#pragma once

#include <iterator>

namespace nigiri {

template<typename T>
concept Clearable = requires(T a) {
  a.clear();
};

template<std::input_iterator It>
void clear_all(It first, It last) {
  for (; first != last; ++first) {
    if constexpr (Clearable<std::iter_value_t<It>>) {
      first->clear();
    }
  }
}

}