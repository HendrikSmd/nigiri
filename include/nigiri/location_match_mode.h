#pragma once

#include <string_view>

namespace nigiri::routing {

enum class location_match_mode {
  kExact,  // only use exactly the specified location
  kOnlyChildren,  // use also children (tracks at this location)
  kEquivalent,  // use equivalent locations (includes children)
  kIntermodal  // use coordinate
};

std::string_view location_match_mode_str(location_match_mode mode);

}  // namespace nigiri::routing
