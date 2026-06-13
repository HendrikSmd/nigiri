#include "nigiri/location_match_mode.h"

#include <string_view>

namespace nigiri::routing {

std::string_view location_match_mode_str(location_match_mode const mode) {
  using namespace std::literals;
  switch (mode) {
    case location_match_mode::kExact: return "exact"sv;
    case location_match_mode::kOnlyChildren: return "only_children"sv;
    case location_match_mode::kEquivalent: return "equivalent"sv;
    case location_match_mode::kIntermodal: return "intermodal"sv;
  }
  std::unreachable();
}

} // namespace nigiri::routing
