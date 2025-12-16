#pragma once

#include "nigiri/routing/raptor/para/route_partition.h"
#include "nigiri/timetable.h"
#include "boost/json.hpp"

#include <string>


namespace nigiri::routing::para {

inline boost::json::array to_array(geo::latlng const& coord) { return {coord.lng(), coord.lat()}; }

inline bool has_distinct(const std::vector<cell_idx_t>& cells) {
  if (cells.size() < 2) {
    return false;
  }

  const auto& first = cells.front();
  for (size_t i = 1U; i < cells.size(); ++i) {
    if (cells[i] != first) {
      return true;
    }
  }

  return false;
}

inline boost::json::object location_to_feature(timetable const& tt,
                                               route_partition const& rtp,
                                               location_idx_t const l_idx) {
  std::vector<cell_idx_t> cell_idxs;
  const auto& routes_of_loc = tt.location_routes_[l_idx];
  std::ranges::transform(routes_of_loc, std::back_inserter(cell_idxs), [&](const route_idx_t& r) {
    return rtp.route_to_cell_idx_[r];
  });


  return boost::json::object{
    {"type", "Feature"},
    {"geometry",
      boost::json::object{
        {"type", "Point"},
        {"coordinates", to_array(tt.locations_.coordinates_[l_idx])}
      }
    },
    {"properties",
      boost::json::object{
        {"cell_idx", has_distinct(cell_idxs) ?
          boost::json::value{-1} :
          boost::json::value{cista::to_idx(cell_idxs.front())}
        }
      }
    }
  };
}

inline void emplace_features(timetable const& tt,
                            route_partition const& rtp,
                            boost::json::array& features) {
  for (auto l_idx = location_idx_t{0}; l_idx < tt.n_locations(); ++l_idx) {
    if (tt.location_routes_[l_idx].empty()) {
      continue;
    }
    features.emplace_back(location_to_feature(tt, rtp, l_idx));
  }
}

inline std::string to_featurecollection(timetable const& tt, route_partition const& rtp) {
  boost::json::array features;
  emplace_features(tt, rtp, features);
  return boost::json::serialize(boost::json::object{
    {"type", "FeatureCollection"},
    {"features", features}
  });
}



}