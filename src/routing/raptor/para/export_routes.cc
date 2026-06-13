#include "nigiri/routing/raptor/para/export_routes.h"

#include <boost/json.hpp>

void nigiri::routing::para::export_routes(timetable const& tt,
                                          std::ostream& out,
                                          plain_route_rank_store const& store) {
  boost::json::array features;

  for (auto route_idx = route_idx_t{0U}; route_idx < tt.n_routes(); ++route_idx) {
    auto const& stop_seq = tt.route_location_seq_[route_idx];
    if (stop_seq.size() < 2) {
      continue;
    }

    boost::json::array coordinates;
    for (auto const stop_val : stop_seq) {
      stop const s{stop_val};
      auto const loc_idx = s.location_idx();
      auto const& coord = tt.locations_.coordinates_[loc_idx];
      coordinates.emplace_back(boost::json::array{coord.lng(), coord.lat()});
    }

    auto const rank = store.route_ranks_[route_idx];

    boost::json::object properties;
    properties["route_idx"] = to_idx(route_idx);
    properties["rank"] = to_idx(rank);

    boost::json::object geometry;
    geometry["type"] = "LineString";
    geometry["coordinates"] = std::move(coordinates);

    boost::json::object feature;
    feature["type"] = "Feature";
    feature["geometry"] = std::move(geometry);
    feature["properties"] = std::move(properties);

    features.emplace_back(std::move(feature));
  }

  boost::json::object geojson;
  geojson["type"] = "FeatureCollection";
  geojson["features"] = std::move(features);

  out << boost::json::serialize(geojson);
}
