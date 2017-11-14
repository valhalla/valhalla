#include "loki/worker.h"
#include "loki/search.h"
#include <functional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <sstream>

#include "baldr/json.h"
#include "baldr/pathlocation.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/connectivity_map.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

json::MapPtr serialize(const boost::optional<std::string>& id, const PathLocation& location, GraphReader& reader, bool verbose, bool istransit) {
    //serialze all the edges
    auto json = json::map
    ({
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}},
    });
    json->emplace("istransit", istransit);
    return json;
  }

  json::MapPtr serialize(const boost::optional<std::string>& id, const PointLL& ll, const std::string& reason, bool verbose, bool istransit) {
    auto json = json::map({
      {"input_lat", json::fp_t{ll.lat(), 6}},
      {"input_lon", json::fp_t{ll.lng(), 6}},
    });
    json->emplace("istransit", istransit);
    if(verbose)
      json->emplace("reason", reason);

    return json;
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_transit_available(rapidjson::Document& request) {
      //strip off unused information
      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{120};
        locations = parse_locations(request, "locations", 130, valhalla_exception_t{112});
        if(request.HasMember("costing"))
          parse_costing(request);

    }

    json::ArrayPtr loki_worker_t::transit_available(rapidjson::Document& request) {
      init_transit_available(request);

      auto json = json::array({});
      bool verbose = GetOptionalFromRapidJson<bool>(request, "/verbose").get_value_or(false);
      auto radius = GetFromRapidJson<float>(request, "/radius", default_radius);
      auto id = GetOptionalFromRapidJson<std::string>(request, "/id");
      std::unordered_map<size_t, size_t> color_counts;
      try{
        // correlate the various locations to the underlying graph
        const auto projections = loki::Search(locations, reader, edge_filter, node_filter);
        const auto transit_level = TileHierarchy::levels().rbegin()->second.level + 1;
        const auto& tiles = TileHierarchy::levels().find(TileHierarchy::levels().rbegin()->first)->second.tiles;

        if (transit_level == 3) {
          for (const auto& location : locations) {
            for (const auto& edge : projections.at(location).edges) {
              // Get a list of tiles required within the radius of the projected point
              PointLL ll = edge.projected;
              DistanceApproximator approximator(ll);
              float latdeg = (radius / kMetersPerDegreeLat);
              float lngdeg = (radius
                  / DistanceApproximator::MetersPerLngDegree(ll.lat()));
              AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
                                  Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
              std::vector<int32_t> tilelist = tiles.TileList(bbox);
              for (auto id : tilelist) {
                auto transit_color = connectivity_map->get_color(GraphId(id));
                auto itr = color_counts.find(transit_color);
                if(itr == color_counts.cend())
                  color_counts[transit_color] = 1;
                else
                  ++itr->second;
              }
            }

            bool istransit = false;
            for(const auto& c : color_counts) {
              if (c.second != 0) {
                LOG_INFO("Transit is available! " + std::to_string(c.second));
                istransit = true;
                break;
              }
            }
            try {
              json->emplace_back(serialize(id, location, reader, verbose, istransit));
            } catch(const std::exception& e) {
              json->emplace_back(serialize(id, location.latlng_, "No data found for location", verbose, istransit));
            }
          }
        }
      } catch (const std::exception&) {
        throw valhalla_exception_t { 171 };
      }
      return json;
    }
  }
}
