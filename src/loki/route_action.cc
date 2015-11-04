#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>

using namespace prime_server;
using namespace valhalla::baldr;

namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  void check_locations(const size_t location_count, const size_t max_locations) {
    //check that location size does not exceed max.
    if (location_count > max_locations)
      throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations) + ".");
  }

  void check_distance(const GraphReader& reader, const std::vector<Location>& locations, float max_distance){
    //see if any locations pairs are unreachable or too far apart
    auto lowest_level = reader.GetTileHierarchy().levels().rbegin();
    for(auto location = ++locations.cbegin(); location != locations.cend(); ++location) {

      //check connectivity
      uint32_t a_id = lowest_level->second.tiles.TileId(std::prev(location)->latlng_);
      uint32_t b_id = lowest_level->second.tiles.TileId(location->latlng_);
      if(!reader.AreConnected({a_id, lowest_level->first, 0}, {b_id, lowest_level->first, 0}))
        throw std::runtime_error("Locations are in unconnected regions. Go check/edit the map at osm.org");

      //check if distance between latlngs exceed max distance limit for each mode of travel
      auto path_distance = std::prev(location)->latlng_.Distance(location->latlng_);
      max_distance-=path_distance;
      if (max_distance < 0)
        throw std::runtime_error("Path distance exceeds the max distance limit.");

      LOG_INFO("location_distance::" + std::to_string(path_distance));
    }
  }
}

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::route(const ACTION_TYPE& action, boost::property_tree::ptree& request) {
      auto costing = request.get<std::string>("costing");
      check_locations(locations.size(), max_locations.find(costing)->second);
      check_distance(reader, locations, max_distance.find(costing)->second);

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, costing_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

      std::stringstream stream;
      boost::property_tree::write_info(stream, request);

      //ok send on the request with correlated origin and destination filled out
      //using the boost ptree info format
      //TODO: make a protobuf request object and pass that along, can be come
      //part of thors path proto object and then get copied into odins trip object
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());
      return result;
    }
  }
}
