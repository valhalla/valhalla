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

  //TODO: move json header to baldr
  //TODO: make objects serialize themselves

  json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader, bool verbose) {
    auto array = json::array({});
    std::unordered_multimap<uint64_t, PointLL> ids;
    for(const auto& edge : location.edges()) {
      try {
        //get the osm way id
        auto tile = reader.GetGraphTile(edge.id);
        auto* directed_edge = tile->directededge(edge.id);
        auto edge_info = tile->edgeinfo(directed_edge->edgeinfo_offset());
        //check if we did this one before
        auto range = ids.equal_range(edge_info->wayid());
        bool duplicate = false;
        for(auto id = range.first; id != range.second; ++id) {
          if(id->second == location.vertex()) {
            duplicate = true;
            break;
          }
        }
      }
      catch(...) {
        //this really shouldnt ever get hit
        LOG_WARN("Expected edge not found in graph but found by loki::search!");
      }
    }
    return array;
  }
}

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::matrix(const ACTION_TYPE& action, boost::property_tree::ptree& request) {
      auto matrix_type = "";
      switch (action) {
       case ONE_TO_MANY:
         matrix_type = "one_to_many";
         break;
       case MANY_TO_ONE:
         matrix_type = "many_to_one";
         break;
       case MANY_TO_MANY:
         matrix_type = "many_to_many";
         break;
     }
      auto max_area = config.get<float>("service_limits." + std::string(matrix_type) + ".max_area");
      auto max_locations = config.get<size_t>("service_limits." + std::string(matrix_type) + ".max_locations");
      //check that location size does not exceed max.
      if (locations.size() > max_locations)
        throw std::runtime_error("Number of locations exceeds the max location limit.");
      LOG_INFO("Location size::" + std::to_string(locations.size()));

      //see if any locations pairs are unreachable or too far apart
      auto lowest_level = reader.GetTileHierarchy().levels().rbegin();
      for(auto location = ++locations.cbegin(); location != locations.cend(); ++location) {
        //check connectivity
        uint32_t a_id = lowest_level->second.tiles.TileId(std::prev(location)->latlng_);
        uint32_t b_id = lowest_level->second.tiles.TileId(location->latlng_);
        if(!reader.AreConnected({a_id, lowest_level->first, 0}, {b_id, lowest_level->first, 0}))
          throw std::runtime_error("Locations are in unconnected regions. Go check/edit the map at osm.org");

        //check if distance between latlngs exceed max distance limit for each mode of travel
        auto path_distance = std::sqrt(midgard::DistanceApproximator::DistanceSquared(std::prev(location)->latlng_, location->latlng_));
        if (path_distance > max_area)
          throw std::runtime_error("Path distance exceeds the max area limit.");

        LOG_INFO("Location distance::" + std::to_string(path_distance));
      }
      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, costing_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }
      request.put("matrix_type", std::string(matrix_type));

      std::stringstream stream;
      boost::property_tree::write_info(stream, request);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());
      return result;
    }
  }
}
