#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>

using namespace prime_server;
using namespace valhalla::baldr;

namespace {
  enum ACTION_TYPE { TIMEDISTANCEMATRIX };
  enum MATRIX_TYPE {  ONE_TO_MANY, MANY_TO_ONE, MANY_TO_MANY };
  const std::unordered_map<std::string, ACTION_TYPE> ACTION{
    {"/timedistancematrix", TIMEDISTANCEMATRIX}
  };
  const std::unordered_map<std::string, MATRIX_TYPE> MATRIX{
    {"one_to_many", ONE_TO_MANY},
    {"many_to_one", MANY_TO_ONE},
    {"many_to_many", MANY_TO_MANY}
  };
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

    worker_t::result_t loki_worker_t::timedistancematrix(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      auto json = json::array({});

      auto matrix_type = MATRIX.find(request.get<std::string>("matrix_type"));
      auto costing = request.get<std::string>("costing");
      if(costing.empty())
        throw std::runtime_error("No edge/node costing provided");

      //see if any locations pairs are unreachable or too far apart
      auto lowest_level = reader.GetTileHierarchy().levels().rbegin();
      auto max_distance = config.get<float>("timedistance_limits." + matrix_type->first + ".max_distance" + request.get<std::string>("costing"));
      auto max_locations = config.get<int>("timedistance_limits." + matrix_type->first + ".max_locations");
      //check that location size does not exceed max.
      if (locations.size() > max_locations)
        throw std::runtime_error("Number of locations exceeds the max location limit.");

      LOG_INFO("Location size::" + std::to_string(locations.size()));

      for(auto location = ++locations.cbegin(); location != locations.cend(); ++location) {

        //check connectivity
        uint32_t a_id = lowest_level->second.tiles.TileId(std::prev(location)->latlng_);
        uint32_t b_id = lowest_level->second.tiles.TileId(location->latlng_);
        if(!reader.AreConnected({a_id, lowest_level->first, 0}, {b_id, lowest_level->first, 0}))
          throw std::runtime_error("Locations are in unconnected regions. Go check/edit the map at osm.org");

        //check if distance between latlngs exceed max distance limit for each mode of travel
        auto path_distance = std::sqrt(midgard::DistanceApproximator::DistanceSquared(std::prev(location)->latlng_, location->latlng_));
        if (path_distance > max_distance)
          throw std::runtime_error("Path distance exceeds the max distance limit.");

        LOG_INFO("Location distance::" + std::to_string(path_distance));
      }

      switch ( matrix_type->second ) {
        case MATRIX_TYPE::ONE_TO_MANY:
          json->emplace_back(matrix_type->first, locations.begin(),locations, reader, costing_filter, costing);
          break;
        case MATRIX_TYPE::MANY_TO_ONE:
          json->emplace_back(matrix_type->first, locations.end(),locations, reader, costing_filter, costing);
          break;
        case MATRIX_TYPE::MANY_TO_MANY:
          json->emplace_back(matrix_type->first, locations, reader, costing_filter, costing);
          break;
        default:
          throw std::runtime_error("Invalid matrix_type provided. Try one_to_many, many_to_one or many_to_many.");
      }

      //jsonp callback if need be
      std::ostringstream stream;
      auto jsonp = request.get_optional<std::string>("jsonp");
      if(jsonp)
        stream << *jsonp << '(';
      stream << *json;
      if(jsonp)
        stream << ')';

      worker_t::result_t result{false};
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }
  }
}
