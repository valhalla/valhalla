#include <functional>
#include <unordered_map>
#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>

#include "loki/service.h"
#include "loki/search.h"

using namespace prime_server;
using namespace valhalla::baldr;
using namespace valhalla::loki;

namespace std {
  template <>
  struct hash<loki_worker_t::ACTION_TYPE>
  {
    std::size_t operator()(const loki_worker_t::ACTION_TYPE& a) const {
      return std::hash<int>()(a);
    }
  };
}

namespace {
  const std::unordered_map<std::string, loki_worker_t::ACTION_TYPE> STRING_TO_ACTION {
    {"one_to_many", loki_worker_t::ONE_TO_MANY},
    {"many_to_one", loki_worker_t::MANY_TO_ONE},
    {"many_to_many", loki_worker_t::MANY_TO_MANY},
    {"optimized_route", loki_worker_t::OPTIMIZED_ROUTE}
  };

  const std::unordered_map<loki_worker_t::ACTION_TYPE, std::string> ACTION_TO_STRING {
    {loki_worker_t::ONE_TO_MANY, "one_to_many"},
    {loki_worker_t::MANY_TO_ONE, "many_to_one"},
    {loki_worker_t::MANY_TO_MANY, "many_to_many"},
    {loki_worker_t::OPTIMIZED_ROUTE, "optimized_route"}
  };

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  void check_locations(const size_t location_count, const size_t max_locations) {
    //check that location size does not exceed max.
    if (location_count > max_locations)
      throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations) + ".");
  }

  void check_distance(const GraphReader& reader, const std::vector<Location>& locations, const size_t origin, const size_t start, const size_t end, float matrix_max_distance, float& max_location_distance) {

    //see if any locations pairs are unreachable or too far apart
    auto lowest_level = reader.GetTileHierarchy().levels().rbegin();
    //one to many should be distance between:a,b a,c ; many to one: a,c b,c ; many to many should be all pairs
    for(size_t i = start; i < end; ++i) {
      //check if distance between latlngs exceed max distance limit the chosen matrix type
      auto path_distance = locations[origin].latlng_.Distance(locations[i].latlng_);

      //only want to log the maximum distance between 2 locations for matrix
      LOG_DEBUG("path_distance -> " + std::to_string(path_distance));
      if (path_distance >= max_location_distance) {
          max_location_distance = path_distance;
          LOG_DEBUG("max_location_distance -> " + std::to_string(max_location_distance));
      }

      if (path_distance > matrix_max_distance)
        throw std::runtime_error("Path distance exceeds the max distance limit");
      }
  }
}

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::matrix(const ACTION_TYPE& action, boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      auto costing = request.get<std::string>("costing");
      if (costing == "multimodal") {
        http_response_t response(400, "Bad Request", ACTION_TO_STRING.find(action)->second + " does not support multimodal costing",  headers_t{CORS});
        response.from_info(request_info);
        return {false, {response.to_string()}};
      }
      //we want optimized_route to use the same location and distance limits as many_to_many
      auto action_str = ACTION_TO_STRING.find(action == loki_worker_t::OPTIMIZED_ROUTE ? loki_worker_t::MANY_TO_MANY : action)->second;
      //check that location size does not exceed max.
      check_locations(locations.size(), max_locations.find(action_str)->second);

      //check the distances
      auto max_location_distance = std::numeric_limits<float>::min();
      switch (action) {
        case ONE_TO_MANY:
          check_distance(reader,locations,0,0,locations.size(),max_distance.find(action_str)->second, max_location_distance);
          break;
        case MANY_TO_ONE:
          check_distance(reader,locations,locations.size()-1,0,locations.size()-1,max_distance.find(action_str)->second, max_location_distance);
          break;
        case MANY_TO_MANY:
        case OPTIMIZED_ROUTE:
          for(size_t i = 0; i < locations.size()-1; ++i)
            check_distance(reader,locations,i,(i+1),locations.size(),max_distance.find(action_str)->second, max_location_distance);
          break;
      }

      //correlate the various locations to the underlying graph
      std::unordered_map<size_t, size_t> color_counts;
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, edge_filter, node_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
        //TODO: get transit level for transit costing
        //TODO: if transit send a non zero radius
        auto colors = connectivity_map.get_colors(reader.GetTileHierarchy().levels().rbegin()->first, correlated, 0);
        for(auto color : colors){
          auto itr = color_counts.find(color);
          if(itr == color_counts.cend())
            color_counts[color] = 1;
          else
            ++itr->second;
        }
      }

      //are all the locations in the same color regions
      bool connected = false;
      for(const auto& c : color_counts) {
        if(c.second == locations.size()) {
          connected = true;
          break;
        }
      }
      if(!connected)
        throw std::runtime_error("Locations are in unconnected regions. Go check/edit the map at osm.org");

      valhalla::midgard::logging::Log("max_location_distance::" + std::to_string(max_location_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
      //pass on to thor with type of matrix
      request.put("matrix_type", ACTION_TO_STRING.find(action)->second);
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      /*for (auto message : result.messages){
        LOG_INFO("SENDING TO THOR:: " + message);
      }*/

      return result;
    }
  }
}
