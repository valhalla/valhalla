#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/datetime.h>
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
      //check if distance between latlngs exceed max distance limit for each mode of travel
      auto path_distance = std::prev(location)->latlng_.Distance(location->latlng_);
      max_distance-=path_distance;
      if (max_distance < 0)
        throw std::runtime_error("Path distance exceeds the max distance limit");

      valhalla::midgard::logging::Log("location_distance::" + std::to_string(path_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
    }
  }
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_route(const boost::property_tree::ptree& request) {
      parse_locations(request);
      if(locations.size() < 2)
        throw std::runtime_error("Insufficient number of locations provided");
      parse_costing(request);
    }

    worker_t::result_t loki_worker_t::route(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      init_route(request);
      auto costing = request.get<std::string>("costing");
      check_locations(locations.size(), max_locations.find(costing)->second);
      check_distance(reader, locations, max_distance.find(costing)->second);

      // Validate walking distances (make sure they are in the accepted range)
      if (costing == "multimodal" || costing == "transit") {
        auto transit_start_end_max_distance =
            request.get<int>("costing_options.pedestrian.transit_start_end_max_distance", min_transit_walking_dis);
        auto transit_transfer_max_distance =
            request.get<int>("costing_options.pedestrian.transit_transfer_max_distance", min_transit_walking_dis);

        if (transit_start_end_max_distance < min_transit_walking_dis || transit_start_end_max_distance > max_transit_walking_dis) {
          throw std::runtime_error("Outside the valid walking distance at the beginning or end of a multimodal route.  Min: " +
                                   std::to_string(min_transit_walking_dis) + " Max: " + std::to_string(max_transit_walking_dis) +
                                   " (Meters)");
        }
        if (transit_transfer_max_distance < min_transit_walking_dis || transit_transfer_max_distance > max_transit_walking_dis) {
          throw std::runtime_error("Outside the valid walking distance between stops of a multimodal route.  Min: " +
                                   std::to_string(min_transit_walking_dis) + " Max: " + std::to_string(max_transit_walking_dis) +
                                   " (Meters)");
        }
      }

      auto date_type = request.get_optional<int>("date_time.type");
      //default to current date_time for mm or transit.
      if (!date_type && (costing == "multimodal" || costing == "transit")) {
        request.add("date_time.type", 0);
        date_type = request.get_optional<int>("date_time.type");
      }

      //check the date stuff
      auto date_time_value = request.get_optional<std::string>("date_time.value");
      if (date_type) {
        //not yet on this
        if(date_type == 2 && (costing == "multimodal" || costing == "transit")) {
          http_response_t response(501, "Not Implemented", "Arrive by for multimodal not implemented yet", headers_t{CORS});
          response.from_info(request_info);
          return {false, {response.to_string()}};
        }
        //what kind
        switch(*date_type) {
        case 0: //current
          request.get_child("locations").front().second.add("date_time", "current");
          break;
        case 1: //depart
          if(!date_time_value)
            throw std::runtime_error("Date and time required for origin for date_type of depart at.");
          if (!DateTime::is_iso_local(*date_time_value))
            throw std::runtime_error("Date and time is invalid.  Format is YYYY-MM-DDTHH:MM");
          request.get_child("locations").front().second.add("date_time", *date_time_value);
          break;
        case 2: //arrive
          if(!date_time_value)
            throw std::runtime_error("Date and time required for destination for date_type of arrive by");
          if (!DateTime::is_iso_local(*date_time_value))
            throw std::runtime_error("Date and time is invalid.  Format is YYYY-MM-DDTHH:MM");
          request.get_child("locations").back().second.add("date_time", *date_time_value);
          break;
        default:
          throw std::runtime_error("Invalid date_type");
          break;
        }
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

      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);

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
