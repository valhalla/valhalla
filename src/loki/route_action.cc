#include "loki/worker.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include "baldr/datetime.h"
#include "baldr/tilehierarchy.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {
  void check_locations(const size_t location_count, const size_t max_locations) {
    //check that location size does not exceed max.
    if (location_count > max_locations)
      throw valhalla_exception_t{150, std::to_string(max_locations)};
  }

  void check_distance(const GraphReader& reader, const std::vector<Location>& locations, float max_distance){
    //see if any locations pairs are unreachable or too far apart
    auto lowest_level = TileHierarchy::levels().rbegin();
    for(auto location = ++locations.cbegin(); location != locations.cend(); ++location) {
      //check if distance between latlngs exceed max distance limit for each mode of travel
      auto path_distance = std::prev(location)->latlng_.Distance(location->latlng_);
      max_distance -= path_distance;
      if (max_distance < 0)
        throw valhalla_exception_t{154};
      valhalla::midgard::logging::Log("location_distance::" + std::to_string(path_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
    }
  }
}

namespace valhalla {
  namespace loki {


    void loki_worker_t::init_route(rapidjson::Document& request) {
      locations = parse_locations(request, "locations");
      //need to check location size here instead of in parse_locations because of locate action needing a different size
      if(locations.size() < 2)
        throw valhalla_exception_t{120};
      parse_costing(request);
    }

    void loki_worker_t::route(rapidjson::Document& request) {
      init_route(request);
      auto costing = GetOptionalFromRapidJson<std::string>(request, "/costing");
      check_locations(locations.size(), max_locations.find(*costing)->second);
      check_distance(reader, locations, max_distance.find(*costing)->second);
      auto& allocator = request.GetAllocator();

      // Validate walking distances (make sure they are in the accepted range)
      if (*costing == "multimodal" || *costing == "transit") {
        auto transit_start_end_max_distance = GetOptionalFromRapidJson<int>(request,
            "/costing_options/pedestrian/transit_start_end_max_distance").get_value_or(min_transit_walking_dis);
        auto transit_transfer_max_distance = GetOptionalFromRapidJson<int>(request,
            "/costing_options/pedestrian/transit_transfer_max_distance").get_value_or(min_transit_walking_dis);

        if (transit_start_end_max_distance < min_transit_walking_dis || transit_start_end_max_distance > max_transit_walking_dis) {
          throw valhalla_exception_t{155, " Min: " + std::to_string(min_transit_walking_dis) + " Max: " + std::to_string(max_transit_walking_dis) +
                                   " (Meters)"};
        }
        if (transit_transfer_max_distance < min_transit_walking_dis || transit_transfer_max_distance > max_transit_walking_dis) {
          throw valhalla_exception_t{156, " Min: " + std::to_string(min_transit_walking_dis) + " Max: " + std::to_string(max_transit_walking_dis) +
                                   " (Meters)"};
        }
      }
      auto date_type_pointer = rapidjson::Pointer("/date_time/type");
      //default to current date_time for mm or transit.
      if (! date_type_pointer.Get(request) && (*costing == "multimodal" || *costing == "transit")) {
        date_type_pointer.Set(request, 0);
      }
      auto& locations_array = request["locations"];
      //check the date stuff
      auto date_time_value = GetOptionalFromRapidJson<std::string>(request, "/date_time/value");
      if (boost::optional<int> date_type = GetOptionalFromRapidJson<int>(request, "/date_time/type")) {
        //not yet on this
        if(*date_type == 2 && (*costing == "multimodal" || *costing == "transit"))
          throw valhalla_exception_t{141};

        //what kind
        switch(*date_type) {
        case 0: //current
          locations_array.Begin()->AddMember("date_time", "current", allocator);
          break;
        case 1: //depart
          if(!date_time_value)
            throw valhalla_exception_t{160};
          if (!DateTime::is_iso_local(*date_time_value))
            throw valhalla_exception_t{162};
          locations_array.Begin()->AddMember("date_time", *date_time_value, allocator);
          break;
        case 2: //arrive
          if(!date_time_value)
            throw valhalla_exception_t{161};
          if (!DateTime::is_iso_local(*date_time_value))
            throw valhalla_exception_t{162};
          locations_array.End()->AddMember("date_time", *date_time_value, allocator);
          break;
        default:
          throw valhalla_exception_t{163};
          break;
        }
      }

      //correlate the various locations to the underlying graph
      std::unordered_map<size_t, size_t> color_counts;
      try{
        const auto projections = loki::Search(locations, reader, edge_filter, node_filter);
        for(size_t i = 0; i < locations.size(); ++i) {
          const auto& correlated = projections.at(locations[i]);
          rapidjson::Pointer("/correlated_" + std::to_string(i)).Set(request, correlated.ToRapidJson(i,allocator));
          //TODO: get transit level for transit costing
          //TODO: if transit send a non zero radius
          if (!connectivity_map)
            continue;
          auto colors = connectivity_map->get_colors(TileHierarchy::levels().rbegin()->first, correlated, 0);
          for(auto color : colors){
            auto itr = color_counts.find(color);
            if(itr == color_counts.cend())
              color_counts[color] = 1;
            else
              ++itr->second;
          }
        }
      }
      catch(const std::exception&) {
        throw valhalla_exception_t{171};
      }

      //are all the locations in the same color regions
      if (!connectivity_map)
        return;
      bool connected = false;
      for(const auto& c : color_counts) {
        if(c.second == locations.size()) {
          connected = true;
          break;
        }
      }
      if(!connected)
        throw valhalla_exception_t{170};
    }
  }
}
