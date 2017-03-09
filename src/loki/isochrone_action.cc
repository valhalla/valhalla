#include "loki/service.h"
#include "loki/search.h"
#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla::baldr;

namespace {
const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_isochrones(rapidjson::Document& request) {
      //strip off unused information
      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{400, 120};
      for(auto& l : locations)
        l.heading_.reset();

      //make sure the isoline definitions are valid
      auto contours = GetOptionalFromRapidJson<rapidjson::Value::ConstArray>(request, "/contours");
      if(! contours)
        throw valhalla_exception_t{400, 113};
      //check that the number of contours is ok
      if(contours->Size() > max_contours)
        throw valhalla_exception_t{400, 152, std::to_string(max_contours)};
      size_t prev = 0;
      for(const auto& contour : *contours) {
        const size_t c = GetOptionalFromRapidJson<size_t>(contour, "/time").get_value_or(-1);
        if(c < prev || c == -1)
          throw valhalla_exception_t{400, 111};
        if(c > max_time)
          throw valhalla_exception_t{400, 151, std::to_string(max_time)};
        prev = c;
      }
      parse_costing(request);
    }

    worker_t::result_t loki_worker_t::isochrones(rapidjson::Document& request, http_request_info_t& request_info) {
      init_isochrones(request);
      //check that location size does not exceed max
      if (locations.size() > max_locations.find("isochrone")->second)
        throw valhalla_exception_t{400, 150, std::to_string(max_locations.find("isochrone")->second)};

      auto costing = GetOptionalFromRapidJson<std::string>(request, "/costing").get_value_or("");
      auto date_type = GetOptionalFromRapidJson<int>(request, "/date_time/type");

      auto& allocator = request.GetAllocator();
      //default to current date_time for mm or transit.
      if (! date_type && (costing == "multimodal" || costing == "transit")) {
        rapidjson::SetValueByPointer(request, "/date_time/type", 0);
        date_type = 0;
      }

      //check the date stuff
      auto date_time_value = GetOptionalFromRapidJson<std::string>(request, "/date_time/value");
      if (date_type) {
        //not yet on this
        if(! date_type || *date_type == 2) {
          jsonify_error({501, 142}, request_info);
        }
        //what kind
        switch(*date_type) {
        case 0: //current
          rapidjson::GetValueByPointer(request, "/locations/0")->AddMember("date_time", "current", allocator);
          break;
        case 1: //depart
          if(! date_time_value)
            throw valhalla_exception_t{400, 160};
          if (!DateTime::is_iso_local(*date_time_value))
            throw valhalla_exception_t{400, 162};
          rapidjson::GetValueByPointer(request, "/locations/0")->AddMember("date_time", *date_time_value, allocator);
          break;
        default:
          throw valhalla_exception_t{400, 163};
          break;
        }
      }

      try{
        //correlate the various locations to the underlying graph
        const auto projections = loki::Search(locations, reader, edge_filter, node_filter);
        for(size_t i = 0; i < locations.size(); ++i) {
          rapidjson::Pointer("/correlated_" + std::to_string(i)).Set(request, projections.at(locations[i]).ToRapidJson(i, allocator));
        }
      }
      catch(const std::exception&) {
        throw valhalla_exception_t{400, 171};
      }

      //pass it on
      worker_t::result_t result{true};
      result.messages.emplace_back(rapidjson::to_string(request));

      return result;
    }

  }
}
