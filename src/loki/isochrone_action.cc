#include "loki/service.h"
#include "loki/search.h"
#include <valhalla/baldr/datetime.h>
#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla::baldr;

namespace {
const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_isochrones(const boost::property_tree::ptree& request) {
      //strip off unused information
      parse_locations(request);
      if(locations.size() < 1)
        throw std::runtime_error("Insufficient number of locations provided");
      for(auto& l : locations)
        l.heading_.reset();

      //make sure the isoline definitions are valid
      auto contours = request.get_child_optional("contours");
      if(!contours)
        throw std::runtime_error("Insufficiently specified required parameter 'contours'");
      //check that the number of contours is ok
      if(contours->size() > max_contours)
        throw std::runtime_error("Exceeded max contours of " + std::to_string(max_contours) + ".");
      size_t prev = 0;
      for(const auto& contour : *contours) {
        auto c = contour.second.get<size_t>("time", -1);
        if(c < prev || c == -1)
          throw std::runtime_error("Insufficiently specified required parameter 'time'");
        if(c > max_time)
          throw std::runtime_error("Exceeded max time of " + std::to_string(max_time) + ".");
        prev = c;
      }
      parse_costing(request);
    }

    worker_t::result_t loki_worker_t::isochrones(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      init_isochrones(request);
      //check that location size does not exceed max
      if (locations.size() > max_locations.find("isochrone")->second)
        throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations.find("isochrone")->second) + ".");

      auto costing = request.get<std::string>("costing");
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
        if(date_type == 2) {
          http_response_t response(501, "Not Implemented", "Arrive by not implemented for isochrones", headers_t{CORS});
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
        default:
          throw std::runtime_error("Invalid date_type");
          break;
        }
      }

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, edge_filter, node_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

      //pass it on
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      return result;
    }

  }
}
