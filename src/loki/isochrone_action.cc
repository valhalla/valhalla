#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>

using namespace prime_server;
using namespace valhalla::baldr;

namespace valhalla {
  namespace loki {

  void loki_worker_t::init_isochrones(const ACTION_TYPE& action,  boost::property_tree::ptree& request) {
    //we require locations
    auto request_locations = request.get_child_optional("locations");

    if (!request_locations)
      throw std::runtime_error("Insufficiently specified required parameter 'locations'");

    for(const auto& location : *request_locations) {
      try{
        locations.push_back(baldr::Location::FromPtree(location.second));
      }
      catch (...) {
        throw std::runtime_error("Failed to parse location");
      }
    }
    if(locations.size() < 2)
      throw std::runtime_error("Insufficient number of locations provided");

    valhalla::midgard::logging::Log("location_count::" + std::to_string(request_locations->size()), " [ANALYTICS] ");

    //using the costing we can determine what type of edge filtering to use
    auto costing = request.get_optional<std::string>("costing");
    if (costing)
      valhalla::midgard::logging::Log("costing_type::" + *costing, " [ANALYTICS] ");
    else throw std::runtime_error("No edge/node costing provided");

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

    // TODO - have a way of specifying mode at the location
    if(*costing == "multimodal")
      *costing = "pedestrian";

    // Get the costing options. Get the base options from the config and the
    // options for the specified costing method
    std::string method_options = "costing_options." + *costing;
    auto config_costing = config.get_child_optional(method_options);
    if(!config_costing)
      throw std::runtime_error("No costing method found for '" + *costing + "'");
    auto request_costing = request.get_child_optional(method_options);
    if(request_costing) {
      // If the request has any options for this costing type, merge the 2
      // costing options - override any config options that are in the request.
      // and add any request options not in the config.
      // TODO: suboptions are probably getting smashed when we do this, preserve them
      boost::property_tree::ptree overridden = *config_costing;
      for(const auto& r : *request_costing)
        overridden.put_child(r.first, r.second);
      auto c = factory.Create(*costing, overridden);
      edge_filter = c->GetEdgeFilter();
      node_filter = c->GetNodeFilter();
    }// No options to override so use the config options verbatim
    else {
      auto c = factory.Create(*costing, *config_costing);
      edge_filter = c->GetEdgeFilter();
      node_filter = c->GetNodeFilter();
    }
  }

    worker_t::result_t loki_worker_t::isochrones(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //check that location size does not exceed max
      if (locations.size() > max_locations.find("isochrone")->second)
        throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations.find("isochrone")->second) + ".");

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, edge_filter, node_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

      //let thor know this is isolines
      request.put("isochrone", 1);
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      return result;
    }

  }
}
