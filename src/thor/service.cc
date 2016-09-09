#include <vector>
#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/geojson.h>

#include <prime_server/prime_server.hpp>

#include "thor/service.h"
#include "thor/isochrone.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {
  constexpr double kMilePerMeter = 0.000621371;
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  std::vector<baldr::PathLocation> store_correlated_locations(const boost::property_tree::ptree& request, const std::vector<baldr::Location>& locations) {
    //we require correlated locations
    std::vector<baldr::PathLocation> correlated;
    correlated.reserve(locations.size());
    size_t i = 0;
    do {
      auto path_location = request.get_child_optional("correlated_" + std::to_string(i));
      if(!path_location)
        break;
      try {
        correlated.emplace_back(PathLocation::FromPtree(locations, *path_location));
      }
      catch (...) {
        throw std::runtime_error("Failed to parse correlated location");
      }
    }while(++i);
    return correlated;
  }

  worker_t::result_t jsonify_error(uint64_t code, const std::string& status, const std::string& error, http_request_t::info_t& request_info, const boost::optional<std::string>& jsonp) {

    //build up the json map
    auto json_error = json::map({});
    json_error->emplace("error", error);
    json_error->emplace("status", status);
    json_error->emplace("code", code);

    //serialize it
    std::stringstream ss;
    if(jsonp)
      ss << *jsonp << '(';
    ss << *json_error;
    if(jsonp)
      ss << ')';

    worker_t::result_t result{false};
    http_response_t response(code, status, ss.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());

    return result;
  }
}

namespace valhalla {
  namespace thor {

    thor_worker_t::thor_worker_t(const boost::property_tree::ptree& config): mode(valhalla::sif::TravelMode::kPedestrian),
      config(config), reader(config.get_child("mjolnir")),
      long_request(config.get<float>("thor.logging.long_request")){
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("transit", sif::CreateTransitCost);
      factory.Register("truck", sif::CreateTruckCost);
    }

    thor_worker_t::~thor_worker_t(){}

    worker_t::result_t thor_worker_t::work(const std::list<zmq::message_t>& job, void* request_info) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Thor Request " + std::to_string(info.id));
      try{
        //get some info about what we need to do
        boost::property_tree::ptree request;
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::stringstream stream(request_str);
        try {
          boost::property_tree::read_json(stream, request);
          jsonp = request.get_optional<std::string>("jsonp");
        }
        catch(const std::exception& e) {
          valhalla::midgard::logging::Log("500::" + std::string(e.what()), " [ANALYTICS] ");
          return jsonify_error(500, "Internal Server Error", e.what(), info, jsonp);
        }
        catch(...) {
          valhalla::midgard::logging::Log("500::non-std::exception", " [ANALYTICS] ");
          return jsonify_error(500, "Internal Server Error", "Failed to parse intermediate request format", info, jsonp);
        }

        // Initialize request - get the PathALgorithm to use
        ACTION_TYPE action = static_cast<ACTION_TYPE>(request.get<int>("action"));
        //what action is it
        switch (action) {
          case ONE_TO_MANY:
          case MANY_TO_ONE:
          case MANY_TO_MANY:
          case SOURCES_TO_TARGETS:
            return matrix(action, request, info);
          case OPTIMIZED_ROUTE:
            return optimized_route(request, request_str, info.do_not_track);
          case ISOCHRONE:
            return isochrone(request, info);
          case ROUTE:
          case VIAROUTE:
            return route(request, request_str, request.get_optional<int>("date_time.type"), info.do_not_track);
          case ATTRIBUTES:
            return attributes(request, info);
          default:
            throw std::runtime_error("Unknown action"); //this should never happen
        }
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error(400, "Bad Request", e.what(), info, jsonp);
      }
    }

    // Get the costing options. Get the base options from the config and the
    // options for the specified costing method. Merge in any request costing
    // options.
    valhalla::sif::cost_ptr_t thor_worker_t::get_costing(const boost::property_tree::ptree& request,
                                          const std::string& costing) {
      std::string method_options = "costing_options." + costing;
      auto config_costing = config.get_child_optional(method_options);
      if(!config_costing)
        throw std::runtime_error("No costing method found for '" + costing + "'");
      auto request_costing = request.get_child_optional(method_options);
      if(request_costing) {
        // If the request has any options for this costing type, merge the 2
        // costing options - override any config options that are in the request.
        // and add any request options not in the config.
        boost::property_tree::ptree overridden = *config_costing;
        for (const auto& r : *request_costing) {
          overridden.put_child(r.first, r.second);
        }
        return factory.Create(costing, overridden);
      }
      // No options to override so use the config options verbatim
      return factory.Create(costing, *config_costing);
    }

    std::string thor_worker_t::parse_costing(const boost::property_tree::ptree& request) {
      // Parse out the type of route - this provides the costing method to use
      auto costing = request.get<std::string>("costing");

      // Set travel mode and construct costing
      if (costing == "multimodal") {
        // For multi-modal we construct costing for all modes and set the
        // initial mode to pedestrian. (TODO - allow other initial modes)
        mode_costing[0] = get_costing(request, "auto");
        mode_costing[1] = get_costing(request, "pedestrian");
        mode_costing[2] = get_costing(request, "bicycle");
        mode_costing[3] = get_costing(request, "transit");
        mode = valhalla::sif::TravelMode::kPedestrian;
      } else {
        valhalla::sif::cost_ptr_t cost = get_costing(request, costing);
        mode = cost->travelmode();
        mode_costing[static_cast<uint32_t>(mode)] = cost;
      }
      valhalla::midgard::logging::Log("travel_mode::" + std::to_string(static_cast<uint32_t>(mode)), " [ANALYTICS] ");
      return costing;
    }

    void thor_worker_t::parse_locations(const boost::property_tree::ptree& request) {
      //we require locations
      auto request_locations = request.get_child_optional("locations");
      auto request_sources = request.get_child_optional("sources");
      auto request_targets = request.get_child_optional("targets");
      if(request_locations) {
        for(const auto& location : *request_locations) {
          try{ locations.push_back(baldr::Location::FromPtree(location.second)); }
          catch (...) { throw std::runtime_error("Failed to parse location"); }
        }
        correlated = store_correlated_locations(request, locations);
      }//if we have a sources and targets request here we will divvy up the correlated amongst them
      else if(request_sources && request_targets) {
        for(const auto& s : *request_sources) {
          try{ locations.push_back(baldr::Location::FromPtree(s.second)); }
          catch (...) { throw std::runtime_error("Failed to parse source"); }
        }
        for(const auto& t : *request_targets) {
          try{ locations.push_back(baldr::Location::FromPtree(t.second)); }
          catch (...) { throw std::runtime_error("Failed to parse target"); }
        }
        correlated = store_correlated_locations(request, locations);

        correlated_s.insert(correlated_s.begin(), correlated.begin(), correlated.begin() + request_sources->size());
        correlated_t.insert(correlated_t.begin(), correlated.begin() + request_sources->size(), correlated.end());
      }//we need something
      else
        throw std::runtime_error("Insufficiently specified required parameter 'locations'");

      //type - 0: current, 1: depart, 2: arrive
      auto date_time_type = request.get_optional<int>("date_time.type");
      auto date_time_value = request.get_optional<std::string>("date_time.value");

      if (date_time_type == 0) //current.
        locations.front().date_time_ = "current";
      else if (date_time_type == 1) //depart at
        locations.front().date_time_ = date_time_value;
      else if (date_time_type == 2) //arrive)
        locations.back().date_time_ = date_time_value;
    }

    void thor_worker_t::cleanup() {
      jsonp = boost::none;
      astar.Clear();
      bidir_astar.Clear();
      multi_modal_astar.Clear();
      locations.clear();
      correlated.clear();
      correlated_s.clear();
      correlated_t.clear();
      isochrone_gen.Clear();
      if(reader.OverCommitted())
        reader.Clear();
    }

    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from thor proxy
      auto upstream_endpoint = config.get<std::string>("thor.service.proxy") + "_out";
      //sends them on to odin
      auto downstream_endpoint = config.get<std::string>("odin.service.proxy") + "_in";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");

      //listen for requests
      zmq::context_t context;
      thor_worker_t thor_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
        std::bind(&thor_worker_t::work, std::ref(thor_worker), std::placeholders::_1, std::placeholders::_2),
        std::bind(&thor_worker_t::cleanup, std::ref(thor_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }

  }
}
