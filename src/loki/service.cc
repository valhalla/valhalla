#include <functional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

#include "loki/service.h"
#include "loki/search.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace {

  const std::unordered_map<std::string, loki_worker_t::ACTION_TYPE> ACTION{
    {"/route", loki_worker_t::ROUTE},
    {"/viaroute", loki_worker_t::VIAROUTE},
    {"/locate", loki_worker_t::LOCATE},
    {"/one_to_many", loki_worker_t::ONE_TO_MANY},
    {"/many_to_one", loki_worker_t::MANY_TO_ONE},
    {"/many_to_many", loki_worker_t::MANY_TO_MANY}
  };
  const std::vector<std::string> matrix_types={"one_to_many","many_to_one","many_to_many"};

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  boost::property_tree::ptree from_request(const loki_worker_t::ACTION_TYPE& action, const http_request_t& request) {
    boost::property_tree::ptree pt;

    //parse the input
    try {
      //throw the json into the ptree
      auto json = request.query.find("json");
      if(json != request.query.end() && json->second.size()) {
        std::istringstream is(json->second.front());
        boost::property_tree::read_json(is, pt);
      }//no json parameter, check the body
      else if(!request.body.empty()) {
        std::istringstream is(request.body);
        boost::property_tree::read_json(is, pt);
      }
    }
    catch(...) {
      throw std::runtime_error("Failed to parse json request");
    }

    //throw the query params into the ptree
    for(const auto& kv : request.query) {
      //skip json or empty entries
      if(kv.first == "json" || kv.first.size() == 0 || kv.second.size() == 0)
        continue;

      //turn single value entries into single key value
      if(kv.second.size() == 1) {
        pt.add(kv.first, kv.second.front());
        continue;
      }

      //make an array of values for this key
      boost::property_tree::ptree array;
      for(const auto& value : kv.second) {
        boost::property_tree::ptree element;
        element.put("", value);
        array.push_back(std::make_pair("", element));
      }
      pt.add_child(kv.first, array);
    }

    //if its osrm compatible lets make the location object conform to our standard input
    if(action == loki_worker_t::VIAROUTE) {
      auto& array = pt.put_child("locations", boost::property_tree::ptree());
      for(const auto& location : pt.get_child("loc")) {
        Location l = Location::FromCsv(location.second.get_value<std::string>());
        boost::property_tree::ptree element;
        element.put("lon", l.latlng_.first);
        element.put("lat", l.latlng_.second);
        array.push_back(std::make_pair("", element));
      }
      pt.erase("loc");
    }

    return pt;
  }
}

namespace valhalla {
  namespace loki {
    loki_worker_t::loki_worker_t(const boost::property_tree::ptree& config):config(config), reader(config.get_child("mjolnir.hierarchy")) {
      auto costing_options = config.get_child("costing_options");
      for (const auto& kv : costing_options) {
        auto max_locations = config.get_optional<std::string>("service_limits." + kv.first + ".max_locations");
        if (!max_locations)
          throw std::runtime_error("The config costing options for max_locations are incorrectly loaded.");

        auto max_distance = config.get_optional<std::string>("service_limits." + kv.first + ".max_distance");
        if (!max_distance)
          throw std::runtime_error("The config costing options for max_distance are incorrectly loaded.");
      }

      //load the loki config actions into an array
      const boost::property_tree::ptree &config_actions = config.get_child("loki.actions");
      if(config_actions.empty())
        throw std::runtime_error("The config actions for Loki are incorrectly loaded.");

      //iterate over the array of actions and store into an unordered set
      for (const auto& kv : config_actions) {
        action_set.emplace("/" + kv.second.get_value<std::string>());
        action_str.append("'/" + kv.second.get_value<std::string>() + "' ");
      }

      for (const std::string& mtype : matrix_types) {
        auto matrix_max_area = config.get<float>("service_limits." + mtype + ".max_area");
        if (!matrix_max_area)
          throw std::runtime_error("The config max_area for " + mtype + " is not found.");
        auto matrix_max_locations = config.get<size_t>("service_limits." + mtype + ".max_locations");
        if (!matrix_max_locations)
          throw std::runtime_error("The config max_locations for " + mtype + " is not found.");
      }

      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", sif::CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
    }
    worker_t::result_t loki_worker_t::work(const std::list<zmq::message_t>& job, void* request_info) {
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Loki Request " + std::to_string(info.id));
      //request should look like:
      //  /[route|viaroute|locate]?loc=&json=&jsonp=

      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

        //block all but get and post
        if(request.method != method_t::POST && request.method != method_t::GET) {
          worker_t::result_t result{false};
          http_response_t response(405, "Method Not Allowed", "Try a POST or GET request instead", headers_t{CORS});
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }

        auto action = ACTION.find(request.path);
        //is the request path action in the action set?
        if ((action_set.find(request.path) == action_set.cend()) || action == ACTION.cend()) {
            worker_t::result_t result{false};
            http_response_t response(404, "Action Not Found", "Try any of: " + action_str, headers_t{CORS});
            response.from_info(info);
            result.messages.emplace_back(response.to_string());
            return result;
        }

        //parse the query's json
        auto request_pt = from_request(action->second, request);
        init_request(action->second, request_pt);
        switch (action->second) {
          case ROUTE:
          case VIAROUTE:
            return route(action->second, request_pt);
          case LOCATE:
            return locate(request_pt, info);
          case ONE_TO_MANY:
          case MANY_TO_ONE:
          case MANY_TO_MANY:
            return matrix(action->second, request_pt);
        }

        //apparently you wanted something that we figured we'd support but havent written yet
        worker_t::result_t result{false};
        http_response_t response(501, "Not Implemented", "Not Implemented", headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
      catch(const std::exception& e) {
        LOG_INFO(std::string("Bad Request: ") + e.what());
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what(), headers_t{CORS});
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
    }
    void loki_worker_t::init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request) {
      auto costing = request.get_optional<std::string>("costing");
      size_t max_locations = std::numeric_limits<size_t>::max(); //TODO: locate should really have a limit..
      if (costing)
        max_locations = config.get<size_t>("service_limits." + *costing + ".max_locations");

      //we require locations
      auto request_locations = request.get_child_optional("locations");
      if(!request_locations)
        throw std::runtime_error("Insufficiently specified required parameter '" + std::string(action == VIAROUTE ? "loc'" : "locations'"));
      for(const auto& location : *request_locations) {
        try{
          locations.push_back(baldr::Location::FromPtree(location.second));
        }
        catch (...) {
          throw std::runtime_error("Failed to parse location");
        }
        if(action != LOCATE && locations.size() > max_locations)
          throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations));
      }
      if(locations.size() < (action == LOCATE ? 1 : 2))
        throw std::runtime_error("Insufficient number of locations provided");
      LOG_INFO("location_count::" + std::to_string(request_locations->size()));

      //using the costing we can determine what type of edge filtering to use
      if(!costing) {
        //locate doesnt require a filter
        if(action == LOCATE) {
          costing_filter = loki::PassThroughFilter;
          return;
        }//but everything else does
        else
          throw std::runtime_error("No edge/node costing provided");
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
        for(const auto& r : *request_costing)
          config_costing->put_child(r.first, r.second);
      }
      costing_filter = factory.Create(*costing, *config_costing)->GetFilter();
    }

    void loki_worker_t::cleanup() {
      locations.clear();
      if(reader.OverCommitted())
        reader.Clear();
    }

    void run_service(const boost::property_tree::ptree& config) {
      //gets requests from the http server
      auto upstream_endpoint = config.get<std::string>("loki.service.proxy") + "_out";
      //sends them on to thor
      auto downstream_endpoint = config.get<std::string>("thor.service.proxy") + "_in";
      //or returns just location information back to the server
      auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");

      //listen for requests
      zmq::context_t context;
      loki_worker_t loki_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
        std::bind(&loki_worker_t::work, std::ref(loki_worker), std::placeholders::_1, std::placeholders::_2),
        std::bind(&loki_worker_t::cleanup, std::ref(loki_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
  }
}
