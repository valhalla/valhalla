#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>

#include "loki/service.h"
#include "loki/search.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


namespace {
  enum ACTION_TYPE {ROUTE, VIAROUTE, LOCATE, NEAREST};
  const std::unordered_map<std::string, ACTION_TYPE> ACTION{
    {"/route", ROUTE},
    {"/viaroute", VIAROUTE},
    {"/locate", LOCATE},
    {"/nearest", NEAREST}
  };

  boost::property_tree::ptree from_request(const ACTION_TYPE& action, const http_request_t& request) {
    boost::property_tree::ptree pt;

    //throw the json into the ptree
    auto json = request.query.find("json");
    if(json != request.query.end() && json->second.size()) {
      std::istringstream is(json->second.front());
      boost::property_tree::read_json(is, pt);
    }

    //throw the query params into the ptree
    for(const auto& kv : request.query) {
      //skip json or empty entries
      if(kv.first == "json" || kv.second.size() == 0)
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
    if(action == VIAROUTE) {
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

  //TODO: move json header to baldr
  //TODO: make objects serialize themselves

  json::ArrayPtr serialize_edges(const PathLocation& location, GraphReader& reader) {
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
        //only serialize it if we didnt do it before
        if(!duplicate) {
          ids.emplace(edge_info->wayid(), location.vertex());
          array->emplace_back(
            json::map({
              {"way_id", edge_info->wayid()},
              {"correlated_lat", json::fp_t{location.latlng_.lat(), 6}},
              {"correlated_lon", json::fp_t{location.latlng_.lng(), 6}}
            })
          );
        }
      }
      catch(...) {
        //this really shouldnt ever get hit
        LOG_WARN("Expected edge no found in graph but found by loki::search!");
      }
    }
    return array;
  }

  json::MapPtr serialize(const PathLocation& location, GraphReader& reader) {
    return json::map({
      {"ways", serialize_edges(location, reader)},
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}}
    });
  }

  json::MapPtr serialize(const PointLL& ll, const std::string& reason) {
    return json::map({
      {"ways", static_cast<nullptr_t>(nullptr)},
      {"input_lat", json::fp_t{ll.lat(), 6}},
      {"input_lon", json::fp_t{ll.lng(), 6}},
      {"reason", reason}
    });
  }

  //TODO: throw this in the header to make it testable?
  class loki_worker_t {
   public:
    loki_worker_t(const boost::property_tree::ptree& config):config(config), reader(config.get_child("mjolnir.hierarchy")) {
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
    }
    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
      auto& info = *static_cast<http_request_t::info_t*>(request_info);
      LOG_INFO("Got Loki Request " + std::to_string(info.id));
      //request should look like:
      //  /[route|viaroute|locate|nearest]?loc=&json=&jsonp=

      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        auto action = ACTION.find(request.path);
        if(action == ACTION.cend()) {
          worker_t::result_t result{false};
          http_response_t response(404, "Not Found", "Try any of: '/route' '/viaroute' '/locate' '/nearest'");
          response.from_info(info);
          result.messages.emplace_back(response.to_string());
          return result;
        }

        //parse the querys json
        auto request_pt = from_request(action->second, request);
        switch (action->second) {
          case ROUTE:
          case VIAROUTE:
            init_request(action->second, request_pt);
            return route(action->second, request_pt, info);
          case LOCATE:
            init_request(action->second, request_pt);
            return locate(request_pt, info);
        }

        worker_t::result_t result{false};
        http_response_t response(501, "Not Implemented");
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
      catch(const std::exception& e) {
        worker_t::result_t result{false};
        http_response_t response(400, "Bad Request", e.what());
        response.from_info(info);
        result.messages.emplace_back(response.to_string());
        return result;
      }
    }
    void init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request) {
      //we require locations
      try {
        for(const auto& location : request.get_child("locations"))
          locations.push_back(baldr::Location::FromPtree(location.second));
        if(locations.size() < 1)
          throw;
        //TODO: bail if this is too many
      }
      catch(...) {
        throw std::runtime_error("insufficiently specified required parameter '" + std::string(action == VIAROUTE ? "loc'" : "locations'"));
      }

      // Parse out the type of route - this provides the costing method to use
      std::string costing;
      try {
        costing = request.get<std::string>("costing");
      }
      catch(...) {
        throw std::runtime_error("No edge/node costing provided");
      }

      // Get the costing options. Get the base options from the config and the
      // options for the specified costing method
      std::string method_options = "costing_options." + costing;
      boost::property_tree::ptree config_costing = config.get_child(method_options);
      const auto& request_costing = request.get_child_optional(method_options);
      if (request_costing) {
        // If the request has any options for this costing type, merge the 2
        // costing options - override any config options that are in the request.
        // and add any request options not in the config.
        for (const auto& r : *request_costing) {
          config_costing.put_child(r.first, r.second);
        }
      }
      cost = factory.Create(costing, config_costing);
    }
    worker_t::result_t route(const ACTION_TYPE& action, boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //currently we dont support multipoint, but we will
      if(locations.size() != 2) {
        worker_t::result_t result{false};
        http_response_t response(501, "Not Implemented", "Only two locations supported at the moment");
        response.from_info(request_info);
        result.messages.emplace_back(response.to_string());
        return result;
      }

      //correlate the various locations to the underlying graph
      auto correlated = loki::Search(locations[0], reader, cost->GetFilter());
      request.put_child("origin", correlated.ToPtree(0));
      correlated = loki::Search(locations[1], reader, cost->GetFilter());
      request.put_child("destination", correlated.ToPtree(1));

      //let tyr know if its valhalla or osrm format
      if(action == ::VIAROUTE)
        request.put("osrm", "compatibility");
      std::stringstream stream;
      boost::property_tree::write_info(stream, request);

      //ok send on the request with correlated origin and destination filled out
      //using the boost ptree info format
      //TODO: make a protobuf request object and pass that along, can be come
      //part of thors path proto object and then get copied into odins trip object
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());
      return result;
    }
    worker_t::result_t locate(const boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //correlate the various locations to the underlying graph
      auto json = json::array({});
      for(const auto& location : locations) {
        try {
          auto correlated = loki::Search(location, reader, cost->GetFilter());
          json->emplace_back(serialize(correlated, reader));
        }
        catch(const std::exception& e) {
          json->emplace_back(serialize(location.latlng_, e.what()));
        }
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
      http_response_t response(200, "OK", stream.str(), headers_t{{"Content-type", "application/json;charset=utf-8"}});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }
    void cleanup() {
      locations.clear();
    }
   protected:
    boost::property_tree::ptree config;
    std::vector<Location> locations;
    sif::CostFactory<sif::DynamicCost> factory;
    valhalla::sif::cost_ptr_t cost;
    valhalla::baldr::GraphReader reader;
  };
}

namespace valhalla {
  namespace loki {
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
