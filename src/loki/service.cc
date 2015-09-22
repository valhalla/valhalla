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
#include <valhalla/midgard/distanceapproximator.h>
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
  enum ACTION_TYPE {ROUTE, VIAROUTE, LOCATE};
  const std::unordered_map<std::string, ACTION_TYPE> ACTION{
    {"/route", ROUTE},
    {"/viaroute", VIAROUTE},
    {"/locate", LOCATE}
  };
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  boost::property_tree::ptree from_request(const ACTION_TYPE& action, const http_request_t& request) {
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
        //only serialize it if we didnt do it before
        if(!duplicate) {
          ids.emplace(edge_info->wayid(), location.vertex());
          //they want MOAR!
          if(verbose) {
            array->emplace_back(
              json::map({
                {"way_id", edge_info->wayid()},
                {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
                {"correlated_lon", json::fp_t{location.vertex().lng(), 6}}
              })
            );
          }//spare the details
          else {
            array->emplace_back(
              json::map({
                {"way_id", edge_info->wayid()},
                {"correlated_lat", json::fp_t{location.vertex().lat(), 6}},
                {"correlated_lon", json::fp_t{location.vertex().lng(), 6}}
              })
            );
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

  json::MapPtr serialize(const PathLocation& location, GraphReader& reader, bool verbose) {
    return json::map({
      {"ways", serialize_edges(location, reader, verbose)},
      {"input_lat", json::fp_t{location.latlng_.lat(), 6}},
      {"input_lon", json::fp_t{location.latlng_.lng(), 6}}
    });
  }

  json::MapPtr serialize(const PointLL& ll, const std::string& reason) {
    return json::map({
      {"ways", static_cast<std::nullptr_t>(nullptr)},
      {"input_lat", json::fp_t{ll.lat(), 6}},
      {"input_lon", json::fp_t{ll.lng(), 6}},
      {"reason", reason}
    });
  }

  class loki_worker_t {
   public:
    loki_worker_t(const boost::property_tree::ptree& config):config(config), reader(config.get_child("mjolnir.hierarchy")),
      max_route_locations(config.get<size_t>("service_limits.max_route_locations")) {

      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", sif::CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
    }
    worker_t::result_t work(const std::list<zmq::message_t>& job, void* request_info) {
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

        //what do they want to do
        auto action = ACTION.find(request.path);
        if(action == ACTION.cend()) {
          worker_t::result_t result{false};
          http_response_t response(404, "Not Found", "Try any of: '/route' '/locate'", headers_t{CORS});
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
            return route(action->second, request_pt, info);
          case LOCATE:
            return locate(request_pt, info);
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
    void init_request(const ACTION_TYPE& action, const boost::property_tree::ptree& request) {
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
        if(action != LOCATE && locations.size() > max_route_locations)
          throw std::runtime_error("Exceeded max route locations of " + std::to_string(max_route_locations));
      }
      if(locations.size() < (action == LOCATE ? 1 : 2))
        throw std::runtime_error("Insufficient number of locations provided");
      LOG_INFO("location_count::" + std::to_string(request_locations->size()));

      //using the costing we can determine what type of edge filtering to use
      auto costing = request.get_optional<std::string>("costing");
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
    worker_t::result_t route(const ACTION_TYPE& action, boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //see if any locations pairs are unreachable or too far apart
      auto lowest_level = reader.GetTileHierarchy().levels().rbegin();
      auto max_distance = config.get<float>("service_limits.max_distance." + request.get<std::string>("costing"));
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

        LOG_INFO("location_distance::" + std::to_string(path_distance));
      }

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, costing_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

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
          auto correlated = loki::Search(location, reader, costing_filter);
          json->emplace_back(serialize(correlated, reader, request.get<bool>("verbose", false)));
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
      http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());
      return result;
    }
    void cleanup() {
      locations.clear();
      if(reader.OverCommitted())
        reader.Clear();
    }
   protected:
    boost::property_tree::ptree config;
    std::vector<Location> locations;
    sif::CostFactory<sif::DynamicCost> factory;
    sif::EdgeFilter costing_filter;
    valhalla::baldr::GraphReader reader;
    size_t max_route_locations;
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
