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
#include <valhalla/midgard/encoded.h>
#include <valhalla/sif/autocost.h>
#include <valhalla/sif/bicyclecost.h>
#include <valhalla/sif/pedestriancost.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/errorcode_util.h>

#include "loki/service.h"
#include "loki/search.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace {

  const std::unordered_map<std::string, loki_worker_t::ACTION_TYPE> PATH_TO_ACTION{
    {"/route", loki_worker_t::ROUTE},
    {"/viaroute", loki_worker_t::VIAROUTE},
    {"/locate", loki_worker_t::LOCATE},
    {"/one_to_many", loki_worker_t::ONE_TO_MANY},
    {"/many_to_one", loki_worker_t::MANY_TO_ONE},
    {"/many_to_many", loki_worker_t::MANY_TO_MANY},
    {"/sources_to_targets", loki_worker_t::SOURCES_TO_TARGETS},
    {"/optimized_route", loki_worker_t::OPTIMIZED_ROUTE},
    {"/isochrone", loki_worker_t::ISOCHRONE},
    {"/trace_route", loki_worker_t::TRACE_ROUTE},
    {"/trace_attributes", loki_worker_t::TRACE_ATTRIBUTES}
  };

  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};

  boost::property_tree::ptree from_request(const loki_worker_t::ACTION_TYPE& action, const http_request_t& request) {
    boost::property_tree::ptree pt;

    //parse the input
    try {
      //throw the json into the ptree
      auto json = request.query.find("json");
      if (json != request.query.end() && json->second.size()
        && json->second.front().size()) {
        std::istringstream is(json->second.front());
        boost::property_tree::read_json(is, pt);
      }//no json parameter, check the body
      else if(!request.body.empty()) {
        std::istringstream is(request.body);
        boost::property_tree::read_json(is, pt);
      }
    }
    catch(...) {
      throw valhalla_exception_t{400, 100};
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
    worker_t::result_t loki_worker_t::jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info) const {

      //build up the json map
      auto json_error = json::map({});
      json_error->emplace("status", exception.status_code_body);
      json_error->emplace("status_code", static_cast<uint64_t>(exception.status_code));
      json_error->emplace("error", std::string(exception.error_code_message));
      json_error->emplace("error_code", static_cast<uint64_t>(exception.error_code));

      //serialize it
      std::stringstream ss;
      if(jsonp)
        ss << *jsonp << '(';
      ss << *json_error;
      if(jsonp)
        ss << ')';

      worker_t::result_t result{false};
      http_response_t response(exception.status_code, exception.status_code_body, ss.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
      response.from_info(request_info);
      result.messages.emplace_back(response.to_string());

      return result;
    }

    void loki_worker_t::parse_locations(const boost::property_tree::ptree& request) {
      //we require locations
      auto request_locations = request.get_child_optional("locations");
      if (!request_locations)
        throw valhalla_exception_t{400, 110};

      for(const auto& location : *request_locations) {
        try{
          locations.push_back(baldr::Location::FromPtree(location.second));
        }
        catch (...) {
          throw valhalla_exception_t{400, 130};
        }
      }
      valhalla::midgard::logging::Log("location_count::" + std::to_string(request_locations->size()), " [ANALYTICS] ");
    }

    void loki_worker_t::parse_trace(boost::property_tree::ptree& request) {
      //we require uncompressed shape or encoded polyline
      auto input_shape = request.get_child_optional("shape");
      auto encoded_polyline = request.get_optional<std::string>("encoded_polyline");

      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape) {
            shape.push_back(baldr::Location::FromPtree(latlng.second).latlng_);
          }
        }//compressed shape
        //if we receive as encoded then we need to add as shape to request
        else if (encoded_polyline) {
          shape = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
          boost::property_tree::ptree shape_child;
          for(const auto& pt : shape) {
            boost::property_tree::ptree point_child;
            point_child.put("lon", static_cast<double>(pt.first));
            point_child.put("lat", static_cast<double>(pt.second));
            shape_child.push_back(std::make_pair("", point_child));
          }
          request.add_child("shape", shape_child);
        }/* else if (gpx) {
          //TODO:Add support
        } else if (geojson){
          //TODO:Add support
        }*/
        else
          throw valhalla_exception_t{400, 126};
      }
      catch (const std::exception& e) {
        //TODO: pass on e.what() to generic exception
        throw valhalla_exception_t{400, 114};
      }

      //not enough
      if(shape.size() < 2)
        throw valhalla_exception_t{400, 123};
      //too much
      else if(shape.size() > max_shape)
        throw valhalla_exception_t{400, 153, "(" + std::to_string(shape.size()) +"). The limit is " + std::to_string(max_shape)};

      valhalla::midgard::logging::Log("trace_size::" + std::to_string(shape.size()), " [ANALYTICS] ");

    }

    void loki_worker_t::parse_costing(const boost::property_tree::ptree& request) {
      //using the costing we can determine what type of edge filtering to use
      auto costing = request.get_optional<std::string>("costing");
      if (costing)
        valhalla::midgard::logging::Log("costing_type::" + *costing, " [ANALYTICS] ");
      else
        throw valhalla_exception_t{400, 124};

      // TODO - have a way of specifying mode at the location
      if(*costing == "multimodal")
        *costing = "pedestrian";

      // Get the costing options if in the config or get the empty default.
      // Creates the cost in the cost factory
      std::string method_options = "costing_options." + *costing;
      auto costing_options = request.get_child(method_options,{});
      try{
        auto c = factory.Create(*costing, costing_options);
        edge_filter = c->GetEdgeFilter();
        node_filter = c->GetNodeFilter();
      }
      catch(const std::runtime_error&) {
        throw valhalla_exception_t{400, 125, "'" + *costing + "'"};
      }
    }

    loki_worker_t::loki_worker_t(const boost::property_tree::ptree& config):
        config(config), reader(config.get_child("mjolnir")), connectivity_map(config.get_child("mjolnir")),
        long_request(config.get<float>("loki.logging.long_request")),
        max_contours(config.get<unsigned int>("service_limits.isochrone.max_contours")),
        max_time(config.get<unsigned int>("service_limits.isochrone.max_time")),
        max_shape(config.get<size_t>("service_limits.trace_route.max_shape")) {

      // Keep a string noting which actions we support, throw if one isnt supported
      for (const auto& kv : config.get_child("loki.actions")) {
        auto path = "/" + kv.second.get_value<std::string>();
        if(PATH_TO_ACTION.find(path) == PATH_TO_ACTION.cend())
          throw valhalla_exception_t{400, 105, path};
        action_str.append("'" + path + "' ");
        actions.insert(path);
      }
      // Make sure we have at least something to support!
      if(action_str.empty())
        throw valhalla_exception_t{400, 102};

      //Build max_locations and max_distance maps
      for (const auto& kv : config.get_child("service_limits")) {
        if (kv.first != "skadi" && kv.first != "trace_route")
          max_locations.emplace(kv.first, config.get<size_t>("service_limits." + kv.first + ".max_locations"));
        if (kv.first != "skadi" && kv.first != "isochrone" && kv.first != "trace_route")
          max_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_distance"));
      }
      //this should never happen
      if (max_locations.empty())
        throw valhalla_exception_t{400, 103};

      if (max_distance.empty())
        throw valhalla_exception_t{400, 104};

      min_transit_walking_dis =
        config.get<int>("service_limits.pedestrian.min_transit_walking_distance");
      max_transit_walking_dis =
        config.get<int>("service_limits.pedestrian.max_transit_walking_distance");

      // Register edge/node costing methods
      // TODO: move this into the loop above
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", sif::CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("hov", sif::CreateHOVCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("truck", sif::CreateTruckCost);
      factory.Register("transit", sif::CreateTransitCost);

    }

    worker_t::result_t loki_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const worker_t::interrupt_function_t&) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Loki Request " + std::to_string(info.id));

      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

        //block all but get and post
        if(request.method != method_t::POST && request.method != method_t::GET)
          return jsonify_error({405, 101}, info);

        //is the request path action in the action set?
        auto action = PATH_TO_ACTION.find(request.path);
        if (action == PATH_TO_ACTION.cend() || actions.find(request.path) == actions.cend())
          return jsonify_error({404, 106, action_str}, info);

        //parse the query's json
        auto request_pt = from_request(action->second, request);
        jsonp = request_pt.get_optional<std::string>("jsonp");
        //let further processes more easily know what kind of request it was
        request_pt.put<int>("action", action->second);
        //let further processes know about tracking
        auto do_not_track = request.headers.find("DNT");
        info.spare = do_not_track != request.headers.cend() && do_not_track->second == "1";

        worker_t::result_t result{false};
        //do request specific processing
        switch (action->second) {
          case ROUTE:
          case VIAROUTE:
            result = route(request_pt, info);
            break;
          case LOCATE:
            result = locate(request_pt, info);
            break;
          case ONE_TO_MANY:
          case MANY_TO_ONE:
          case MANY_TO_MANY:
          case SOURCES_TO_TARGETS:
          case OPTIMIZED_ROUTE:
            result = matrix(action->second, request_pt, info);
            break;
          case ISOCHRONE:
            result = isochrones(request_pt, info);
            break;
          case TRACE_ATTRIBUTES:
          case TRACE_ROUTE:
            result = trace_route(request_pt, info);
            break;
          default:
            //apparently you wanted something that we figured we'd support but havent written yet
            return jsonify_error({501, 107}, info);
        }
        //get processing time for loki
        auto e = std::chrono::system_clock::now();
        std::chrono::duration<float, std::milli> elapsed_time = e - s;
        //log request if greater than X (ms)
        auto work_units = locations.size() ? locations.size() : 1;
        if (!info.spare && elapsed_time.count() / work_units > long_request) {
          std::stringstream ss;
          boost::property_tree::json_parser::write_json(ss, request_pt, false);
          LOG_WARN("loki::request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
          LOG_WARN("loki::request exceeded threshold::"+ ss.str());
          midgard::logging::Log("valhalla_loki_long_request", " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({e.status_code, e.error_code, e.extra}, info);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({400, 199, std::string(e.what())}, info);
      }
    }

    void loki_worker_t::cleanup() {
      jsonp = boost::none;
      locations.clear();
      sources.clear();
      targets.clear();
      shape.clear();
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
      auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

      //listen for requests
      zmq::context_t context;
      loki_worker_t loki_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint, interrupt_endpoint,
        std::bind(&loki_worker_t::work, std::ref(loki_worker), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        std::bind(&loki_worker_t::cleanup, std::ref(loki_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }
  }
}
