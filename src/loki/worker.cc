#include <functional>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/pedestriancost.h"
#include "sif/motorscootercost.h"
#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "tyr/actor.h"

#include "loki/worker.h"
#include "loki/search.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace valhalla {
  namespace loki {
    std::vector<baldr::Location> loki_worker_t::parse_locations(const rapidjson::Document& request, const std::string& node,
      unsigned location_parse_error_code, boost::optional<valhalla_exception_t> required_exception) {
      std::vector<baldr::Location> parsed;
      auto request_locations = GetOptionalFromRapidJson<rapidjson::Value::ConstArray>(request, std::string("/" + node).c_str());
      if (request_locations) {
        for(const auto& location : *request_locations) {
          try { parsed.push_back(baldr::Location::FromRapidJson(location, default_reachability, default_radius)); }
          catch (...) { throw valhalla_exception_t{location_parse_error_code}; }
          if(parsed.back().minimum_reachability_ > max_reachability)
            parsed.back().minimum_reachability_ = max_reachability;
          if(parsed.back().radius_ > max_radius)
            parsed.back().radius_ = max_radius;
        }
        if (!healthcheck)
          valhalla::midgard::logging::Log(node + "_count::" + std::to_string(request_locations->Size()), " [ANALYTICS] ");
      }
      else if(required_exception)
        throw *required_exception;
      return parsed;
    }

    void loki_worker_t::parse_costing(rapidjson::Document& request) {
      //using the costing we can determine what type of edge filtering to use
      auto costing = GetOptionalFromRapidJson<std::string>(request, "/costing");
      if (!costing)
        throw valhalla_exception_t{124};
      else if (!healthcheck)
        valhalla::midgard::logging::Log("costing_type::" + *costing, " [ANALYTICS] ");

      // TODO - have a way of specifying mode at the location
      if(*costing == "multimodal")
        *costing = "pedestrian";

      // Get the costing options if in the config or make a blank one.
      // Creates the cost in the cost factory
      auto* method_options_ptr = rapidjson::Pointer{"/costing_options/" + *costing}.Get(request);
      auto& allocator = request.GetAllocator();
      if(!method_options_ptr) {
        auto* costing_options = rapidjson::Pointer{"/costing_options"}.Get(request);
        if(!costing_options) {
          request.AddMember(rapidjson::Value("costing_options", allocator), rapidjson::Value(rapidjson::kObjectType), allocator);
          costing_options = rapidjson::Pointer{"/costing_options"}.Get(request);
        }
        costing_options->AddMember(rapidjson::Value(*costing, allocator), rapidjson::Value{rapidjson::kObjectType}, allocator);
        method_options_ptr = rapidjson::Pointer{"/costing_options/" + *costing}.Get(request);
      }

      try{
        cost_ptr_t c;
        c = factory.Create(*costing, *method_options_ptr);
        edge_filter = c->GetEdgeFilter();
        node_filter = c->GetNodeFilter();
      }
      catch(const std::runtime_error&) {
        throw valhalla_exception_t{125, "'" + *costing + "'"};
      }

      // See if we have avoids and take care of them
      auto avoid_locations = parse_locations(request, "avoid_locations", 133, boost::none);
      if(!avoid_locations.empty()) {
        if(avoid_locations.size() > max_avoid_locations)
          throw valhalla_exception_t{157, std::to_string(max_avoid_locations)};
        try {
          auto results = loki::Search(avoid_locations, reader, edge_filter, node_filter);
          std::unordered_set<uint64_t> avoids;
          for(const auto& result : results) {
            for(const auto& edge : result.second.edges) {
              auto inserted = avoids.insert(edge.id);
              GraphId shortcut;
              if(inserted.second && (shortcut = reader.GetShortcut(edge.id)).Is_Valid())
                avoids.insert(shortcut);
            }
          }
          rapidjson::Value avoid_edges{rapidjson::kArrayType};
          for(auto avoid : avoids)
            avoid_edges.PushBack(rapidjson::Value(avoid), allocator);
          method_options_ptr->AddMember("avoid_edges", avoid_edges, allocator);
        }//swallow all failures on optional avoids
        catch(...) {
          LOG_WARN("Failed to find avoid_locations");
        }
      }
    }

    loki_worker_t::loki_worker_t(const boost::property_tree::ptree& config):
        config(config), reader(config.get_child("mjolnir")),
        connectivity_map(config.get<bool>("loki.use_connectivity", true) ? new connectivity_map_t(config.get_child("mjolnir")) : nullptr),
        long_request(config.get<float>("loki.logging.long_request")),
        max_contours(config.get<size_t>("service_limits.isochrone.max_contours")),
        max_time(config.get<size_t>("service_limits.isochrone.max_time")),
        max_shape(config.get<size_t>("service_limits.trace.max_shape")),
        healthcheck(false) {

      // Keep a string noting which actions we support, throw if one isnt supported
      for (const auto& kv : config.get_child("loki.actions")) {
        auto path = "/" + kv.second.get_value<std::string>();
        if(PATH_TO_ACTION.find(path) == PATH_TO_ACTION.cend())
          throw std::runtime_error("Path action not supported " + path);
        action_str.append("'" + path + "' ");
        actions.insert(path);
      }
      // Make sure we have at least something to support!
      if(action_str.empty())
        throw std::runtime_error("The config actions for Loki are incorrectly loaded");

      //Build max_locations and max_distance maps
      for (const auto& kv : config.get_child("service_limits")) {
        if(kv.first == "max_avoid_locations" || kv.first == "max_reachability" || kv.first == "max_radius")
          continue;
        if (kv.first != "skadi" && kv.first != "trace")
          max_locations.emplace(kv.first, config.get<size_t>("service_limits." + kv.first + ".max_locations"));
        if (kv.first != "skadi")
          max_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_distance"));
        if (kv.first != "skadi" && kv.first != "trace" && kv.first != "isochrone") {
          max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_matrix_distance"));
          max_matrix_locations.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_matrix_locations"));
        }
      }
      //this should never happen
      if (max_locations.empty())
        throw std::runtime_error("Missing max_locations configuration");

      if (max_distance.empty())
        throw std::runtime_error("Missing max_distance configuration");

      if (max_matrix_distance.empty())
        throw std::runtime_error("Missing max_matrix_distance configuration");

      if (max_matrix_locations.empty())
        throw std::runtime_error("Missing max_matrix_locations configuration");

      min_transit_walking_dis =
        config.get<size_t>("service_limits.pedestrian.min_transit_walking_distance");
      max_transit_walking_dis =
        config.get<size_t>("service_limits.pedestrian.max_transit_walking_distance");

      max_avoid_locations = config.get<size_t>("service_limits.max_avoid_locations");
      max_reachability = config.get<unsigned int>("service_limits.max_reachability");
      default_reachability = config.get<unsigned int>("loki.service_defaults.minimum_reachability");
      max_radius = config.get<unsigned long>("service_limits.max_radius");
      default_radius = config.get<unsigned long>("loki.service_defaults.radius");
      max_gps_accuracy = config.get<float>("service_limits.trace.max_gps_accuracy");
      max_search_radius = config.get<float>("service_limits.trace.max_search_radius");
      max_best_paths = config.get<unsigned int>("service_limits.trace.max_best_paths");
      max_best_paths_shape = config.get<size_t>("service_limits.trace.max_best_paths_shape");


      // Register edge/node costing methods
      // TODO: move this into the loop above
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", sif::CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("hov", sif::CreateHOVCost);
      factory.Register("motor_scooter", sif::CreateMotorScooterCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("truck", sif::CreateTruckCost);
      factory.Register("transit", sif::CreateTransitCost);
    }

    void loki_worker_t::cleanup() {
      locations.clear();
      sources.clear();
      targets.clear();
      shape.clear();
      if(reader.OverCommitted())
        reader.Clear();
    }

#ifdef HAVE_HTTP
    worker_t::result_t loki_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt_function) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Loki Request " + std::to_string(info.id));
      boost::optional<std::string> jsonp;
      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

        //is the request path action in the action set?
        auto action = PATH_TO_ACTION.find(request.path);
        if (action == PATH_TO_ACTION.cend() || actions.find(request.path) == actions.cend())
          return jsonify_error({106, action_str}, info);

        //parse the query's json
        auto request_rj = from_request(request);
        jsonp = GetOptionalFromRapidJson<std::string>(request_rj, "/jsonp");
        //let further processes more easily know what kind of request it was
        rapidjson::SetValueByPointer(request_rj, "/action", action->second);
        //flag healthcheck requests; do not send to logstash
        healthcheck = GetOptionalFromRapidJson<bool>(request_rj, "/healthcheck").get_value_or(false);
        //let further processes know about tracking
        auto do_not_track = request.headers.find("DNT");
        info.spare = do_not_track != request.headers.cend() && do_not_track->second == "1";

        // Set the interrupt function
        service_worker_t::set_interrupt(interrupt_function);

        worker_t::result_t result{true};
        //do request specific processing
        switch (action->second) {
          case ROUTE:
          case VIAROUTE:
            route(request_rj);
            result.messages.emplace_back(rapidjson::to_string(request_rj));
            break;
          case LOCATE:
            result = to_response(locate(request_rj), jsonp, info);
            break;
          case ONE_TO_MANY:
          case MANY_TO_ONE:
          case MANY_TO_MANY:
          case SOURCES_TO_TARGETS:
          case OPTIMIZED_ROUTE:
            matrix(action->second, request_rj);
            result.messages.emplace_back(rapidjson::to_string(request_rj));
            break;
          case ISOCHRONE:
            isochrones(request_rj);
            result.messages.emplace_back(rapidjson::to_string(request_rj));
            break;
          case TRACE_ATTRIBUTES:
          case TRACE_ROUTE:
            trace(action->second, request_rj);
            result.messages.emplace_back(rapidjson::to_string(request_rj));
            break;
          case TRANSIT_AVAILABLE:
            result = to_response(transit_available(request_rj), jsonp, info);
            break;
          default:
            //apparently you wanted something that we figured we'd support but havent written yet
            return jsonify_error({107}, info);
        }
        //get processing time for loki
        auto e = std::chrono::system_clock::now();
        std::chrono::duration<float, std::milli> elapsed_time = e - s;
        //log request if greater than X (ms)
        auto work_units = locations.size() ? locations.size() : 1;
        if (!healthcheck && !info.spare && elapsed_time.count() / work_units > long_request) {
          LOG_WARN("loki::request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
          LOG_WARN("loki::request exceeded threshold::"+ rapidjson::to_string(request_rj));
          midgard::logging::Log("valhalla_loki_long_request", " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error(e, info, jsonp);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({199, std::string(e.what())}, info, jsonp);
      }
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
#endif
  }
}
