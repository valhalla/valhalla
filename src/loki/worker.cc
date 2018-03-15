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
    std::vector<baldr::Location> loki_worker_t::parse_locations(const valhalla_request_t& request, const std::string& node,
      unsigned location_parse_error_code, boost::optional<valhalla_exception_t> required_exception) {
      std::vector<baldr::Location> parsed;
      auto request_locations = rapidjson::get_optional<rapidjson::Value::ConstArray>(request.document, std::string("/" + node).c_str());
      if (request_locations) {
        for(const auto& location : *request_locations) {
          try { parsed.push_back(baldr::Location::FromRapidJson(location, default_reachability, default_radius)); }
          catch (...) { throw valhalla_exception_t{location_parse_error_code}; }
          if(parsed.back().minimum_reachability_ > max_reachability)
            parsed.back().minimum_reachability_ = max_reachability;
          if(parsed.back().radius_ > max_radius)
            parsed.back().radius_ = max_radius;
        }
        if (!request.options.do_not_track())
          valhalla::midgard::logging::Log(node + "_count::" + std::to_string(request_locations->Size()), " [ANALYTICS] ");
      }
      else if(required_exception)
        throw *required_exception;
      return parsed;
    }

    void loki_worker_t::parse_costing(valhalla_request_t& request) {
      //using the costing we can determine what type of edge filtering to use
      auto costing = rapidjson::get_optional<std::string>(request.document, "/costing");
      if (!costing)
        throw valhalla_exception_t{124};
      else if (!request.options.do_not_track())
        valhalla::midgard::logging::Log("costing_type::" + *costing, " [ANALYTICS] ");

      // TODO - have a way of specifying mode at the location
      if(*costing == "multimodal")
        *costing = "pedestrian";

      // Get the costing options if in the config or make a blank one.
      // Creates the cost in the cost factory
      auto* method_options_ptr = rapidjson::Pointer{"/costing_options/" + *costing}.Get(request.document);
      auto& allocator = request.document.GetAllocator();
      if(!method_options_ptr) {
        auto* costing_options = rapidjson::Pointer{"/costing_options"}.Get(request.document);
        if(!costing_options) {
          request.document.AddMember(rapidjson::Value("costing_options", allocator), rapidjson::Value(rapidjson::kObjectType), allocator);
          costing_options = rapidjson::Pointer{"/costing_options"}.Get(request.document);
        }
        costing_options->AddMember(rapidjson::Value(*costing, allocator), rapidjson::Value{rapidjson::kObjectType}, allocator);
        method_options_ptr = rapidjson::Pointer{"/costing_options/" + *costing}.Get(request.document);
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
        max_trace_shape(config.get<size_t>("service_limits.trace.max_shape")),
        sample(config.get<std::string>("additional_data.elevation", "test/data/")),
        max_elevation_shape(config.get<size_t>("service_limits.skadi.max_shape")),
        min_resample(config.get<float>("service_limits.skadi.min_resample")) {

      // Keep a string noting which actions we support, throw if one isnt supported
      odin::DirectionsOptions::Action action;
      for (const auto& kv : config.get_child("loki.actions")) {
        auto path = kv.second.get_value<std::string>();
        if(!odin::DirectionsOptions::Action_Parse(path, &action))
          throw std::runtime_error("Action not supported " + path);
        action_str.append("'/" + path + "' ");
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
      valhalla_request_t request;
      try{
        //request parsing
        auto http_request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        request = std::move(valhalla_request_t(http_request));

        //check there is a valid action
        if(!request.options.has_action())
          return jsonify_error({106, action_str}, info, request);

        // Set the interrupt function
        service_worker_t::set_interrupt(interrupt_function);

        worker_t::result_t result{true};
        //do request specific processing
        switch (request.options.action()) {
          case odin::DirectionsOptions::route:
            route(request);
            result.messages.emplace_back(rapidjson::to_string(request.document));
            result.messages.emplace_back(request.options.SerializeAsString());
            break;
          case odin::DirectionsOptions::locate:
            result = to_response_json(locate(request), info, request);
            break;
          case odin::DirectionsOptions::sources_to_targets:
          case odin::DirectionsOptions::optimized_route:
            matrix(request);
            result.messages.emplace_back(rapidjson::to_string(request.document));
            result.messages.emplace_back(request.options.SerializeAsString());
            break;
          case odin::DirectionsOptions::isochrone:
            isochrones(request);
            result.messages.emplace_back(rapidjson::to_string(request.document));
            result.messages.emplace_back(request.options.SerializeAsString());
            break;
          case odin::DirectionsOptions::trace_attributes:
          case odin::DirectionsOptions::trace_route:
            trace(request);
            result.messages.emplace_back(rapidjson::to_string(request.document));
            result.messages.emplace_back(request.options.SerializeAsString());
            break;
          case odin::DirectionsOptions::height:
            result = to_response_json(height(request), info, request);
            break;
          case odin::DirectionsOptions::transit_available:
            result = to_response_json(transit_available(request), info, request);
            break;
          default:
            //apparently you wanted something that we figured we'd support but havent written yet
            return jsonify_error({107}, info, request);
        }
        //get processing time for loki
        auto e = std::chrono::system_clock::now();
        std::chrono::duration<float, std::milli> elapsed_time = e - s;
        //log request if greater than X (ms)
        auto work_units = locations.size() ? locations.size() : shape.size() * 20;
        if (!request.options.do_not_track() && elapsed_time.count() / work_units > long_request) {
          LOG_WARN("loki::request elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
          LOG_WARN("loki::request exceeded threshold::"+ rapidjson::to_string(request.document));
          midgard::logging::Log("valhalla_loki_long_request", " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error(e, info, request);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({199, std::string(e.what())}, info, request);
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
