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
#include "midgard/logging.h"
#include "midgard/constants.h"
#include "baldr/json.h"
#include "baldr/geojson.h"
#include "baldr/errorcode_util.h"

#include <prime_server/prime_server.hpp>

#include "thor/service.h"
#include "thor/isochrone.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::meili;
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
        throw valhalla_exception_t{400, 420};
      }
    }while(++i);
    return correlated;
  }
}

namespace valhalla {
  namespace thor {

    const std::unordered_map<std::string, thor_worker_t::SHAPE_MATCH> thor_worker_t::STRING_TO_MATCH {
      {"edge_walk", thor_worker_t::EDGE_WALK},
      {"map_snap", thor_worker_t::MAP_SNAP},
      {"walk_or_snap", thor_worker_t::WALK_OR_SNAP}
    };

    thor_worker_t::thor_worker_t(const boost::property_tree::ptree& config):
      mode(valhalla::sif::TravelMode::kPedestrian),
      config(config), matcher_factory(config), reader(matcher_factory.graphreader()),
      long_request(config.get<float>("thor.logging.long_request")){
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("hov", sif::CreateHOVCost);
      factory.Register("pedestrian", sif::CreatePedestrianCost);
      factory.Register("transit", sif::CreateTransitCost);
      factory.Register("truck", sif::CreateTruckCost);

      for (const auto& item : config.get_child("meili.customizable")) {
        trace_customizable.insert(item.second.get_value<std::string>());
      }

      // Select the matrix algorithm based on the conf file (defaults to
      // select_optimal if not present)
      auto conf_algorithm = config.get<std::string>("thor.source_to_target_algorithm",
                                                          "select_optimal");
      if (conf_algorithm == "timedistancematrix") {
        source_to_target_algorithm = TIME_DISTANCE_MATRIX;
      } else if (conf_algorithm == "costmatrix") {
        source_to_target_algorithm = COST_MATRIX;
      } else {
        source_to_target_algorithm = SELECT_OPTIMAL;
      }

      interrupt_callback = nullptr;
    }

    thor_worker_t::~thor_worker_t(){}

    worker_t::result_t thor_worker_t::jsonify_error(const valhalla_exception_t& exception, http_request_info_t& request_info) const {

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

    worker_t::result_t thor_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const worker_t::interrupt_function_t& interrupt) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
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
          return jsonify_error({500, 499, std::string(e.what())}, info);
        }
        catch(...) {
          valhalla::midgard::logging::Log("500::non-std::exception", " [ANALYTICS] ");
          return jsonify_error({500, 401}, info);
        }

        // Set the interrupt function
        interrupt_callback = &interrupt;

        //flag healthcheck requests; do not send to logstash
        healthcheck = request.get<bool>("healthcheck", false);
        // Initialize request - get the PathALgorithm to use
        ACTION_TYPE action = static_cast<ACTION_TYPE>(request.get<int>("action"));
        // Allow the request to be aborted
        astar.set_interrupt(&interrupt);
        bidir_astar.set_interrupt(&interrupt);
        multi_modal_astar.set_interrupt(&interrupt);
        //what action is it
        switch (action) {
          case ONE_TO_MANY:
          case MANY_TO_ONE:
          case MANY_TO_MANY:
          case SOURCES_TO_TARGETS:
            return matrix(action, request, info);
          case OPTIMIZED_ROUTE:
            return optimized_route(request, request_str, info.spare);
          case ISOCHRONE:
            return isochrone(request, info);
          case ROUTE:
          case VIAROUTE:
            return route(request, request_str, request.get_optional<int>("date_time.type"), info.spare);
          case TRACE_ROUTE:
            return trace_route(request, request_str, info.spare);
          case TRACE_ATTRIBUTES:
            return trace_attributes(request, request_str, info);
          default:
            throw valhalla_exception_t{400, 400}; //this should never happen
        }
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({e.status_code, e.error_code, e.extra}, info);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({400, 499, std::string(e.what())}, info);
      }
    }

    // Get the costing options if in the config or get the empty default.
    // Creates the cost in the cost factory
    valhalla::sif::cost_ptr_t thor_worker_t::get_costing(const boost::property_tree::ptree& request,
                                          const std::string& costing) {
      std::string method_options = "costing_options." + costing;
      auto costing_options = request.get_child(method_options, {});
      return factory.Create(costing, costing_options);
    }

    std::string thor_worker_t::parse_costing(const boost::property_tree::ptree& request) {
      // Parse out the type of route - this provides the costing method to use
      auto costing = request.get<std::string>("costing");

      // Set travel mode and construct costing
      if (costing == "multimodal" || costing == "transit") {
        // For multi-modal we construct costing for all modes and set the
        // initial mode to pedestrian. (TODO - allow other initial modes)
        mode_costing[0] = get_costing(request, "auto");
        mode_costing[1] = get_costing(request, "pedestrian");
        mode_costing[2] = get_costing(request, "bicycle");
        mode_costing[3] = get_costing(request, "transit");
        mode = valhalla::sif::TravelMode::kPedestrian;
      } else {
        valhalla::sif::cost_ptr_t cost = get_costing(request, costing);
        mode = cost->travel_mode();
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
          catch (...) { throw valhalla_exception_t{400, 421}; }
        }
        correlated = store_correlated_locations(request, locations);
      }//if we have a sources and targets request here we will divvy up the correlated amongst them
      else if(request_sources && request_targets) {
        for(const auto& s : *request_sources) {
          try{ locations.push_back(baldr::Location::FromPtree(s.second)); }
          catch (...) { throw valhalla_exception_t{400, 422}; }
        }
        for(const auto& t : *request_targets) {
          try{ locations.push_back(baldr::Location::FromPtree(t.second)); }
          catch (...) { throw valhalla_exception_t{400, 423}; }
        }
        correlated = store_correlated_locations(request, locations);

        correlated_s.insert(correlated_s.begin(), correlated.begin(), correlated.begin() + request_sources->size());
        correlated_t.insert(correlated_t.begin(), correlated.begin() + request_sources->size(), correlated.end());
      }//we need something
      else
        throw valhalla_exception_t{400, 410};

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

    void thor_worker_t::parse_shape(const boost::property_tree::ptree& request) {
      //we require locations
      auto request_shape = request.get_child("shape");

      for(const auto& pt : request_shape) {
        try{
          shape.push_back(baldr::Location::FromPtree(pt.second).latlng_);
        }
        catch (...) {
          throw std::runtime_error("Failed to parse shape");
        }
      }

    }

    void thor_worker_t::parse_trace_config(const boost::property_tree::ptree& request) {
      auto costing = request.get<std::string>("costing");
      trace_config.put<std::string>("mode", costing);

      if (trace_customizable.empty()) {
        return;
      }

      auto trace_options = request.get_child_optional("trace_options");
      if (!trace_options) {
        return;
      }
      
      for (const auto& pair : *trace_options) {
        const auto& name = pair.first;
        const auto& values = pair.second.data();
        if (trace_customizable.find(name) != trace_customizable.end()
            && !values.empty() ){
          try {
            // Possibly throw std::invalid_argument or std::out_of_range
            trace_config.put<float>(name, std::stof(values));
          } catch (const std::invalid_argument& ex) {
            throw std::invalid_argument("Invalid argument: unable to parse " + name + " to float");
          } catch (const std::out_of_range& ex) {
            throw std::out_of_range("Invalid argument: " + name + " is out of float range");
          }
        }
      }
    }

    void thor_worker_t::log_admin(valhalla::odin::TripPath& trip_path) {
      if (!healthcheck) {
        std::unordered_set<std::string> state_iso;
        std::unordered_set<std::string> country_iso;
        std::stringstream s_ss, c_ss;
        if (trip_path.admin_size() > 0) {
          for (const auto& admin : trip_path.admin()) {
            if (admin.has_state_code())
              state_iso.insert(admin.state_code());
            if (admin.has_country_code())
              country_iso.insert(admin.country_code());
          }
          for (const std::string& x: state_iso)
            s_ss << " " << x;
          for (const std::string& x: country_iso)
            c_ss << " " << x;
          if (!s_ss.eof()) valhalla::midgard::logging::Log("admin_state_iso::" + s_ss.str() + ' ', " [ANALYTICS] ");
          if (!c_ss.eof()) valhalla::midgard::logging::Log("admin_country_iso::" + c_ss.str() + ' ', " [ANALYTICS] ");
        }
      }
    }

    void thor_worker_t::cleanup() {
      jsonp = boost::none;
      astar.Clear();
      bidir_astar.Clear();
      multi_modal_astar.Clear();
      locations.clear();
      shape.clear();
      correlated.clear();
      correlated_s.clear();
      correlated_t.clear();
      isochrone_gen.Clear();
      matcher_factory.ClearFullCache();
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
      auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

      //listen for requests
      zmq::context_t context;
      thor_worker_t thor_worker(config);
      prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint, interrupt_endpoint,
        std::bind(&thor_worker_t::work, std::ref(thor_worker), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        std::bind(&thor_worker_t::cleanup, std::ref(thor_worker)));
      worker.work();

      //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
    }

  }
}
