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
#include "exception.h"

#include "thor/worker.h"
#include "thor/isochrone.h"
#include "tyr/actor.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::meili;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {
  //maximum edge score (24 hours)
  constexpr float kMaxScore = 86400.0f;

  constexpr double kMilePerMeter = 0.000621371;

  std::vector<odin::Location> store_correlated_locations(const rapidjson::Document& request, const std::vector<baldr::Location>& locations) {
    //we require correlated locations
    std::vector<odin::Location> correlated;
    correlated.reserve(locations.size());
    size_t i = 0;
    do {
      auto path_location = rapidjson::get_child_optional(request, ("/correlated_" + std::to_string(i)).c_str());
      if(!path_location)
        break;

      try {
        correlated.emplace_back(PathLocation::FromRapidJson(locations, *path_location));
        auto minScoreEdge = *std::min_element (correlated.back().edges.begin(), correlated.back().edges.end(),
          [](PathLocation::PathEdge i, PathLocation::PathEdge j)->bool {
            return i.score < j.score;
          });

        for(auto& e : correlated.back().edges) {
          e.score -= minScoreEdge.score;
          if (e.score > kMaxScore) {
            e.score = kMaxScore;
          }
        }
      }
      catch (...) {
        throw valhalla_exception_t{420};
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
      matcher_factory(config), reader(matcher_factory.graphreader()),
      long_request(config.get<float>("thor.logging.long_request")){
      // Register edge/node costing methods
      factory.Register("auto", sif::CreateAutoCost);
      factory.Register("auto_shorter", sif::CreateAutoShorterCost);
      factory.Register("bus", sif::CreateBusCost);
      factory.Register("bicycle", sif::CreateBicycleCost);
      factory.Register("hov", sif::CreateHOVCost);
      factory.Register("motor_scooter", sif::CreateMotorScooterCost);
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
      for (const auto& kv : config.get_child("service_limits")) {
        if(kv.first == "max_avoid_locations" || kv.first == "max_reachability" || kv.first == "max_radius")
          continue;
        if (kv.first != "skadi" && kv.first != "trace" && kv.first != "isochrone") {
          max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_matrix_distance"));
        }
      }

      if (conf_algorithm == "timedistancematrix") {
        source_to_target_algorithm = TIME_DISTANCE_MATRIX;
      } else if (conf_algorithm == "costmatrix") {
        source_to_target_algorithm = COST_MATRIX;
      } else {
        source_to_target_algorithm = SELECT_OPTIMAL;
      }
    }

    thor_worker_t::~thor_worker_t(){}

#ifdef HAVE_HTTP
    worker_t::result_t thor_worker_t::work(const std::list<zmq::message_t>& job, void* request_info, const std::function<void ()>& interrupt_function) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      auto& info = *static_cast<http_request_info_t*>(request_info);
      LOG_INFO("Got Thor Request " + std::to_string(info.id));
      valhalla_request_t request;
      try{
        //crack open the original request
        std::string request_str(static_cast<const char*>(job.front().data()), job.front().size());
        std::string serialized_options(static_cast<const char*>(job.back().data()), job.back().size());
        request = std::move(valhalla_request_t(request_str, serialized_options));

        // Set the interrupt function
        service_worker_t::set_interrupt(interrupt_function);

        boost::optional<int> date_time_type = rapidjson::get_optional<int>(request.document, "/date_time/type");

        worker_t::result_t result{true};
        double denominator = 0;
        size_t order_index = 0;
        //do request specific processing
        switch (request.options.action()) {
          case odin::DirectionsOptions::sources_to_targets:
            result = to_response_json(matrix(request), info, request);
            denominator = request.options.sources_size() + request.options.targets_size();
            break;
          case odin::DirectionsOptions::optimized_route:
            // Forward the original request
            result.messages.emplace_back(std::move(request_str));
            for (auto& trippath : optimized_route(request)) {
              for (auto& location : *trippath.mutable_location())
                location.set_original_index(optimal_order[order_index++]);
              --order_index;
              result.messages.emplace_back(trippath.SerializeAsString());
            }
            denominator = std::max(request.options.sources_size(), request.options.targets_size());
            break;
          case odin::DirectionsOptions::isochrone:
            result = to_response_json(isochrones(request), info, request);
            denominator = request.options.sources_size() * request.options.targets_size();
            break;
          case odin::DirectionsOptions::route:
            // Forward the original request
            result.messages.emplace_back(std::move(request_str));
            result.messages.emplace_back(std::move(serialized_options));
            for (const auto& trippath : route(request, date_time_type))
              result.messages.emplace_back(trippath.SerializeAsString());
            denominator = correlated.size();
            break;
          case odin::DirectionsOptions::trace_route:
            // Forward the original request
            result.messages.emplace_back(std::move(request_str));
            result.messages.emplace_back(std::move(serialized_options));
            result.messages.emplace_back(trace_route(request).SerializeAsString());
            denominator = trace.size() / 1100;
            break;
          case odin::DirectionsOptions::trace_attributes:
            result = to_response_json(trace_attributes(request), info, request);
            denominator = trace.size() / 1100;
            break;
          default:
            throw valhalla_exception_t{400}; //this should never happen
        }

        double elapsed_time = std::chrono::duration<float, std::milli>(std::chrono::system_clock::now() - s).count();
        if (!request.options.do_not_track() && elapsed_time / denominator > long_request) {
          LOG_WARN("thor::" + odin::DirectionsOptions::Action_Name(request.options.action()) + " request elapsed time (ms)::"+ std::to_string(elapsed_time));
          LOG_WARN("thor::" + odin::DirectionsOptions::Action_Name(request.options.action()) + " request exceeded threshold::"+ request_str);
          midgard::logging::Log("valhalla_thor_long_request_"+odin::DirectionsOptions::Action_Name(request.options.action()), " [ANALYTICS] ");
        }

        return result;
      }
      catch(const valhalla_exception_t& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error(e, info, request);
      }
      catch(const std::exception& e) {
        valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
        return jsonify_error({499, std::string(e.what())}, info, request);
      }
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
#endif

    // Get the costing options if in the config or get the empty default.
    // Creates the cost in the cost factory
    valhalla::sif::cost_ptr_t thor_worker_t::get_costing(const rapidjson::Document& request,
                                          const std::string& costing) {
      auto costing_options = rapidjson::get_child_optional(request, ("/costing_options/" + costing).c_str());
      if(costing_options)
        return factory.Create(costing, *costing_options);
      return factory.Create(costing, boost::property_tree::ptree{});
    }

    std::string thor_worker_t::parse_costing(const valhalla_request_t& request) {
      // Parse out the type of route - this provides the costing method to use
      auto costing = rapidjson::get<std::string>(request.document,  "/costing");

      // Set travel mode and construct costing
      if (costing == "multimodal" || costing == "transit") {
        // For multi-modal we construct costing for all modes and set the
        // initial mode to pedestrian. (TODO - allow other initial modes)
        mode_costing[0] = get_costing(request.document, "auto");
        mode_costing[1] = get_costing(request.document, "pedestrian");
        mode_costing[2] = get_costing(request.document, "bicycle");
        mode_costing[3] = get_costing(request.document, "transit");
        mode = valhalla::sif::TravelMode::kPedestrian;
      } else {
        valhalla::sif::cost_ptr_t cost = get_costing(request.document, costing);
        mode = cost->travel_mode();
        mode_costing[static_cast<uint32_t>(mode)] = cost;
      }
      valhalla::midgard::logging::Log("travel_mode::" + std::to_string(static_cast<uint32_t>(mode)), " [ANALYTICS] ");
      return costing;
    }

    void thor_worker_t::parse_locations(const valhalla_request_t& request) {
      //we require locations
      auto request_locations = rapidjson::get_optional<rapidjson::Value::ConstArray>(request.document, "/locations");
      auto request_sources = rapidjson::get_optional<rapidjson::Value::ConstArray>(request.document, "/sources");
      auto request_targets = rapidjson::get_optional<rapidjson::Value::ConstArray>(request.document, "/targets");
      if(request_locations) {
        for(const auto& location : *request_locations) {
          try{ locations.push_back(baldr::Location::FromRapidJson(location)); }
          catch (...) { throw valhalla_exception_t{421}; }
        }
        correlated = store_correlated_locations(request.document, locations);
      }//if we have a sources and targets request here we will divy up the correlated amongst them
      else if(request_sources && request_targets) {
        for(const auto& s : *request_sources) {
          try{ locations.push_back(baldr::Location::FromRapidJson(s)); }
          catch (...) { throw valhalla_exception_t{422}; }
        }
        for(const auto& t : *request_targets) {
          try{ locations.push_back(baldr::Location::FromRapidJson(t)); }
          catch (...) { throw valhalla_exception_t{423}; }
        }
        correlated = store_correlated_locations(request.document, locations);
      }

      //type - 0: current, 1: depart, 2: arrive
      if (!request.document.HasMember("/date_time/type"))
        return;

      int date_time_type = rapidjson::get<float>(request.document, "/date_time/type");
      auto date_time_value = rapidjson::get_optional<std::string>(request.document, "/date_time/value");

      if (date_time_type == 0) //current.
        locations.front().date_time_ = "current";
      else if (date_time_type == 1) //depart at
        locations.front().date_time_ = date_time_value;
      else if (date_time_type == 2) //arrive)
        locations.back().date_time_ = date_time_value;
    }

    void thor_worker_t::parse_measurements(const valhalla_request_t& request) {
      // Create a matcher
      try {
        matcher.reset(matcher_factory.Create(trace_config));
      } catch (const std::invalid_argument& ex) {
        throw std::runtime_error(std::string(ex.what()));
      }

      //we require locations
      auto request_shape = rapidjson::get<rapidjson::Value::ConstArray>(request.document, "/shape");
      try{
        for(const auto& pt : request_shape) {
          float lat = rapidjson::get<float>(pt, "/lat");
          float lon = midgard::circular_range_clamp<float>(rapidjson::get<float>(pt, "/lon"), -180, 180);
          double time = rapidjson::get<double>(pt, "/time", -1.0);
          float accuracy = rapidjson::get<float>(pt, "/accuracy", matcher->config().get<float>("gps_accuracy"));
          float radius = rapidjson::get<float>(pt, "/radius", matcher->config().get<float>("search_radius"));
          trace.emplace_back(meili::Measurement{{lon, lat}, accuracy, radius, time});
        }
      }
      catch (...) {
        throw valhalla_exception_t{424};
      }
    }

    void thor_worker_t::parse_trace_config(const valhalla_request_t& request) {
      auto costing = rapidjson::get<std::string>(request.document, "/costing");
      trace_config.put<std::string>("mode", costing);

      if (trace_customizable.empty()) {
        return;
      }

      auto trace_options = rapidjson::get_optional<rapidjson::Value::ConstObject>(request.document, "/trace_options");
      if (!trace_options) {
        return;
      }

      for (const auto& pair : *trace_options) {
        std::string name = pair.name.GetString();
        if (trace_customizable.find(name) != trace_customizable.end()){
          try {
            // Possibly throw std::invalid_argument or std::out_of_range
            trace_config.put<float>(name, pair.value.GetFloat());
          } catch (const std::invalid_argument& ex) {
            throw std::invalid_argument("Invalid argument: unable to parse " + name + " to float");
          } catch (const std::out_of_range& ex) {
            throw std::out_of_range("Invalid argument: " + name + " is out of float range");
          }
        }
      }
    }

    void thor_worker_t::log_admin(const valhalla::odin::TripPath& trip_path) {
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

    void thor_worker_t::cleanup() {
      astar.Clear();
      bidir_astar.Clear();
      multi_modal_astar.Clear();
      locations.clear();
      trace.clear();
      correlated.clear();
      isochrone_gen.Clear();
      matcher_factory.ClearFullCache();
      if(reader.OverCommitted())
        reader.Clear();
    }

  }
}
