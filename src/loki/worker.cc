#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "baldr/json.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "sif/motorcyclecost.h"
#include "sif/motorscootercost.h"
#include "sif/pedestriancost.h"
#include "tyr/actor.h"

#include "loki/search.h"
#include "loki/worker.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::loki;

namespace valhalla {
namespace loki {
void loki_worker_t::parse_locations(google::protobuf::RepeatedPtrField<valhalla::Location>* locations,
                                    boost::optional<valhalla_exception_t> required_exception) {
  if (locations->size()) {
    for (auto& location : *locations) {
      if (!location.has_minimum_reachability())
        location.set_minimum_reachability(default_reachability);
      else if (location.minimum_reachability() > max_reachability)
        location.set_minimum_reachability(max_reachability);

      if (!location.has_radius())
        location.set_radius(default_radius);
      else if (location.radius() > max_radius)
        location.set_radius(max_radius);

      if (!location.has_heading_tolerance())
        location.set_heading_tolerance(default_heading_tolerance);

      if (!location.has_node_snap_tolerance())
        location.set_node_snap_tolerance(default_node_snap_tolerance);

      if (!location.has_search_cutoff())
        location.set_search_cutoff(default_search_cutoff);

      if (!location.has_street_side_tolerance())
        location.set_street_side_tolerance(default_street_side_tolerance);
    }
  } else if (required_exception) {
    throw *required_exception;
  }
}

void loki_worker_t::parse_costing(Api& api) {
  auto& options = *api.mutable_options();
  // using the costing we can determine what type of edge filtering to use
  if (!options.has_costing()) {
    throw valhalla_exception_t{124};
  }

  auto costing_type = options.costing();
  auto costing_str = Costing_Enum_Name(costing_type);

  if (!options.do_not_track()) {
    valhalla::midgard::logging::Log("costing_type::" + costing_str, " [ANALYTICS] ");
  }

  // TODO - have a way of specifying mode at the location
  if (costing_type == Costing::multimodal) {
    costing_type = Costing::pedestrian;
  }

  try {
    costing = factory.Create(costing_type, options);
  } catch (const std::runtime_error&) { throw valhalla_exception_t{125, "'" + costing_str + "'"}; }

  // See if we have avoids and take care of them
  if (options.avoid_locations_size() > max_avoid_locations) {
    throw valhalla_exception_t{157, std::to_string(max_avoid_locations)};
  }

  // Process avoid locations. Add to a list of edgeids and percent along the edge.
  if (options.avoid_locations_size()) {
    try {
      auto avoid_locations = PathLocation::fromPBF(options.avoid_locations());
      auto results = loki::Search(avoid_locations, *reader, costing.get());
      std::unordered_set<uint64_t> avoids;
      for (const auto& result : results) {
        for (const auto& edge : result.second.edges) {
          auto inserted = avoids.insert(edge.id);

          // If this edge Id was inserted add it to the request options (along with percent along)
          // Also insert shortcut edge if one includes this edge
          if (inserted.second) {
            // Add edge and percent along to pbf
            auto* avoid = options.add_avoid_edges();
            avoid->set_id(edge.id);
            avoid->set_percent_along(edge.percent_along);

            // Check if a shortcut exists
            GraphId shortcut = reader->GetShortcut(edge.id);
            if (shortcut.Is_Valid()) {
              // Check if this shortcut has not been added
              auto shortcut_inserted = avoids.insert(shortcut);
              if (shortcut_inserted.second) {
                avoids.insert(shortcut);

                // Add to pbf (with 0 percent along)
                auto* avoid = options.add_avoid_edges();
                avoid->set_id(shortcut);
                avoid->set_percent_along(0);
              }
            }
          }
        }
      }
    } // swallow all failures on optional avoids
    catch (...) {
      LOG_WARN("Failed to find avoid_locations");
    }
  }

  // If more alternates are requested than we support we cap it
  if (options.alternates() > max_alternates)
    options.set_alternates(max_alternates);
}

loki_worker_t::loki_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : config(config), reader(graph_reader),
      connectivity_map(config.get<bool>("loki.use_connectivity", true)
                           ? new connectivity_map_t(config.get_child("mjolnir"))
                           : nullptr),
      long_request(config.get<float>("loki.logging.long_request")),
      max_contours(config.get<size_t>("service_limits.isochrone.max_contours")),
      max_time(config.get<size_t>("service_limits.isochrone.max_time")),
      max_trace_shape(config.get<size_t>("service_limits.trace.max_shape")),
      sample(config.get<std::string>("additional_data.elevation", "test/data/")),
      max_elevation_shape(config.get<size_t>("service_limits.skadi.max_shape")),
      min_resample(config.get<float>("service_limits.skadi.min_resample")) {
  // If we weren't provided with a graph reader make our own
  if (!reader)
    reader.reset(new baldr::GraphReader(config.get_child("mjolnir")));

  // Keep a string noting which actions we support, throw if one isnt supported
  Options::Action action;
  for (const auto& kv : config.get_child("loki.actions")) {
    auto path = kv.second.get_value<std::string>();
    if (!Options_Action_Enum_Parse(path, &action)) {
      throw std::runtime_error("Action not supported " + path);
    }
    action_str.append("'/" + path + "' ");
  }
  // Make sure we have at least something to support!
  if (action_str.empty()) {
    throw std::runtime_error("The config actions for Loki are incorrectly loaded");
  }

  // Build max_locations and max_distance maps
  for (const auto& kv : config.get_child("service_limits")) {
    if (kv.first == "max_avoid_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius" || kv.first == "max_timedep_distance" ||
        kv.first == "max_alternates") {
      continue;
    }
    if (kv.first != "skadi" && kv.first != "trace") {
      max_locations.emplace(kv.first,
                            config.get<size_t>("service_limits." + kv.first + ".max_locations"));
    }
    if (kv.first != "skadi") {
      max_distance.emplace(kv.first,
                           config.get<float>("service_limits." + kv.first + ".max_distance"));
    }
    if (kv.first != "skadi" && kv.first != "trace" && kv.first != "isochrone") {
      max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                              ".max_matrix_distance"));
      max_matrix_locations.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                               ".max_matrix_locations"));
    }
  }
  // this should never happen
  if (max_locations.empty()) {
    throw std::runtime_error("Missing max_locations configuration");
  }

  if (max_distance.empty()) {
    throw std::runtime_error("Missing max_distance configuration");
  }

  if (max_matrix_distance.empty()) {
    throw std::runtime_error("Missing max_matrix_distance configuration");
  }

  if (max_matrix_locations.empty()) {
    throw std::runtime_error("Missing max_matrix_locations configuration");
  }

  min_transit_walking_dis =
      config.get<size_t>("service_limits.pedestrian.min_transit_walking_distance");
  max_transit_walking_dis =
      config.get<size_t>("service_limits.pedestrian.max_transit_walking_distance");

  max_avoid_locations = config.get<size_t>("service_limits.max_avoid_locations");
  max_reachability = config.get<unsigned int>("service_limits.max_reachability");
  default_reachability = config.get<unsigned int>("loki.service_defaults.minimum_reachability");
  max_radius = config.get<unsigned int>("service_limits.max_radius");
  default_radius = config.get<unsigned int>("loki.service_defaults.radius");
  default_heading_tolerance = config.get<unsigned int>("loki.service_defaults.heading_tolerance");
  default_node_snap_tolerance = config.get<unsigned int>("loki.service_defaults.node_snap_tolerance");
  default_search_cutoff = config.get<unsigned int>("loki.service_defaults.search_cutoff");
  default_street_side_tolerance =
      config.get<unsigned int>("loki.service_defaults.street_side_tolerance");
  max_gps_accuracy = config.get<float>("service_limits.trace.max_gps_accuracy");
  max_search_radius = config.get<float>("service_limits.trace.max_search_radius");
  max_best_paths = config.get<unsigned int>("service_limits.trace.max_best_paths");
  max_best_paths_shape = config.get<size_t>("service_limits.trace.max_best_paths_shape");
  max_alternates = config.get<unsigned int>("service_limits.max_alternates");

  // Register standard edge/node costing methods
  factory.RegisterStandardCostingModels();
}

void loki_worker_t::cleanup() {
  if (reader->OverCommitted()) {
    reader->Trim();
  }
}

#ifdef HAVE_HTTP

prime_server::worker_t::result_t
loki_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {
  // get time for start of request
  auto s = std::chrono::system_clock::now();
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Loki Request " + std::to_string(info.id));
  Api request;
  try {
    // request parsing
    auto http_request =
        prime_server::http_request_t::from_string(static_cast<const char*>(job.front().data()),
                                                  job.front().size());
    ParseApi(http_request, request);
    const auto& options = request.options();

    // check there is a valid action
    if (!options.has_action()) {
      return jsonify_error({106, action_str}, info, request);
    }

    // Set the interrupt function
    service_worker_t::set_interrupt(interrupt_function);

    prime_server::worker_t::result_t result{true};
    // do request specific processing
    switch (options.action()) {
      case Options::route:
      case Options::expansion:
        route(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::locate:
        result = to_response_json(locate(request), info, request);
        break;
      case Options::sources_to_targets:
      case Options::optimized_route:
        matrix(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::isochrone:
        isochrones(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::trace_attributes:
      case Options::trace_route:
        trace(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::height:
        result = to_response_json(height(request), info, request);
        break;
      case Options::transit_available:
        result = to_response_json(transit_available(request), info, request);
        break;
      default:
        // apparently you wanted something that we figured we'd support but havent written yet
        return jsonify_error({107}, info, request);
    }
    // get processing time for loki
    auto e = std::chrono::system_clock::now();
    std::chrono::duration<float, std::milli> elapsed_time = e - s;
    // log request if greater than X (ms)
    auto work_units = options.locations_size()
                          ? options.locations_size()
                          : (options.sources_size() ? options.sources_size() + options.targets_size()
                                                    : options.shape_size() * 20);
    if (!options.do_not_track() && elapsed_time.count() / work_units > long_request) {
      LOG_WARN("loki::request elapsed time (ms)::" + std::to_string(elapsed_time.count()));
      LOG_WARN("loki::request exceeded threshold::" + std::to_string(info.id));
      midgard::logging::Log("valhalla_loki_long_request", " [ANALYTICS] ");
    }

    return result;
  } catch (const valhalla_exception_t& e) {
    valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
    return jsonify_error(e, info, request);
  } catch (const std::exception& e) {
    valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
    return jsonify_error({199, std::string(e.what())}, info, request);
  }
}

void run_service(const boost::property_tree::ptree& config) {
  // gets requests from the http server
  auto upstream_endpoint = config.get<std::string>("loki.service.proxy") + "_out";
  // sends them on to thor
  auto downstream_endpoint = config.get<std::string>("thor.service.proxy") + "_in";
  // or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  // listen for requests
  zmq::context_t context;
  loki_worker_t loki_worker(config);
  prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
                                interrupt_endpoint,
                                std::bind(&loki_worker_t::work, std::ref(loki_worker),
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3),
                                std::bind(&loki_worker_t::cleanup, std::ref(loki_worker)));
  worker.work();

  // TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}
#endif
} // namespace loki
} // namespace valhalla
