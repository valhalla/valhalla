#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
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

#include "loki/polygon_search.h"
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
                                    Api& request,
                                    std::optional<valhalla_exception_t> required_exception) {
  bool has_302 = false, has_303 = false;

  if (locations->size()) {
    for (auto& location : *locations) {
      if (!location.has_minimum_reachability_case())
        location.set_minimum_reachability(default_reachability);
      else if (location.minimum_reachability() > max_reachability)
        location.set_minimum_reachability(max_reachability);

      if (!location.has_radius_case())
        location.set_radius(default_radius);
      else if (location.radius() > max_radius)
        location.set_radius(max_radius);

      if (!location.has_heading_tolerance_case())
        location.set_heading_tolerance(default_heading_tolerance);

      if (!location.has_node_snap_tolerance_case())
        location.set_node_snap_tolerance(default_node_snap_tolerance);

      const bool has_level =
          location.has_search_filter() && location.search_filter().level() != baldr::kMaxLevel;

      if (!location.has_search_cutoff_case() && !has_level) {
        // no level and no cutoff, provide regular default
        location.set_search_cutoff(default_search_cutoff);
      } else if (location.has_search_cutoff_case() && has_level) {
        // level and cutoff, clamp value to limit candidate search in case of bogus level input
        if (location.search_cutoff() > kMaxIndoorSearchCutoff) {
          has_303 = true;
          location.set_search_cutoff(kMaxIndoorSearchCutoff);
        }
      } else if (!location.has_search_cutoff_case() && has_level) {
        // level and no cutoff, set special default
        location.set_search_cutoff(kDefaultIndoorSearchCutoff);
        has_302 = true;
      }
      // if there is a level search filter and
      if (!location.has_street_side_tolerance_case())
        location.set_street_side_tolerance(default_street_side_tolerance);

      if (!location.has_street_side_max_distance_case())
        location.set_street_side_max_distance(default_street_side_max_distance);
    }
    if (has_302)
      add_warning(request, 302, std::to_string(kDefaultIndoorSearchCutoff));
    if (has_303)
      add_warning(request, 303, std::to_string(kMaxIndoorSearchCutoff));

  } else if (required_exception) {
    throw *required_exception;
  }
}

void loki_worker_t::parse_costing(Api& api, bool allow_none) {
  auto& options = *api.mutable_options();
  // using the costing we can determine what type of edge filtering to use
  if (!allow_none && options.costing_type() == Costing::none_) {
    throw valhalla_exception_t{124};
  }

  if (!allow_hard_exclusions) {
    bool exclusion_detected = false;
    for (auto& pair : options.costings()) {
      auto opts = pair.second.options();
      exclusion_detected = exclusion_detected || opts.exclude_bridges() || opts.exclude_tolls() ||
                           opts.exclude_tunnels() || opts.exclude_highways() ||
                           opts.exclude_ferries();
      opts.set_exclude_bridges(false);
      opts.set_exclude_tolls(false);
      opts.set_exclude_tunnels(false);
      opts.set_exclude_highways(false);
      opts.set_exclude_ferries(false);
    }
    if (exclusion_detected) {
      add_warning(api, 208);
    }
  }

  const auto& costing_str = Costing_Enum_Name(options.costing_type());
  try {
    // For the begin and end of multimodal we expect you to be walking
    if (options.costing_type() == Costing::multimodal) {
      options.set_costing_type(Costing::pedestrian);
      costing = factory.Create(options);
      options.set_costing_type(Costing::multimodal);
    } // otherwise use the provided costing
    else {
      costing = factory.Create(options);
    }
  } catch (const std::runtime_error&) { throw valhalla_exception_t{125, "'" + costing_str + "'"}; }

  if (options.exclude_polygons_size()) {
    const auto edges =
        edges_in_rings(options.exclude_polygons(), *reader, costing, max_exclude_polygons_length);
    auto& co = *options.mutable_costings()->find(options.costing_type())->second.mutable_options();
    for (const auto& edge_id : edges) {
      auto* avoid = co.add_exclude_edges();
      avoid->set_id(edge_id);
      // TODO: set correct percent_along in edges_in_rings (for origin & destination edges)
      avoid->set_percent_along(0);
    }
  }

  // Process avoid locations. Add to a list of edgeids and percent along the edge.
  if (options.exclude_locations_size()) {
    // See if we have avoids and take care of them
    if (static_cast<size_t>(options.exclude_locations_size()) > max_exclude_locations) {
      throw valhalla_exception_t{157, std::to_string(max_exclude_locations)};
    }
    try {
      auto exclude_locations = PathLocation::fromPBF(options.exclude_locations());
      auto results = loki::Search(exclude_locations, *reader, costing);
      std::unordered_set<uint64_t> avoids;
      auto& co = *options.mutable_costings()->find(options.costing_type())->second.mutable_options();
      for (const auto& result : results) {
        for (const auto& edge : result.second.edges) {
          auto inserted = avoids.insert(edge.id);

          // If this edge Id was inserted add it to the request options (along with percent along)
          // Also insert shortcut edge if one includes this edge
          if (inserted.second) {
            // Add edge and percent along to pbf
            auto* avoid = co.add_exclude_edges();
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
                auto* avoid_shortcut = co.add_exclude_edges();
                avoid_shortcut->set_id(shortcut);
                avoid_shortcut->set_percent_along(0);
              }
            }
          }
        }
      }
    } // swallow all failures on optional avoids
    catch (...) {
      LOG_WARN("Failed to find exclude_locations");
    }
  }

  // If more alternates are requested than we support we cap it
  if (options.action() != Options::trace_attributes && options.alternates() > max_alternates)
    options.set_alternates(max_alternates);
  if (options.action() == Options::trace_attributes && options.alternates() > max_trace_alternates)
    options.set_alternates(max_trace_alternates);
}

loki_worker_t::loki_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : service_worker_t(config), config(config),
      reader(graph_reader ? graph_reader
                          : std::make_shared<baldr::GraphReader>(config.get_child("mjolnir"))),
      connectivity_map(config.get<bool>("loki.use_connectivity", true)
                           ? new connectivity_map_t(config.get_child("mjolnir"), reader)
                           : nullptr),
      max_contours(config.get<size_t>("service_limits.isochrone.max_contours")),
      max_contour_min(config.get<size_t>("service_limits.isochrone.max_time_contour")),
      max_contour_km(config.get<size_t>("service_limits.isochrone.max_distance_contour")),
      max_trace_shape(config.get<size_t>("service_limits.trace.max_shape")),
      sample(config.get<std::string>("additional_data.elevation", "")),
      max_elevation_shape(config.get<size_t>("service_limits.skadi.max_shape")),
      min_resample(config.get<float>("service_limits.skadi.min_resample")),
      allow_hard_exclusions(config.get<bool>("service_limits.allow_hard_exclusions", false)) {

  // Keep a string noting which actions we support, throw if one isnt supported
  Options::Action action;
  for (const auto& kv : config.get_child("loki.actions")) {
    auto path = kv.second.get_value<std::string>();
    if (!Options_Action_Enum_Parse(path, &action)) {
      throw std::runtime_error("Action not supported " + path);
    }
    actions.insert(action);
    action_str.append("'/" + path + "' ");
  }
  // Make sure we have at least something to support!
  if (action_str.empty()) {
    throw std::runtime_error("The config actions for Loki are incorrectly loaded");
  }

  // Build max_locations and max_distance maps
  for (const auto& kv : config.get_child("service_limits")) {
    if (kv.first == "max_exclude_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius" || kv.first == "max_timedep_distance" ||
        kv.first == "max_timedep_distance_matrix" || kv.first == "max_alternates" ||
        kv.first == "max_exclude_polygons_length" ||
        kv.first == "max_distance_disable_hierarchy_culling" || kv.first == "skadi" ||
        kv.first == "status" || kv.first == "allow_hard_exclusions") {
      continue;
    }
    if (kv.first != "trace") {
      max_locations.emplace(kv.first,
                            config.get<size_t>("service_limits." + kv.first + ".max_locations"));
      if (kv.first == "centroid" && max_locations["centroid"] > 127)
        throw std::runtime_error("Max locations for centroid action must be < 128");
    }
    max_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first + ".max_distance"));
    if (kv.first != "centroid" && kv.first != "trace" && kv.first != "isochrone") {
      max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                              ".max_matrix_distance"));
      max_matrix_locations.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                               ".max_matrix_location_pairs"));
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
    throw std::runtime_error("Missing max_matrix_location_pairs configuration");
  }

  min_transit_walking_dis =
      config.get<size_t>("service_limits.pedestrian.min_transit_walking_distance");
  max_transit_walking_dis =
      config.get<size_t>("service_limits.pedestrian.max_transit_walking_distance");

  max_exclude_locations = config.get<size_t>("service_limits.max_exclude_locations");
  max_exclude_polygons_length = config.get<float>("service_limits.max_exclude_polygons_length");
  max_reachability = config.get<unsigned int>("service_limits.max_reachability");
  default_reachability = config.get<unsigned int>("loki.service_defaults.minimum_reachability");
  max_radius = config.get<unsigned int>("service_limits.max_radius");
  default_radius = config.get<unsigned int>("loki.service_defaults.radius");
  default_heading_tolerance = config.get<unsigned int>("loki.service_defaults.heading_tolerance");
  default_node_snap_tolerance = config.get<unsigned int>("loki.service_defaults.node_snap_tolerance");
  default_search_cutoff = config.get<unsigned int>("loki.service_defaults.search_cutoff");
  default_street_side_tolerance =
      config.get<unsigned int>("loki.service_defaults.street_side_tolerance");
  default_street_side_max_distance =
      config.get<unsigned int>("loki.service_defaults.street_side_max_distance");
  default_breakage_distance = config.get<float>("meili.default.breakage_distance");
  max_gps_accuracy = config.get<float>("service_limits.trace.max_gps_accuracy");
  max_search_radius = config.get<float>("service_limits.trace.max_search_radius");
  max_trace_alternates = config.get<unsigned int>("service_limits.trace.max_alternates");
  max_trace_alternates_shape = config.get<size_t>("service_limits.trace.max_alternates_shape");
  max_alternates = config.get<unsigned int>("service_limits.max_alternates");
  allow_verbose = config.get<bool>("service_limits.status.allow_verbose", false);
  max_timedep_dist_matrix = config.get<size_t>("service_limits.max_timedep_distance_matrix", 0);
  // assign max_distance_disable_hierarchy_culling
  max_distance_disable_hierarchy_culling =
      config.get<float>("service_limits.max_distance_disable_hierarchy_culling", 0.f);
  allow_hard_exclusions = config.get<bool>("service_limits.allow_hard_exclusions", false);

  // signal that the worker started successfully
  started();
}

void loki_worker_t::cleanup() {
  service_worker_t::cleanup();
  if (reader->OverCommitted()) {
    reader->Trim();
  }
}

void loki_worker_t::set_interrupt(const std::function<void()>* interrupt_function) {
  interrupt = interrupt_function;
  reader->SetInterrupt(interrupt);
}

// Check if total arc distance exceeds the max distance limit for disable_hierarchy_pruning.
// If true, add a warning and set the disable_hierarchy_pruning costing option to false.
void loki_worker_t::check_hierarchy_distance(Api& request) {
  auto& options = *request.mutable_options();
  // Bail early if the mode of travel is not vehicular.
  // Bike and pedestrian don't use hierarchies anyway.
  if (options.costing_type() == Costing_Type_bicycle ||
      options.costing_type() == Costing_Type_pedestrian) {
    return;
  }

  // If disable_hierarchy_pruning is not true, skip the rest.
  auto costing_options = options.mutable_costings()->find(options.costing_type());
  if (!costing_options->second.options().disable_hierarchy_pruning()) {
    return;
  }

  // For route action check if total distances between locations exceed the max limit.
  // For matrix action, check every pair of source and target.
  bool max_distance_exceeded = false;
  if (request.options().action() == Options_Action_sources_to_targets) {
    for (auto& source : *options.mutable_sources()) {
      for (auto& target : *options.mutable_targets()) {
        if (to_ll(source).Distance(to_ll(target)) > max_distance_disable_hierarchy_culling) {
          max_distance_exceeded = true;
          break;
        }
      }
    }
  } else {
    auto locations = options.locations();
    float arc_distance = 0.0f;
    for (auto source = locations.begin(); source != locations.end() - 1; ++source) {
      arc_distance += to_ll(*source).Distance(to_ll(*(source + 1)));
      if (arc_distance > max_distance_disable_hierarchy_culling) {
        max_distance_exceeded = true;
        break;
      }
    }
  }

  // Turn off disable_hierarchy_pruning and add a warning if max limit exceeded.
  if (max_distance_exceeded) {
    add_warning(request, 205);
    costing_options->second.mutable_options()->set_disable_hierarchy_pruning(false);
  }
}

#ifdef ENABLE_SERVICES
prime_server::worker_t::result_t
loki_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {

  // grab the request info and make sure to record any metrics before we are done
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Loki Request " + std::to_string(info.id));
  Api request;
  prime_server::worker_t::result_t result{true, {}, ""};
  try {
    // request parsing
    auto http_request =
        prime_server::http_request_t::from_string(static_cast<const char*>(job.front().data()),
                                                  job.front().size());
    ParseApi(http_request, request);
    const auto& options = request.options();

    // check there is a valid action
    if (actions.find(options.action()) == actions.cend()) {
      throw valhalla_exception_t{106, action_str};
    }

    // Set the interrupt function
    service_worker_t::set_interrupt(&interrupt_function);
    // do request specific processing
    switch (options.action()) {
      case Options::route:
      case Options::centroid:
        route(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::locate:
        result = to_response(locate(request), info, request);
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
        result = to_response(height(request), info, request);
        break;
      case Options::transit_available:
        result = to_response(transit_available(request), info, request);
        break;
      case Options::status:
        status(request);
        result.messages.emplace_back(request.SerializeAsString());
        break;
      case Options::expansion:
        if (options.expansion_action() == Options::route) {
          route(request);
        } else if (options.expansion_action() == Options::isochrone) {
          isochrones(request);
        } else {
          matrix(request);
        }
        result.messages.emplace_back(request.SerializeAsString());
        break;
      default:
        // apparently you wanted something that we figured we'd support but havent written yet
        throw valhalla_exception_t{107};
    }
  } catch (const valhalla_exception_t& e) {
    LOG_WARN("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_error(e, info, request);
  } catch (const std::exception& e) {
    LOG_ERROR("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_error({199, std::string(e.what())}, info, request);
  }

  // keep track of the metrics if the request is going back to the client
  if (!result.intermediate)
    enqueue_statistics(request);

  return result;
}

void run_service(const boost::property_tree::ptree& config) {
  // gracefully shutdown when asked via SIGTERM
  prime_server::quiesce(config.get<unsigned int>("httpd.service.drain_seconds", 28),
                        config.get<unsigned int>("httpd.service.shutting_seconds", 1));

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
