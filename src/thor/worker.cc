#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include <boost/property_tree/ptree.hpp>

#include "thor/isochrone.h"
#include "thor/worker.h"
#include "tyr/actor.h"

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::meili;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

// Default maximum distance allowed for time dependent routes. Since these use
// single direction A* there may be performance issues allowing very long
// routes. Also, for long routes the accuracy of predicted time along the
// route starts to become suspect (due to user breaks and other factors).
constexpr float kDefaultMaxTimeDependentDistance = 500000.0f; // 500 km

// Maximum edge score - base this on costing type.
// Large values can cause very bad performance. Setting this back
// to 2 hours for bike and pedestrian and 12 hours for driving routes.
// TODO - re-evaluate edge scores and balance performance vs. quality.
// Perhaps tie the edge score logic in with the costing type - but
// may want to do this in loki. At this point in thor the costing method
// has not yet been constructed.
const std::unordered_map<std::string, float> kMaxDistances = {
    {"auto", 43200.0f},          {"auto_data_fix", 43200.0f}, {"auto_shorter", 43200.0f},
    {"bicycle", 7200.0f},        {"bus", 43200.0f},           {"hov", 43200.0f},
    {"motor_scooter", 14400.0f}, {"motorcycle", 14400.0f},    {"multimodal", 7200.0f},
    {"pedestrian", 7200.0f},     {"transit", 14400.0f},       {"truck", 43200.0f},
    {"taxi", 43200.0f},
};
// a scale factor to apply to the score so that we bias towards closer results more
constexpr float kDistanceScale = 10.f;
constexpr double kMilePerMeter = 0.000621371;

} // namespace

namespace valhalla {
namespace thor {

thor_worker_t::thor_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : mode(valhalla::sif::TravelMode::kPedestrian), matcher_factory(config, graph_reader),
      reader(graph_reader), controller{},
      long_request(config.get<float>("thor.logging.long_request")) {
  // If we weren't provided with a graph reader make our own
  if (!reader)
    reader = matcher_factory.graphreader();

  // Register standard edge/node costing methods
  factory.RegisterStandardCostingModels();

  // Select the matrix algorithm based on the conf file (defaults to
  // select_optimal if not present)
  auto conf_algorithm = config.get<std::string>("thor.source_to_target_algorithm", "select_optimal");
  for (const auto& kv : config.get_child("service_limits")) {
    if (kv.first == "max_avoid_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius" || kv.first == "max_timedep_distance" ||
        kv.first == "max_alternates") {
      continue;
    }
    if (kv.first != "skadi" && kv.first != "trace" && kv.first != "isochrone") {
      max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                              ".max_matrix_distance"));
    }
  }

  if (conf_algorithm == "timedistancematrix") {
    source_to_target_algorithm = TIME_DISTANCE_MATRIX;
  } else if (conf_algorithm == "costmatrix") {
    source_to_target_algorithm = COST_MATRIX;
  } else {
    source_to_target_algorithm = SELECT_OPTIMAL;
  }

  max_timedep_distance =
      config.get<float>("service_limits.max_timedep_distance", kDefaultMaxTimeDependentDistance);
}

thor_worker_t::~thor_worker_t() {
}

#ifdef HAVE_HTTP
prime_server::worker_t::result_t
thor_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {
  // get time for start of request
  auto s = std::chrono::system_clock::now();
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Thor Request " + std::to_string(info.id));
  Api request;
  try {
    // crack open the original request
    request.ParseFromArray(job.front().data(), job.front().size());
    const auto& options = request.options();

    // Set the interrupt function
    service_worker_t::set_interrupt(interrupt_function);

    prime_server::worker_t::result_t result{true};
    double denominator = 0;
    // do request specific processing
    switch (options.action()) {
      case Options::sources_to_targets:
        result = to_response_json(matrix(request), info, request);
        denominator = options.sources_size() + options.targets_size();
        break;
      case Options::optimized_route: {
        optimized_route(request);
        result.messages.emplace_back(request.SerializeAsString());
        denominator = std::max(options.sources_size(), options.targets_size());
        break;
      }
      case Options::isochrone:
        result = to_response_json(isochrones(request), info, request);
        denominator = options.sources_size() * options.targets_size();
        break;
      case Options::route: {
        route(request);
        result.messages.emplace_back(request.SerializeAsString());
        denominator = options.locations_size();
        break;
      }
      case Options::trace_route: {
        trace_route(request);
        result.messages.emplace_back(request.SerializeAsString());
        denominator = trace.size() / 1100;
        break;
      }
      case Options::trace_attributes:
        result = to_response_json(trace_attributes(request), info, request);
        denominator = trace.size() / 1100;
        break;
      case Options::expansion: {
        result = to_response_json(expansion(request), info, request);
        denominator = options.locations_size();
        break;
      }
      default:
        throw valhalla_exception_t{400}; // this should never happen
    }

    double elapsed_time =
        std::chrono::duration<float, std::milli>(std::chrono::system_clock::now() - s).count();
    if (!options.do_not_track() && elapsed_time / denominator > long_request) {
      LOG_WARN("thor::" + Options_Action_Enum_Name(options.action()) +
               " request elapsed time (ms)::" + std::to_string(elapsed_time));
      LOG_WARN("thor::" + Options_Action_Enum_Name(options.action()) +
               " request exceeded threshold::" + std::to_string(info.id));
      midgard::logging::Log("valhalla_thor_long_request_" +
                                Options_Action_Enum_Name(options.action()),
                            " [ANALYTICS] ");
    }

    return result;
  } catch (const valhalla_exception_t& e) {
    valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
    return jsonify_error(e, info, request);
  } catch (const std::exception& e) {
    valhalla::midgard::logging::Log("400::" + std::string(e.what()), " [ANALYTICS] ");
    return jsonify_error({499, std::string(e.what())}, info, request);
  }
}

void run_service(const boost::property_tree::ptree& config) {
  // gets requests from thor proxy
  auto upstream_endpoint = config.get<std::string>("thor.service.proxy") + "_out";
  // sends them on to odin
  auto downstream_endpoint = config.get<std::string>("odin.service.proxy") + "_in";
  // or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  // listen for requests
  zmq::context_t context;
  thor_worker_t thor_worker(config);
  prime_server::worker_t worker(context, upstream_endpoint, downstream_endpoint, loopback_endpoint,
                                interrupt_endpoint,
                                std::bind(&thor_worker_t::work, std::ref(thor_worker),
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3),
                                std::bind(&thor_worker_t::cleanup, std::ref(thor_worker)));
  worker.work();

  // TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
}
#endif

// Get the costing options if in the config or get the empty default.
// Creates the cost in the cost factory
valhalla::sif::cost_ptr_t thor_worker_t::get_costing(const Costing costing, const Options& options) {
  return factory.Create(costing, options);
}

std::string thor_worker_t::parse_costing(const Api& request) {
  // Parse out the type of route - this provides the costing method to use
  const auto& options = request.options();
  auto costing = options.costing();
  auto costing_str = Costing_Enum_Name(costing);

  // Set travel mode and construct costing
  if (costing == Costing::multimodal || costing == Costing::transit) {
    // For multi-modal we construct costing for all modes and set the
    // initial mode to pedestrian. (TODO - allow other initial modes)
    mode_costing[0] = get_costing(Costing::auto_, options);
    mode_costing[1] = get_costing(Costing::pedestrian, options);
    mode_costing[2] = get_costing(Costing::bicycle, options);
    mode_costing[3] = get_costing(Costing::transit, options);
    mode = valhalla::sif::TravelMode::kPedestrian;
  } else {
    valhalla::sif::cost_ptr_t cost = get_costing(costing, options);
    mode = cost->travel_mode();
    mode_costing[static_cast<uint32_t>(mode)] = cost;
  }
  valhalla::midgard::logging::Log("travel_mode::" + std::to_string(static_cast<uint32_t>(mode)),
                                  " [ANALYTICS] ");
  return costing_str;
}

void thor_worker_t::parse_locations(Api& request) {
  auto& options = *request.mutable_options();
  for (auto* locations :
       {options.mutable_locations(), options.mutable_sources(), options.mutable_targets()}) {
    for (auto& location : *locations) {
      // get the minimum score for all the candidates
      auto minScore = std::numeric_limits<float>::max();
      for (auto* candidates : {location.mutable_path_edges(), location.mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          // completely disable scores for this location
          if (location.has_rank_candidates() && !location.rank_candidates()) {
            candidate.set_distance(0);
            // scale the score to favor closer results more
          } else {
            candidate.set_distance(candidate.distance() * candidate.distance() * kDistanceScale);
          }
          // remember the min score
          if (minScore > candidate.distance()) {
            minScore = candidate.distance();
          }
        }
      }

      // subtract off the min score and cap at max so that path algorithm doesnt go too far
      auto max_score = kMaxDistances.find(Costing_Enum_Name(options.costing()));
      for (auto* candidates : {location.mutable_path_edges(), location.mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          candidate.set_distance(candidate.distance() - minScore);
          if (candidate.distance() > max_score->second) {
            candidate.set_distance(max_score->second);
          }
        }
      }
    }
  }
}

void thor_worker_t::parse_measurements(const Api& request) {
  // Create a matcher
  const auto& options = request.options();
  try {
    matcher.reset(matcher_factory.Create(options));
  } catch (const std::invalid_argument& ex) { throw std::runtime_error(std::string(ex.what())); }

  // we require locations
  try {
    auto default_accuracy = matcher->config().get<float>("gps_accuracy");
    auto default_radius = matcher->config().get<float>("search_radius");
    for (const auto& pt : options.shape()) {
      trace.emplace_back(meili::Measurement{{pt.ll().lng(), pt.ll().lat()},
                                            pt.has_accuracy() ? pt.accuracy() : default_accuracy,
                                            pt.has_radius() ? pt.radius() : default_radius,
                                            pt.time()});
    }
  } catch (...) { throw valhalla_exception_t{424}; }
}

void thor_worker_t::log_admin(const valhalla::TripLeg& trip_path) {
  std::unordered_set<std::string> state_iso;
  std::unordered_set<std::string> country_iso;
  std::stringstream s_ss, c_ss;
  if (trip_path.admin_size() > 0) {
    for (const auto& admin : trip_path.admin()) {
      if (admin.has_state_code()) {
        state_iso.insert(admin.state_code());
      }
      if (admin.has_country_code()) {
        country_iso.insert(admin.country_code());
      }
    }
    for (const std::string& x : state_iso) {
      s_ss << " " << x;
    }
    for (const std::string& x : country_iso) {
      c_ss << " " << x;
    }
    if (!s_ss.eof()) {
      valhalla::midgard::logging::Log("admin_state_iso::" + s_ss.str() + ' ', " [ANALYTICS] ");
    }
    if (!c_ss.eof()) {
      valhalla::midgard::logging::Log("admin_country_iso::" + c_ss.str() + ' ', " [ANALYTICS] ");
    }
  }
}

/*
 * Apply attribute filters from the request to the AttributesController. These filters
 * allow including or excluding specific attributes from the response in route,
 * trace_route, and trace_attributes actions.
 */
void thor_worker_t::parse_filter_attributes(const Api& request, bool is_strict_filter) {
  // Set default controller
  controller = AttributesController();
  const auto& options = request.options();

  if (options.has_filter_action()) {
    switch (options.filter_action()) {
      case (FilterAction::include): {
        if (is_strict_filter)
          controller.disable_all();
        for (const auto& filter_attribute : options.filter_attributes()) {
          try {
            controller.attributes.at(filter_attribute) = true;
          } catch (...) { LOG_ERROR("Invalid filter attribute " + filter_attribute); }
        }
        break;
      }
      case (FilterAction::exclude): {
        for (const auto& filter_attribute : options.filter_attributes()) {
          try {
            controller.attributes.at(filter_attribute) = false;
          } catch (...) { LOG_ERROR("Invalid filter attribute " + filter_attribute); }
        }
        break;
      }
    }
  }
}

void thor_worker_t::cleanup() {
  astar.Clear();
  bidir_astar.Clear();
  multi_modal_astar.Clear();
  trace.clear();
  isochrone_gen.Clear();
  matcher_factory.ClearFullCache();
  if (reader->OverCommitted()) {
    reader->Trim();
  }
  m_path_infos.clear();
  m_map_match_results.clear();
  m_offline_results.clear();
}

} // namespace thor
} // namespace valhalla
