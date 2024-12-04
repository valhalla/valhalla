#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "thor/isochrone.h"
#include "thor/worker.h"
#include "tyr/actor.h"

#include <boost/property_tree/ptree.hpp>

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
    {"taxi", 43200.0f},          {"bikeshare", 7200.0f},
};
// a scale factor to apply to the score so that we bias towards closer results more
constexpr float kDistanceScale = 10.f;

#ifdef ENABLE_SERVICES
std::string serialize_to_pbf(Api& request) {
  std::string buf;
  if (!request.SerializeToString(&buf)) {
    LOG_ERROR("Failed serializing to pbf in Thor::Worker");
    throw valhalla_exception_t{401, "Failed serializing to pbf in Thor::Worker"};
  }
  return buf;
};
#endif

} // namespace

namespace valhalla {
namespace thor {

thor_worker_t::thor_worker_t(const boost::property_tree::ptree& config,
                             const std::shared_ptr<baldr::GraphReader>& graph_reader)
    : service_worker_t(config), mode(valhalla::sif::TravelMode::kPedestrian),
      bidir_astar(config.get_child("thor")), bss_astar(config.get_child("thor")),
      multi_modal_astar(config.get_child("thor")), timedep_forward(config.get_child("thor")),
      timedep_reverse(config.get_child("thor")), costmatrix_(config.get_child("thor")),
      time_distance_matrix_(config.get_child("thor")),
      time_distance_bss_matrix_(config.get_child("thor")), isochrone_gen(config.get_child("thor")),
      reader(graph_reader ? graph_reader
                          : std::make_shared<baldr::GraphReader>(config.get_child("mjolnir"))),
      matcher_factory(config, reader), controller{} {

  // Select the matrix algorithm based on the conf file (defaults to
  // select_optimal if not present)
  auto conf_algorithm = config.get<std::string>("thor.source_to_target_algorithm", "select_optimal");
  for (const auto& kv : config.get_child("service_limits")) {
    if (kv.first == "max_exclude_locations" || kv.first == "max_reachability" ||
        kv.first == "max_radius" || kv.first == "max_timedep_distance" ||
        kv.first == "max_timedep_distance_matrix" || kv.first == "max_alternates" ||
        kv.first == "max_exclude_polygons_length" || kv.first == "skadi" || kv.first == "trace" ||
        kv.first == "isochrone" || kv.first == "centroid" || kv.first == "status" ||
        kv.first == "max_distance_disable_hierarchy_culling" || kv.first == "allow_hard_exclusions") {
      continue;
    }

    max_matrix_distance.emplace(kv.first, config.get<float>("service_limits." + kv.first +
                                                            ".max_matrix_distance"));
  }

  if (conf_algorithm == "timedistancematrix") {
    source_to_target_algorithm = TIME_DISTANCE_MATRIX;
  } else if (conf_algorithm == "costmatrix") {
    source_to_target_algorithm = COST_MATRIX;
  } else {
    source_to_target_algorithm = SELECT_OPTIMAL;
  }

  costmatrix_allow_second_pass = config.get<bool>("thor.costmatrix_allow_second_pass", false);

  max_timedep_distance =
      config.get<float>("service_limits.max_timedep_distance", kDefaultMaxTimeDependentDistance);

  // signal that the worker started successfully
  started();
}

thor_worker_t::~thor_worker_t() {
}

#ifdef ENABLE_SERVICES
prime_server::worker_t::result_t
thor_worker_t::work(const std::list<zmq::message_t>& job,
                    void* request_info,
                    const std::function<void()>& interrupt_function) {

  // get request info and make sure to record any metrics before we are done
  auto& info = *static_cast<prime_server::http_request_info_t*>(request_info);
  LOG_INFO("Got Thor Request " + std::to_string(info.id));
  Api request;
  prime_server::worker_t::result_t result{true, {}, {}};
  try {
    // crack open the original request
    bool success = request.ParseFromArray(job.front().data(), job.front().size());
    if (!success) {
      LOG_ERROR("Failed parsing pbf in Thor::Worker");
      throw valhalla_exception_t{401, "Failed parsing pbf in Thor::Worker"};
    }
    const auto& options = request.options();

    // Set the interrupt function
    service_worker_t::set_interrupt(&interrupt_function);

    // do request specific processing
    switch (options.action()) {
      case Options::sources_to_targets:
        result = to_response(matrix(request), info, request);
        break;
      case Options::optimized_route: {
        optimized_route(request);
        result.messages.emplace_back(serialize_to_pbf(request));
        break;
      }
      case Options::isochrone:
        result = to_response(isochrones(request), info, request);
        break;
      case Options::route: {
        route(request);
        result.messages.emplace_back(serialize_to_pbf(request));
        break;
      }
      case Options::trace_route: {
        trace_route(request);
        result.messages.emplace_back(serialize_to_pbf(request));
        break;
      }
      case Options::trace_attributes:
        result = to_response(trace_attributes(request), info, request);
        break;
      case Options::expansion: {
        result = to_response(expansion(request), info, request);
        break;
      }
      case Options::centroid: {
        centroid(request);
        result.messages.emplace_back(serialize_to_pbf(request));
        break;
      }
      case Options::status: {
        status(request);
        result.messages.emplace_back(serialize_to_pbf(request));
        break;
      }
      default:
        throw valhalla_exception_t{400}; // this should never happen
    }
  } catch (const valhalla_exception_t& e) {
    LOG_WARN("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_error(e, info, request);
  } catch (const std::exception& e) {
    LOG_ERROR("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_error({499, std::string(e.what())}, info, request);
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

std::string thor_worker_t::parse_costing(const Api& request) {
  // Parse out the type of route - this provides the costing method to use
  const auto& options = request.options();
  auto costing = options.costing_type();
  auto costing_str = Costing_Enum_Name(costing);
  mode_costing = factory.CreateModeCosting(options, mode);
  return costing_str;
}

void thor_worker_t::adjust_scores(valhalla::Options& options) {
  for (auto* locations :
       {options.mutable_locations(), options.mutable_sources(), options.mutable_targets()}) {
    for (auto& location : *locations) {
      // get the minimum score for all the candidates
      auto minScore = std::numeric_limits<float>::max();
      for (auto* candidates : {location.mutable_correlation()->mutable_edges(),
                               location.mutable_correlation()->mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          // completely disable scores for this location
          if (location.skip_ranking_candidates()) {
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
      auto max_score = kMaxDistances.find(Costing_Enum_Name(options.costing_type()));
      for (auto* candidates : {location.mutable_correlation()->mutable_edges(),
                               location.mutable_correlation()->mutable_filtered_edges()}) {
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
    const auto& config = matcher->config();
    for (const auto& pt : options.shape()) {
      trace.emplace_back(
          meili::Measurement{{pt.ll().lng(), pt.ll().lat()},
                             pt.has_accuracy_case() ? pt.accuracy()
                                                    : config.emission_cost.gps_accuracy_meters,
                             pt.has_radius_case() ? pt.radius()
                                                  : config.candidate_search.search_radius_meters,
                             pt.time(),
                             PathLocation::fromPBF(pt.type())});
    }
  } catch (...) { throw valhalla_exception_t{424}; }
}

void thor_worker_t::log_admin(const valhalla::TripLeg& trip_path) {
  std::unordered_set<std::string> state_iso;
  std::unordered_set<std::string> country_iso;
  if (trip_path.admin_size() > 0) {
    for (const auto& admin : trip_path.admin()) {
      if (!admin.state_code().empty()) {
        state_iso.insert(admin.state_code());
      }
      if (!admin.country_code().empty()) {
        country_iso.insert(admin.country_code());
      }
    }
  }
}

void thor_worker_t::cleanup() {
  service_worker_t::cleanup();
  bidir_astar.Clear();
  timedep_forward.Clear();
  timedep_reverse.Clear();
  multi_modal_astar.Clear();
  bss_astar.Clear();
  trace.clear();
  costmatrix_.Clear();
  time_distance_matrix_.Clear();
  time_distance_bss_matrix_.Clear();
  isochrone_gen.Clear();
  centroid_gen.Clear();
  matcher_factory.ClearFullCache();
  if (reader->OverCommitted()) {
    reader->Trim();
  }
}

void thor_worker_t::set_interrupt(const std::function<void()>* interrupt_function) {
  interrupt = interrupt_function;
  reader->SetInterrupt(interrupt);
}
} // namespace thor
} // namespace valhalla
