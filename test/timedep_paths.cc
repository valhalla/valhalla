#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "thor/timedep.h"
#include "thor/worker.h"
#include "worker.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

rapidjson::Document to_document(const std::string& request) {
  rapidjson::Document d;
  auto& allocator = d.GetAllocator();
  d.Parse(request.c_str());
  if (d.HasParseError())
    throw valhalla_exception_t{100};
  return d;
}

// Maximum edge score - base this on costing type.
// Large values can cause very bad performance. Setting this back
// to 2 hours for bike and pedestrian and 12 hours for driving routes.
// TODO - re-evaluate edge scores and balance performance vs. quality.
// Perhaps tie the edge score logic in with the costing type - but
// may want to do this in loki. At this point in thor the costing method
// has not yet been constructed.
const std::unordered_map<std::string, float> kMaxDistances = {
    {"auto", 43200.0f},      {"auto_shorter", 43200.0f}, {"bicycle", 7200.0f},
    {"bus", 43200.0f},       {"hov", 43200.0f},          {"motor_scooter", 14400.0f},
    {"multimodal", 7200.0f}, {"pedestrian", 7200.0f},    {"transit", 14400.0f},
    {"truck", 43200.0f},     {"taxi", 43200.0f},
};
// a scale factor to apply to the score so that we bias towards closer results more
constexpr float kDistanceScale = 10.f;

void adjust_scores(Options& options) {
  for (auto* locations :
       {options.mutable_locations(), options.mutable_sources(), options.mutable_targets()}) {
    for (auto& location : *locations) {
      // get the minimum score for all the candidates
      auto minScore = std::numeric_limits<float>::max();
      for (auto* candidates : {location.mutable_path_edges(), location.mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          // completely disable scores for this location
          if (location.has_rank_candidates() && !location.rank_candidates())
            candidate.set_distance(0);
          // scale the score to favor closer results more
          else
            candidate.set_distance(candidate.distance() * candidate.distance() * kDistanceScale);
          // remember the min score
          if (minScore > candidate.distance())
            minScore = candidate.distance();
        }
      }

      // subtract off the min score and cap at max so that path algorithm doesnt go too far
      auto max_score = kMaxDistances.find(Costing_Enum_Name(options.costing()));
      for (auto* candidates : {location.mutable_path_edges(), location.mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          candidate.set_distance(candidate.distance() - minScore);
          if (candidate.distance() > max_score->second)
            candidate.set_distance(max_score->second);
        }
      }
    }
  }
}

const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["sources_to_targets"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
    },
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

} // namespace

void try_path(GraphReader& reader,
              loki_worker_t& loki_worker,
              const bool depart_at,
              const char* test_request,
              const uint32_t expected_edgecount) {
  Api request;
  ParseApi(test_request, Options::route, request);
  loki_worker.route(request);
  adjust_scores(*request.mutable_options());

  // For now this just tests auto costing - could extend to other
  TravelMode mode = TravelMode::kDrive;
  cost_ptr_t costing = CreateAutoCost(request.options().costing(), request.options());
  std::shared_ptr<DynamicCost> mode_costing[4];
  mode_costing[static_cast<uint32_t>(mode)] = costing;

  valhalla::Location origin = request.options().locations(0);
  valhalla::Location dest = request.options().locations(1);
  if (depart_at) {
    TimeDepForward alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    if (pathedges.size() != expected_edgecount) {
      throw std::runtime_error("Depart at path failed: expected edges: " +
                               std::to_string(expected_edgecount));
    }
  } else {
    TimeDepReverse alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    if (pathedges.size() != expected_edgecount) {
      throw std::runtime_error("Arrive by path failed: expected edges: " +
                               std::to_string(expected_edgecount));
    }
  }
}

void test_depart_at_paths() {
  // Test setup
  loki_worker_t loki_worker(config);
  GraphReader reader(config.get_child("mjolnir"));

  // Simple path along oneway edge in the driveable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request1, 1);

  // Simple path along oneway edge opposing the driveable direction -must not
  // return a single edge (edge count is 10)
  const auto test_request2 = R"({"locations":[{"lat":52.078937,"lon":5.115321},
               {"lat":52.079079,"lon":5.115197}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request2, 10);
}

void test_arrive_by_paths() {
  // Test setup
  loki_worker_t loki_worker(config);
  GraphReader reader(config.get_child("mjolnir"));

  // Simple path along oneway edge in the driveable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":2,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, false, test_request1, 1);
}

int main(int argc, char* argv[]) {
  test::suite suite("trivial_paths");
  // logging::Configure({{"type", ""}}); // silence logs

  // Test depart at
  suite.test(TEST_CASE(test_depart_at_paths));

  // Test arrive by
  suite.test(TEST_CASE(test_arrive_by_paths));

  return suite.tear_down();
}
