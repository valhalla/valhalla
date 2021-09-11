#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "worker.h"

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {
// Maximum edge score - base this on costing type.
// Large values can cause very bad performance. Setting this back
// to 2 hours for bike and pedestrian and 12 hours for driving routes.
// TODO - re-evaluate edge scores and balance performance vs. quality.
// Perhaps tie the edge score logic in with the costing type - but
// may want to do this in loki. At this point in thor the costing method
// has not yet been constructed.
const std::unordered_map<std::string, float> kMaxDistances = {
    {"auto", 43200.0f},      {"auto_shorter", 43200.0f},  {"bicycle", 7200.0f},
    {"bus", 43200.0f},       {"motor_scooter", 14400.0f}, {"multimodal", 7200.0f},
    {"pedestrian", 7200.0f}, {"transit", 14400.0f},       {"truck", 43200.0f},
    {"taxi", 43200.0f},
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

const auto config = test::make_config("test/data/utrecht_tiles");

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
  TravelMode mode;
  auto mode_costing = sif::CostFactory().CreateModeCosting(request.options(), mode);

  valhalla::Location origin = request.options().locations(0);
  valhalla::Location dest = request.options().locations(1);
  if (depart_at) {
    TimeDepForward alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    EXPECT_EQ(pathedges.size(), expected_edgecount) << "Depart at path failed";
  } else {
    TimeDepReverse alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    EXPECT_EQ(pathedges.size(), expected_edgecount) << "Arrive by path failed";
  }
}

} // namespace

TEST(TimeDepPaths, test_depart_at_paths) {
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

TEST(TimeDepPaths, test_arrive_by_paths) {
  // Test setup
  loki_worker_t loki_worker(config);
  GraphReader reader(config.get_child("mjolnir"));

  // Simple path along oneway edge in the driveable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":2,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, false, test_request1, 1);
}

int main(int argc, char* argv[]) {
  // logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
