#include "test.h"

#include <string>

#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "thor/worker.h"

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
      for (auto* candidates : {location.mutable_correlation()->mutable_edges(),
                               location.mutable_correlation()->mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          // completely disable scores for this location
          if (location.skip_ranking_candidates())
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
      auto max_score = kMaxDistances.find(Costing_Enum_Name(options.costing_type()));
      for (auto* candidates : {location.mutable_correlation()->mutable_edges(),
                               location.mutable_correlation()->mutable_filtered_edges()}) {
        for (auto& candidate : *candidates) {
          candidate.set_distance(candidate.distance() - minScore);
          if (candidate.distance() > max_score->second)
            candidate.set_distance(max_score->second);
        }
      }
    }
  }
}

const auto cfg = test::make_config("test/data/utrecht_tiles");

void try_path(GraphReader& reader,
              loki_worker_t& loki_worker,
              const char* test_request,
              const uint32_t expected_edgecount) {
  Api request;
  ParseApi(test_request, Options::route, request);
  loki_worker.route(request);
  adjust_scores(*request.mutable_options());

  // For now this just tests auto costing - could extend to other
  request.mutable_options()->set_costing_type(Costing::auto_);
  sif::TravelMode mode;
  auto mode_costing = sif::CostFactory{}.CreateModeCosting(*request.mutable_options(), mode);
  cost_ptr_t costing = mode_costing[static_cast<size_t>(mode)];

  TimeDepForward astar;
  valhalla::Location origin = request.options().locations(0);
  valhalla::Location dest = request.options().locations(1);
  auto pathedges = astar.GetBestPath(origin, dest, reader, mode_costing, mode).front();
  EXPECT_EQ(pathedges.size(), expected_edgecount);
}

} // namespace

TEST(TrivialPaths, test_trivial_paths) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  // Simple path along oneway edge in the drivable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto"})";
  try_path(reader, loki_worker, test_request1, 1);

  // Simple path along oneway edge opposing the drivable direction -must not
  // return a single edge (edge count is 10)
  const auto test_request2 = R"({"locations":[{"lat":52.078937,"lon":5.115321},
               {"lat":52.079079,"lon":5.115197}],"costing":"auto"})";
  try_path(reader, loki_worker, test_request2, 10);

  // Simple path along two-way edge - should return a single edge
  const auto test_request3 = R"({"locations":[{"lat":52.0785070,"lon":5.110835},
               {"lat":52.078882,"lon":5.1104848}],"costing":"auto"})";
  try_path(reader, loki_worker, test_request3, 1);

  // Simple path along two way edge (opposite direction to request 3) - should edge opposing the
  // drivable direction -must not return a single edge
  const auto test_request4 = R"({"locations":[{"lat":52.078882,"lon":5.1104848},
               {"lat":52.0785070,"lon":5.110835}],"costing":"auto"})";
  try_path(reader, loki_worker, test_request4, 1);

  // Test avoidance of parking aisles. Path should avoid the shortcut via a parking aisle.
  // drivable direction -must not return a single edge
  const auto test_request5 = R"({"locations":[{"lat":52.072534,"lon":5.125980},
               {"lat":52.072862,"lon":5.124025}],"costing":"auto"})";
  try_path(reader, loki_worker, test_request5, 5);
}

int main(int argc, char* argv[]) {
  // logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
