#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/dynamiccost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancematrix.h"
#include "thor/worker.h"

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {

// Quick costing class derived for testing so that any changes to regular costing
// won't change the outcome of the tests. Some of the logic for this class is just
// copy pasted from AutoCost as it stands when this test was written.
class SimpleCost final : public DynamicCost {
public:
  /**
   * Constructor.
   * @param  options Request options in a pbf
   */
  SimpleCost(const CostingOptions& options) : DynamicCost(options, TravelMode::kDrive, kAutoAccess) {
  }

  ~SimpleCost() {
  }

  bool Allowed(const DirectedEdge* edge,
               const EdgeLabel& pred,
               const graph_tile_ptr& /*tile*/,
               const GraphId& edgeid,
               const uint64_t /*current_time*/,
               const uint32_t /*tz_index*/,
               uint8_t& /*restriction_idx*/) const override {
    if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
        (pred.restrictions() & (1 << edge->localedgeidx())) ||
        edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
        (!allow_destination_only_ && !pred.destonly() && edge->destonly())) {
      return false;
    }
    return true;
  }

  bool AllowedReverse(const DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const DirectedEdge* opp_edge,
                      const graph_tile_ptr& /*tile*/,
                      const GraphId& opp_edgeid,
                      const uint64_t /*current_time*/,
                      const uint32_t /*tz_index*/,
                      uint8_t& /*restriction_idx*/) const override {
    if (!IsAccessible(opp_edge) ||
        (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
        (opp_edge->restrictions() & (1 << pred.opp_local_idx())) ||
        opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
        (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly())) {
      return false;
    }
    return true;
  }

  Cost EdgeCost(const baldr::DirectedEdge* /*edge*/,
                const baldr::TransitDeparture* /*departure*/,
                const uint32_t /*curr_time*/) const override {
    throw std::runtime_error("We shouldnt be testing transit edges");
  }

  Cost EdgeCost(const DirectedEdge* edge,
                const graph_tile_ptr& /*tile*/,
                const uint32_t /*seconds*/,
                uint8_t& /*flow_sources*/) const override {
    float sec = static_cast<float>(edge->length());
    return {sec / 10.0f, sec};
  }

  Cost TransitionCost(const DirectedEdge* /*edge*/,
                      const NodeInfo* /*node*/,
                      const EdgeLabel& /*pred*/) const override {
    return {5.0f, 5.0f};
  }

  Cost TransitionCostReverse(const uint32_t /*idx*/,
                             const NodeInfo* /*node*/,
                             const DirectedEdge* /*opp_edge*/,
                             const DirectedEdge* /*opp_pred_edge*/,
                             const bool /*has_measured_speed*/) const override {
    return {5.0f, 5.0f};
  }

  float AStarCostFactor() const override {
    return 0.1f;
  }

  bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr&, uint16_t) const override {
    auto access_mask = (ignore_access_ ? kAllAccess : access_mask_);
    bool accessible = (edge->forwardaccess() & access_mask) ||
                      (ignore_oneways_ && (edge->reverseaccess() & access_mask));
    if (edge->is_shortcut() || !accessible)
      return 0.0f;
    else {
      return 1.0f;
    }
  }
};

cost_ptr_t CreateSimpleCost(const CostingOptions& options) {
  return std::make_shared<SimpleCost>(options);
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

const auto config = test::make_config("test/data/utrecht_tiles");

const auto test_request = R"({
    "sources":[
      {"lat":52.106337,"lon":5.101728},
      {"lat":52.111276,"lon":5.089717},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.094273,"lon":5.075254}
    ],
    "costing":"auto"
  })";

const auto test_request_osrm = R"({
    "sources":[
      {"lat":52.106337,"lon":5.101728},
      {"lat":52.111276,"lon":5.089717},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.094273,"lon":5.075254}
    ],
    "costing":"auto"
  }&format=osrm)";

std::vector<TimeDistance> matrix_answers = {{28, 28},     {2027, 1837}, {2390, 2209}, {4163, 3838},
                                            {1519, 1398}, {1808, 1638}, {2042, 1937}, {3944, 3639},
                                            {2298, 2107}, {687, 637},   {0, 0},       {2808, 2623},
                                            {5552, 5177}, {3942, 3707}, {4344, 4104}, {1815, 1680}};
} // namespace

const uint32_t kThreshold = 1;
bool within_tolerance(const uint32_t v1, const uint32_t v2) {
  return (v1 > v2) ? v1 - v2 <= kThreshold : v2 - v1 <= kThreshold;
}

TEST(Matrix, test_matrix) {
  loki_worker_t loki_worker(config);

  Api request;
  ParseApi(test_request, Options::sources_to_targets, request);
  loki_worker.matrix(request);
  adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] = CreateSimpleCost(
      request.options().costing_options(static_cast<int>(request.options().costing())));

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results =
      cost_matrix.SourceToTarget(request.options().sources(), request.options().targets(), reader,
                                 mode_costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].dist, matrix_answers[i].dist, kThreshold)
        << "result " + std::to_string(i) + "'s distance is not close enough" +
               " to expected value for CostMatrix";

    EXPECT_NEAR(results[i].time, matrix_answers[i].time, kThreshold)
        << "result " + std::to_string(i) + "'s time is not close enough" +
               " to expected value for CostMatrix";
  }

  TimeDistanceMatrix timedist_matrix;
  results = timedist_matrix.SourceToTarget(request.options().sources(), request.options().targets(),
                                           reader, mode_costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].dist, matrix_answers[i].dist, kThreshold)
        << "result " + std::to_string(i) + "'s distance is not equal to" +
               " the expected value for TimeDistMatrix";

    EXPECT_NEAR(results[i].time, matrix_answers[i].time, kThreshold)
        << "result " + std::to_string(i) +
               "'s time is not equal to the expected value for TimeDistMatrix";
  }
}

// TODO: it was commented before. Why?
TEST(Matrix, DISABLED_test_matrix_osrm) {
  loki_worker_t loki_worker(config);

  Api request;
  ParseApi(test_request_osrm, Options::sources_to_targets, request);

  loki_worker.matrix(request);
  adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] = CreateSimpleCost(
      request.options().costing_options(static_cast<int>(request.options().costing())));

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results;
  results = cost_matrix.SourceToTarget(request.options().sources(), request.options().targets(),
                                       reader, mode_costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_EQ(results[i].dist, matrix_answers[i].dist)
        << "result " + std::to_string(i) +
               "'s distance is not close enough to expected value for CostMatrix.";

    EXPECT_EQ(results[i].time, matrix_answers[i].time)
        << "result " + std::to_string(i) +
               "'s time is not close enough to expected value for CostMatrix.";
  }

  TimeDistanceMatrix timedist_matrix;
  results = timedist_matrix.SourceToTarget(request.options().sources(), request.options().targets(),
                                           reader, mode_costing, TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_EQ(results[i].dist, matrix_answers[i].dist)
        << "result " + std::to_string(i) +
               "'s distance is not equal to the expected value for TimeDistMatrix.";

    EXPECT_EQ(results[i].time, matrix_answers[i].time)
        << "result " + std::to_string(i) +
               "'s time is not equal to the expected value for TimeDistMatrix";
  }
}

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
