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
  SimpleCost(const Costing& options) : DynamicCost(options, sif::TravelMode::kDrive, kAutoAccess) {
  }

  ~SimpleCost() {
  }

  bool Allowed(const DirectedEdge* edge,
               const bool /*is_dest*/,
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
                const baldr::TimeInfo& /*time_info*/,
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
                             const bool /*has_measured_speed*/,
                             const InternalTurn /*internal_turn*/) const override {
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

cost_ptr_t CreateSimpleCost(const Costing& options) {
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
    {"auto", 43200.0f},      {"auto_shorter", 43200.0f},  {"bicycle", 7200.0f},
    {"bus", 43200.0f},       {"motor_scooter", 14400.0f}, {"multimodal", 7200.0f},
    {"pedestrian", 7200.0f}, {"transit", 14400.0f},       {"truck", 43200.0f},
    {"taxi", 43200.0f},
};
// a scale factor to apply to the score so that we bias towards closer results more
constexpr float kDistanceScale = 10.f;

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

std::vector<TimeDistance> matrix_answers = {{28, 28},     {2027, 1837}, {2403, 2213}, {4163, 3838},
                                            {1519, 1398}, {1808, 1638}, {2061, 1951}, {3944, 3639},
                                            {2311, 2111}, {701, 641},   {0, 0},       {2821, 2626},
                                            {5562, 5177}, {3952, 3707}, {4367, 4107}, {1825, 1680}};
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
  thor_worker_t::adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] =
      CreateSimpleCost(request.options().costings().find(request.options().costing_type())->second);

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results =
      cost_matrix.SourceToTarget(request.options().sources(), request.options().targets(), reader,
                                 mode_costing, sif::TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].dist, matrix_answers[i].dist, kThreshold)
        << "result " + std::to_string(i) + "'s distance is not close enough" +
               " to expected value for CostMatrix";

    EXPECT_NEAR(results[i].time, matrix_answers[i].time, kThreshold)
        << "result " + std::to_string(i) + "'s time is not close enough" +
               " to expected value for CostMatrix";
  }

  CostMatrix cost_matrix_abort_source;
  results = cost_matrix_abort_source.SourceToTarget(request.options().sources(),
                                                    request.options().targets(), reader, mode_costing,
                                                    sif::TravelMode::kDrive, 100000.0);

  uint32_t found = 0;
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist < kMaxCost) {
      ++found;
    }
  }
  EXPECT_EQ(found, 15) << " not the number of results as expected";

  CostMatrix cost_matrix_abort_target;
  results = cost_matrix_abort_target.SourceToTarget(request.options().sources(),
                                                    request.options().targets(), reader, mode_costing,
                                                    sif::TravelMode::kDrive, 50000.0);

  found = 0;
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist < kMaxCost) {
      ++found;
    }
  }
  EXPECT_EQ(found, 10) << " not the number of results as expected";

  TimeDistanceMatrix timedist_matrix;
  results = timedist_matrix.SourceToTarget(request.options().sources(), request.options().targets(),
                                           reader, mode_costing, sif::TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].dist, matrix_answers[i].dist, kThreshold)
        << "result " + std::to_string(i) + "'s distance is not equal to" +
               " the expected value for TimeDistMatrix";

    EXPECT_NEAR(results[i].time, matrix_answers[i].time, kThreshold)
        << "result " + std::to_string(i) +
               "'s time is not equal to the expected value for TimeDistMatrix";
  }
}

TEST(Matrix, test_timedistancematrix_results_sequence) {
  // Input request is the same as `test_request`, but without the last target
  const auto test_request_more_sources = R"({
    "sources":[
      {"lat":52.106337,"lon":5.101728},
      {"lat":52.111276,"lon":5.089717},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005}
    ],
    "costing":"auto"
  })";

  loki_worker_t loki_worker(config);

  Api request;
  ParseApi(test_request_more_sources, Options::sources_to_targets, request);
  loki_worker.matrix(request);
  thor_worker_t::adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] =
      CreateSimpleCost(request.options().costings().find(request.options().costing_type())->second);

  TimeDistanceMatrix timedist_matrix;
  std::vector<TimeDistance> results =
      timedist_matrix.SourceToTarget(request.options().sources(), request.options().targets(), reader,
                                     mode_costing, sif::TravelMode::kDrive, 400000.0);

  // expected results are the same as `matrix_answers`, but without the last column
  std::vector<TimeDistance> expected_results = {
      {28, 28},     {2027, 1837}, {2403, 2213}, {1519, 1398}, {1808, 1638}, {2061, 1951},
      {2311, 2111}, {701, 641},   {0, 0},       {5562, 5177}, {3952, 3707}, {4367, 4107},
  };

  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].dist, expected_results[i].dist, kThreshold)
        << "result " + std::to_string(i) + "'s distance is not equal to" +
               " the expected value for TimeDistMatrix";

    EXPECT_NEAR(results[i].time, expected_results[i].time, kThreshold)
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
  thor_worker_t::adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] =
      CreateSimpleCost(request.options().costings().find(request.options().costing_type())->second);

  CostMatrix cost_matrix;
  std::vector<TimeDistance> results;
  results = cost_matrix.SourceToTarget(request.options().sources(), request.options().targets(),
                                       reader, mode_costing, sif::TravelMode::kDrive, 400000.0);
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
                                           reader, mode_costing, sif::TravelMode::kDrive, 400000.0);
  for (uint32_t i = 0; i < results.size(); ++i) {
    EXPECT_EQ(results[i].dist, matrix_answers[i].dist)
        << "result " + std::to_string(i) +
               "'s distance is not equal to the expected value for TimeDistMatrix.";

    EXPECT_EQ(results[i].time, matrix_answers[i].time)
        << "result " + std::to_string(i) +
               "'s time is not equal to the expected value for TimeDistMatrix";
  }
}

const auto test_request_partial = R"({
    "sources":[
      {"lat":52.103948,"lon":5.06813}
    ],
    "targets":[
      {"lat":52.106126,"lon":5.101497},
      {"lat":52.100469,"lon":5.087099},
      {"lat":52.103105,"lon":5.081005},
      {"lat":52.094273,"lon":5.075254}
    ],
    "costing":"auto",
    "matrix_locations":2
  })";

TEST(Matrix, partial_matrix) {
  loki_worker_t loki_worker(config);

  Api request;
  ParseApi(test_request_partial, Options::sources_to_targets, request);
  loki_worker.matrix(request);
  thor_worker_t::adjust_scores(*request.mutable_options());

  GraphReader reader(config.get_child("mjolnir"));

  sif::mode_costing_t mode_costing;
  mode_costing[0] =
      CreateSimpleCost(request.options().costings().find(request.options().costing_type())->second);

  TimeDistanceMatrix timedist_matrix;
  std::vector<TimeDistance> results =
      timedist_matrix.SourceToTarget(request.options().sources(), request.options().targets(), reader,
                                     mode_costing, sif::TravelMode::kDrive, 400000.0,
                                     request.options().matrix_locations());
  uint32_t found = 0;
  for (uint32_t i = 0; i < results.size(); ++i) {
    if (results[i].dist > 0) {
      ++found;
    }
  }
  EXPECT_EQ(found, 2) << " partial result did not find 2 results as expected";
}

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
