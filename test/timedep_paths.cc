#include "test.h"

#include <string>

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

const auto cfg = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");

void try_path(GraphReader& reader,
              loki_worker_t& loki_worker,
              const bool depart_at,
              const char* test_request,
              const uint32_t expected_edgecount) {
  Api request;
  ParseApi(test_request, Options::route, request);
  loki_worker.route(request);
  thor_worker_t::adjust_scores(*request.mutable_options());

  // For now this just tests auto costing - could extend to other
  travel_mode_t mode;
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
  loki_worker_t loki_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  // Simple path along oneway edge in the drivable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request1, 1);

  // Simple path along oneway edge opposing the drivable direction -must not
  // return a single edge (edge count is 10)
  const auto test_request2 = R"({"locations":[{"lat":52.078937,"lon":5.115321},
               {"lat":52.079079,"lon":5.115197}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request2, 10);
}

TEST(TimeDepPaths, test_arrive_by_paths) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  // Simple path along oneway edge in the drivable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":2,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, false, test_request1, 1);
}

class TimeDepForwardTest : public thor::TimeDepForward {
public:
  explicit TimeDepForwardTest(const boost::property_tree::ptree& config = {})
      : TimeDepForward(config) {
  }

  void Clear() {
    TimeDepForward::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(TimeDepPaths, test_forward_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  TimeDepForwardTest time_dep(config);
  time_dep.Clear();
}

TEST(TimeDepPaths, test_forward_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_astar", 10);

  TimeDepForwardTest time_dep(config);
  time_dep.Clear();
}

class TimeDepReverseTest : public thor::TimeDepReverse {
public:
  explicit TimeDepReverseTest(const boost::property_tree::ptree& config = {})
      : TimeDepReverse(config) {
  }

  void Clear() {
    TimeDepReverse::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(TimeDepPaths, test_reverse_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  TimeDepReverseTest time_dep(config);
  time_dep.Clear();
}

TEST(TimeDepPaths, test_reverse_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_dijkstras", 10);

  TimeDepReverseTest time_dep(config);
  time_dep.Clear();
}

int main(int argc, char* argv[]) {
  // logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
