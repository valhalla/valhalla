#include "proto/options.pb.h"
#include "sif/costconstants.h"
#include "test.h"

#include <iostream>
#include <string>
#include <valhalla/baldr/rapidjson_utils.h>
#include <vector>

#include "loki/worker.h"
#include "midgard/logging.h"
#include "odin/worker.h"
#include "thor/worker.h"

#include "sif/costfactory.h"
#include "sif/dynamiccost.h"
#include "thor/matrixalgorithm.h"
#include "thor/timedistancebssmatrix.h"

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;
using namespace valhalla::odin;

namespace rj = rapidjson;

namespace {

// Note that the "loki/radius" is intentionally left to 10, since the bss_connection is a duplication
// of the existing way on which the bike share sation is projected. It would be advisable to not set
// radius to 0 so that the algorithm will choose the best projection. Otherwise, the location may be
// projected uniquely on the bss_connection.
const auto cfg =
    test::make_config("test/data/paris_bss_tiles", {{"loki.service_defaults.radius", "10"}});
} // namespace

// The distances returned by route and matrix are not always equal to each other
// We set here 4% as the bias tolerance.
const float kDistancePercentThreshold = 0.04;
const uint32_t kTimeThreshold = 2;

class MatrixBssTest : public ::testing::Test {
public:
  MatrixBssTest() {
    Options options;
    options.set_costing_type(Costing::bikeshare);
    rapidjson::Document doc;
    sif::ParseCosting(doc, "/costing_options", options);
    sif::TravelMode mode;
    mode_costing = sif::CostFactory().CreateModeCosting(options, mode);
  }

  std::string make_matrix_request(const std::vector<std::pair<float, float>>& sources,
                                  const std::vector<std::pair<float, float>>& targets) {
    rj::Document req;
    auto& alloc = req.GetAllocator();

    auto make_req_array = [&alloc](const auto& locations) -> rj::Value {
      auto array = rj::Value{rj::kArrayType};
      for (const auto& s : locations) {
        array.PushBack(rj::Value()
                           .SetObject()
                           .AddMember("lat", s.first, alloc)
                           .AddMember("lon", s.second, alloc),
                       alloc);
      }
      return array;
    };

    req.SetObject()
        .AddMember("sources", make_req_array(sources), alloc)
        .AddMember("targets", make_req_array(targets), alloc)
        .AddMember("costing", "bikeshare", alloc);

    std::cout << "matrix request: " << rj::to_string(req) << "\n";

    return rj::to_string(req);
  }

  std::string make_matrix_request(const std::pair<float, float>& source,
                                  const std::pair<float, float>& target) {
    rj::Document req;
    auto& alloc = req.GetAllocator();

    auto locations = rj::Value{rj::kArrayType};

    locations
        .PushBack(rj::Value()
                      .SetObject()
                      .AddMember("lat", source.first, alloc)
                      .AddMember("lon", source.second, alloc),
                  alloc)
        .PushBack(rj::Value()
                      .SetObject()
                      .AddMember("lat", target.first, alloc)
                      .AddMember("lon", target.second, alloc),
                  alloc);

    req.SetObject().AddMember("locations", locations, alloc).AddMember("costing", "bikeshare", alloc);

    std::cout << "route req: " << rj::to_string(req) << "\n";

    return rj::to_string(req);
  }

  void test(const std::vector<std::pair<float, float>>& sources,
            const std::vector<std::pair<float, float>>& targets) {

    Api matrix_request;
    ParseApi(make_matrix_request(sources, targets), Options::sources_to_targets, matrix_request);
    loki_worker.matrix(matrix_request);

    timedist_matrix_bss.SourceToTarget(matrix_request, reader, mode_costing,
                                       sif::TravelMode::kPedestrian, 400000.0);

    auto s_size = sources.size();
    auto t_size = targets.size();

    // we compute the real route by iterating over the sources and targets
    for (size_t i = 0; i < s_size; ++i) {
      for (size_t j = 0; j < t_size; ++j) {

        Api route_request;
        ParseApi(make_matrix_request(sources[i], targets[j]), valhalla::Options::route,
                 route_request);
        loki_worker.route(route_request);
        thor_worker.route(route_request);
        odin_worker.narrate(route_request);

        const auto& legs = route_request.directions().routes(0).legs();
        EXPECT_EQ(legs.size(), 1) << "Should have 1 leg";

        int route_time = legs.begin()->summary().time();
        int route_length = legs.begin()->summary().length() * 1000;

        size_t m_result_idx = i * t_size + j;
        int matrix_time = matrix_request.matrix().times()[m_result_idx];
        int matrix_length = matrix_request.matrix().distances()[m_result_idx];

        EXPECT_NEAR(matrix_time, route_time, kTimeThreshold);
        EXPECT_NEAR(matrix_length, route_length, route_length * kDistancePercentThreshold);
      }
    }
  }

private:
  loki_worker_t loki_worker{cfg};
  thor_worker_t thor_worker{cfg};
  odin_worker_t odin_worker{cfg};

  GraphReader reader{cfg.get_child("mjolnir")};
  mode_costing_t mode_costing;
  TimeDistanceBSSMatrix timedist_matrix_bss;
};

TEST_F(MatrixBssTest, OneToMany) {

  test(
      // sources lat - lon
      {
          {48.858376, 2.358229},
      },
      // targets lat - lon
      {
          {48.865032, 2.362484},
          {48.862484, 2.365708},
          {48.86911, 2.36019},
          {48.865448, 2.363641},
      });
}

TEST_F(MatrixBssTest, ManyToOne) {

  test(
      // sources lat - lon
      {
          {48.858376, 2.358229},
          {48.859636, 2.362984},
          {48.857826, 2.366695},
          {48.85788, 2.36125},
      },
      // targets lat - lon
      {
          {48.865032, 2.362484},
      });
}

TEST_F(MatrixBssTest, ManyToMany) {
  test(
      // sources lat - lon
      {
          {48.858376, 2.358229},
          {48.859636, 2.362984},
          {48.857826, 2.366695},
          {48.85788, 2.36125},
      },
      // targets lat - lon
      {
          {48.865032, 2.362484},
          {48.862484, 2.365708},
          {48.86911, 2.36019},
          {48.865448, 2.363641},
      });
}

// this fails, meaning the reverse matrix tree is not the same as the AStar forward tree
TEST_F(MatrixBssTest, DISABLED_ManyToManyMoreSources) {
  test(
      // sources lat - lon
      {
          {48.858376, 2.358229},
          {48.859636, 2.362984},
          {48.857826, 2.366695},
          {48.85788, 2.36125},
      },
      // targets lat - lon
      {
          {48.865032, 2.362484},
          {48.862484, 2.365708},
          {48.86911, 2.36019},
      });
}

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
