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
#include "thor/costmatrix.h"
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

valhalla::sif::cost_ptr_t create_costing() {
  valhalla::Options options;
  for (int i = 0; i < valhalla::Costing_MAX; ++i)
    options.add_costing_options();

  for (auto costing_str : {"pedestrian", "bicycle"}) {
    valhalla::Costing costing;
    if (valhalla::Costing_Enum_Parse(costing_str, &costing)) {
      options.set_costing(costing);
    }
  }
  return valhalla::sif::CostFactory{}.Create(options);
}

// Note that the "loki/radius" is intentionally left to 10, since the bss_connection is a duplication
// of the existing way on which the bike share sation is projected. It would be advisable to not set
// radius to 0 so that the algorithm will choose the best projection. Otherwise, the location may be
// projected uniquely on the bss_connection.
const auto config = test::json_to_pt(R"({
	      "mjolnir":{"tile_dir":"test/data/paris_bss_tiles", "concurrency": 1},
	      "loki":{
	        "actions":["sources_to_targets"],
	        "logging":{"long_request": 100},
	        "service_defaults":{
				"minimum_reachability": 2,
				"radius": 10,
				"search_cutoff": 35000, 
				"node_snap_tolerance": 5, 
				"street_side_tolerance": 5, 
				"heading_tolerance": 60, 
				"street_side_max_distance": 1000}
	      },
	      "thor":{"logging":{"long_request": 100}},
	      "odin":{"logging":{"long_request": 100}},
	      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
	      "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor","search_radius"],
	              "mode":"auto","grid":{"cache_size":100240,"size":500},
	              "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
	              "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
	              "search_radius":15.0,"sigma_z":4.07,"turn_penalty_factor":200}},
	      "service_limits": {
      		"auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      		"auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      		"bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      		"bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      		"hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      		"taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      		"isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time_contour": 240, "max_distance_contour":200},
      		"max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      		"multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      		"pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      		"skadi": {"max_shape": 750000,"min_resample": 10.0},
      		"trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      		"transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      		"truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
	        "bikeshare": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    		}
	    })");

} // namespace

// The distances returned by route and matrix are not always equal to each other
// We set here 4% as the bias tolerance.
const float kDistancePercentThreshold = 0.04;
const uint32_t kTimeThreshold = 2;

class MatrixBssTest : public ::testing::Test {
public:
  MatrixBssTest() {
    Options options;
    options.set_costing(Costing::bikeshare);
    rapidjson::Document doc;
    sif::ParseCostingOptions(doc, "/costing_options", options);
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

    auto matrix_results =
        timedist_matrix_bss.SourceToTarget(matrix_request.options().sources(),
                                           matrix_request.options().targets(), reader, mode_costing,
                                           TravelMode::kPedestrian, 400000.0);

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
        int matrix_time = matrix_results[m_result_idx].time;
        int matrix_length = matrix_results[m_result_idx].dist;

        EXPECT_NEAR(matrix_time, route_time, kTimeThreshold);
        EXPECT_NEAR(matrix_length, route_length, route_length * kDistancePercentThreshold);
      }
    }
  }

private:
  loki_worker_t loki_worker{config};
  thor_worker_t thor_worker{config};
  odin_worker_t odin_worker{config};

  GraphReader reader{config.get_child("mjolnir")};
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

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
