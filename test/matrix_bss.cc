#include "proto/options.pb.h"
#include "sif/costconstants.h"
#include "test.h"
//#include "utils.h"

#include <iostream>
#include <string>
#include <vector>

#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/costfactory.h"
#include "sif/dynamiccost.h"
#include "thor/costmatrix.h"
#include "thor/timedistancebssmatrix.h"
#include "thor/worker.h"

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

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

const uint32_t kDistanceThreshold = 2;
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

  void test(const std::string& test_request, const std::vector<TimeDistance>& matrix_answers) {
    Api request;
    ParseApi(test_request, Options::sources_to_targets, request);
    loki_worker.matrix(request);

    auto results =
        timedist_matrix_bss.SourceToTarget(request.options().sources(), request.options().targets(),
                                           reader, mode_costing, TravelMode::kPedestrian, 400000.0);

    for (uint32_t i = 0; i < results.size(); ++i) {
      EXPECT_NEAR(results[i].dist, matrix_answers[i].dist, kDistanceThreshold)
          << "result " + std::to_string(i) + "'s distance is not equal to" +
                 " the expected value for TimeDistMatrix "
          << results[i].dist << " expected: " << matrix_answers[i].dist;

      EXPECT_NEAR(results[i].time, matrix_answers[i].time, kTimeThreshold)
          << "result " + std::to_string(i) +
                 "'s time is not equal to the expected value for TimeDistMatrix"
          << results[i].time << " expected " << matrix_answers[i].time;
    }
  }

private:
  loki_worker_t loki_worker{config};
  GraphReader reader{config.get_child("mjolnir")};
  mode_costing_t mode_costing;
  TimeDistanceBSSMatrix timedist_matrix_bss;
};

TEST_F(MatrixBssTest, OneToMany) {
  const auto test_request = R"({
	    "sources":[
	      {"lat":48.858376,"lon":2.358229}
	    ],
	    "targets":[
	      {"lat":48.865032,"lon":2.362484},
	      {"lat":48.862484,"lon":2.365708},
	      {"lat":48.86911, "lon":2.36019},
	      {"lat":48.865448,"lon":2.363641}
	    ],
	    "costing":"bikeshare"
	  })";
  std::vector<TimeDistance> matrix_answers = {{624, 1057}, {739, 1667}, {742, 1699}, {693, 1151}};
  test(test_request, matrix_answers);
}

TEST_F(MatrixBssTest, ManyToOne) {
  const auto test_request = R"({
	    "sources":[
	      {"lat":48.858376,"lon":2.358229},
	      {"lat":48.859636,"lon":2.362984},
	      {"lat":48.857826,"lon":2.366695},
	      {"lat":48.85788 ,"lon":2.36125 }
	    ],
	    "targets":[
	      {"lat":48.865032,"lon":2.362484}
	    ],
	    "costing":"bikeshare"
	  })";
  std::vector<TimeDistance> matrix_answers = {{624, 1057}, {616, 1031}, {659, 1332}, {679, 1121}};
  test(test_request, matrix_answers);
}

TEST_F(MatrixBssTest, ManyToMany) {
  const auto test_request = R"({
	    "sources":[
	      {"lat":48.858376,"lon":2.358229},
	      {"lat":48.859636,"lon":2.362984},
	      {"lat":48.857826,"lon":2.366695},
	      {"lat":48.85788 ,"lon":2.36125 }
	    ],
	    "targets":[
	      {"lat":48.865032,"lon":2.362484},
	      {"lat":48.862484,"lon":2.365708},
	      {"lat":48.86911, "lon": 2.36019},
	      {"lat":48.865448,"lon":2.363641}
	    ],
	    "costing":"bikeshare"
	  })";

  std::vector<TimeDistance> matrix_answers = {{624, 1057}, {739, 1667}, {742, 1699}, {693, 1151},
                                              {616, 1031}, {313, 441},  {734, 1673}, {557, 782},
                                              {659, 1332}, {489, 691},  {693, 1583}, {727, 1425},
                                              {679, 1121}, {486, 685},  {797, 1763}, {747, 1214}};

  test(test_request, matrix_answers);
}

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
