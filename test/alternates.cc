#include "test.h"

#include <iostream>
#include <string>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

boost::property_tree::ptree get_conf() {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
      "loki":{
        "actions":["route"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
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
        "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time_contour": 120, "max_distance_contour":200},
        "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
        "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
        "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
        "skadi": {"max_shape": 750000,"min_resample": 10.0},
        "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

struct route_tester {
  route_tester()
      : conf(get_conf()), reader(new GraphReader(conf.get_child("mjolnir"))),
        loki_worker(conf, reader), thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    std::pair<std::list<TripLeg>, std::list<DirectionsLeg>> results;
    thor_worker.route(request);
    odin_worker.narrate(request);
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

void test_alternates(int num_alternates) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.111893,"lon":5.125282},
      {"lat":52.113731,"lon":5.091155}],"costing":"auto",
      "alternates":)" +
      std::to_string(num_alternates) + "}";

  auto response = tester.test(request);
  const auto& routes = response.trip().routes();

  if (routes.size() != num_alternates + 1)
    throw std::logic_error("Expected " + std::to_string(num_alternates + 1) + " routes, got " +
                           std::to_string(routes.size()));
}

TEST(Alternates, test_zero_alternates) {
  test_alternates(0);
}

TEST(Alternates, test_one_alternate) {
  test_alternates(1);
}

TEST(Alternates, test_two_alternates) {
  test_alternates(2);
}
} // namespace

class Alternates : public ::testing::Environment {
public:
  void SetUp() override {
  }

  void TearDown() override {
  }
};

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new Alternates);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
