#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/serializers.h"
#include <boost/property_tree/ptree.hpp>

#include "gurka/gurka.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::loki;
using namespace valhalla::tyr;

namespace {

boost::property_tree::ptree get_conf() {
  std::stringstream ss;
  ss << R"({
      "mjolnir":{"tile_dir":"test/data/utrecht_tiles","data_processing": {"use_urban_tag": true}},
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
    thor_worker.route(request);
    odin_worker.narrate(request);
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  boost::property_tree::ptree conf;
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

// http://localhost:8002/route?json={"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","directions_options":{"units":"miles","format":"osrm"}}&access_token=
TEST(Urban2, test_urban) {
  bool is_urban;
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","units":"miles","format":"osrm","filters":{"action":"include","attributes":["edge.is_urban"]}})";
  auto response = tester.test(request);

  // get the osrm json
  auto json_str = serializeDirections(response);
  rapidjson::Document json;
  json.Parse(json_str);

  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& route : json["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("is_urban")) {
            is_urban = intersection["is_urban"].GetBool();
            EXPECT_EQ(is_urban, true);
          }
        }
      }
    }
  }
}

TEST(Urban2, test_urban_excluded_by_default) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.10160225589803,"lon":5.116925239562988},{"lat":52.09403591712907,"lon":5.113234519958496}],"costing":"auto","units":"miles","format":"osrm"})";
  auto response = tester.test(request);

  // get the osrm json
  auto json_str = serializeDirections(response);
  rapidjson::Document json;
  json.Parse(json_str);

  ASSERT_FALSE(json.HasParseError());

  // loop over all routes all legs
  for (const auto& route : json["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("is_urban"), false);
        }
      }
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
