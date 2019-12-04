#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "sif/autocost.h"
#include "sif/bicyclecost.h"
#include "thor/isochrone.h"
#include "thor/worker.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

const auto config = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["sources_to_targets"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
    },
    "thor":{
      "logging":{"long_request": 100}
    },
    "meili":{
      "grid": {
        "cache_size": 100240,
        "size": 500
      }
    },
    "service_limits": {
      "auto": {"max_distance": 5000000.0, "max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "auto_shorter": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "bicycle": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "bus": {"max_distance": 5000000.0,"max_locations": 50,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "hov": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "taxi": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50},
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

} // namespace

void try_isochrone(GraphReader& reader,
                   loki_worker_t& loki_worker,
                   thor_worker_t& thor_worker,
                   const char* test_request,
                   const std::string& expected) {
  Api request;
  ParseApi(test_request, Options::isochrone, request);
  loki_worker.isochrones(request);

  // Process isochrone request
  auto result = thor_worker.isochrones(request);

  // Check if the result contains the expected string
  if (result.find(expected) == std::string::npos) {
    throw std::runtime_error("isochrones failed: expected " + expected);
  }
}

void test_isochrones() {
  // Test setup
  loki_worker_t loki_worker(config);
  thor_worker_t thor_worker(config);
  GraphReader reader(config.get_child("mjolnir"));

// Test auto isochrone with one contour
// 32bit builds fail in release mode we'll look at this separately
#if _WIN64 || __amd64__
  std::string expected1 = "\"type\":\"LineString\"";
  const auto test_request1 =
      R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"auto","contours":[{"time":15}],"polygons":false,"denoise":0.2,"generalize":150})";
  try_isochrone(reader, loki_worker, thor_worker, test_request1, expected1);

  // Try pedestrian isochrone with one contour, polygon=true
  std::string expected2 = "\"type\":\"Polygon\"";
  const auto test_request2 =
      R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"bicycle","contours":[{"time":15}],"polygons":true,"denoise":0.2})";
  try_isochrone(reader, loki_worker, thor_worker, test_request2, expected2);
#endif
}

int main(int argc, char* argv[]) {
  test::suite suite("isochrones");

  // Silence logs (especially long request logging)
  logging::Configure({{"type", ""}});

  suite.test(TEST_CASE(test_isochrones));

  return suite.tear_down();
}
