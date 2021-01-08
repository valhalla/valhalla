#include "test.h"

#include "midgard/logging.h"
#include "thor/attributes_controller.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include <thread>
#include <unistd.h>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;

namespace {

// fake config
const auto conf = test::json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "street_side_max_distance": 1000, "heading_tolerance": 60}
    },
    "thor":{"logging":{"long_request": 110}},
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
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time_contour": 120, "max_distance_contour":200},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,"max_alternates":2,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

TEST(ThorWorker, test_parse_filter_attributes_defaults) {
  tyr::actor_t actor(conf, true);

  auto result = test::json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}]})"));

  EXPECT_FALSE(result.get_child_optional("shape_attributes")) << "Expected excluded shape_attributes";

  EXPECT_TRUE(result.get_child_optional("edges")) << "Expected included edges";

  EXPECT_TRUE(result.get_child_optional("shape")) << "Expected included shape";
  EXPECT_FALSE(result.get_child_optional("edge.show_incidents"))
      << "Expected excluded edge.show_incidents";
}

TEST(ThorWorker, test_parse_filter_attributes_excludes) {
  tyr::actor_t actor(conf, true);

  std::vector<std::string> test_cases = {
      actor.trace_attributes(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "filters":{"attributes":["shape"], "action":"exclude"}})"),
      // TODO currently EnhancedTripPath sets any excluded fields to their default value
      // so this test can't check for excluded keys. Come back and add tests for
      // shape_attributes once we have them since those will be untouched by ETP.
      // actor.trace_route(
      //     R"({"costing":"auto","shape_match":"map_snap","shape":[
      //     {"lat":52.09110,"lon":5.09806},
      //     {"lat":52.09098,"lon":5.09679}],
      //     "filters":{"attributes":["shape"], "action":"exclude"}})"),
      // actor.route(
      //     R"({"costing":"auto","locations":[
      //     {"lat":52.09110,"lon":5.09806},
      //     {"lat":52.09098,"lon":5.09679}],
      //     "filters":{"attributes":["shape"], "action":"exclude"}})"),
  };
  std::vector<std::string> excluded_keys = {"shape", "trip.legs..shape", "trip.legs..shape"};

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto result = test::json_to_pt(test_cases[i]);
    EXPECT_FALSE(result.get_child_optional(excluded_keys[i]))
        << "Expected excluded shape | found " + excluded_keys[i] + "=" +
               result.get<std::string>(excluded_keys[i]);
  }
}

TEST(ThorWorker, test_parse_filter_attributes_includes) {
  tyr::actor_t actor(conf, true);

  std::vector<std::string> test_cases = {
      actor.trace_attributes(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],"filters":{"attributes":["shape"], "action":"include"}})"),
      actor.trace_route(
          R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "filters":{"attributes":["shape"], "action":"include"}})"),
      actor.route(
          R"({"costing":"auto","locations":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],"filters":{"attributes":["shape"],
          "action":"include"}})"),
  };
  std::vector<std::string> included_keys = {"shape", "trip.legs..shape", "trip.legs..shape"};

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto result = test::json_to_pt(test_cases[i]);
    EXPECT_TRUE(result.get_child_optional(included_keys[i]))
        << "Expected " + included_keys[i] + " to be present";
  }
}

TEST(ThorWorker, test_linear_references) {
  std::vector<std::string> requests = {
      R"({"costing":"auto","linear_references":true,"locations":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}],
          "action":"include"})",
  };
  const std::vector<std::string>& expected = {
      "CwOgEyUK5SKXAP/H//wiBw==",
      "CwOf+CUK4iKXAP/k//8iBw==",
      "CwOf6yUK4SKXAP/Y//0iBw==",
  };
  tyr::actor_t actor(conf, true);
  for (const auto& request : requests) {
    auto result = test::json_to_pt(actor.route(request));
    std::vector<std::string> references;
    for (const auto& reference : result.get_child("trip.linear_references"))
      references.push_back(reference.second.get_value<std::string>());
    EXPECT_EQ(references.size(), 3);
    EXPECT_EQ(expected, references);
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
