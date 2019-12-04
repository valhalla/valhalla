#include "test.h"

#include "midgard/logging.h"
#include "thor/attributes_controller.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include <boost/property_tree/ptree.hpp>
#include <thread>
#include <unistd.h>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::thor;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

// fake config
const auto conf = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
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

void test_parse_filter_attributes_defaults() {
  tyr::actor_t actor(conf, true);

  auto result = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09098,"lon":5.09679}]})"));

  if (result.get_child_optional("shape_attributes"))
    throw std::logic_error("Expected excluded shape_attributes | found shape_attributes=" +
                           result.get<std::string>("shape_attributes"));

  if (!result.get_child_optional("edges"))
    throw std::logic_error("Expected included edges");

  if (!result.get_child_optional("shape"))
    throw std::logic_error("Expected included shape");
}

void test_parse_filter_attributes_excludes() {
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
    auto result = json_to_pt(test_cases[i]);
    if (result.get_child_optional(excluded_keys[i]))
      throw std::logic_error("Expected excluded shape | found " + excluded_keys[i] + "=" +
                             result.get<std::string>(excluded_keys[i]));
  }
}

void test_parse_filter_attributes_includes() {
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
    auto result = json_to_pt(test_cases[i]);
    if (!result.get_child_optional(included_keys[i]))
      throw std::logic_error("Expected " + included_keys[i] + " to be present");
  }
}
} // namespace

int main(void) {
  test::suite suite("Thor Worker");

  suite.test(TEST_CASE(test_parse_filter_attributes_defaults));

  suite.test(TEST_CASE(test_parse_filter_attributes_excludes));

  suite.test(TEST_CASE(test_parse_filter_attributes_includes));

  return suite.tear_down();
}
