#include "test.h"

#include <stdexcept>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "tyr/actor.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

namespace {

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  rapidjson::read_json(ss, pt);
  return pt;
}

boost::property_tree::ptree make_conf() {
  // fake up config against pine grove traffic extract
  auto conf = json_to_pt(R"({
      "mjolnir":{"tile_dir":"test/traffic_matcher_tiles"},
      "loki":{
        "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes","transit_available"],
        "logging":{"long_request": 100},
        "service_defaults":{"minimum_reachability": 50,"radius": 0,"search_cutoff": 35000, "node_snap_tolerance": 5, "street_side_tolerance": 5, "heading_tolerance": 60}
      },
      "thor":{"logging":{"long_request": 110}},
      "skadi":{"actons":["height"],"logging":{"long_request": 5}},
      "meili":{"customizable": ["breakage_distance"],
               "mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
               "max_route_distance_factor":3,"max_route_time_factor":3,"max_search_radius":100,"route":true,
               "search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}},
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
        "trace": { "max_best_paths": 4, "max_best_paths_shape": 100, "max_distance": 200000.0, "max_gps_accuracy": 100.0, "max_search_radius": 100, "max_shape": 16000 },
        "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
        "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
      }
    })");

  conf.get_child("mjolnir").put("tile_dir", VALHALLA_SOURCE_DIR "test/traffic_matcher_tiles");
  return conf;
}

void test_actor() {
  auto conf = make_conf();
  tyr::actor_t actor(conf);

  actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
      {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route_json = actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
          {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})");
  actor.cleanup();
  auto route = json_to_pt(route_json);
  route_json.find("Tulpehocken");

  actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes_json = actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
      {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})");
  actor.cleanup();
  auto attributes = json_to_pt(attributes_json);
  attributes_json.find("Tulpehocken");

  actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit_json =
      actor.transit_available(R"({"locations":[{"lat":35.647452, "lon":-79.597477, "radius":20},
      {"lat":34.766908, "lon":-80.325936,"radius":10}]})");
  actor.cleanup();
  auto transit = json_to_pt(transit_json);
  transit_json.find(std::to_string(false));

  // TODO: test the rest of them
}

void test_interrupt() {
  auto conf = make_conf();
  tyr::actor_t actor(conf);
  struct test_exception_t {};

  try {
    actor.route(R"({"locations":[{"lat":40.546115,"lon":-76.385076,"type":"break"},
        {"lat":40.544232,"lon":-76.385752,"type":"break"}],"costing":"auto"})",
                []() -> void { throw test_exception_t{}; });
    throw std::logic_error("this should have thrown already");
  } catch (const test_exception_t& e) {}

  try {
    actor.trace_attributes(R"({"shape":[{"lat":40.546115,"lon":-76.385076},
        {"lat":40.544232,"lon":-76.385752}],"costing":"auto","shape_match":"map_snap"})",
                           []() -> void { throw test_exception_t{}; });
    throw std::logic_error("this should have thrown already");
  } catch (const test_exception_t& e) {}

  // TODO: test the rest of them
}

} // namespace

int main() {
  test::suite suite("actor");

  suite.test(TEST_CASE(test_actor));

  suite.test(TEST_CASE(test_interrupt));

  return suite.tear_down();
}
