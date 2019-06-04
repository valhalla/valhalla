#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <stdexcept>

#include "baldr/rapidjson_utils.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "tyr/actor.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::midgard;

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

void test_shape_attributes_included() {
  tyr::actor_t actor(conf);

  auto result_json = actor.trace_attributes(
      R"({"shape":[
        {"lat":52.09110,"lon":5.09806},
        {"lat":52.09050,"lon":5.09769},
        {"lat":52.09098,"lon":5.09679}
      ],"costing":"auto","shape_match":"map_snap"})");

  rapidjson::Document doc;
  doc.Parse(result_json);
  if (doc.HasParseError()) {
    throw std::logic_error("");
  }

  auto shape =  midgard::decode<std::vector<PointLL>>(rapidjson::Pointer("/shape").Get(doc)->GetString());
  auto shape_attributes_time =  rapidjson::Pointer("/shape_attributes/time").Get(doc)->GetArray();
  auto shape_attributes_length =  rapidjson::Pointer("/shape_attributes/length").Get(doc)->GetArray();
  auto shape_attributes_speed =  rapidjson::Pointer("/shape_attributes/speed").Get(doc)->GetArray();
  auto edges =  rapidjson::Pointer("/edges").Get(doc)->GetArray();

  if (shape_attributes_time.Size() != shape.size() - 1)
    throw std::logic_error("Expected: " + std::to_string(shape.size() - 1) + " | Found: " + std::to_string(shape_attributes_time.Size()));
  if (shape_attributes_length.Size() != shape.size() - 1)
      throw std::logic_error("Expected: " + std::to_string(shape.size() - 1) + " | Found: " + std::to_string(shape_attributes_length.Size()));
  if (shape_attributes_speed.Size() != shape.size() - 1)
        throw std::logic_error("Expected: " + std::to_string(shape.size() - 1) + " | Found: " + std::to_string(shape_attributes_speed.Size()));

  // Measures the length between point
  for (int i = 1; i < shape.size(); i++) {
    std::cout << "Time:: " + std::to_string(shape_attributes_time[i-1].GetDouble()) + "\n";
    std::cout << "Length:: " + std::to_string(shape_attributes_length[i-1].GetFloat()) + "\n";
    std::cout << "Speed:: " + std::to_string(shape_attributes_speed[i-1].GetDouble()) + "\n";
    // Measuring that the length between shape pts is approx. to the shape attributes length
    if (!midgard::equal(shape[i].Distance(shape[i-1]) * .001f, shape_attributes_length[i-1].GetFloat(), .01f)) {
      throw std::logic_error("Expected: " + std::to_string(shape_attributes_length[i-1].GetFloat()) + " | Found: " + std::to_string(shape[i].Distance(shape[i-1]) * .001f));
    }
  }
  for (int e = 1; e < edges.Size(); e++) {
    auto edge_speed = edges[e]["speed"].GetDouble();
    std::cout << "edge_speed :: " + std::to_string(edge_speed) + "\n";
  }
  // TODO: test a multi edge route, decode the shape and measure that the length between each
  // is appx the same as the length in the shape attributes. Get the edge time from request and validate
  // that that makes sense as well.
}

} // namespace

int main() {
  test::suite suite("shape_attributes");

  suite.test(TEST_CASE(test_shape_attributes_included));

  return suite.tear_down();
}
