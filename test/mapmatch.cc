#include "test.h"

#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include <boost/optional/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "baldr/json.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/util.h"
#include "tyr/actor.h"

using namespace valhalla;

namespace std {
std::string to_string(const midgard::PointLL& p) {
  return "[" + to_string(p.first) + "," + to_string(p.second) + "]";
}
} // namespace std

namespace {

template <class container_t> std::string print(const container_t& container) {
  std::string output;
  for (const auto& e : container)
    output += std::to_string(e) + ",";
  if (container.size())
    output.pop_back();
  return output;
}

float round_up(float val, int multiple) {
  return int((val + multiple - 1) / multiple) * multiple;
}

std::string to_locations(const std::vector<PointLL>& shape,
                         const std::vector<float>& accuracies,
                         int frequency) {
  std::string locations = "[";
  int freq = 0;
  for (size_t i = 0; i < shape.size(); ++i) {
    // round accuracy up to nearest 5m
    int accuracy = round_up(accuracies[i] + 1.f, 5);
    std::string acc = R"(,"accuracy":)" + std::to_string(accuracy);
    // add this point on
    const auto& p = shape[i];
    locations +=
        R"({"lat":)" + std::to_string(p.second) + R"(,"lon":)" + std::to_string(p.first) + acc;
    // get the time component
    freq += frequency;
    if (freq > 0)
      locations += R"(,"time":)" + std::to_string(freq);
    locations += "},";
  }
  locations.back() = ']';
  return locations;
}

boost::property_tree::ptree json_to_pt(const std::string& json) {
  std::stringstream ss;
  ss << json;
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(ss, pt);
  return pt;
}

// fake config
const auto conf = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0}
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
      "isochrone": {"max_contours": 4,"max_distance": 25000.0,"max_locations": 1,"max_time": 120},
      "max_avoid_locations": 50,"max_radius": 200,"max_reachability": 100,
      "multimodal": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 0.0,"max_matrix_locations": 0},
      "pedestrian": {"max_distance": 250000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50,"max_transit_walking_distance": 10000,"min_transit_walking_distance": 1},
      "skadi": {"max_shape": 750000,"min_resample": 10.0},
      "trace": {"max_distance": 200000.0,"max_gps_accuracy": 100.0,"max_search_radius": 100,"max_shape": 16000,"max_best_paths":4,"max_best_paths_shape":100},
      "transit": {"max_distance": 500000.0,"max_locations": 50,"max_matrix_distance": 200000.0,"max_matrix_locations": 50},
      "truck": {"max_distance": 5000000.0,"max_locations": 20,"max_matrix_distance": 400000.0,"max_matrix_locations": 50}
    }
  })");

std::string json_escape(const std::string& unescaped) {
  std::stringstream ss;
  baldr::json::OstreamVisitor v(ss);
  v(unescaped);
  std::string escaped = ss.str().substr(1);
  escaped.pop_back();
  return escaped;
}

int seed = 973;
int bound = 81;
std::string make_test_case(PointLL& start, PointLL& end) {
  static std::default_random_engine generator(seed);
  static std::uniform_real_distribution<float> distribution(0, 1);
  float distance = 0;
  do {
    // get two points in and around utrecht
    start = PointLL(5.0819f + .053f * distribution(generator),
                    52.0698f + .0334f * distribution(generator));
    end = PointLL(5.0819f + .053f * distribution(generator),
                  52.0698f + .0334f * distribution(generator));
    distance = start.Distance(end);
    // try again if they are too close or too far apart
  } while (distance < 1000 || distance > 2000);
  return R"({"costing":"auto","locations":[{"lat":)" + std::to_string(start.second) + R"(,"lon":)" +
         std::to_string(start.first) +
         R"(},{"lat":)" + std::to_string(end.second) + R"(,"lon":)" + std::to_string(end.first) +
         "}]}";
}

void test_matcher() {
  // generate a bunch of tests
  tyr::actor_t actor(conf, true);
  int tested = 0;
  while (tested < bound) {
    // get a route shape
    PointLL start, end;
    auto test_case = make_test_case(start, end);
    std::cout << test_case << std::endl;
    boost::property_tree::ptree route;
    try {
      route = json_to_pt(actor.route(test_case));
    } catch (...) {
      std::cout << "route failed" << std::endl;
      continue;
    }
    auto encoded_shape = route.get_child("trip.legs").front().second.get<std::string>("shape");
    auto shape = midgard::decode<std::vector<midgard::PointLL>>(encoded_shape);
    // skip any routes that have loops in them as edge walk fails in that case...
    // TODO: fix edge walk
    std::unordered_set<std::string> names;
    bool looped = false;
    const auto& maneuvers = route.get_child("trip.legs").front().second.get_child("maneuvers");
    for (const auto& maneuver : maneuvers) {
      if (maneuver.second.find("street_names") == maneuver.second.not_found())
        continue;
      for (const auto& name : maneuver.second.get_child("street_names"))
        looped = looped || !names.insert(name.second.get_value<std::string>()).second;
    }
    // get the edges along that route shape
    boost::property_tree::ptree walked;
    try {
      walked = json_to_pt(actor.trace_attributes(
          R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" +
          json_escape(encoded_shape) + "\"}"));
    } catch (...) {
      std::cout << test_case << std::endl;
      std::cout << R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" +
                       json_escape(encoded_shape) + "\"}"
                << std::endl;
      throw std::logic_error("Edge walk failed with exact shape");
    }
    std::vector<uint64_t> walked_edges;
    std::vector<gps_segment_t> segments;
    for (const auto& edge : walked.get_child("edges")) {
      walked_edges.push_back(edge.second.get<uint64_t>("id"));
      auto b = edge.second.get<size_t>("begin_shape_index");
      auto e = edge.second.get<size_t>("end_shape_index") + 1;
      segments.emplace_back(
          gps_segment_t{std::vector<PointLL>(shape.cbegin() + b, shape.cbegin() + e),
                        static_cast<float>(edge.second.get<float>("speed") * 1e3) / 3600.f});
    }
    // simulate gps from the route shape
    std::vector<float> accuracies;
    auto simulation = simulate_gps(segments, accuracies, 50, 100.f, 1);
    auto locations = to_locations(simulation, accuracies, 1);
    // get a trace-attributes from the simulated gps
    auto matched = json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"map_snap","shape":)" + locations + "}"));
    std::vector<uint64_t> matched_edges;
    for (const auto& edge : matched.get_child("edges"))
      matched_edges.push_back(edge.second.get<uint64_t>("id"));
    // because of noise we can have off by 1 happen at the beginning or end so we trim to make sure
    auto walked_it = std::search(walked_edges.begin(), walked_edges.end(),
                                 matched_edges.begin() + 1, matched_edges.end() - 1);
    if (walked_it == walked_edges.end()) {
      if (looped) {
        std::cout << "route had a possible loop" << std::endl;
        continue;
      }
      auto decoded_match = midgard::decode<std::vector<PointLL>>(matched.get<std::string>("shape"));
      std::string geojson =
          R"({"type":"FeatureCollection","features":[{"geometry":{"type":"LineString","coordinates":[)";
      geojson += print(shape);
      geojson +=
          R"(]},"type":"Feature","properties":{"stroke":"#00ff00","stroke-width":2}},{"geometry":{"type":"LineString","coordinates":[)";
      geojson += print(simulation);
      geojson +=
          R"(]},"type":"Feature","properties":{"stroke":"#0000ff","stroke-width":2}},{"geometry":{"type":"LineString","coordinates":[)";
      geojson += print(decoded_match);
      geojson +=
          R"(]},"type":"Feature","properties":{"stroke":"#ff0000","stroke-width":2}},{"geometry":{"type":"MultiPoint","coordinates":[)";
      geojson += print(std::vector<PointLL>{start, end});
      geojson += R"(]},"type":"Feature","properties":{}}]})";
      std::cout << geojson << std::endl;
      throw std::logic_error("The match did not match the walk");
    }
    std::cout << "Iteration " << tested << " complete" << std::endl;
    ++tested;
  }
}

void test_distance_only() {
  tyr::actor_t actor(conf, true);
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"max_route_distance_factor":10,"max_route_time_factor":1,"turn_penalty_factor":0},
          "costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"accuracy":10},
          {"lat":52.09050,"lon":5.09769,"accuracy":100},
          {"lat":52.09098,"lon":5.09679,"accuracy":10}]})"));
  std::unordered_set<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.insert(name.second.get_value<std::string>());
  if (names.find("Jan Pieterszoon Coenstraat") == names.end())
    throw std::logic_error("Using distance only it should have taken a small detour");
}

void test_time_rejection() {
  tyr::actor_t actor(conf, true);
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"max_route_distance_factor":10,"max_route_time_factor":3,"turn_penalty_factor":0},
          "costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"accuracy":10,"time":2},
          {"lat":52.09050,"lon":5.09769,"accuracy":100,"time":4},
          {"lat":52.09098,"lon":5.09679,"accuracy":10,"time":6}]})"));
  std::unordered_set<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.insert(name.second.get_value<std::string>());
  if (names.find("Jan Pieterszoon Coenstraat") != names.end()) {
    throw std::logic_error("Using time it should not take a small detour");
  }
}

void test32bit() {
  tyr::actor_t actor(conf, true);
  std::string test_case = "{\"costing\":\"auto\",\"locations\":[{\"lat\":52.096672,\"lon\":5."
                          "110825},{\"lat\":52.081371,\"lon\":5.125671}]}";
  actor.route(test_case);
}

void test_topk_validate() {
  // tests a fork in the road
  tyr::actor_t actor(conf, true);

  // tests a previous segfault due to using a claimed state
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
         {"lat":52.088548,"lon":5.15357,"accuracy":30},
         {"lat":52.088627,"lon":5.153269,"accuracy":30},
         {"lat":52.08864,"lon":5.15298,"accuracy":30},
         {"lat":52.08861,"lon":5.15272,"accuracy":30},
         {"lat":52.08863,"lon":5.15253,"accuracy":30},
         {"lat":52.08851,"lon":5.15249,"accuracy":30}]})"));

  // this tests a fix for an infinite loop because there is only 1 result and we ask for 4
  matched = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":4,"shape_match":"map_snap","shape":[
         {"lat":52.09579,"lon":5.13137,"accuracy":5},
         {"lat":52.09652,"lon":5.13184,"accuracy":5}]})"));
  if (matched.get_child("alternate_paths").size() > 0)
    throw std::logic_error("There should be only one result");
}

void test_topk_fork_alternate() {
  // tests a fork in the road
  tyr::actor_t actor(conf, true);
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"search_radius":0},"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
          {"lat":52.08511,"lon":5.15085,"accuracy":10},
          {"lat":52.08533,"lon":5.15109,"accuracy":20},
          {"lat":52.08539,"lon":5.15100,"accuracy":20}]})"));

  /*** Primary path - left at the fork
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.150850,52.085110]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151090,52.085331]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151000,52.085388]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.150851,52.085110]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.295,"distance_from_trace_point":0.097}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151000,52.085323]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":1,"distance_along_edge":0.152,"distance_from_trace_point":6.149}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.150990,52.085388]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":1,"distance_along_edge":0.296,"distance_from_trace_point":0.713}}
    ]}
   */
  std::vector<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.push_back(name.second.get_value<std::string>());
  if (names != std::vector<std::string>{"Louis Saalbornlaan", "Cor Ruyslaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error("The most obvious result is stay left but got: " + streets);
  }
  if (matched.get<float>("confidence_score") != 1.0f)
    throw std::logic_error("Confidence of the first result is always 1");

  /*** Alternate path - right at the fork
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.150850,52.085110]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151090,52.085331]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151000,52.085388]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.150851,52.085110]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.295,"distance_from_trace_point":0.097}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151095,52.085327]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":1,"distance_along_edge":0.254,"distance_from_trace_point":0.532}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151106,52.085339]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":1,"distance_along_edge":0.293,"distance_from_trace_point":9.044}}
    ]}
   */
  names.clear();
  auto alternate = matched.get_child("alternate_paths").front().second;
  for (const auto& edge : alternate.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.push_back(name.second.get_value<std::string>());
  if (names != std::vector<std::string>{"Louis Saalbornlaan", "Louis Saalbornlaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error("The second most obvious result is stay right but got: " + streets);
  }
  if (alternate.get<float>("confidence_score") >= 1.0f)
    throw std::logic_error("Confidence of the second result is always less than 1");
  if (matched.get<float>("raw_score") >= alternate.get<float>("raw_score"))
    throw std::logic_error(
        "The raw score of the first result is always less than that of the second");
}

void test_topk_loop_alternate() {
  // tests a loop in the road
  tyr::actor_t actor(conf, true);
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
           {"lat":52.0886,"lon":5.1535,"accuracy":10},
           {"lat":52.088619,"lon":5.15315,"accuracy":20},
           {"lat":52.08855,"lon":5.152652,"accuracy":25},
           {"lat":52.0883,"lon":5.152183,"accuracy":20},
           {"lat":52.088062,"lon":5.151963,"accuracy":20}]})"));

  /*** Primary path - stay left on the same road
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153500,52.088600]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153150,52.088619]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152652,52.088551]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152183,52.088299]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":3}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151963,52.088062]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":4}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153483,52.088573]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.698,"distance_from_trace_point":3.174}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153239,52.088531]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":2,"distance_along_edge":0.049,"distance_from_trace_point":11.437}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152822,52.088379]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":4,"distance_along_edge":0.136,"distance_from_trace_point":22.209}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152305,52.088184]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":3,"matched_point_type":"matched","edge_index":5,"distance_along_edge":0.118,"distance_from_trace_point":15.111}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151974,52.088051]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":4,"matched_point_type":"matched","edge_index":5,"distance_along_edge":0.801,"distance_from_trace_point":1.468}}
    ]}
   */
  std::vector<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.push_back(name.second.get_value<std::string>());
  if (names != std::vector<std::string>{"Louis Bouwmeesterlaan", "Louis Bouwmeesterlaan",
                                        "Louis Bouwmeesterlaan", "Louis Bouwmeesterlaan",
                                        "Louis Bouwmeesterlaan", "Louis Bouwmeesterlaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error("The most obvious result is stay left on the same road - but got: " +
                           streets);
  }
  if (matched.get<float>("confidence_score") != 1.0f)
    throw std::logic_error("Confidence of the first result is always 1");

  /*** Alternate path - loop around to the right
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153500,52.088600]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153150,52.088619]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152652,52.088551]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152183,52.088299]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":3}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151963,52.088062]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":4}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153483,52.088573]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.698,"distance_from_trace_point":3.174}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.153173,52.088627]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":2,"distance_along_edge":0.308,"distance_from_trace_point":1.769}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152542,52.088661]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":4,"distance_along_edge":0.183,"distance_from_trace_point":14.339}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.152305,52.088184]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":3,"matched_point_type":"matched","edge_index":6,"distance_along_edge":0.118,"distance_from_trace_point":15.111}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.151974,52.088051]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":4,"matched_point_type":"matched","edge_index":6,"distance_along_edge":0.801,"distance_from_trace_point":1.468}}
    ]}
   */
  names.clear();
  auto alternate = matched.get_child("alternate_paths").front().second;
  for (const auto& edge : alternate.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.push_back(name.second.get_value<std::string>());
  if (names != std::vector<std::string>{"Louis Bouwmeesterlaan", "Louis Bouwmeesterlaan",
                                        "Eduard Verkadelaan", "Eduard Verkadelaan",
                                        "Eduard Verkadelaan", "Eduard Verkadelaan",
                                        "Louis Bouwmeesterlaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error(
        "The second most obvious result is loop around to the right - but got: " + streets);
  }
  if (alternate.get<float>("confidence_score") >= 1.0f)
    throw std::logic_error("Confidence of the second result is always less than 1");
  if (matched.get<float>("raw_score") >= alternate.get<float>("raw_score"))
    throw std::logic_error(
        "The raw score of the first result is always less than that of the second");
}

void test_topk_frontage_alternate() {
  // tests a parallel frontage road
  tyr::actor_t actor(conf, true);
  auto matched = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
           {"lat":52.07956040090567,"lon":5.138160288333893,"accuracy":10},
           {"lat":52.07957358807355,"lon":5.138508975505829,"accuracy":10},
           {"lat":52.07959666560798,"lon":5.138905942440034,"accuracy":10},
           {"lat":52.0796213915245,"lon":5.139262676239015,"accuracy":10},
           {"lat":52.079637875461195,"lon":5.139581859111787,"accuracy":10},
           {"lat":52.07964776582031,"lon":5.139828622341157,"accuracy":10},
           {"lat":52.07965600778458,"lon":5.1402121782302865,"accuracy":10}]})"));

  /*** Primary path - use main road
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138160,52.079559]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138509,52.079575]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138906,52.079597]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139263,52.079620]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":3}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139582,52.079639]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":4}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139829,52.079647]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":5}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.140212,52.079655]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":6}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138153,52.079605]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.254,"distance_from_trace_point":5.085}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138501,52.079624]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.385,"distance_from_trace_point":5.511}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138898,52.079647]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.534,"distance_from_trace_point":5.511}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139255,52.079670]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":3,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.668,"distance_from_trace_point":5.511}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139574,52.079689]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":4,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.788,"distance_from_trace_point":5.511}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139820,52.079704]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":5,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.881,"distance_from_trace_point":6.357}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.140204,52.079727]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":6,"matched_point_type":"matched","edge_index":1,"distance_along_edge":0.070,"distance_from_trace_point":8.033}}
    ]}
   */
  std::vector<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.push_back(name.second.get_value<std::string>());
  if (names != std::vector<std::string>{"Rubenslaan", "Rubenslaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error("The most obvious result is stay straight on the same road - but got: " +
                           streets);
  }
  if (matched.get<float>("confidence_score") != 1.0f)
    throw std::logic_error("Confidence of the first result is always 1");

  /*** Alternate path - use one way frontage road
    {"type":"FeatureCollection","features":[
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138160,52.079559]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138509,52.079575]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138906,52.079597]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139263,52.079620]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":3}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139582,52.079639]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":4}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139829,52.079647]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":5}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.140212,52.079655]},"properties":{"marker-color":"#abd9e9","marker-size":"small","trace_point_index":6}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138169,52.079498]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":0,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.223,"distance_from_trace_point":6.774}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138517,52.079517]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":1,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.369,"distance_from_trace_point":6.351}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.138914,52.079540]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":2,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.536,"distance_from_trace_point":6.351}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139271,52.079559]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":3,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.686,"distance_from_trace_point":6.774}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139591,52.079575]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":4,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.821,"distance_from_trace_point":7.197}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.139837,52.079586]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":5,"matched_point_type":"matched","edge_index":0,"distance_along_edge":0.924,"distance_from_trace_point":6.771}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.140018,52.079597]},"properties":{"marker-color":"#2c7bb6","marker-size":"medium","matched_point_index":6,"matched_point_type":"matched","edge_index":0,"distance_along_edge":1.000,"distance_from_trace_point":14.626}}
    ]}
   */
  names.clear();
  auto alternate = matched.get_child("alternate_paths").front().second;
  for (const auto& edge : alternate.get_child("edges")) {
    const auto json_names = edge.second.get_child_optional("names");
    if (json_names) {
      for (const auto& name : json_names.get())
        names.push_back(name.second.get_value<std::string>());
    } else {
      names.push_back("<empty>");
    }
  }
  if (names != std::vector<std::string>{"Rubenslaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    throw std::logic_error(
        "The second most obvious result is fronatge road to the right - but got: " + streets);
  }
  if (alternate.get<float>("confidence_score") >= 1.0f)
    throw std::logic_error("Confidence of the second result is always less than 1");
  if (matched.get<float>("raw_score") >= alternate.get<float>("raw_score"))
    throw std::logic_error(
        "The raw score of the first result is always less than that of the second");
}

} // namespace

int main(int argc, char* argv[]) {
  test::suite suite("map matcher");
  midgard::logging::Configure({{"type", ""}}); // silence logs
  if (argc > 1)
    seed = std::stoi(argv[1]);
  if (argc > 2)
    bound = std::stoi(argv[2]);

  suite.test(TEST_CASE(test32bit));

  suite.test(TEST_CASE(test_matcher));

  suite.test(TEST_CASE(test_distance_only));

  suite.test(TEST_CASE(test_time_rejection));

  suite.test(TEST_CASE(test_topk_validate));

  suite.test(TEST_CASE(test_topk_fork_alternate));

  suite.test(TEST_CASE(test_topk_loop_alternate));

  suite.test(TEST_CASE(test_topk_frontage_alternate));

  return suite.tear_down();
}
