#include "test.h"

#include <vector>
#include <random>
#include <utility>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "meili/map_matcher_factory.h"
#include "meili/map_matcher.h"
#include "tyr/actor.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "midgard/logging.h"
#include "baldr/json.h"
#include "midgard/distanceapproximator.h"
#include "mjolnir/util.h"

using namespace valhalla;

namespace std {
  std::string to_string(const midgard::PointLL& p) {
    return "[" + to_string(p.first) + "," + to_string(p.second) + "]";
  }
}

namespace {

  template <class container_t>
  std::string print(const container_t& container) {
    std::string output;
    for(const auto& e : container)
      output += std::to_string(e) + ",";
    if(container.size())
      output.pop_back();
    return output;
  }

  float round_up(float val, int multiple) {
    return int((val + multiple - 1) / multiple) * multiple;
  }

  std::string to_locations(const std::vector<PointLL>& shape, const std::vector<float>& accuracies, int frequency) {
    std::string locations = "[";
    int freq = 0;
    for(size_t i = 0; i < shape.size(); ++i) {
      //round accuracy up to nearest 5m
      int accuracy = round_up(accuracies[i] + 1.f, 5);
      std::string acc = R"(,"accuracy":)" + std::to_string(accuracy);
      //add this point on
      const auto& p = shape[i];
      locations += R"({"lat":)" + std::to_string(p.second) + R"(,"lon":)" + std::to_string(p.first) + acc;
      //get the time component
      freq += frequency;
      if(freq > 0)
        locations += R"(,"time":)" + std::to_string(freq);
      locations += "},";
    }
    locations.back() = ']';
    return locations;
  }

  boost::property_tree::ptree json_to_pt(const std::string& json) {
    std::stringstream ss; ss << json;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(ss, pt);
    return pt;
  }

  //fake config
  const auto conf = json_to_pt(R"({
    "mjolnir":{"tile_dir":"test/data/utrecht_tiles", "concurrency": 1},
    "loki":{
      "actions":["locate","route","one_to_many","many_to_one","many_to_many","sources_to_targets","optimized_route","isochrone","trace_route","trace_attributes"],
      "logging":{"long_request": 100},
      "service_defaults":{"minimum_reachability": 50,"radius": 0}
    },
    "thor":{"logging":{"long_request": 110}},
    "skadi":{"actons":["height"],"logging":{"long_request": 5}},
    "meili":{"customizable": ["turn_penalty_factor","max_route_distance_factor","max_route_time_factor"],
             "mode":"auto","grid":{"cache_size":100240,"size":500},
             "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
             "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":200,"route":true,
             "search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}},
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

  int seed = 973; int bound = 81;
  std::string make_test_case(PointLL& start, PointLL& end) {
    static std::default_random_engine generator(seed);
    static std::uniform_real_distribution<float> distribution(0, 1);
    float distance = 0;
    do {
      //get two points in and around utrecht
      start = PointLL(5.0819f + .053f * distribution(generator), 52.0698f + .0334f * distribution(generator));
      end = PointLL(5.0819f + .053f * distribution(generator), 52.0698f + .0334f * distribution(generator));
      distance = start.Distance(end);
      //try again if they are too close or too far apart
    }while(distance < 1000 || distance > 2000);
    return R"({"costing":"auto","locations":[{"lat":)" + std::to_string(start.second) + R"(,"lon":)" + std::to_string(start.first) +
      R"(},{"lat":)" + std::to_string(end.second) + R"(,"lon":)" + std::to_string(end.first) + "}]}";
  }

  void test_matcher() {
    //generate a bunch of tests
    tyr::actor_t actor(conf, true);
    int tested = 0;
    while(tested < bound) {
      //get a route shape
      PointLL start, end;
      auto test_case = make_test_case(start, end);
      std::cout << test_case << std::endl;
      boost::property_tree::ptree route;
      try { route = json_to_pt(actor.route(tyr::ROUTE, test_case)); }
      catch (...) { std::cout << "route failed" << std::endl; continue; }
      auto encoded_shape = route.get_child("trip.legs").front().second.get<std::string>("shape");
      auto shape = midgard::decode<std::vector<midgard::PointLL> >(encoded_shape);
      //skip any routes that have loops in them as edge walk fails in that case...
      //TODO: fix edge walk
      std::unordered_set<std::string> names;
      bool looped = false;
      const auto& maneuvers = route.get_child("trip.legs").front().second.get_child("maneuvers");
      for(const auto& maneuver : maneuvers) {
        if(maneuver.second.find("street_names") == maneuver.second.not_found())
          continue;
        for(const auto& name : maneuver.second.get_child("street_names"))
          looped = looped || !names.insert(name.second.get_value<std::string>()).second;
      }
      //get the edges along that route shape
      boost::property_tree::ptree walked;
      try {
        walked = json_to_pt(actor.trace_attributes(
          R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" + json_escape(encoded_shape) + "\"}"));
      } catch (...) {
        std::cout << test_case << std::endl;
        std::cout << R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" + json_escape(encoded_shape) + "\"}" << std::endl;
        throw std::logic_error("Edge walk failed with exact shape");
      }
      std::vector<uint64_t> walked_edges;
      for(const auto& edge : walked.get_child("edges"))
        walked_edges.push_back(edge.second.get<uint64_t>("id"));
      //simulate gps from the route shape
      std::vector<float> accuracies;
      auto simulation = midgard::simulate_gps(walked.get_child("edges"), shape, accuracies, 50, 100.f, 1);
      auto locations = to_locations(simulation, accuracies, 1);
      //get a trace-attributes from the simulated gps
      auto matched = json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"map_snap","shape":)" + locations + "}"));
      std::vector<uint64_t> matched_edges;
      for(const auto& edge : matched.get_child("edges"))
        matched_edges.push_back(edge.second.get<uint64_t>("id"));
      //because of noise we can have off by 1 happen at the beginning or end so we trim to make sure
      auto walked_it = std::search(walked_edges.begin(), walked_edges.end(), matched_edges.begin() + 1, matched_edges.end() - 1);
      if(walked_it == walked_edges.end()) {
        if(looped) {
          std::cout << "route had a possible loop" << std::endl;
          continue;
        }
        auto decoded_match = midgard::decode<std::vector<midgard::PointLL> >(matched.get<std::string>("shape"));
        std::string geojson = R"({"type":"FeatureCollection","features":[{"geometry":{"type":"LineString","coordinates":[)";
        geojson += print(shape);
        geojson += R"(]},"type":"Feature","properties":{"stroke":"#00ff00","stroke-width":2}},{"geometry":{"type":"LineString","coordinates":[)";
        geojson += print(simulation);
        geojson += R"(]},"type":"Feature","properties":{"stroke":"#0000ff","stroke-width":2}},{"geometry":{"type":"LineString","coordinates":[)";
        geojson += print(decoded_match);
        geojson += R"(]},"type":"Feature","properties":{"stroke":"#ff0000","stroke-width":2}},{"geometry":{"type":"MultiPoint","coordinates":[)";
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
    for(const auto& edge : matched.get_child("edges"))
      for(const auto& name : edge.second.get_child("names"))
        names.insert(name.second.get_value<std::string>());
    if(names.find("Jan Pieterszoon Coenstraat") == names.end())
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
    for(const auto& edge : matched.get_child("edges"))
      for(const auto& name : edge.second.get_child("names"))
        names.insert(name.second.get_value<std::string>());
    if(names.find("Jan Pieterszoon Coenstraat") != names.end()) {
      throw std::logic_error("Using time it should not take a small detour");
    }
  }

  void test32bit() {
    tyr::actor_t actor(conf, true);
    std::string test_case = "{\"costing\":\"auto\",\"locations\":[{\"lat\":52.096672,\"lon\":5.110825},{\"lat\":52.081371,\"lon\":5.125671}]}";
    actor.route(tyr::ROUTE, test_case);
  }

  void test_topk() {
    tyr::actor_t actor(conf, true);
    auto matched = json_to_pt(actor.trace_attributes(
            R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
                {"lat":52.08511,"lon":5.15085,"accuracy":50},
                {"lat":52.08533,"lon":5.15109,"accuracy":50},
                {"lat":52.08539,"lon":5.15100,"accuracy":50}]})"));

    std::vector<std::string> names;
    for(const auto& edge : matched.get_child("edges"))
      for(const auto& name : edge.second.get_child("names"))
        names.push_back(name.second.get_value<std::string>());
    if(names != std::vector<std::string>{"Louis Saalbornlaan", "Cor Ruyslaan"}) {
      std::string streets;
      for(const auto& n : names)
        streets += n + " ";
      throw std::logic_error("The most obvious result is stay left but got: " + streets);
    }

    names.clear();
    auto alternate = matched.get_child("alternate_paths").front().second;
    for(const auto& edge : alternate.get_child("edges"))
      for(const auto& name : edge.second.get_child("names"))
        names.push_back(name.second.get_value<std::string>());
    if(names != std::vector<std::string>{"Louis Saalbornlaan", "Louis Saalbornlaan"}) {
      std::string streets;
      for(const auto& n : names)
        streets += n + " ";
      throw std::logic_error("The second most obvious result is stay right but got: " + streets);
    }

    matched = json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
         {"lat":52.088548,"lon":5.15357,"accuracy":30},
         {"lat":52.088627,"lon":5.153269,"accuracy":30},
         {"lat":52.08864,"lon":5.15298,"accuracy":30},
         {"lat":52.08861,"lon":5.15272,"accuracy":30},
         {"lat":52.08863,"lon":5.15253,"accuracy":30},
         {"lat":52.08851,"lon":5.15249,"accuracy":30}]})"));
  }

}

int main(int argc, char* argv[]) {
  test::suite suite("map matcher");
  midgard::logging::Configure({{"type", ""}}); //silence logs
  if(argc > 1)
    seed = std::stoi(argv[1]);
  if(argc > 2)
    bound = std::stoi(argv[2]);

  suite.test(TEST_CASE(test32bit));

  suite.test(TEST_CASE(test_matcher));

  suite.test(TEST_CASE(test_distance_only));

  suite.test(TEST_CASE(test_time_rejection));

  suite.test(TEST_CASE(test_topk));

  return suite.tear_down();
}
