#include <algorithm>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include "baldr/json.h"
#include "loki/worker.h"
#include "midgard/distanceapproximator.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "worker.h"

#include "test.h"

using namespace valhalla;
using namespace valhalla::midgard;

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
                         int frequency = 0,
                         int start_time = 8 * 60 * 60) {
  std::string locations = "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    // round accuracy up to nearest 5m
    int accuracy = round_up(accuracies[i] + 1.f, 5);
    std::string acc = R"(,"accuracy":)" + std::to_string(accuracy);
    // add this point on
    const auto& p = shape[i];
    locations +=
        R"({"lat":)" + std::to_string(p.second) + R"(,"lon":)" + std::to_string(p.first) + acc;
    // get the time component
    start_time += frequency;
    if (frequency > 0)
      locations += R"(,"time":)" + std::to_string(start_time);
    locations += "},";
  }
  locations.back() = ']';
  return locations;
}

// fake config
const auto conf = test::make_config("test/data/utrecht_tiles",
                                    {
                                        {"meili.default.max_search_radius", "200"},
                                        {"meili.default.search_radius", "15.0"},
                                        {"meili.default.turn_penalty_factor", "200"},
                                    });

struct api_tester {
  api_tester()
      : conf_(conf), reader(new valhalla::baldr::GraphReader(conf.get_child("mjolnir"))),
        loki_worker(conf, reader), thor_worker(conf, reader), odin_worker(conf) {
  }
  Api match(const std::string& request_json) {
    Api request;
    ParseApi(request_json, Options::trace_route, request);
    loki_worker.trace(request);
    thor_worker.trace_route(request);
    odin_worker.narrate(request);
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  Api route(const std::string& request_json) {
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
  boost::property_tree::ptree conf_;
  std::shared_ptr<valhalla::baldr::GraphReader> reader;
  valhalla::loki::loki_worker_t loki_worker;
  valhalla::thor::thor_worker_t thor_worker;
  valhalla::odin::odin_worker_t odin_worker;
};

std::string json_escape(const std::string& unescaped) {
  std::stringstream ss;
  baldr::json::OstreamVisitor v(ss);
  v(unescaped);
  std::string escaped = ss.str().substr(1);
  escaped.pop_back();
  return escaped;
}

std::string output_shape(const valhalla::Api& api) {
  std::stringstream shape;
  for (const auto& r : api.directions().routes()) {
    shape << "new route" << std::endl;
    for (const auto& l : r.legs()) {
      shape << std::fixed << std::setprecision(3) << "Time : " << l.summary().time()
            << ", length : " << l.summary().length() << ", shape : " << l.shape() << std::endl;
    }
  }
  return shape.str();
}

void compare_results(const valhalla::Api& expected, const valhalla::Api& result) {
  // check the number of routes match
  ASSERT_EQ(result.trip().routes_size(), expected.trip().routes_size())
      << "Expected " << expected.trip().routes_size() << " routes but got "
      << expected.trip().routes_size();

  // loop over the routes and compare them
  auto route_answer = result.trip().routes().begin();
  for (int r = 0; r < expected.trip().routes_size(); ++r) {
    const auto& route = expected.trip().routes(r);
    ASSERT_EQ(route.legs_size(), route_answer->legs_size())
        << "Expected " << route.legs_size() << " legs but got " << route_answer->legs_size();

    // check each legs time and distance
    auto leg_answer = route_answer->legs().begin();
    for (int l = 0; l < route.legs_size(); ++l) {
      const auto& leg = route.legs(l);

      auto expected_seconds = leg.node().rbegin()->cost().elapsed_cost().seconds();
      auto answer_seconds = leg_answer->node().rbegin()->cost().elapsed_cost().seconds();
      ASSERT_NEAR(expected_seconds, answer_seconds, .1)
          << "Expected leg with elapsed time " << expected_seconds << " but got " << answer_seconds;

      auto expected_distance = expected.directions().routes(r).legs(l).summary().length();
      auto answer_distance = result.directions().routes(r).legs(l).summary().length();
      ASSERT_NEAR(expected_distance, answer_distance, .1)
          << "Expected leg with length " << expected_distance << " but got " << answer_distance;
      ++leg_answer;
    }

    // move to next result
    ++route_answer;
  }
}

int seed = 527;
int bound = 81;

std::string make_test_case(PointLL& start, PointLL& end) {
  static std::mt19937 generator(seed);
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
         std::to_string(start.first) + R"(},{"lat":)" + std::to_string(end.second) + R"(,"lon":)" +
         std::to_string(end.first) + "}]}";
}

TEST(Mapmatch, test_matcher) {
  // generate a bunch of tests
  tyr::actor_t actor(conf, true);
  int tested = 0;
  while (tested < bound) {
    // get a route shape
    PointLL start, end;
    auto test_case = make_test_case(start, end);
    boost::property_tree::ptree route;
    std::string route_json;
    try {
      route_json = actor.route(test_case);
      route = test::json_to_pt(route_json);
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
    std::string walked_json;

    try {
      Api api;
      walked_json = actor.trace_attributes(
          R"({"date_time":{"type":1,"value":"2019-10-31T18:30"},"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" +
              json_escape(encoded_shape) + "\"}",
          nullptr, &api);
      EXPECT_NE(api.trip().routes(0).legs(0).location(0).path_edges_size(), 0);
      EXPECT_NE(api.trip().routes(0).legs(0).location().rbegin()->path_edges_size(), 0);
      walked = test::json_to_pt(walked_json);
    } catch (...) {
      std::cout << test_case << std::endl;
      std::cout << R"({"costing":"auto","shape_match":"edge_walk","encoded_polyline":")" +
                       json_escape(encoded_shape) + "\"}"
                << std::endl;
      FAIL() << "Edge walk failed with exact shape";
    }

    // check the shape makes sense
    auto walked_encoded_shape = walked.get<std::string>("shape");
    auto walked_shape = midgard::decode<std::vector<midgard::PointLL>>(walked_encoded_shape);

    /*EXPECT_EQ(walked_shape.size() , shape.size())
      << "Differing shape lengths " + std::to_string(shape.size()) +
         " != " + std::to_string(walked_shape.size()) + "\n" + encoded_shape +
         "\n" + walked_encoded_shape;*/

    // build up some gps segments for simulation from the real shape
    std::vector<uint64_t> walked_edges;
    std::vector<gps_segment_t> segments;
    for (const auto& edge : walked.get_child("edges")) {
      walked_edges.push_back(edge.second.get<uint64_t>("id"));
      auto b = edge.second.get<size_t>("begin_shape_index");
      auto e = edge.second.get<size_t>("end_shape_index") + 1;

      segments.emplace_back(
          gps_segment_t{std::vector<PointLL>(walked_shape.cbegin() + b, walked_shape.cbegin() + e),
                        static_cast<float>(edge.second.get<float>("speed") * 1e3) / 3600.f});
    }

    // simulate gps from the route shape
    std::vector<float> accuracies;
    auto simulation = simulate_gps(segments, accuracies, 50, 75.f, 1);
    auto locations = to_locations(simulation, accuracies, 1);
    // get a trace-attributes from the simulated gps
    auto matched = test::json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"map_snap","shape":)" + locations + "}"));
    std::vector<uint64_t> matched_edges;
    for (const auto& edge : matched.get_child("edges"))
      matched_edges.push_back(edge.second.get<uint64_t>("id"));

    // because of noise we can have off by 1 happen at the beginning or end so we trim to make sure
    auto walked_it = std::search(walked_edges.begin(), walked_edges.end(), matched_edges.begin() + 1,
                                 matched_edges.end() - 1);
    if (walked_it == walked_edges.end()) {
      if (looped) {
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
      FAIL() << "The match did not match the walk";
    }

    for (const auto& point : matched.get_child("matched_points")) {
      auto const edge_index = point.second.get<uint64_t>("edge_index");
      auto const match_type = point.second.get<std::string>("type");

      if (match_type == "matched" && edge_index == meili::kInvalidEdgeIndex)
        FAIL() << "Bad edge index for matched point.";
      if (match_type == "unmatched" && edge_index != meili::kInvalidEdgeIndex)
        FAIL() << "Bad edge index for ummatched point.";
    }
    ++tested;
  }
}

TEST(Mapmatch, test_distance_only) {
  tyr::actor_t actor(conf, true);
  auto matched = test::json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"max_route_distance_factor":10,"max_route_time_factor":1,"turn_penalty_factor":0},
          "costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"accuracy":10},
          {"lat":52.09050,"lon":5.09769,"accuracy":100},
          {"lat":52.09098,"lon":5.09679,"accuracy":10}]})"));
  std::unordered_set<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.insert(name.second.get_value<std::string>());
  EXPECT_NE(names.find("Jan Pieterszoon Coenstraat"), names.end())
      << "Using distance only it should have taken a small detour";
}

TEST(Mapmatch, test_matched_points) {
  tyr::actor_t actor(conf, true);
  auto matched = test::json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"max_route_distance_factor":10,"max_route_time_factor":1,"turn_penalty_factor":0},
          "costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"accuracy":10},
          {"lat":52.09050,"lon":5.09769,"accuracy":100},
          {"lat":52.09098,"lon":5.09679,"accuracy":10}]})"));

  auto const edges_number = matched.get_child("edges").size();
  std::vector<size_t> edge_indexes;
  for (const auto& point : matched.get_child("matched_points")) {
    auto const match_type = point.second.get<std::string>("type");
    auto const edge_index = point.second.get<uint64_t>("edge_index");
    edge_indexes.push_back(edge_index);

    if (match_type == "matched" && edge_index == meili::kInvalidEdgeIndex)
      FAIL() << "Bad edge index for matched point.";
    if (match_type == "unmatched" && edge_index != meili::kInvalidEdgeIndex)
      FAIL() << "Bad edge index for ummatched point.";
  }
  EXPECT_EQ(edge_indexes[0], 0);
  EXPECT_EQ(edge_indexes.back(), edges_number - 1);
}

TEST(Mapmatch, test_trace_route_breaks) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"type":"break"},
          {"lat":52.09050,"lon":5.09769,"type":"break"},
          {"lat":52.09098,"lon":5.09679,"type":"break"}]})",
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"type":"break"},
          {"lat":52.09050,"lon":5.09769,"type":"via"},
          {"lat":52.09098,"lon":5.09679,"type":"break"}]})",
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806},
          {"lat":52.09050,"lon":5.09769},
          {"lat":52.09098,"lon":5.09679}]})",
      R"({"costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.091100,"lon":5.098060,"type":"break","radius":5},
          {"lat":52.091100,"lon":5.097290,"type":"break","radius":5},
          {"lat":52.090993,"lon":5.096991,"type":"break","radius":5},
          {"lat":52.090970,"lon":5.096771,"type":"break","radius":5}]})",
      R"({"costing":"auto","shape_match":"map_snap","encoded_polyline":"quijbBqpnwHfJxc@bBdJrDfSdAzFX|AHd@bG~[|AnIdArGbAo@z@m@`EuClO}MjE}E~NkPaAuC"})"};
  std::vector<size_t> test_answers = {2, 1, 1, 1, 1};

  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& legs = matched.get_child("trip.legs");
    EXPECT_EQ(legs.size(), test_answers[i]);
  }
}

TEST(Mapmatch, test_edges_discontinuity_with_multi_routes) {
  // here everything is a leg and the discontinuities are the routes
  // we have to use osrm format because valhalla format doesnt support multi route
  std::vector<std::string> test_cases = {
      R"({"date_time":{"type":1,"value":"2019-11-10T09:00"},
                 "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                 {"lat":52.0609632,"lon":5.0917676,"type":"break"},
                 {"lat":52.0607180,"lon":5.0950566,"type":"break"},
                 {"lat":52.0797372,"lon":5.1293068,"type":"break"},
                 {"lat":52.0792731,"lon":5.1343818,"type":"break"},
                 {"lat":52.0763011,"lon":5.1574637,"type":"break"},
                 {"lat":52.0782167,"lon":5.1592370,"type":"break"}]})",
      R"({"date_time":{"type":0},
                  "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat":52.0609632,"lon":5.0917676,"type":"break"},
                  {"lat":52.0607180,"lon":5.0950566,"type":"via"},
                  {"lat":52.0797372,"lon":5.1293068,"type":"via"},
                  {"lat":52.0792731,"lon":5.1343818,"type":"via"},
                  {"lat":52.0763011,"lon":5.1574637,"type":"via"},
                  {"lat":52.0782167,"lon":5.1592370,"type":"break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat":52.0609632,"lon":5.0917676,"type":"break","time":7},
                  {"lat":52.0607185,"lon":5.0940566,"type":"break_through","time":11},
                  {"lat":52.0607180,"lon":5.0950566,"type":"break_through","time":15},
                  {"lat":52.0797372,"lon":5.1293068,"type":"break_through","time":19},
                  {"lat":52.0792731,"lon":5.1343818,"type":"break_through","time":23},
                  {"lat":52.0763011,"lon":5.1574637,"type":"break_through","time":27},
                  {"lat":52.0782167,"lon":5.1592370,"type":"break","time":13}]})",
      R"({"date_time":{"type":2,"value":"2019-11-10T09:00"},
          "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat":52.0609632,"lon":5.0917676,"type":"break"},
                  {"lat":52.0607185,"lon":5.0940566,"type":"via"},
                  {"lat":52.0607180,"lon":5.0950566,"type":"via"},
                  {"lat":52.0797372,"lon":5.1293068,"type":"via"},
                  {"lat":52.0792731,"lon":5.1343818,"type":"via"},
                  {"lat":52.0763011,"lon":5.1574637,"type":"via"},
                  {"lat":52.0782167,"lon":5.1592370,"type":"break"}]})",
      R"({"date_time":{"type":1,"value":"2019-11-10T09:00"},
          "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat": 52.068882, "lon": 5.120852, "type": "break"},
                  {"lat": 52.069671, "lon": 5.121185, "type": "break"},
                  {"lat": 52.070380, "lon": 5.121523, "type": "break"},
                  {"lat": 52.070947, "lon": 5.121828, "type": "break"},
                  {"lat": 52.071827, "lon": 5.122220, "type": "break"},
                  {"lat": 52.072526, "lon": 5.122553, "type": "break"},
                  {"lat": 52.073489, "lon": 5.122880, "type": "break"},
                  {"lat": 52.074554, "lon": 5.122955, "type": "break"},
                  {"lat": 52.075190, "lon": 5.123067, "type": "break"},
                  {"lat": 52.075718, "lon": 5.123121, "type": "break"}]})",
      R"({"date_time":{"type":0},
        "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat": 52.068882, "lon": 5.120852, "type": "break"},
                  {"lat": 52.069671, "lon": 5.121185, "type": "through"},
                  {"lat": 52.070380, "lon": 5.121523, "type": "through"},
                  {"lat": 52.070947, "lon": 5.121828, "type": "through"},
                  {"lat": 52.071827, "lon": 5.122220, "type": "through"},
                  {"lat": 52.072526, "lon": 5.122553, "type": "through"},
                  {"lat": 52.073489, "lon": 5.122880, "type": "through"},
                  {"lat": 52.074554, "lon": 5.122955, "type": "through"},
                  {"lat": 52.075190, "lon": 5.123067, "type": "through"},
                  {"lat": 52.075718, "lon": 5.123121, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat": 52.068882, "lon": 5.120852, "type": "break","time":7},
                  {"lat": 52.069671, "lon": 5.121185, "type": "break","time":9},
                  {"lat": 52.070380, "lon": 5.121523, "type": "break","time":11},
                  {"lat": 52.070947, "lon": 5.121828, "type": "break","time":13},
                  {"lat": 52.071827, "lon": 5.1227, "type": "break", "radius":1,"time":15},
                  {"lat": 52.072526, "lon": 5.122553, "type": "break","time":17},
                  {"lat": 52.073489, "lon": 5.122880, "type": "break","time":19},
                  {"lat": 52.074554, "lon": 5.122955, "type": "break","time":21},
                  {"lat": 52.075190, "lon": 5.123067, "type": "break","time":23},
                  {"lat": 52.075718, "lon": 5.123121, "type": "break","time":25}]})",
      R"({"date_time":{"type":2,"value":"2019-11-10T09:00"},
        "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
                  {"lat": 52.068882, "lon": 5.120852, "type": "break"},
                  {"lat": 52.069671, "lon": 5.121185, "type": "through"},
                  {"lat": 52.070380, "lon": 5.121523, "type": "through"},
                  {"lat": 52.070947, "lon": 5.121828, "type": "through"},
                  {"lat": 52.071827, "lon": 5.1227, "type": "through", "radius": 1},
                  {"lat": 52.072526, "lon": 5.122553, "type": "through"},
                  {"lat": 52.073489, "lon": 5.122880, "type": "through"},
                  {"lat": 52.074554, "lon": 5.122955, "type": "through"},
                  {"lat": 52.075190, "lon": 5.123067, "type": "through"},
                  {"lat": 52.075718, "lon": 5.123121, "type": "break"}]})"};

  using a_t = std::tuple<size_t, size_t, bool>;
  std::vector<a_t> test_answers = {a_t{3, 3, true},  a_t{3, 3, true}, a_t{2, 3, true},
                                   a_t{3, 3, false}, a_t{1, 9, true}, a_t{1, 1, true},
                                   a_t{2, 7, true},  a_t{2, 2, false}};
  // for type 2, we currently do not support them, thus we are not expecting any time stamp
  for (size_t i = 0; i < test_cases.size(); ++i) {
    api_tester tester;
    auto response = tester.match(test_cases[i]);
    EXPECT_EQ(response.trip().routes_size(), std::get<0>(test_answers[i]));

    size_t leg_count = 0;
    for (const auto& route : response.trip().routes()) {
      leg_count += route.legs_size();
      for (const auto& leg : route.legs()) {
        if (leg.location(0).has_date_time()) {
          EXPECT_TRUE(std::get<2>(test_answers[i]))
              << "Found a leg with a start time when it shouldnt have had one";
        } else {
          EXPECT_FALSE(std::get<2>(test_answers[i]))
              << "Found a leg without a start time when it should have had one";
        }
      }
    }
    EXPECT_EQ(leg_count, std::get<1>(test_answers[i]));
  }
}

TEST(Mapmatch, test_disconnected_edges_expect_no_route) {
  std::vector<std::string> test_cases = {R"({"costing":"auto","shape_match":"map_snap","shape":[
        {"lat":52.0630834,"lon":5.1037227,"type":"break"},
        {"lat":52.0633099,"lon":5.1047193,"type":"break"},
        {"lat":52.0640117,"lon":5.1040429,"type":"break"},
        {"lat":52.0644313,"lon":5.1041697,"type":"break"}]})"};
  std::vector<size_t> test_answers = {0};
  size_t illegal_path = 0;
  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    try {
      auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    } catch (...) { EXPECT_EQ(illegal_path++, i) << "Expected no route but got one"; }
  }
}

TEST(Mapmatch, test_matching_indices_and_waypoint_indices) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
        {"lat": 52.068882, "lon": 5.120852, "type": "break"},
        {"lat": 52.069671, "lon": 5.121185, "type": "via"},
        {"lat": 52.070380, "lon": 5.121523, "type": "via"},
        {"lat": 52.070947, "lon": 5.121828, "type": "via"},
        {"lat": 52.071827, "lon": 5.1227, "type": "via"},
        {"lat": 52.072526, "lon": 5.122553, "type": "via"},
        {"lat": 52.073489, "lon": 5.122880, "type": "via"},
        {"lat": 52.074554, "lon": 5.122955, "type": "via"},
        {"lat": 52.075190, "lon": 5.123067, "type": "via"},
        {"lat": 52.075718, "lon": 5.123121, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
        {"lat": 52.0609632, "lon": 5.0917676, "type": "break"},
        {"lat": 52.0607180, "lon": 5.0950566, "type": "break"},
        {"lat": 52.0797372, "lon": 5.1293068, "type": "break"},
        {"lat": 52.0792731, "lon": 5.1343818, "type": "break"},
        {"lat": 52.0763011, "lon": 5.1574637, "type": "break"},
        {"lat": 52.0782167, "lon": 5.1592370, "type": "via"},
        {"lat": 52.0801357, "lon": 5.1605372, "type": "break"}]})"};
  std::vector<std::vector<std::pair<std::string, std::string>>> answers{{{"0", "0"},
                                                                         {"0", "null"},
                                                                         {"0", "null"},
                                                                         {"0", "1"},
                                                                         {"1", "0"},
                                                                         {"1", "null"},
                                                                         {"1", "null"},
                                                                         {"1", "null"},
                                                                         {"1", "1"}},
                                                                        {
                                                                            {"0", "0"},
                                                                            {"0", "1"},
                                                                            {"1", "0"},
                                                                            {"1", "1"},
                                                                            {"2", "0"},
                                                                            {"2", "null"},
                                                                            {"2", "1"},
                                                                        }};

  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& tracepoints = matched.get_child("tracepoints");
    int j = 0;
    for (const auto& tracepoint : tracepoints) {
      std::pair<std::string, std::string> result;
      try {
        result = {tracepoint.second.get<std::string>("matchings_index"),
                  tracepoint.second.get<std::string>("waypoint_index")};
      } catch (...) {
        // handle the tracepoint null case
        continue;
      }
      EXPECT_EQ(result, answers[i][j]) << "expect matching_index and waypoint_index: (" +
                                              answers[i][j].first + "," + answers[i][j].second +
                                              "), " + "but got: (" + result.first + "," +
                                              result.second + ")";
      ++j;
    }
  }
}

TEST(Mapmatch, test_time_rejection) {
  tyr::actor_t actor(conf, true);
  auto matched = test::json_to_pt(actor.trace_attributes(
      R"({"trace_options":{"max_route_distance_factor":10,"max_route_time_factor":3,"turn_penalty_factor":0},
          "costing":"auto","shape_match":"map_snap","shape":[
          {"lat":52.09110,"lon":5.09806,"accuracy":10,"time":2},
          {"lat":52.09050,"lon":5.09769,"accuracy":100,"time":4},
          {"lat":52.09098,"lon":5.09679,"accuracy":10,"time":6}]})"));
  std::unordered_set<std::string> names;
  for (const auto& edge : matched.get_child("edges"))
    for (const auto& name : edge.second.get_child("names"))
      names.insert(name.second.get_value<std::string>());
  EXPECT_EQ(names.find("Jan Pieterszoon Coenstraat"), names.end())
      << "Using time it should not take a small detour";
}

TEST(Mapmatch, test32bit) {
  tyr::actor_t actor(conf, true);
  std::string test_case =
      R"({"costing":"auto","locations":[{"lat":52.096672,"lon":5.110825},
          {"lat":52.081371,"lon":5.125671,"name":"foo","street":"bar","city":"baz",
          "state":"qux","postal_code":"corge","country":"quux","url":"zoinks",
          "date_time":"2001-06-07T15:17","heading_tolerance":90,"node_snap_tolerance":5,
          "way_id":1234,"minimum_reachability":50,"rank_candidates":false,
          "preferred_side":"neither","search_cutoff":100,"street_side_tolerance":5}],"date_time":{"type":0}})";
  actor.route(test_case);
}

TEST(Mapmatch, test_trace_route_edge_walk_expected_error_code) {
  // tests expected error_code for trace_route edge_walk
  auto expected_error_code = 443;
  tyr::actor_t actor(conf, true);

  try {
    auto response = test::json_to_pt(actor.trace_route(
        R"({"costing":"auto","shape_match":"edge_walk","shape":[
         {"lat":52.088548,"lon":5.15357,"accuracy":30,"time":2},
         {"lat":52.088627,"lon":5.153269,"accuracy":30,"time":4},
         {"lat":52.08864,"lon":5.15298,"accuracy":30,"time":6},
         {"lat":52.08861,"lon":5.15272,"accuracy":30,"time":8},
         {"lat":52.08863,"lon":5.15253,"accuracy":30,"time":10},
         {"lat":52.08851,"lon":5.15249,"accuracy":30,"time":12}]})"));
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, expected_error_code);
    // If we get here then all good - return
    return;
  }

  // If we get here then fail the test!
  FAIL() << "Expected trace_route edge_walk exception was not found";
}

TEST(Mapmatch, test_trace_route_map_snap_expected_error_code) {
  // tests expected error_code for trace_route edge_walk
  auto expected_error_code = 442;
  tyr::actor_t actor(conf, true);

  try {
    auto response = test::json_to_pt(actor.trace_route(
        R"({"costing":"auto","shape_match":"map_snap","shape":[
         {"lat":52.088548,"lon":5.15357,"radius":5,"time":2},
         {"lat":52.088627,"lon":5.153269,"radius":5,"time":4},
         {"lat":52.08864,"lon":5.15298,"radius":5,"time":6},
         {"lat":52.08861,"lon":5.15272,"radius":5,"time":8},
         {"lat":52.08863,"lon":5.15253,"radius":5,"time":10},
         {"lat":52.08851,"lon":5.15249,"radius":5,"time":12}]})"));
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, expected_error_code);
    // If we get here then all good - return
    return;
  }

  // If we get here then fail the test!
  FAIL() << "Expected trace_route map_snap exception was not found";
}

TEST(Mapmatch, test_trace_attributes_edge_walk_expected_error_code) {
  // tests expected error_code for trace_attributes edge_walk
  auto expected_error_code = 443;
  tyr::actor_t actor(conf, true);

  try {
    auto response = test::json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"edge_walk","shape":[
         {"lat":52.088548,"lon":5.15357,"accuracy":30,"time":2},
         {"lat":52.088627,"lon":5.153269,"accuracy":30,"time":4},
         {"lat":52.08864,"lon":5.15298,"accuracy":30,"time":6},
         {"lat":52.08861,"lon":5.15272,"accuracy":30,"time":8},
         {"lat":52.08863,"lon":5.15253,"accuracy":30,"time":10},
         {"lat":52.08851,"lon":5.15249,"accuracy":30,"time":12}]})"));
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, expected_error_code);
    // If we get here then all good - return
    return;
  }

  // If we get here then fail the test!
  FAIL() << "Expected trace_attributes edge_walk exception was not found";
}

TEST(Mapmatch, test_trace_attributes_map_snap_expected_error_code) {
  // tests expected error_code for trace_attributes edge_walk
  auto expected_error_code = 444;
  tyr::actor_t actor(conf, true);

  try {
    auto response = test::json_to_pt(actor.trace_attributes(
        R"({"costing":"auto","shape_match":"map_snap","shape":[
         {"lat":52.088548,"lon":5.15357,"radius":5,"time":2},
         {"lat":52.088627,"lon":5.153269,"radius":5,"time":4},
         {"lat":52.08864,"lon":5.15298,"radius":5,"time":6},
         {"lat":52.08861,"lon":5.15272,"radius":5,"time":8},
         {"lat":52.08863,"lon":5.15253,"radius":5,"time":10},
         {"lat":52.08851,"lon":5.15249,"radius":5,"time":12}]})"));
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, expected_error_code);
    // If we get here then all good - return
    return;
  }

  // If we get here then fail the test!
  FAIL() << "Expected trace_attributes map_snap exception was not found";
}

TEST(Mapmatch, test_topk_validate) {
  // tests a fork in the road
  tyr::actor_t actor(conf, true);

  // tests a previous segfault due to using a claimed state
  auto matched = test::json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
         {"lat":52.088548,"lon":5.15357,"accuracy":30,"time":2},
         {"lat":52.088627,"lon":5.153269,"accuracy":30,"time":4},
         {"lat":52.08864,"lon":5.15298,"accuracy":30,"time":6},
         {"lat":52.08861,"lon":5.15272,"accuracy":30,"time":8},
         {"lat":52.08863,"lon":5.15253,"accuracy":30,"time":10},
         {"lat":52.08851,"lon":5.15249,"accuracy":30,"time":12}]})"));

  // this tests a fix for an infinite loop because there is only 1 result and we ask for 4
  matched = test::json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":4,"shape_match":"map_snap","shape":[
         {"lat":52.09579,"lon":5.13137,"accuracy":5,"time":2},
         {"lat":52.09652,"lon":5.13184,"accuracy":5,"time":4}]})"));

  EXPECT_EQ(matched.get_child("alternate_paths").size(), 0) << "There should be only one result";
}

TEST(Mapmatch, test_topk_fork_alternate) {
  // tests a fork in the road
  tyr::actor_t actor(conf, true);
  auto json = actor.trace_attributes(
      R"({"trace_options":{"search_radius":0},"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
          {"lat":52.08511,"lon":5.15085,"accuracy":10,"time":2},
          {"lat":52.08533,"lon":5.15109,"accuracy":20,"time":4},
          {"lat":52.08539,"lon":5.15100,"accuracy":20,"time":6}]})");
  auto matched = test::json_to_pt(json);

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
    FAIL() << "The most obvious result is stay left but got: " + streets;
  }
  EXPECT_EQ(matched.get<float>("confidence_score"), 1.0f)
      << "Confidence of the first result is always 1";

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
    FAIL() << "The second most obvious result is stay right but got: " + streets;
  }
  EXPECT_LT(alternate.get<float>("confidence_score"), 1.0f)
      << "Confidence of the second result is always less than 1";
  EXPECT_LT(matched.get<float>("raw_score"), alternate.get<float>("raw_score"))
      << "The raw score of the first result is always less than that of the second";
}

TEST(Mapmatch, test_topk_loop_alternate) {
  // tests a loop in the road
  tyr::actor_t actor(conf, true);
  auto matched = test::json_to_pt(actor.trace_attributes(
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
    FAIL() << "The most obvious result is stay left on the same road - but got: " + streets;
  }

  EXPECT_EQ(matched.get<float>("confidence_score"), 1.0f)
      << "Confidence of the first result is always 1";

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
    FAIL() << "The second most obvious result is loop around to the right - but got: " + streets;
  }
  EXPECT_LT(alternate.get<float>("confidence_score"), 1.0f)
      << "Confidence of the second result is always less than 1";
  EXPECT_LT(matched.get<float>("raw_score"), alternate.get<float>("raw_score"))
      << "The raw score of the first result is always less than that of the second";
}

TEST(Mapmatch, test_topk_frontage_alternate) {
  // tests a parallel frontage road
  tyr::actor_t actor(conf, true);
  auto matched = test::json_to_pt(actor.trace_attributes(
      R"({"costing":"auto","best_paths":2,"shape_match":"map_snap","shape":[
           {"lat":52.07956040090567,"lon":5.138160288333893,"accuracy":10,"time":2},
           {"lat":52.07957358807355,"lon":5.138508975505829,"accuracy":10,"time":4},
           {"lat":52.07959666560798,"lon":5.138905942440034,"accuracy":10,"time":6},
           {"lat":52.0796213915245,"lon":5.139262676239015,"accuracy":10,"time":8},
           {"lat":52.079637875461195,"lon":5.139581859111787,"accuracy":10,"time":10},
           {"lat":52.07964776582031,"lon":5.139828622341157,"accuracy":10,"time":12},
           {"lat":52.07985600778458,"lon":5.1404121782302865,"accuracy":10,"time":14}]})"));

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
    FAIL() << "The most obvious result is stay straight on the same road - but got: " + streets;
  }
  EXPECT_EQ(matched.get<float>("confidence_score"), 1.0f)
      << "Confidence of the first result is always 1";

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
  if (names != std::vector<std::string>{"Rubenslaan", "Rubenslaan", "Rubenslaan"}) {
    std::string streets;
    for (const auto& n : names)
      streets += n + ", ";
    FAIL() << "The second most obvious result is frontage road to the right - but got: " + streets;
  }

  EXPECT_LT(alternate.get<float>("confidence_score"), 1.0f)
      << "Confidence of the second result is always less than 1";
  EXPECT_LT(matched.get<float>("raw_score"), alternate.get<float>("raw_score"))
      << "The raw score of the first result is always less than that of the second";
}

TEST(Mapmatch, test_now_matches) {
  tyr::actor_t actor(conf, true);

  // once with map matching
  std::string test_case =
      R"({"date_time":{"type":0},"shape_match":"map_snap","costing":"auto",
         "encoded_polyline":"oeyjbBqfjwHeO~M}x@`u@wDmh@oCcd@sAiVcAaKe@cBaNe[u^qg@qH`u@cL{Tmr@c{AtTu_@xVsd@"})";
  auto route_json = actor.trace_route(test_case);

  // again with walking
  auto route = test::json_to_pt(route_json);
  auto encoded_shape = route.get_child("trip.legs").front().second.get<std::string>("shape");
  test_case =
      R"({"date_time":{"type":0},"shape_match":"edge_walk","costing":"auto","encoded_polyline":")" +
      json_escape(encoded_shape) + "\"}";
  Api api;
  actor.trace_route(test_case, nullptr, &api);
  EXPECT_NE(api.trip().routes(0).legs(0).location(0).path_edges_size(), 0);
  EXPECT_NE(api.trip().routes(0).legs(0).location().rbegin()->path_edges_size(), 0);
}

TEST(Mapmatch, test_leg_duration_trimming) {
  std::vector<std::vector<std::string>> test_cases = {
      // 2 routes, one leg per route
      {R"([{"lat": 52.0865058, "lon": 5.1201, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0865512, "lon": 5.1201, "type": "via", "node_snap_tolerance":0},
          {"lat": 52.0867449, "lon": 5.12, "type": "break", "node_snap_tolerance":0}])",
       R"([{"lat": 52.12705182, "lon": 5.0892165, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.1267117, "lon": 5.0898420, "type": "via", "node_snap_tolerance":0},
          {"lat": 52.1261379, "lon": 5.0907894, "type": "break", "node_snap_tolerance":0}])"},
      // 2 routes, multiple legs per route
      {R"([{"lat": 52.0865058, "lon": 5.1201, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0865512, "lon": 5.1201, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0867449, "lon": 5.12, "type": "break", "node_snap_tolerance":0}])",
       R"([{"lat": 52.12705182, "lon": 5.0892165, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.1267117, "lon": 5.0898420, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.1261379, "lon": 5.0907894, "type": "break", "node_snap_tolerance":0}])"},
      // 2 routes close together in space
      {R"([{"lat": 52.0940283, "lon": 5.1133286, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0938604, "lon": 5.1133609, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0935827, "lon": 5.1133893, "type": "break", "node_snap_tolerance":0}])",
       R"([{"lat": 52.0939489, "lon": 5.1136976, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0939837, "lon": 5.1139511, "type": "break", "node_snap_tolerance":0}])"},
      // 1 route with 1 leg across 2 edges and then another leg on the same second edge
      {R"([{"lat": 52.0957652, "lon": 5.1101366, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0959457, "lon": 5.1106847, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0962535, "lon": 5.1116988, "type": "break", "node_snap_tolerance":0}])"},
      // 1 route with 1 leg across 2 edges
      {R"([{"lat": 52.0957652, "lon": 5.1101366, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0959457, "lon": 5.1106847, "type": "break", "node_snap_tolerance":0}])"},
      // 1 route with 1 leg on 1 edge
      {R"([{"lat": 52.0959457, "lon": 5.1106847, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0962535, "lon": 5.1116988, "type": "break", "node_snap_tolerance":0}])"},
      // 1 routes, lots of legs on same edge
      {R"([{"lat": 52.108781, "lon": 5.1057369, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.108982, "lon": 5.1054472, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.109180, "lon": 5.1051449, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.109407, "lon": 5.1047962, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.109589, "lon": 5.1044422, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.109806, "lon": 5.1041525, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.110030, "lon": 5.1038306, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.110244, "lon": 5.1034766, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.110521, "lon": 5.1031029, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.110735, "lon": 5.1027846, "type": "break", "node_snap_tolerance":0}])"},
      {R"([{"lat": 52.0826293, "lon": 5.1267623, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0835867, "lon": 5.1276355, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0837127, "lon": 5.1277763, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0839615, "lon": 5.1280204, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0841756, "lon": 5.1282906, "type": "break", "node_snap_tolerance":0}])"},
      {R"([{"lat": 52.0609108, "lon": 5.0924059, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0605926, "lon": 5.0962937, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0604866, "lon": 5.0975675, "type": "break", "node_snap_tolerance":0},
          {"lat": 52.0601766, "lon": 5.1005663, "type": "break", "node_snap_tolerance":0}])"},
  };

  api_tester tester;
  for (const auto& test_case : test_cases) {
    // for routing we need to do each route separately, and we manually mash them into one object
    valhalla::Api route_api;
    for (const auto& locations : test_case) {
      auto route_test_case = R"({"costing":"auto","locations":)" + locations + '}';
      auto single_route_api = tester.route(route_test_case);
      route_api.mutable_trip()->mutable_routes()->MergeFrom(single_route_api.trip().routes());
    }

    // for map matching we do it all in one shot by mashing all the locations together
    // NOTE: if want a test case with a discontinuity make sure a map match is not possible at it
    std::string match_test_case = R"({"costing":"auto", "shape_match":"map_snap","shape":)";
    for (const auto& locations : test_case) {
      if (match_test_case.back() == ']')
        match_test_case.back() = ',';
      match_test_case += locations.substr(match_test_case.back() == ',');
    }
    match_test_case += '}';
    auto match_api = tester.match(match_test_case);
    printf("\n%s\n", match_test_case.c_str());

    // they should not disagree (unless the map match is very vague)
    EXPECT_EQ(route_api.trip().routes_size(), match_api.trip().routes_size())
        << "Number of routes differs";

    for (size_t i = 0; i < route_api.trip().routes_size(); ++i) {
      const auto& rlegs = route_api.trip().routes(i).legs();
      const auto& mlegs = match_api.trip().routes(i).legs();
      EXPECT_EQ(rlegs.size(), mlegs.size()) << "Number of legs differs";
      printf("Route %zu\n", i);
      for (size_t j = 0; j < rlegs.size(); ++j) {
        auto rtime = rlegs.Get(j).node().rbegin()->cost().elapsed_cost().seconds();
        auto mtime = mlegs.Get(j).node().rbegin()->cost().elapsed_cost().seconds();
        printf("r: %.2f %s\n", rtime, rlegs.Get(j).shape().c_str());
        printf("m: %.2f %s\n", mtime, mlegs.Get(j).shape().c_str());
        EXPECT_NEAR(rtime, mtime, 0.1) << "Leg time differs";
      }
    }
  }
}

TEST(Mapmatch, test_discontinuity_on_same_edge) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0948884, "lon": 5.1112737, "type": "break"},
          {"lat": 52.0949312, "lon": 5.1118285, "type": "break"},
          {"lat": 52.1054861, "lon": 5.1289207, "type": "break"},
          {"lat": 52.0948920, "lon": 5.1113874, "type": "break"},
          {"lat": 52.0949182, "lon": 5.1116986, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0962541, "lon": 5.1129487, "type": "break"},
          {"lat": 52.0959946, "lon": 5.1120336, "type": "break"},
          {"lat": 52.1054861, "lon": 5.1289207, "type": "break"},
          {"lat": 52.0961760, "lon": 5.1126547, "type": "break"},
          {"lat": 52.0960736, "lon": 5.1123291, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0963956, "lon": 5.1133696, "type": "break"},
          {"lat": 52.0959223, "lon": 5.1118128, "type": "break"},
          {"lat": 52.1108625, "lon": 5.1325334, "type": "break"},
          {"lat": 52.0962053, "lon": 5.1128136, "type": "break"},
          {"lat": 52.0960736, "lon": 5.1123291, "type": "break"},
          {"lat": 52.0963029, "lon": 5.1131075, "type": "break"},
          {"lat": 52.0959906, "lon": 5.1120590, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0956110, "lon": 5.0978352, "type": "break"},
          {"lat": 52.0956407, "lon": 5.0974388, "type": "break"},
          {"lat": 52.0956612, "lon": 5.0971513, "type": "break"},
          {"lat": 52.0956258, "lon": 5.0973227, "type": "break"},
          {"lat": 52.0956068, "lon": 5.0976801, "type": "break"},
          {"lat": 52.0956105, "lon": 5.0975165, "type": "break"},
          {"lat": 52.0956333, "lon": 5.0972287, "type": "break"}]})"};

  std::vector<int> test_ans_num_routes{2, 2, 3, 2};
  std::vector<std::vector<int>> test_ans_num_legs{{1, 1}, {1, 1}, {1, 1, 1}, {2, 2}};

  api_tester tester;
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto result = tester.match(test_cases[i]);
    EXPECT_EQ(result.trip().routes_size(), test_ans_num_routes[i]);
    int j = 0, k = 0;
    for (const auto& route : result.trip().routes()) {
      ASSERT_EQ(route.legs_size(), test_ans_num_legs[i][j++])
          << "Expected " + std::to_string(test_ans_num_legs[i][j - 1]) + " legs but got " +
                 std::to_string(route.legs_size());

      for (const auto& leg : route.legs()) {
        valhalla::Api route_api;
        auto route_test_case =
            R"({"costing":"auto","locations":[{"lat":)" + std::to_string(leg.location(0).ll().lat()) +
            R"(,"lon":)" + std::to_string(leg.location(0).ll().lng()) +
            R"(,"node_snap_tolerance":0},{"lat":)" + std::to_string(leg.location(1).ll().lat()) +
            R"(,"lon":)" + std::to_string(leg.location(1).ll().lng()) +
            R"(,"node_snap_tolerance":0}]})";

        auto single_route_api = tester.route(route_test_case);
        double route_duration = single_route_api.trip()
                                    .routes(0)
                                    .legs(0)
                                    .node()
                                    .rbegin()
                                    ->cost()
                                    .elapsed_cost()
                                    .seconds();
        double duration = leg.node().rbegin()->cost().elapsed_cost().seconds();
        ASSERT_NEAR(duration, route_duration, .1) << "Expected legs with duration " +
                                                         std::to_string(route_duration) +
                                                         " but got " + std::to_string(duration);
      }
    }
  }
}

TEST(Mapmatch, test_discontinuity_duration_trimming) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.1055358, "lon": 5.1208866, "type": "break"},
          {"lat": 52.1044362, "lon": 5.1259572, "type": "break"},
          {"lat": 52.1130862, "lon": 5.1445529, "type": "break"},
          {"lat": 52.1130460, "lon": 5.1444851, "type": "break"},
          {"lat": 52.1130186, "lon": 5.1444687, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.1047614, "lon": 5.1245468, "type": "break"},
          {"lat": 52.1022218, "lon": 5.1299002, "type": "break"},
          {"lat": 52.1131029, "lon": 5.1440879, "type": "break"},
          {"lat": 52.1131350, "lon": 5.1440195, "type": "break"},
          {"lat": 52.1131857, "lon": 5.1439104, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.1089306, "lon": 5.1226142, "type": "break"},
          {"lat": 52.1060622, "lon": 5.1256574, "type": "break"},
          {"lat": 52.0873837, "lon": 5.1442371, "type": "break"},
          {"lat": 52.0872970, "lon": 5.1439155, "type": "break"},
          {"lat": 52.0872151, "lon": 5.1436803, "type": "break"}]})"};

  std::vector<int> test_ans_num_routes{2, 2, 2};
  std::vector<std::vector<int>> test_ans_num_legs{{1, 2}, {1, 2}, {1, 2}};
  std::vector<std::vector<float>> test_ans_leg_duration{{26.459, 0.97, 0.454},
                                                        {48.159, 0.64, 0.961},
                                                        {93.388, 2.431, 1.899}};

  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& routes = matched.get_child("matchings");
    EXPECT_EQ(routes.size(), test_ans_num_routes[i]);
    int j = 0, k = 0;
    for (const auto& route : routes) {
      const auto& legs = route.second.get_child("legs");
      ASSERT_EQ(legs.size(), test_ans_num_legs[i][j++])
          << "Expected " + std::to_string(test_ans_num_legs[i][j - 1]) + " legs but got " +
                 std::to_string(legs.size());
      for (const auto& leg : legs) {
        float duration = leg.second.get<float>("duration");
        ASSERT_NEAR(duration, test_ans_leg_duration[i][k++], .1)
            << "Expected legs with duration " + std::to_string(test_ans_leg_duration[i][k - 1]) +
                   " but got " + std::to_string(duration);
      }
    }
  }
}

TEST(Mapmatch, test_transition_matching) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.1003455, "lon": 5.1194303, "type": "break"},
          {"lat": 52.1003954, "lon": 5.1190220, "type": "break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.1011859, "lon": 5.1209135, "type": "break"},
          {"lat": 52.1009284, "lon": 5.1204603, "type": "break"}]})",
  };

  std::vector<int> test_ans_num_routes{1, 1};
  std::vector<float> durations{3.4, 5.0};
  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& routes = matched.get_child("matchings");
    EXPECT_EQ(routes.size(), test_ans_num_routes[i]);
    for (const auto& route : routes) {
      const auto& legs = route.second.get_child("legs");
      for (const auto& leg : legs) {
        float duration = leg.second.get<float>("duration");
        ASSERT_NEAR(duration, durations[i], .1) << "Expected legs with duration " +
                                                       std::to_string(durations[i]) + " but got " +
                                                       std::to_string(duration);
      }
    }
  }
}

TEST(Mapmatch, test_loop_matching) {
  // NOTE THAT: test case 0 and 3 has discontinuity on loops
  std::vector<std::string> test_cases = {
      R"({"shape":[
          {"lat": 52.0992698, "lon": 5.1071285, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.0990768, "lon": 5.1069392, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.0995259, "lon": 5.1073563, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto", "shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1183497, "lon": 5.1171364, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1181338, "lon": 5.1188697, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1182095, "lon": 5.1170544, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto", "shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1181394, "lon": 5.1168568, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1181338, "lon": 5.1188697, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1183749, "lon": 5.1173171, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto", "shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1185567, "lon": 5.1226105, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1189432, "lon": 5.1244406, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1183977, "lon": 5.1223398, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1201857, "lon": 5.1153547, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1191862, "lon": 5.1165680, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1206734, "lon": 5.1161527, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1191920, "lon":5.1026068, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1189946, "lon": 5.1029042, "type": "break_through", "node_snap_tolerance": 0},
          {"lat": 52.1192212, "lon": 5.1024556, "type": "break_through", "node_snap_tolerance": 0}],
          "costing":"auto","shape_match":"map_snap"})",
  };

  api_tester tester;
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto route_case = R"({"locations)" + test_cases[i].substr(7);
    auto routed = tester.route(route_case);
    auto matched = tester.match(test_cases[i]);
    compare_results(routed, matched);
  }
}

TEST(Mapmatch, test_intersection_matching) {
  std::vector<std::string> test_cases = {
      R"({"shape":[
          {"lat": 52.098127, "lon": 5.129618, "type": "break", "node_snap_tolerance": 0},
          {"lat": 52.098128, "lon": 5.129725, "type": "break", "node_snap_tolerance": 0}],
          "costing":"auto","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.098134, "lon": 5.130043, "type": "break", "node_snap_tolerance": 5},
          {"lat": 52.098125, "lon": 5.130946, "type": "break", "node_snap_tolerance": 5},
          {"lat": 52.098064, "lon": 5.131499, "type": "break", "node_snap_tolerance": 5},
          {"lat": 52.098087, "lon": 5.131504, "type": "break", "node_snap_tolerance": 5}],
          "costing":"auto","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.095164, "lon": 5.128560, "type": "break", "node_snap_tolerance": 5},
          {"lat": 52.095295, "lon": 5.130906, "type": "break", "node_snap_tolerance": 5},
          {"lat": 52.094478, "lon": 5.130406, "type": "break", "node_snap_tolerance": 5}],
          "costing":"auto","shape_match":"map_snap"})"};

  api_tester tester;
  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = tester.match(test_cases[i]);
    auto route_case = R"({"locations)" + test_cases[i].substr(7);
    auto routed = tester.route(route_case);
    compare_results(routed, matched);
  }
}

TEST(Mapmatch, test_degenerate_match) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0981280, "lon": 5.1297250, "type": "break", "time":10},
          {"lat": 52.0981280, "lon": 5.1297250, "type": "break", "time":169}],
          "trace_options": {"interpolation_distance": 0}})",
  };
  tyr::actor_t actor(conf, true);

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& routes = matched.get_child("matchings");
    for (const auto& route : routes) {
      const auto& legs = route.second.get_child("legs");
      for (const auto& leg : legs) {
        double duration = leg.second.get<double>("duration");
        ASSERT_TRUE(duration >= 0);
      }
    }
  }
}

TEST(Mapmatch, interpolation) {
  std::vector<std::string> test_cases = {
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.082829, "lon": 5.087129, "type": "break"},
          {"lat": 52.082870, "lon": 5.086956, "type": "break"},
          {"lat": 52.082870, "lon": 5.086960, "type": "break"},
          {"lat": 52.082855, "lon": 5.087024, "type": "break"}]})",
      R"({"trace_options": {"interpolation_distance": 20},
          "costing":"auto","format":"osrm","shape_match":"map_snap","shape":[
          {"lat": 52.0749799, "lon": 5.1141067, "type": "break"},
          {"lat": 52.0750399, "lon": 5.1141172, "type": "break"},
          {"lat": 52.0750431, "lon": 5.1141172, "type": "break"},
          {"lat": 52.0750392, "lon": 5.1141197, "type": "break"}]})"};
  tyr::actor_t actor(conf, true);

  for (size_t i = 0; i < test_cases.size(); ++i) {
    auto matched = test::json_to_pt(actor.trace_route(test_cases[i]));
    const auto& routes = matched.get_child("matchings");
    ASSERT_EQ(1, routes.size());
    for (const auto& route : routes) {
      const auto& legs = route.second.get_child("legs");
      ASSERT_EQ(3, legs.size());
    }
  }
}

TEST(Mapmatch, duplicated_end_points) {
  std::vector<std::string> test_cases = {
      R"({"shape":[
          {"lat": 52.09617, "lon": 5.121475, "type": "break", "radius": 10},
          {"lat": 52.09617, "lon": 5.121475, "type": "break", "radius": 10},
          {"lat": 52.09617, "lon": 5.121475, "type": "break", "radius": 10}],
          "costing":"auto","format":"osrm","shape_match":"map_snap",
          "trace_options": {"interpolation_distance": 30}})",
      R"({"shape":[
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1217638, "lon": 5.1011698, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1217638, "lon": 5.1011698, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap"})",
      R"({"shape":[
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1214847, "lon": 5.1011657, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"},
          {"lat": 52.1215641, "lon": 5.1010741, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap",
          "trace_options": {"interpolation_distance": 0}})",
      R"({"shape":[
          {"lat": 52.09531,"lon": 5.11413, "type": "break"},
          {"lat": 52.09531,"lon": 5.11413, "type": "break"},
          {"lat": 52.09531,"lon": 5.11413, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap",
          "trace_options": {"interpolation_distance": 0}})",
      R"({"shape":[
          {"lat": 52.09531,"lon": 5.11413, "type": "break"},
          {"lat": 52.09531,"lon": 5.11413, "type": "break"},
          {"lat": 52.09531,"lon": 5.11413, "type": "break"}],
          "costing":"auto","format":"osrm","shape_match":"map_snap"})"};

  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    Api req_rep;
    actor.trace_route(test_cases[i], nullptr, &req_rep);
    // all of them should be a single route
    ASSERT_EQ(1, req_rep.trip().routes_size());
    // there should be locations - 1 legs in each route
    ASSERT_EQ(req_rep.options().shape_size() - 1, req_rep.trip().routes(0).legs_size());
  }
}

TEST(Mapmatch, no_edge_candidates) {
  // clang-format off
  std::vector<std::pair<size_t, std::string>> test_cases = {
    // First two cases are too far from the road to have candidates, so throw 443 NoSegment.
    {443, R"({"shape":[
             {"lat":"52.0954408","lon":"5.1135341","type":"break","radius":15},
             {"lat":"52.0954819","lon":"5.1135190","type":"break","radius":15}],
              "costing":"auto","shape_match":"map_snap","format":"osrm",
              "trace_options":{"interpolation_distance":0}})"},
    {443, R"({"shape":[
             {"lat":"52.0954408","lon":"5.1135341","type":"break","radius":15},
             {"lat":"52.0954819","lon":"5.1135190","type":"break","radius":15},
             {"lat":"52.0954010","lon":"5.1135599","type":"break","radius":15}],
             "costing":"auto","shape_match":"map_snap","format":"osrm",
             "trace_options":{"interpolation_distance":0}})"},
    // Only a single point is too far, so we should throw 442 NoRoute.
    {442, R"({"shape":[
             {"lat":"52.0954342","lon":"5.1140063","type":"break","radius":15},
             {"lat":"52.0954819","lon":"5.1135190","type":"break","radius":15}],
             "costing":"auto","shape_match":"map_snap","format":"osrm",
             "trace_options":{"interpolation_distance":0}})"}
  };
  // clang-format on
  tyr::actor_t actor(conf, true);
  for (size_t i = 0; i < test_cases.size(); ++i) {
    Api req_rep;
    try {
      actor.trace_route(test_cases[i].second, nullptr, &req_rep);
    } catch (const valhalla::valhalla_exception_t& e) { ASSERT_EQ(e.code, test_cases[i].first); }
  }
}

TEST(Mapmatch, test_breakage_distance_error_handling) {
  // tests expected error_code for trace_route edge_walk
  auto expected_error_code = 172;
  tyr::actor_t actor(conf, true);
  try {
    auto response = test::json_to_pt(actor.trace_route(
        R"({"costing":"auto","shape_match":"edge_walk","shape":[
         {"lat":52.0764279,"lon":5.0323097,"type":"break","radius":15},
         {"lat":52.1022785,"lon":5.1391531,"type":"break","radius":15}]})"));
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, expected_error_code);
    // If we get here then all good - return
    return;
  }

  // If we get here then fail the test!
  FAIL() << "Expected trace_route breakage distance exceed exception was not found";
}

// Spot check OpenLR linear references when asked for
TEST(Mapmatch, openlr_parameter_true_osrm_api) {
  // clang-format off
  const std::string request = R"({"costing":"auto","linear_references": true,"format":"osrm","shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})";
  // clang-format on
  tyr::actor_t actor(conf, true);
  const auto& response = test::json_to_pt(actor.trace_route(request));
  const auto& matches = response.get_child("matchings");
  EXPECT_EQ(matches.size(), 1);
  const std::vector<std::string>& expected = {
      "CwOduyULYiKJAAAV//0iGw==",
      "CwOdxCULYCKJAAAN//8iGw==",
      "CwOdySULXyKJAAAf//EiGw==",
      "CwOd1yULWCKLAQBV/84iGw==",
  };
  for (const auto& match : matches) {
    std::vector<std::string> references;
    for (const auto& reference : match.second.get_child("linear_references"))
      references.push_back(reference.second.get_value<std::string>());
    EXPECT_EQ(references.size(), 4);
    EXPECT_EQ(expected, references);
  }
}

// Replica of above test, but uses the Valhalla's native API
TEST(Mapmatch, openlr_parameter_true_native_api) {
  // clang-format off
  const std::string request = R"({"costing":"auto","linear_references": true,"shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})";
  // clang-format on
  tyr::actor_t actor(conf, true);
  const auto& response = test::json_to_pt(actor.trace_route(request));
  const std::vector<std::string>& expected = {
      "CwOduyULYiKJAAAV//0iGw==",
      "CwOdxCULYCKJAAAN//8iGw==",
      "CwOdySULXyKJAAAf//EiGw==",
      "CwOd1yULWCKLAQBV/84iGw==",
  };
  std::vector<std::string> references;
  for (const auto& reference : response.get_child("trip.linear_references"))
    references.push_back(reference.second.get_value<std::string>());
  EXPECT_EQ(references.size(), 4);
  EXPECT_EQ(expected, references);
}

// Verify that OpenLR references are not present in API response when not asked for
TEST(Mapmatch, openlr_parameter_falsy_api) {
  const std::vector<std::string> requests = {
      R"({"costing":"auto","linear_references":false,"format":"osrm","shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})",
      R"({"costing":"auto","format":"osrm","shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})",
  };
  tyr::actor_t actor(conf, true);
  for (const auto& request : requests) {
    const auto& response = test::json_to_pt(actor.trace_route(request));
    const auto& matches = response.get_child("matchings");
    EXPECT_EQ(matches.size(), 1);
    for (const auto& match : matches) {
      EXPECT_THROW(match.second.get_child("linear_references"), std::runtime_error);
    }
  }
}

// Replica of above test, but uses the Valhalla's native API
TEST(Mapmatch, openlr_parameter_falsy_native_api) {
  const std::vector<std::string> requests = {
      R"({"costing":"auto","linear_references":false,"shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})",
      R"({"costing":"auto","shape_match":"map_snap","shape":[{"lon":5.08531221,"lat":52.0938563,"type":"break"},{"lon":5.0865867,"lat":52.0930211,"type":"break"}]})",
  };
  tyr::actor_t actor(conf, true);
  for (const auto& request : requests) {
    const auto& response = test::json_to_pt(actor.trace_route(request));
    EXPECT_THROW(response.get_child("trip.linear_references"), std::runtime_error);
  }
}
} // namespace

int main(int argc, char* argv[]) {
  midgard::logging::Configure({{"type", ""}});
  if (argc > 1 && std::string(argv[1]).find("gtest") == std::string::npos) {
    if (argc > 1)
      seed = std::stoi(argv[1]);
    if (argc > 2)
      bound = std::stoi(argv[2]);
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
