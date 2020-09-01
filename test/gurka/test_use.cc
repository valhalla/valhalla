#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

TEST(Use, Standalone) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_area"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use");

  auto result = gurka::route(map, "A", "C", "auto");

  // rest_area
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).edge().use(), TripLeg::Use::TripLeg_Use_kRestAreaUse); // BC

  // service_area
  result = gurka::route(map, "A", "F", "auto");

  leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(3).edge().use(), TripLeg::Use::TripLeg_Use_kServiceAreaUse); // EF
}

TEST(Use, test_rest_area_use_excluded_by_default) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_are"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               "test/data/gurka_rest_area_use_excluded_by_default", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);

  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["A"].lat()) +
                          R"(},{"lon":)" + std::to_string(map.nodes["C"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["C"].lat()) + "}";

  Api api;
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})",
                          {}, &api);

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("rest_area"), false);
        }
      }
    }
  }
}

TEST(Use, test_rest_area_use) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_area"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_rest_area_use", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);

  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["A"].lat()) +
                          R"(},{"lon":)" + std::to_string(map.nodes["C"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["C"].lat()) + "}";

  Api api;
  auto json = actor.route(
      R"({"costing":"auto","format":"osrm","filters":{"action":"include","attributes":["edge.rest_area_use"]},"locations":[)" +
          locations + R"(]})",
      {}, &api);

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("rest_area")) {
            EXPECT_EQ(intersection.HasMember("rest_area"), true);
          }
        }
      }
    }
  }
}
TEST(Use, test_service_area_use_excluded_by_default) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_area"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {},
                               "test/data/gurka_service_area_use_excluded_by_default", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);

  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["A"].lat()) +
                          R"(},{"lon":)" + std::to_string(map.nodes["F"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["F"].lat()) + "}";

  Api api;
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})",
                          {}, &api);

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("service_area"), false);
        }
      }
    }
  }
}

TEST(Use, test_service_area_use) {
  const std::string ascii_map = R"(
                          A
                          |
                          | 
                          B----C
                          |
                          |
                          D
                          |
                          |
                          E----F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "motorway"}}},
       {"BC", {{"highway", "motorway"}, {"service", "rest_area"}}},
       {"BD", {{"highway", "motorway"}}},
       {"DE", {{"highway", "motorway"}}},
       {"EF", {{"highway", "motorway"}, {"service", "rest_area"}, {"amenity", "yes"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_service_area_use", build_config);
  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);

  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["A"].lat()) +
                          R"(},{"lon":)" + std::to_string(map.nodes["F"].lng()) +
                          R"(,"lat":)" + std::to_string(map.nodes["F"].lat()) + "}";

  Api api;
  auto json = actor.route(
      R"({"costing":"auto","format":"osrm","filters":{"action":"include","attributes":["edge.service_area_use"]},"locations":[)" +
          locations + R"(]})",
      {}, &api);

  // get the osrm json
  rapidjson::Document d;
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("service_area")) {
            EXPECT_EQ(intersection.HasMember("service_area"), true);
          }
        }
      }
    }
  }
}
