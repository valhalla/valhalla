#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{"mjolnir.shortcuts", "false"}};

class Use : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

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

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_use", build_config);
  }
};
gurka::map Use::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(Use, EdgeUse) {
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

TEST_F(Use, test_rest_area_use_excluded_by_default) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["C"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["C"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})",
                          {}, &api);

  // get the osrm json
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("rest_stops"), false);
        }
      }
    }
  }
}

TEST_F(Use, test_rest_area_use) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["C"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["C"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(
      R"({"costing":"auto","format":"osrm","filters":{"action":"include","attributes":["edge.use.rest_area"]},"locations":[)" +
          locations + R"(]})",
      {}, &api);

  // get the osrm json
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("rest_stops")) {
            EXPECT_EQ(intersection.HasMember("rest_stops"), true);
            for (const auto& stop : intersection["rest_stops"].GetArray()) {
              EXPECT_TRUE(stop["type"].IsString());
              // EXPECT_TRUE(stop["type"] == "rest_area");
            }
          }
        }
      }
    }
  }
}
TEST_F(Use, test_service_area_use_excluded_by_default) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["A"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["A"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(R"({"costing":"auto","format":"osrm","locations":[)" + locations + R"(]})",
                          {}, &api);

  // get the osrm json
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          EXPECT_EQ(intersection.HasMember("rest_stops"), false);
        }
      }
    }
  }
}

TEST_F(Use, test_service_area_use) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["B"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["B"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(
      R"({"costing":"auto","format":"osrm","filters":{"action":"include","attributes":["edge.use.service_area"]},"locations":[)" +
          locations + R"(]})",
      {}, &api);

  // get the osrm json
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("rest_stops")) {
            EXPECT_EQ(intersection.HasMember("rest_stops"), true);
            for (const auto& stop : intersection["rest_stops"].GetArray()) {
              EXPECT_TRUE(stop["type"].IsString());
              EXPECT_TRUE(stop["type"] == "service_area");
            }
          }
        }
      }
    }
  }
}

TEST_F(Use, test_all_use) {
  std::string locations = R"({"lon":)" + std::to_string(map.nodes["B"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["B"].lat()) + R"(},{"lon":)" +
                          std::to_string(map.nodes["F"].lng()) + R"(,"lat":)" +
                          std::to_string(map.nodes["F"].lat()) + "}";

  auto reader = std::make_shared<baldr::GraphReader>(map.config.get_child("mjolnir"));
  valhalla::tyr::actor_t actor(map.config, *reader, true);
  auto json = actor.route(
      R"({"costing":"auto","format":"osrm","filters":{"action":"include","attributes":["edge.use.rest_area", "edge.use.service_area"]},"locations":[)" +
          locations + R"(]})",
      {}, &api);

  // get the osrm json
  d.Parse(json);
  EXPECT_FALSE(d.HasParseError());

  for (const auto& route : d["routes"].GetArray()) {
    for (const auto& leg : route["legs"].GetArray()) {
      for (const auto& step : leg["steps"].GetArray()) {
        for (const auto& intersection : step["intersections"].GetArray()) {
          if (intersection.HasMember("rest_stops")) {
            EXPECT_EQ(intersection.HasMember("rest_stops"), true);
            const auto& stop = intersection["rest_stops"].GetObject();
            EXPECT_TRUE(stop["type"].IsString());
            EXPECT_FALSE(stop["type"] == "rest_area");
            EXPECT_TRUE(stop["type"] == "service_area");
          }
        }
      }
    }
  }
}
