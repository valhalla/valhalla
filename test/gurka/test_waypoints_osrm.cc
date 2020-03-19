#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, WaypointsOsrmSingleEdge) {
  const std::string ascii_map = R"(
    A----B
         |
         C
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_waypoints_osrm_1");
  auto result = gurka::route(map, "A", "B", "auto");

  result.mutable_options()->set_format(valhalla::Options_Format_osrm);
  auto json = tyr::serializeDirections(result);

  rapidjson::Document result_json;
  result_json.Parse(json.c_str());
  if (result_json.HasParseError()) {
    FAIL();
  }

  std::cout << json.c_str() << std::endl;
  EXPECT_TRUE(result_json.HasMember("waypoints"));
  EXPECT_TRUE(result_json["waypoints"].IsArray());
  EXPECT_EQ(result_json["waypoints"].Size(), 2);
  EXPECT_TRUE(result_json["waypoints"][0].IsObject());
  EXPECT_EQ(result_json["waypoints"][0]["name"], "AB");
  EXPECT_EQ(result_json["waypoints"][0]["road_class"], "motorway");

  EXPECT_TRUE(result_json["waypoints"][1].IsObject());
  EXPECT_EQ(result_json["waypoints"][1]["name"], "AB");
  EXPECT_EQ(result_json["waypoints"][1]["road_class"], "motorway");
}

TEST(Standalone, WaypointsOsrmMultiLeg) {
  const std::string ascii_map = R"(
    A----B
         |
         C)";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}},
      {"BC", {{"highway", "service"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_waypoints_osrm_2");
  auto result = gurka::route(map, {"A", "B", "C"}, "auto");

  result.mutable_options()->set_format(valhalla::Options_Format_osrm);
  auto json = tyr::serializeDirections(result);

  rapidjson::Document result_json;
  result_json.Parse(json.c_str());
  if (result_json.HasParseError()) {
    FAIL();
  }

  std::cout << json.c_str() << std::endl;
  EXPECT_TRUE(result_json.HasMember("waypoints"));
  EXPECT_TRUE(result_json["waypoints"].IsArray());
  EXPECT_EQ(result_json["waypoints"].Size(), 3);
  EXPECT_TRUE(result_json["waypoints"][0].IsObject());
  EXPECT_EQ(result_json["waypoints"][0]["name"], "AB");
  EXPECT_EQ(result_json["waypoints"][0]["road_class"], "motorway");

  EXPECT_TRUE(result_json["waypoints"][1].IsObject());
  EXPECT_EQ(result_json["waypoints"][1]["name"], "AB");
  EXPECT_EQ(result_json["waypoints"][1]["road_class"], "motorway");

  EXPECT_TRUE(result_json["waypoints"][2].IsObject());
  EXPECT_EQ(result_json["waypoints"][2]["name"], "BC");
  EXPECT_EQ(result_json["waypoints"][2]["road_class"], "service_other");
}
