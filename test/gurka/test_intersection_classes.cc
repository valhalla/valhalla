#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, IntersectionClassesAtRamp) {
  const std::string ascii_map = R"(
      A----B----C----D
                 \
                  E----F
  )";
  const gurka::ways ways = {
      {"ABCD", {{"highway", "primary"}}},
      {"CE", {{"highway", "motorway_link"}}},
      {"EF", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_intersection_classes_1");
  auto result = gurka::route(map, "A", "E", "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  // assert CE is restored to a motorway_link after reclassifying
  EXPECT_EQ(leg.node(1).edge().name(0).value(), "CE");
  EXPECT_EQ(leg.node(1).edge().use(), TripLeg_Use_kRampUse);
  EXPECT_EQ(leg.node(1).edge().road_class(), valhalla::RoadClass::kMotorway);

  result.mutable_options()->set_format(valhalla::Options_Format_osrm);
  auto json = tyr::serializeDirections(result);

  rapidjson::Document result_json;
  result_json.Parse(json.c_str());
  if (result_json.HasParseError()) {
    FAIL();
  }

  // assert motorway in intersection.classes[]
  EXPECT_EQ(result_json["routes"][0]["legs"][0]["steps"][1]["name"], "CE");
  EXPECT_TRUE(
      result_json["routes"][0]["legs"][0]["steps"][1]["intersections"][0].HasMember("classes"));
  EXPECT_EQ(result_json["routes"][0]["legs"][0]["steps"][1]["intersections"][0]["classes"][0],
            "motorway");
}
