#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{
    {"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"}}};

class ViaWaypoints : public testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;
    const std::string ascii_map = R"(
    A-B-C-D-E
        |
        1
        F---2G
        |
        |	
        H---3I
    )";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
                              {"CF", {{"highway", "primary"}}}, {"FH", {{"highway", "primary"}}},
                              {"FG", {{"highway", "primary"}}}, {"HI", {{"highway", "primary"}}}};

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_via_waypoints", build_config);
  }
};
gurka::map ViaWaypoints::map = {};
Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(ViaWaypoints, test_via_waypoints_response1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "1", "E"}, "auto",
                                 {{"/locations/0/type", "break_through"},
                                  {"/locations/1/type", "through"},
                                  {"/locations/2/type", "break_through"}});
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FG", "FG", "CF", "CD", "DE"});

  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 1);

  // Expect via waypoint array at leg level
  auto leg = d["routes"][0]["legs"][0].GetObject();
  EXPECT_TRUE(leg.HasMember("via_waypoints"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("waypoint_index"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("geometry_index"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("distance_from_leg_start"));
}

TEST_F(ViaWaypoints, test_via_waypoints_response2) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "2", "3", "E"}, "auto",
                                 {{"/locations/0/type", "break_through"},
                                  {"/locations/1/type", "through"},
                                  {"/locations/2/type", "through"},
                                  {"/locations/3/type", "break_through"}});
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

  gurka::assert::raw::expect_path(result, {"AB", "BC", "CF", "FG", "FG", "FH", "HI", "HI", "FH", "CF",
                                           "CD", "DE"});

  ASSERT_EQ(d["routes"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"].Size(), 1);
  ASSERT_EQ(d["routes"][0]["legs"][0]["via_waypoints"].Size(), 2);

  // Expect via waypoint array at leg level
  auto leg = d["routes"][0]["legs"][0].GetObject();
  EXPECT_TRUE(leg.HasMember("via_waypoints"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("waypoint_index"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("geometry_index"));
  EXPECT_TRUE(leg["via_waypoints"][0].HasMember("distance_from_leg_start"));

  EXPECT_TRUE(leg.HasMember("via_waypoints"));
  EXPECT_TRUE(leg["via_waypoints"][1].HasMember("waypoint_index"));
  EXPECT_TRUE(leg["via_waypoints"][1].HasMember("geometry_index"));
  EXPECT_TRUE(leg["via_waypoints"][1].HasMember("distance_from_leg_start"));
}
