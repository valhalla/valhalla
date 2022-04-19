#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, Maxspeed) {
  const std::string ascii_map = R"(
      A----B----C----D----E----F
                               |
                               G
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"maxspeed", "50"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "60mph"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", ""}}},
      {"DE", {{"highway", "motorway"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "none"}}},
      {"FG", {{"highway", "motorway"}, {"maxspeed", "40"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxspeed");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto",
                                 {{"/filters/action", "include"},
                                  {"/filters/attributes/0", "shape_attributes.speed_limit"}});

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  EXPECT_EQ(leg.node(0).edge().speed_limit(), 50);  // AB
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 97);  // BC
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 0);   // CD
  EXPECT_EQ(leg.node(3).edge().speed_limit(), 0);   // DE
  EXPECT_EQ(leg.node(4).edge().speed_limit(), 255); // EF
  EXPECT_EQ(leg.node(5).edge().speed_limit(), 40);  // FG

  // Test osrm output
  //
  // "maxspeed": [
  //  {"speed":50,"unit":"km\/h"},
  //  {"speed":97,"unit":"km\/h"},
  //  {"unknown":true},
  //  {"unknown":true},
  //  {"none":true},
  //  {"speed":40,"unit":"km\/h"}
  // ]
  auto d = gurka::convert_to_json(result, valhalla::Options_Format_osrm);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"].Size(), 6);
  EXPECT_EQ(d["routes"][0]["legs"][0]["steps"].Size(), 3);

  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][0]["speed"].GetInt(), 50);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][0]["unit"].GetString(), "km/h");
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][1]["speed"].GetInt(), 97);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][1]["unit"].GetString(), "km/h");
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][2]["unknown"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][3]["unknown"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][4]["none"].GetBool(), true);
  EXPECT_EQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][5]["speed"].GetInt(), 40);
  EXPECT_STREQ(d["routes"][0]["legs"][0]["annotation"]["maxspeed"][5]["unit"].GetString(), "km/h");

  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][0]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][0]["speedLimitUnit"].GetString(), "km/h");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][1]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][1]["speedLimitUnit"].GetString(), "km/h");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][2]["speedLimitSign"].GetString(), "vienna");
  EXPECT_STREQ(d["routes"][0]["legs"][0]["steps"][2]["speedLimitUnit"].GetString(), "km/h");
}
