#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//------------------------------------------------------------------
// We detect the deceleration lane and do not call out "Keep left to stay on A92."
//------------------------------------------------------------------
TEST(ForkTest, StandardDecelerationLaneDontCallKeepLeft) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    A-------B--------------------C----------------------------------------------------------D
                                        E
                                          F
  )";

  const gurka::ways ways =
      {// we start out with two lanes
       {"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "A92"},
         {"maxspeed", "120"}}},
       // BC is three lanes because it "grew" a deceleration lane.  length = 200 m
       {"BC",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "A92"},
         {"maxspeed", "120"}}},
       // now back to two lanes
       {"CD",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "A92"},
         {"maxspeed", "120"}}},
       // this ramp forms a small angle ~15 deg
       {"CE",
        {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "A92 link"}}},
       {"EF",
        {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "A92 link"}}}};

  const gurka::nodes nodes;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 2);

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, 0, "Drive east on A92.",
      "Drive east. Then, in 900 meters, You will arrive at your destination.", "",
      "Drive east on A92. Then, in 900 meters, You will arrive at your destination.",
      "Continue for 900 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

//------------------------------------------------------------------
// We detect the deceleration lane and do not call out "Keep right to stay on M1."
//------------------------------------------------------------------
TEST(ForkTest, StandardDecelerationLaneDontCallKeepRight) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
                                          F
                                        E
    A-------B--------------------C----------------------------------------------------------D
  )";

  const gurka::ways ways =
      {// we start out with two lanes
       {"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "M1"},
         {"driving_side", "left"},
         {"maxspeed", "120"}}},
       // BC is three lanes because it "grew" a deceleration lane.  length = 200 m
       {"BC",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "M1"},
         {"driving_side", "left"},
         {"maxspeed", "120"}}},
       // now back to two lanes
       {"CD",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "M1"},
         {"driving_side", "left"},
         {"maxspeed", "120"}}},
       // this ramp forms a small angle ~15 deg
       {"CE",
        {{"highway", "motorway_link"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D5"},
         {"driving_side", "left"}}},
       {"EF",
        {{"highway", "motorway_link"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D5"},
         {"driving_side", "left"}}}};

  const gurka::nodes nodes;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 2);

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, 0, "Drive east on M1.",
      "Drive east. Then, in 900 meters, You will arrive at your destination.", "",
      "Drive east on M1. Then, in 900 meters, You will arrive at your destination.",
      "Continue for 900 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

//------------------------------------------------------------------
// The deceleration lane is too short. Have to call out "Keep right".
//------------------------------------------------------------------
TEST(ForkTest, DecelerationLaneTooShort) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    A---------------------------B------------C----------------------------------------------------------D
                                                    E
                                                       F
  )";

  const gurka::ways ways =
      {// we start out with two lanes
       {"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // BC just "grew" a third lane.  length = 130 m.
       // This is too short to be considered a standard deceleration lane.
       {"BC",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // now back to two lanes - but wait this is a different highway.
       // turn angle is ~15 deg.
       {"CD",
        {{"highway", "motorway_link"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D2"},
         {"maxspeed", "120"}}},
       // this is the continuation of D4
       {"CE", {{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D4"}}},
       {"EF", {{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D4"}}}};

  const gurka::nodes nodes;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);

  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, 0, "Drive east on D4.",
                                            "Drive east. Then Keep right to stay on D4.", "",
                                            "Drive east on D4. Then Keep right to stay on D4.",
                                            "Continue for 400 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, 1, "Keep right to stay on D4.", "", "Keep right to stay on D4.",
      "Keep right to stay on D4. Then You will arrive at your destination.",
      "Continue for 100 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 2,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

//------------------------------------------------------------------
// The deceleration lane is too long (so not really a deceleration lane).
// Have to call out "Keep right".
//------------------------------------------------------------------
TEST(ForkTest, DecelerationLaneTooLong) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    Z------------A-----------------B-----------------C----------------D
                                                            E
                                                               F
  )";

  const gurka::ways ways =
      {// we start out with two lanes
       {"ZA",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // grow a lane
       {"AB",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // AB & BC combined are too long to be considered a deceleration lane
       {"BC",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // now back to two lanes - but wait this is a different highway.
       // turn angle is ~15 deg.
       {"CD",
        {{"highway", "motorway_link"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D2"},
         {"maxspeed", "120"}}},
       // this is the continuation of D4
       {"CE", {{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D4"}}},
       {"EF", {{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D4"}}}};

  const gurka::nodes nodes;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);

  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, 0, "Drive east on D4.",
                                            "Drive east. Then Keep right to stay on D4.", "",
                                            "Drive east on D4. Then Keep right to stay on D4.",
                                            "Continue for 400 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(
      result, 1, "Keep right to stay on D4.", "", "Keep right to stay on D4.",
      "Keep right to stay on D4. Then You will arrive at your destination.",
      "Continue for 100 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 2,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

//------------------------------------------------------------------
// The route starts on what might be a deceleration lane. However,
// the deceleration-lane-detection logic cannot walk backwards far
// enough to make the determination. Therefore the logic calls out
// the intermediate maneuver to "keep left...".
//------------------------------------------------------------------
TEST(ForkTest, RouteStartsOnPossibleDecelLane) {
  constexpr double gridsize = 10;

  const std::string ascii_map = R"(
    B------------------C----------------------------------------------------------D
                              E
                                 F
  )";

  const gurka::ways ways =
      {// BC is three lanes. it is long enough to be considered a deceleration
       // lane, but the route starts at B so we cannot assess with certainty
       // that it is one.
       {"BC",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       // now back to two lanes - but wait this is a different highway.
       // turn angle is ~15 deg.
       {"CD",
        {{"highway", "motorway"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"name", "D4"},
         {"maxspeed", "120"}}},
       {"CE", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D5"}}},
       {"EF", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}, {"name", "D5"}}}};

  const gurka::nodes nodes;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
  std::shared_ptr<baldr::GraphReader> reader =
      test::make_clean_graphreader(map.config.get_child("mjolnir"));

  auto result = gurka::do_action(valhalla::Options::route, map, {"B", "D"}, "auto");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs(0).maneuver_size(), 3);

  gurka::assert::raw::
      expect_instructions_at_maneuver_index(result, 0, "Drive east on D4.",
                                            "Drive east. Then Keep left to stay on D4.", "",
                                            "Drive east on D4. Then Keep left to stay on D4.",
                                            "Continue for 200 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1, "Keep left to stay on D4.", "",
                                                            "Keep left to stay on D4.",
                                                            "Keep left to stay on D4.",
                                                            "Continue for 600 meters.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 2,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}
