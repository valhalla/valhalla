#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

//=======================================================================================
class ForkTest : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
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
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> ForkTest::reader;
gurka::map ForkTest::map = {};

//------------------------------------------------------------------
TEST_F(ForkTest, test) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");

  // TODO: should not announce stay left since we detect a deceleration lane
  for (const auto& leg : result.directions().routes(0).legs()) {
    for (const auto& maneuver : leg.maneuver()) {
      std::cout << maneuver.text_instruction() << "\n";
    }
  }
}

//=======================================================================================
class ForkTest2 : public ::testing::Test {
protected:
  static gurka::map map;
  static std::shared_ptr<baldr::GraphReader> reader;

  static void SetUpTestSuite() {
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
           {"name", "D4"},
           {"maxspeed", "120"}}},
         // BC is three lanes because it "grew" a deceleration lane.  length = 200 m
         {"BC",
          {{"highway", "motorway"},
           {"oneway", "yes"},
           {"lanes", "3"},
           {"name", "D4"},
           {"maxspeed", "120"}}},
         // now back to two lanes - but wait this is a different highway. ramp forms a small angle ~15
         // deg
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
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/ForkTest", {});
    reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  }
};

std::shared_ptr<baldr::GraphReader> ForkTest2::reader;
gurka::map ForkTest2::map = {};

//------------------------------------------------------------------
TEST_F(ForkTest2, test) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  // TODO: since the "ramp" goes straight, announce the "stay right"
  for (const auto& leg : result.directions().routes(0).legs()) {
    for (const auto& maneuver : leg.maneuver()) {
      std::cout << maneuver.text_instruction() << "\n";
    }
  }
}
