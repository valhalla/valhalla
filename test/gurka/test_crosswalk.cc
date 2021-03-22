#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(TestSuite, TestName) {

  const std::string& ascii_map = R"(
        A----B----E
             |
             |
             C
             |
             |
             D
    )";
  const gurka::ways ways = {{"AB", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
                            {"BC", {{"name", ""}, {"highway", "footway"}, {"footway", "crossing"}}},
                            {"CD", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
                            {"BE", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/crosswalk");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  gurka::assert::raw::expect_path(result, {"", "", ""});

  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kPedestrianCrossingUse);

  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Turn right onto the crosswalk.",
                                                            "Turn right onto the crosswalk.",
                                                            "Turn right onto the crosswalk.",
                                                            "Continue for 300 meters.");
}
