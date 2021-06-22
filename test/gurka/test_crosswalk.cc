#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Crosswalk, CrosswalkInstructions) {

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
                                                            "Turn right.",
                                                            "Turn right onto the crosswalk.",
                                                            "Turn right onto the crosswalk.",
                                                            "Continue for 300 meters.");
}

// This test ensures that we don't call out crosswalks if there is a straight path.
TEST(Crosswalk, StraightRoute) {

  const std::string& ascii_map = R"(
        A----B----C----D----E-----F
             |    |    |
             |    |    |
             G    I    H

    )";
  const gurka::ways ways = {
      {"AB", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
      {"BC", {{"name", ""}, {"highway", "footway"}, {"footway", "crossing"}}},
      {"CD", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
      {"DE", {{"name", ""}, {"highway", "footway"}, {"footway", "crossing"}}},
      {"EF", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
      {"BG", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
      {"CI", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
      {"DI", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/crosswalk");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "pedestrian");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);

  // verify there are only two maneuvers (start and end)
  auto legs = result.directions().routes(0).legs();
  EXPECT_EQ(legs.Get(0).maneuver().size(), 2);

  // Verify instructions for start and end maneuvers
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 0, "Walk east on the walkway.",
                                                            "Walk east.", "",
                                                            "Walk east on the walkway.",
                                                            "Continue for 1.5 kilometers.");

  gurka::assert::raw::expect_instructions_at_maneuver_index(result, 1,
                                                            "You have arrived at your destination.",
                                                            "",
                                                            "You will arrive at your destination.",
                                                            "You have arrived at your destination.",
                                                            "");
}

// This test ensures we don't call out crosswalks when turning from a non-footway.
TEST(Crosswalk, TransitionFromNonFootways) {

  const std::string& ascii_map = R"(
        A----B----E---F
             |
             |
             C
    )";
  const gurka::ways ways = {{"AB", {{"name", ""}, {"highway", "footway"}, {"footway", "crossing"}}},
                            {"CB", {{"name", ""}, {"highway", "secondary"}}},
                            {"BE", {{"name", ""}, {"highway", "footway"}, {"footway", "crossing"}}},
                            {"EF", {{"name", ""}, {"highway", "footway"}, {"footway", "sidewalk"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/crosswalk");
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "pedestrian");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  // Ensure the first right turn edge is a crosswalk
  EXPECT_EQ(leg.node(1).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kPedestrianCrossingUse);

  // verify there are only three maneuvers (start, turn and end)
  auto legs = result.directions().routes(0).legs();
  EXPECT_EQ(legs.Get(0).maneuver().size(), 3);

  // Ensure the first right turn instruction does not call out crosswalk even though the edge use is
  // TripLeg_Use_kPedestrianCrossingUse
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Turn right onto the walkway.",
                                                            "Turn right.",
                                                            "Turn right onto the walkway.",
                                                            "Turn right onto the walkway.",
                                                            "Continue for 500 meters.");
}
