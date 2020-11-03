#include "gurka.h"
#include <gtest/gtest.h>

#include "odin/enhancedtrippath.h"

using namespace valhalla;

void validate_turn_lanes(valhalla::Api& result,
                         std::vector<std::pair<int, std::string>> expected_lanes) {
  odin::EnhancedTripLeg etl(*result.mutable_trip()->mutable_routes(0)->mutable_legs(0));

  ASSERT_EQ(expected_lanes.size(), etl.node_size() - 1);

  for (size_t i = 1; i < etl.node_size(); ++i) {
    auto prev_edge = etl.GetPrevEdge(i);
    ASSERT_TRUE(prev_edge) << "Expected prev_edge on node index " << i;
    ASSERT_EQ(prev_edge->turn_lanes_size(), expected_lanes[i - 1].first)
        << "Incorrect lane count on node index " << i;
    ASSERT_EQ(prev_edge->TurnLanesToString(), expected_lanes[i - 1].second)
        << "Incorrect turn:lanes on node index " << i;
  }
}

/*************************************************************/
TEST(Standalone, TurnLanes) {

  const std::string ascii_map = R"(
    A----B----C
         |
         D)";

  const gurka::ways ways =
      {{"ABC", {{"highway", "primary"}, {"lanes", "4"}, {"turn:lanes", "left|||right"}}},
       {"DB",
        {{"highway", "primary"},
         {"lanes:forward", "3"},
         {"lanes:backward", "0"},
         {"turn:lanes", "left|none|none"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_1");

  auto result = gurka::route(map, "D", "C", "auto");

  gurka::assert::osrm::expect_steps(result, {"DB", "ABC"});
  gurka::assert::raw::expect_path(result, {"DB", "ABC"});

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  gurka::assert::raw::expect_path_length(result, 0.7, 0.001);

  validate_turn_lanes(result, {
                                  {3, "[ left | through | through;right ACTIVE ]"},
                                  {4, "[ left | through | through | right ]"},
                              });

  result = gurka::route(map, "A", "D", "auto");
  validate_turn_lanes(result, {
                                  {4, "[ left | through | through | right ACTIVE ]"},
                                  {0, ""},
                              });

  result = gurka::route(map, "C", "D", "auto");
  validate_turn_lanes(result, {
                                  {4, "[ left ACTIVE | through | through | right ]"},
                                  {0, ""},
                              });
}

// Split lane example - 5-way intersection
// (40.856684, -73.869336)
TEST(Standalone, TurnLanesSplitLane) {

  const std::string ascii_map = R"(
        C
        |
        |  D--E
    A---B<
        |  F--G
        |
        H
  )";

  const gurka::ways ways = {{"AB",
                             {{"highway", "trunk"},
                              {"name", "Bronx and Pelham Parkway"},
                              {"oneway", "yes"},
                              {"lanes", "3"},
                              {"turn:lanes", "left|through;left|through;right"}}},
                            {"CBH", {{"highway", "primary"}, {"name", "Boston Road"}}},
                            {"BDE",
                             {{"highway", "trunk"},
                              {"name", "Bronx and Pelham Parkway"},
                              {"oneway", "yes"},
                              {"lanes", "3"},
                              {"turn:lanes", "through|through|through"}}},
                            {"BFG",
                             {{"highway", "tertiary"},
                              {"name", "Pelham Parkway South"},
                              {"oneway", "yes"},
                              {"lanes", "1"},
                              {"turn:lanes", "through"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_2");
  valhalla::Api result;

  // A -> C - takes the leftmost left lane
  result = gurka::route(map, "A", "C", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left ACTIVE | left;through | through;right ]"},
                                  {0, ""},
                              });

  // A -> E - takes the leftmost through lane
  result = gurka::route(map, "A", "E", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left | left;through ACTIVE | through;right ]"},
                                  {0, ""}, // TODO lanes are tossed when all are through
                              });

  // A -> G - takes the rightmost through lane
  result = gurka::route(map, "A", "G", "auto");
  gurka::assert::osrm::expect_steps(result, {"Bronx and Pelham Parkway", "Pelham Parkway South"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {3, "[ left | left;through | through;right ACTIVE ]"},
                                  {0, ""}, // TODO lanes are tossed when all are through
                              });

  // A -> H - takes the right lane
  result = gurka::route(map, "A", "H", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left | left;through | through;right ACTIVE ]"},
                                  {0, ""},
                              });
}

// Shared turn lane example
// (40.053550, -76.309125)
TEST(Standalone, TurnLanesSharedTurnLane) {
  const std::string ascii_map = R"(
    A---B---D---E---G
        |       |
        C       F
  )";

  const gurka::ways ways = {{"AB",
                             {{"highway", "primary"},
                              {"lanes:forward", "3"},
                              {"turn:lanes:forward", "through|through|right"}}},
                            {"BC", {{"highway", "residential"}}},
                            {"BD",
                             {{"highway", "primary"},
                              {"lanes:forward", "3"},
                              {"turn:lanes:forward", "through|through|right"}}},
                            {"DE",
                             {{"highway", "primary"},
                              {"lanes:forward", "3"},
                              {"turn:lanes:forward", "through|through|right"}}},
                            {"EF", {{"highway", "residential"}}},
                            {"EG",
                             {{"highway", "primary"},
                              {"lanes:forward", "2"},
                              {"turn:lanes:forward", "through|through"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_3");

  // A -> G  - only through lanes should be active throughout
  auto result = gurka::route(map, "A", "G", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ through ACTIVE | through | right ]"},
                                  {3, "[ through ACTIVE | through | right ]"},
                                  {3, "[ through ACTIVE | through | right ]"},
                                  {0, ""},
                              });

  // A -> C - right lane should always be active
  result = gurka::route(map, "A", "C", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ through | through | right ACTIVE ]"},
                                  {0, ""},
                              });

  // A -> F - only right lane after B should be active, before that rightmost right lane should be
  // active
  result = gurka::route(map, "A", "F", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ through | through ACTIVE | right ]"},
                                  {3, "[ through | through | right ACTIVE ]"},
                                  {3, "[ through | through | right ACTIVE ]"},
                                  {0, ""},
                              });
}

// Multiple turn lanes with short arrival
TEST(Standalone, TurnLanesMultiLaneShort) {
  const std::string ascii_map = R"(
          1 E 2
            |
            |
    A---B---C---D
            |
            |
          3 F 4
            |
            |
            G
  )";

  const gurka::ways ways = {{"AB",
                             {{"highway", "primary"},
                              {"lanes:forward", "5"},
                              {"turn:lanes:forward", "left|left|through|right|right"}}},
                            {"BC",
                             {{"highway", "primary"},
                              {"lanes:forward", "5"},
                              {"turn:lanes:forward", "left|left|through|right|right"}}},
                            {"CD", {{"highway", "trunk"}}},
                            {"ECFG", {{"highway", "trunk"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_4");

  valhalla::Api result;

  // TODO - side of street not being set correctly
  // // A -> 1 - takes leftmost left lane
  // result = gurka::route(map, "A", "1", "auto");
  // gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
  //                                               DirectionsLeg_Maneuver_Type_kLeft,
  //                                               DirectionsLeg_Maneuver_Type_kDestinationLeft});
  // validate_turn_lanes(result, {
  //                                 {5, "[ left ACTIVE | left | through | right | right ]"},
  //                                 {5, "[ left ACTIVE | left | through | right | right ]"},
  //                                 {0, ""},
  //                             });

  // // A -> 2 - takes rightmost left lane
  // result = gurka::route(map, "A", "2", "auto");
  // gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
  //                                               DirectionsLeg_Maneuver_Type_kLeft,
  //                                               DirectionsLeg_Maneuver_Type_kDestinationRight});
  // validate_turn_lanes(result, {
  //                                 {5, "[ left | left ACTIVE | through | right | right ]"},
  //                                 {5, "[ left | left ACTIVE | through | right | right ]"},
  //                                 {0, ""},
  //                             });

  // A -> E - takes leftmost left lane (left side driving)
  result = gurka::route(map, "A", "E", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ left ACTIVE | left | through | right | right ]"},
                                  {5, "[ left ACTIVE | left | through | right | right ]"},
                                  {0, ""},
                              });

  // A -> 3 - takes rightmost right lane
  result = gurka::route(map, "A", "3", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | right | right ACTIVE ]"},
                                  {5, "[ left | left | through | right | right ACTIVE ]"},
                                  {0, ""},
                              });

  // A -> 4 - takes leftmost right lane
  result = gurka::route(map, "A", "4", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | right ACTIVE | right ]"},
                                  {5, "[ left | left | through | right ACTIVE | right ]"},
                                  {0, ""},
                              });

  // A -> F - takes leftmost right lane (left side driving)
  result = gurka::route(map, "A", "F", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | right ACTIVE | right ]"},
                                  {5, "[ left | left | through | right ACTIVE | right ]"},
                                  {0, ""},
                              });

  // A -> G - takes both right lanes as destination is far away from previous transition
  result = gurka::route(map, "A", "G", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | right ACTIVE | right ACTIVE ]"},
                                  {5, "[ left | left | through | right ACTIVE | right ACTIVE ]"},
                                  {0, ""},
                              });
}
