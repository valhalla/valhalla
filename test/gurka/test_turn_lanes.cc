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

  auto result = gurka::do_action(valhalla::Options::route, map, {"D", "C"}, "auto");

  gurka::assert::osrm::expect_steps(result, {"DB", "ABC"});
  gurka::assert::raw::expect_path(result, {"DB", "ABC"});

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  gurka::assert::raw::expect_path_length(result, 0.7, 0.001);

  validate_turn_lanes(result, {
                                  {3, "[ left | through | through;*right* ACTIVE ]"},
                                  {4, "[ left | through | through | right ]"},
                              });

  result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  validate_turn_lanes(result, {
                                  {4, "[ left | through | through | *right* ACTIVE ]"},
                                  {0, ""},
                              });

  result = gurka::do_action(valhalla::Options::route, map, {"C", "D"}, "auto");
  validate_turn_lanes(result, {
                                  {0, ""},
                                  {0, ""},
                              });

  result = gurka::do_action(valhalla::Options::route, map, {"D", "C"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left | through | through;*right* ACTIVE ]"},
                                  {4, "[ left | through | through | right ]"},
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
  result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ *left* ACTIVE | *left*;through VALID | through;right ]"},
                                  {0, ""},
                              });

  // A -> E - takes the leftmost through lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left | left;*through* ACTIVE | *through*;right VALID ]"},
                                  {0, ""}, // TODO lanes are tossed when all are through
                              });

  // A -> G - takes the rightmost through lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"Bronx and Pelham Parkway", "Pelham Parkway South"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {3, "[ left | left;through | through;*right* ACTIVE ]"},
                                  {0, ""}, // TODO lanes are tossed when all are through
                              });

  // A -> H - takes the right lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ left | left;through | through;*right* ACTIVE ]"},
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
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ *through* ACTIVE | *through* VALID | right ]"},
                                  {3, "[ *through* ACTIVE | *through* VALID | right ]"},
                                  {3, "[ *through* ACTIVE | *through* VALID | right ]"},
                                  {0, ""},
                              });

  // A -> C - right lane should always be active
  result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ through | through | *right* ACTIVE ]"},
                                  {0, ""},
                              });

  // A -> F - only right lane after B should be active, before that rightmost right lane should be
  // active
  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  validate_turn_lanes(result, {
                                  {3, "[ *through* VALID | *through* ACTIVE | right ]"},
                                  {3, "[ through | through | *right* ACTIVE ]"},
                                  {3, "[ through | through | *right* ACTIVE ]"},
                                  {0, ""},
                              });
}

// Multiple turn lanes with short arrival
TEST(Standalone, TurnLanesMultiLaneShort) {
  const std::string ascii_map = R"(
            E
            |
       1    |    2
            |
            |
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
                              {"driving_side", "right"},
                              {"turn:lanes:forward", "left|left|through|right|right"}}},
                            {"BC",
                             {{"highway", "primary"},
                              {"lanes:forward", "5"},
                              {"driving_side", "right"},
                              {"turn:lanes:forward", "left|left|through|right|right"}}},
                            {"CD", {{"highway", "trunk"}, {"driving_side", "right"}}},
                            {"ECFG", {{"highway", "trunk"}, {"driving_side", "right"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_4",
                               {{"mjolnir.data_processing.use_admin_db", "false"}});

  valhalla::Api result;

  // TODO - side of street not being set correctly
  // // A -> 1 - takes leftmost left lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "1"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
  validate_turn_lanes(result, {
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {0, ""},
                              });

  // // A -> 2 - takes rightmost left lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "2"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
  validate_turn_lanes(result, {
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {0, ""},
                              });

  // A -> E - takes leftmost left lane (left side driving)
  result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {5, "[ *left* ACTIVE | *left* ACTIVE | through | right | right ]"},
                                  {0, ""},
                              });

  // A -> 3 - takes rightmost right lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "3"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | *right* VALID | *right* ACTIVE ]"},
                                  {5, "[ left | left | through | *right* VALID | *right* ACTIVE ]"},
                                  {0, ""},
                              });

  // A -> 4 - takes leftmost right lane
  result = gurka::do_action(valhalla::Options::route, map, {"A", "4"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | *right* ACTIVE | *right* VALID ]"},
                                  {5, "[ left | left | through | *right* ACTIVE | *right* VALID ]"},
                                  {0, ""},
                              });

  // makes the data left side driving.
  map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_5",
                          {{"mjolnir.data_processing.use_admin_db", "true"}});

  // A -> F - takes leftmost right lane (left side driving)
  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | *right* ACTIVE | *right* VALID ]"},
                                  {5, "[ left | left | through | *right* ACTIVE | *right* VALID ]"},
                                  {0, ""},
                              });

  // A -> G - takes both right lanes as destination is far away from previous transition
  result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ left | left | through | *right* ACTIVE | *right* ACTIVE ]"},
                                  {5, "[ left | left | through | *right* ACTIVE | *right* ACTIVE ]"},
                                  {0, ""},
                              });
}

TEST(Standalone, TurnLanesUTurns) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
              E D
              | |
              | |
              | |
              | |
         L----F-C----K
              | |
         I----G-B----J
              | |
              | |
              | |
              | |
              H A
    )";

  const gurka::ways ways =
      {{"AB",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "5"},
         {"turn:lanes", "left|left|through|right|right"}}},
       {"EF",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "4"},
         {"turn:lanes", "reverse|left|through|right"}}},
       {"BC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"CD", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"FG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"GH", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"IG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"GB", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"BJ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"KC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"CF", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"FL", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_6");

  valhalla::Api result;

  // Test U-turn using left-only lanes. The left-most left lane should be
  // active & rest invalid
  result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ *left* ACTIVE | left | through | right | right ]"},
                                  {5, "[ *left* ACTIVE | left | through | right | right ]"},
                                  {0, ""},
                                  {0, ""},
                                  {0, ""},
                              });

  // Test using reverse lanes. The reverse lane should be active & the rest
  // invalid
  result = gurka::do_action(valhalla::Options::route, map, {"E", "D"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {4, "[ *reverse* ACTIVE | left | through | right ]"},
                                  {4, "[ *reverse* ACTIVE | left | through | right ]"},
                                  {0, ""},
                                  {0, ""},
                                  {0, ""},
                              });
}

TEST(Standalone, TurnLanesInternalsTurnChannels) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
              E D
              | |
              | |
              | |
              | |
         L----F-C-----K
              | |
         I----G-B--N--J
              | | /
              | |/
              | M
              | |
              H A
    )";

  const gurka::ways ways =
      {{"AM",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "5"},
         {"turn:lanes", "through|through|right"}}},
       {"MB",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "5"},
         {"internal_intersection", "true"},
         {"turn:lanes", "left|through|through"}}},
       {"EF",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "4"},
         {"turn:lanes", "reverse|left|through|right"}}},
       {"BC",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"internal_intersection", "true"},
         {"name", "Broken Land Parkway"}}},
       {"CD", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"FG",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"internal_intersection", "true"},
         {"name", "Broken Land Parkway"}}},
       {"GH", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"IG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"GB",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"internal_intersection", "true"},
         {"name", "Snowden River Parkway"}}},
       {"BN",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"internal_intersection", "true"},
         {"name", "Snowden River Parkway"}}},
       {"NJ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"KC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"CF",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"internal_intersection", "true"},
         {"name", "Snowden River Parkway"}}},
       {"FL", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_7",
                               {{"mjolnir.data_processing.infer_internal_intersections", "false"}});

  valhalla::Api result;
  result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {3, "[ *through* ACTIVE | *through* VALID | right ]"},
                                  {3, "[ *left* ACTIVE | through | through ]"},
                                  {3, "[ *left* ACTIVE | through | through ]"},
                                  {0, ""},
                                  {0, ""},
                              });

  map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_8",
                          {{"mjolnir.data_processing.infer_internal_intersections", "true"}});

  result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {3, "[ *through* ACTIVE | *through* VALID | right ]"},
                                  {3, "[ *left* ACTIVE | through | through ]"},
                                  {3, "[ *left* ACTIVE | through | through ]"},
                                  {0, ""},
                                  {0, ""},
                              });
}

TEST(Standalone, TurnLanesInternalsAndRestrictions) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
                   E D
                   | |
                   | |
         N    P    | |
         |    |    | |
    Q----M----L----F-C----K
         |    |    | |
    R----O----I----G-B----J
                   | |
                   | |
                   | |
                   | |
                   H A
    )";

  const gurka::ways ways =
      {{"ABC",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "5"},
         {"turn:lanes", "left|left|through|right|right"}}},
       {"EF",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Broken Land Parkway"},
         {"lanes", "4"},
         {"turn:lanes", "reverse|left|through|right"}}},
       {"CD", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"FG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"GH", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Broken Land Parkway"}}},
       {"FL",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Patuxent Woods Drive"},
         {"lanes", "3"},
         {"turn:lanes", "||right"}}},
       {"LM",
        {{"highway", "primary"},
         {"oneway", "yes"},
         {"name", "Patuxent Woods Drive"},
         {"lanes", "3"},
         {"turn:lanes", "||right"}}},
       {"MQ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"RO", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"OI", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"IG", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Patuxent Woods Drive"}}},
       {"LI", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Small Internal Street"}}},
       {"MO", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Small Internal Street"}}},
       {"LP", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "First Street"}}},
       {"MN", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Second Street"}}},
       {"GB", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"BJ", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"KC", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}},
       {"CF", {{"highway", "primary"}, {"oneway", "yes"}, {"name", "Snowden River Parkway"}}}};

  const gurka::relations relations = {
      {{
           {gurka::way_member, "CF", "from"},
           {gurka::way_member, "GB", "to"},
           {gurka::way_member, "FG", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
      {{
           {gurka::way_member, "FL", "from"},
           {gurka::way_member, "IG", "to"},
           {gurka::way_member, "LI", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
  };

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_turn_lanes_9");

  valhalla::Api result;

  // Test U-turn using left-only lanes. The left-most left lane should be
  // active & rest invalid
  result = gurka::do_action(valhalla::Options::route, map, {"A", "H"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {5, "[ *left* ACTIVE | left | through | right | right ]"},
                                  {5, "[ *left* ACTIVE | left | through | right | right ]"},
                                  {0, ""},
                                  {0, ""},
                                  {0, ""},
                              });

  // Test using reverse lanes. The reverse lane should be active & the rest
  // invalid
  result = gurka::do_action(valhalla::Options::route, map, {"E", "D"}, "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result, {
                                  {4, "[ *reverse* ACTIVE | left | through | right ]"},
                                  {4, "[ *reverse* ACTIVE | left | through | right ]"},
                                  {0, ""},
                                  {5, "[ left | left | *through* ACTIVE | right | right ]"},
                                  {0, ""},
                              });

  result = gurka::do_action(valhalla::Options::route, map, {"K", "J"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kUturnLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result,
                      {
                          {0, ""},
                          {0, ""},
                          {3, "[ *through* ACTIVE | *through* VALID | right ]"}, // restriction does
                                                                                 // not allow a left
                                                                                 // to be added.
                          {3, "[ *left*;through ACTIVE | through | right ]"}, // no restriction allows
                                                                              // a left to be added.
                          {0, ""},
                          {0, ""},
                          {0, ""},
                          {0, ""},
                          {0, ""},
                      });
}

TEST(Standalone, TurnLanesSerializedResponse) {
  const std::string ascii_map = R"(
        C   J
        |   |
    A---B G I
        | | |
        D-E-H
        |
        F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "trunk"}, {"lanes", "3"}, {"turn:lanes", "left|right|right"}}},
      {"BC",
       {{"highway", "primary"},
        {"oneway", "-1"},
        {"lanes", "3"},
        {"turn:lanes:backward", "reverse;left|left;through|through"}}},
      {"BD",
       {{"highway", "primary"},
        {"oneway", "yes"},
        {"lanes", "3"},
        // TODO add reverse;left and show its not handled
        {"turn:lanes:forward", "reverse;left|left;through|through"}}},
      {"DE", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"EG", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DF",
       {{"highway", "primary"},
        {"oneway", "yes"},
        {"lanes", "2"},
        {"turn:lanes", "through|through"}}},
      {"JI",
       {{"highway", "trunk"},
        {"oneway", "yes"},
        {"lanes", "2"},
        {"turn:lanes:forward", "|reverse;right"}}},
      {"IH",
       {{"highway", "trunk"},
        {"oneway", "yes"},
        {"lanes", "2"},
        {"turn:lanes:forward", "|reverse;right"}}},
      {"EH", {{"highway", "primary"}, {"oneway", "-1"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_10");

  auto verify_lane_schema = [](const rapidjson::Value& lanes) -> void {
    // Verify common lane attributes:
    // - has valid & active booleans
    // - has a non-empty indications array
    for (const auto& lane : lanes.GetArray()) {
      ASSERT_TRUE(lane.HasMember("valid"));
      EXPECT_TRUE(lane["valid"].IsBool());
      ASSERT_TRUE(lane.HasMember("active"));
      EXPECT_TRUE(lane["active"].IsBool());
      ASSERT_TRUE(lane.HasMember("indications"));
      ASSERT_TRUE(lane["indications"].IsArray());
      ASSERT_GT(lane["indications"].Size(), 0);
    }
  };

  {
    // A->E : Both right lanes should be valid and left-most right lane should
    // be active. For following step, both left lanes should be valid and the
    // left-most left lane should be active.
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");
    rapidjson::Document directions = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

    // Assert expected number of routes, legs, steps
    ASSERT_EQ(directions["routes"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"][0]["steps"].Size(), 4);
    const rapidjson::Value& steps = directions["routes"][0]["legs"][0]["steps"];

    {
      // Validate lane object of second step
      const rapidjson::Value& step1_int0 = steps[1]["intersections"][0];
      ASSERT_TRUE(step1_int0.HasMember("lanes"));
      ASSERT_EQ(step1_int0["lanes"].Size(), 3);

      verify_lane_schema(step1_int0["lanes"]);
      // lane 0 - active: false, valid: false
      EXPECT_FALSE(step1_int0["lanes"][0]["active"].GetBool());
      EXPECT_FALSE(step1_int0["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][0]["indications"].Size(), 1);
      EXPECT_EQ(step1_int0["lanes"][0]["indications"][0].GetString(), std::string("left"));
      EXPECT_FALSE(step1_int0["lanes"][0].HasMember("valid_indication"));
      // lane 1 - active: true, valid: true, active_ind: right
      EXPECT_TRUE(step1_int0["lanes"][1]["active"].GetBool());
      EXPECT_TRUE(step1_int0["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][1]["indications"].Size(), 1);
      EXPECT_EQ(step1_int0["lanes"][1]["indications"][0].GetString(), std::string("right"));
      ASSERT_TRUE(step1_int0["lanes"][1].HasMember("valid_indication"));
      EXPECT_EQ(step1_int0["lanes"][1]["valid_indication"].GetString(), std::string("right"));
      // lane 2 - active: false, valid: true, active_ind: right
      EXPECT_FALSE(step1_int0["lanes"][2]["active"].GetBool());
      EXPECT_TRUE(step1_int0["lanes"][2]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][2]["indications"].Size(), 1);
      EXPECT_EQ(step1_int0["lanes"][2]["indications"][0].GetString(), std::string("right"));
      ASSERT_TRUE(step1_int0["lanes"][2].HasMember("valid_indication"));
      EXPECT_EQ(step1_int0["lanes"][2]["valid_indication"].GetString(), std::string("right"));
    }

    {
      // Validate lane object of third step
      const rapidjson::Value& step2_int0 = steps[2]["intersections"][0];
      ASSERT_TRUE(step2_int0.HasMember("lanes"));
      ASSERT_EQ(step2_int0["lanes"].Size(), 3);

      verify_lane_schema(step2_int0["lanes"]);
      // lane 0 - active: true, valid: true
      EXPECT_TRUE(step2_int0["lanes"][0]["active"].GetBool());
      EXPECT_TRUE(step2_int0["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step2_int0["lanes"][0]["indications"].Size(), 2);
      // TODO: Note that the order of the indications array is reverse, ["left", "uturn"]
      // when it should be ["uturn", "left"]. This is due to not knowing whether a uturn
      // is a left or right uturn in serialization.
      EXPECT_EQ(step2_int0["lanes"][0]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step2_int0["lanes"][0]["indications"][1].GetString(), std::string("uturn"));
      EXPECT_TRUE(step2_int0["lanes"][0].HasMember("valid_indication"));
      EXPECT_EQ(step2_int0["lanes"][0]["valid_indication"].GetString(), std::string("left"));
      // lane 1 - active: true, valid: true, active_ind: left
      EXPECT_FALSE(step2_int0["lanes"][1]["active"].GetBool());
      EXPECT_TRUE(step2_int0["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step2_int0["lanes"][1]["indications"].Size(), 2);
      EXPECT_EQ(step2_int0["lanes"][1]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step2_int0["lanes"][1]["indications"][1].GetString(), std::string("straight"));
      ASSERT_TRUE(step2_int0["lanes"][1].HasMember("valid_indication"));
      EXPECT_EQ(step2_int0["lanes"][1]["valid_indication"].GetString(), std::string("left"));
      // lane 2 - active: false, valid: false
      EXPECT_FALSE(step2_int0["lanes"][2]["active"].GetBool());
      EXPECT_FALSE(step2_int0["lanes"][2]["valid"].GetBool());
      ASSERT_EQ(step2_int0["lanes"][2]["indications"].Size(), 1);
      EXPECT_EQ(step2_int0["lanes"][2]["indications"][0].GetString(), std::string("straight"));
      ASSERT_FALSE(step2_int0["lanes"][2].HasMember("valid_indication"));
    }
  }

  {
    // C->G: test that left uturn gets activated
    auto result = gurka::do_action(valhalla::Options::route, map, {"C", "G"}, "auto");
    rapidjson::Document directions = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

    // Assert expected number of routes, legs, steps
    ASSERT_EQ(directions["routes"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"][0]["steps"].Size(), 3);
    const rapidjson::Value& steps = directions["routes"][0]["legs"][0]["steps"];

    {
      // Validate second lane object of first step
      const rapidjson::Value& step0_int1 = steps[0]["intersections"][1];
      ASSERT_TRUE(step0_int1.HasMember("lanes"));
      ASSERT_EQ(step0_int1["lanes"].Size(), 3);

      verify_lane_schema(step0_int1["lanes"]);
      // lane 0 - active: true, valid: true, active_ind: uturn
      EXPECT_TRUE(step0_int1["lanes"][0]["active"].GetBool());
      EXPECT_TRUE(step0_int1["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step0_int1["lanes"][0]["indications"].Size(), 2);
      // TODO: Note that the order of the indications array is reverse, ["left", "uturn"]
      // when it should be ["uturn", "left"]. This is due to not knowing whether a uturn
      // is a left or right uturn in serialization.
      EXPECT_EQ(step0_int1["lanes"][0]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step0_int1["lanes"][0]["indications"][1].GetString(), std::string("uturn"));
      ASSERT_TRUE(step0_int1["lanes"][0].HasMember("valid_indication"));
      EXPECT_EQ(step0_int1["lanes"][0]["valid_indication"].GetString(), std::string("uturn"));
      // lane 1 - active: false, valid: false
      EXPECT_FALSE(step0_int1["lanes"][1]["active"].GetBool());
      EXPECT_FALSE(step0_int1["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step0_int1["lanes"][1]["indications"].Size(), 2);
      EXPECT_EQ(step0_int1["lanes"][1]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step0_int1["lanes"][1]["indications"][1].GetString(), std::string("straight"));
      EXPECT_FALSE(step0_int1["lanes"][1].HasMember("valid_indication"));
      // lane 2 - active: false, valid: false
      EXPECT_FALSE(step0_int1["lanes"][2]["active"].GetBool());
      EXPECT_FALSE(step0_int1["lanes"][2]["valid"].GetBool());
      ASSERT_EQ(step0_int1["lanes"][2]["indications"].Size(), 1);
      EXPECT_EQ(step0_int1["lanes"][2]["indications"][0].GetString(), std::string("straight"));
      EXPECT_FALSE(step0_int1["lanes"][2].HasMember("valid_indication"));
    }

    {
      // Validate first lane object of second step
      const rapidjson::Value& step1_int0 = steps[1]["intersections"][0];
      ASSERT_TRUE(step1_int0.HasMember("lanes"));
      ASSERT_EQ(step1_int0["lanes"].Size(), 3);

      verify_lane_schema(step1_int0["lanes"]);
      // lane 0 - active: true, valid: true, active_ind: uturn
      EXPECT_TRUE(step1_int0["lanes"][0]["active"].GetBool());
      EXPECT_TRUE(step1_int0["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][0]["indications"].Size(), 2);
      // TODO: Note that the order of the indications array is reverse, ["left", "uturn"]
      // when it should be ["uturn", "left"]. This is due to not knowing whether a uturn
      // is a left or right uturn in serialization.
      EXPECT_EQ(step1_int0["lanes"][0]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step1_int0["lanes"][0]["indications"][1].GetString(), std::string("uturn"));
      ASSERT_TRUE(step1_int0["lanes"][0].HasMember("valid_indication"));
      EXPECT_EQ(step1_int0["lanes"][0]["valid_indication"].GetString(), std::string("uturn"));
      // lane 1 - active: false, valid: false
      EXPECT_FALSE(step1_int0["lanes"][1]["active"].GetBool());
      EXPECT_FALSE(step1_int0["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][1]["indications"].Size(), 2);
      EXPECT_EQ(step1_int0["lanes"][1]["indications"][0].GetString(), std::string("left"));
      EXPECT_EQ(step1_int0["lanes"][1]["indications"][1].GetString(), std::string("straight"));
      EXPECT_FALSE(step1_int0["lanes"][1].HasMember("valid_indication"));
      // lane 2 - active: false, valid: false
      EXPECT_FALSE(step1_int0["lanes"][2]["active"].GetBool());
      EXPECT_FALSE(step1_int0["lanes"][2]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][2]["indications"].Size(), 1);
      EXPECT_EQ(step1_int0["lanes"][2]["indications"][0].GetString(), std::string("straight"));
      EXPECT_FALSE(step1_int0["lanes"][2].HasMember("valid_indication"));
    }
  }

  {
    // J->G: test that right uturn gets activated
    auto result = gurka::do_action(valhalla::Options::route, map, {"J", "G"}, "auto");
    rapidjson::Document directions = gurka::convert_to_json(result, valhalla::Options_Format_osrm);

    // Assert expected number of routes, legs, steps
    ASSERT_EQ(directions["routes"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"].Size(), 1);
    ASSERT_EQ(directions["routes"][0]["legs"][0]["steps"].Size(), 3);
    const rapidjson::Value& steps = directions["routes"][0]["legs"][0]["steps"];

    {
      // Validate second intersection of first step
      const rapidjson::Value& step0_int1 = steps[0]["intersections"][1];
      ASSERT_TRUE(step0_int1.HasMember("lanes"));
      ASSERT_EQ(step0_int1["lanes"].Size(), 2);

      verify_lane_schema(step0_int1["lanes"]);
      // lane 0 - active: false, valid: false
      EXPECT_FALSE(step0_int1["lanes"][0]["active"].GetBool());
      EXPECT_FALSE(step0_int1["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step0_int1["lanes"][0]["indications"].Size(), 1);
      EXPECT_EQ(step0_int1["lanes"][0]["indications"][0].GetString(), std::string("straight"));
      ASSERT_FALSE(step0_int1["lanes"][0].HasMember("valid_indication"));

      // lane 1 - active: true, valid: true, active_ind: uturn
      EXPECT_TRUE(step0_int1["lanes"][1]["active"].GetBool());
      EXPECT_TRUE(step0_int1["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step0_int1["lanes"][1]["indications"].Size(), 2);
      EXPECT_EQ(step0_int1["lanes"][1]["indications"][0].GetString(), std::string("right"));
      EXPECT_EQ(step0_int1["lanes"][1]["indications"][1].GetString(), std::string("uturn"));
      EXPECT_TRUE(step0_int1["lanes"][1].HasMember("valid_indication"));
      EXPECT_EQ(step0_int1["lanes"][1]["valid_indication"].GetString(), std::string("uturn"));
    }

    {
      // Validate first intersection of second step
      const rapidjson::Value& step1_int0 = steps[1]["intersections"][0];
      ASSERT_TRUE(step1_int0.HasMember("lanes"));
      ASSERT_EQ(step1_int0["lanes"].Size(), 2);

      verify_lane_schema(step1_int0["lanes"]);
      // lane 0 - active: false, valid: false
      EXPECT_FALSE(step1_int0["lanes"][0]["active"].GetBool());
      EXPECT_FALSE(step1_int0["lanes"][0]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][0]["indications"].Size(), 1);
      EXPECT_EQ(step1_int0["lanes"][0]["indications"][0].GetString(), std::string("straight"));
      ASSERT_FALSE(step1_int0["lanes"][0].HasMember("valid_indication"));

      // lane 1 - active: true, valid: true, active_ind: uturn
      EXPECT_TRUE(step1_int0["lanes"][1]["active"].GetBool());
      EXPECT_TRUE(step1_int0["lanes"][1]["valid"].GetBool());
      ASSERT_EQ(step1_int0["lanes"][1]["indications"].Size(), 2);
      EXPECT_EQ(step1_int0["lanes"][1]["indications"][0].GetString(), std::string("right"));
      EXPECT_EQ(step1_int0["lanes"][1]["indications"][1].GetString(), std::string("uturn"));
      EXPECT_TRUE(step1_int0["lanes"][1].HasMember("valid_indication"));
      EXPECT_EQ(step1_int0["lanes"][1]["valid_indication"].GetString(), std::string("uturn"));
    }
  }
}

TEST(Standalone, TurnLanesForks) {
  constexpr double gridsize_metres = 500;

  const std::string ascii_map = R"(
        H   J     
        |   |          E
    A---B---C---D<
        |   |          F
        G   I
  )";

  const gurka::ways ways =
      {{"AB",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"turn:lanes", "left|through|through;right"}}},
       {"BG", {{"highway", "tertiary"}, {"name", "Knorrstraße"}}},
       {"BH", {{"highway", "tertiary"}, {"name", "Knorrstraße"}}},
       {"BC",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"turn:lanes", "left|through|through;right"}}},
       {"CI", {{"highway", "tertiary"}, {"name", "Am Nordring"}}},
       {"CJ", {{"highway", "tertiary"}, {"name", "Am Nordring"}}},
       {"CD",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"turn:lanes", "through|slight_right"}}},
       {"DE",
        {{"highway", "primary"}, {"name", "Frankfurter Ring"}, {"oneway", "yes"}, {"lanes", "1"}}},
       {"DF",
        {{"highway", "primary"}, {"name", "Frankfurter Ring"}, {"oneway", "yes"}, {"lanes", "2"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_11");

  valhalla::Api result;

  // Keep left
  result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  validate_turn_lanes(result, {{3, "[ left | *through* ACTIVE | *through*;right ACTIVE ]"},
                               {3, "[ left | *through* ACTIVE | *through*;right ACTIVE ]"},
                               {2, "[ *through* ACTIVE | slight_right ]"},
                               {0, ""}});

  // Keep right
  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  validate_turn_lanes(result, {{3, "[ left | *through* ACTIVE | *through*;right ACTIVE ]"},
                               {3, "[ left | *through* ACTIVE | *through*;right ACTIVE ]"},
                               {2, "[ through | *slight_right* ACTIVE ]"},
                               {0, ""}});
}

TEST(Standalone, ShortTurnLanesForks) {
  constexpr double gridsize_metres = 20;

  const std::string ascii_map = R"(
        H   J     
        |   |          E
    A---B---C---D<
        |   |          F
        G   I
  )";

  const gurka::ways ways =
      {{"AB",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"turn:lanes", "left|through|through;right"}}},
       {"BG", {{"highway", "tertiary"}, {"name", "Knorrstraße"}}},
       {"BH", {{"highway", "tertiary"}, {"name", "Knorrstraße"}}},
       {"BC",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "3"},
         {"turn:lanes", "left|through|through;right"}}},
       {"CI", {{"highway", "tertiary"}, {"name", "Am Nordring"}}},
       {"CJ", {{"highway", "tertiary"}, {"name", "Am Nordring"}}},
       {"CD",
        {{"highway", "primary"},
         {"name", "Frankfurter Ring"},
         {"oneway", "yes"},
         {"lanes", "2"},
         {"turn:lanes", "through|slight_right"}}},
       {"DE",
        {{"highway", "primary"}, {"name", "Frankfurter Ring"}, {"oneway", "yes"}, {"lanes", "1"}}},
       {"DF",
        {{"highway", "primary"}, {"name", "Frankfurter Ring"}, {"oneway", "yes"}, {"lanes", "2"}}}};

  const auto layout =
      gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.1079374, 52.0887174});

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_12");

  valhalla::Api result;

  // Keep left
  result = gurka::do_action(valhalla::Options::route, map, {"A", "E"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  validate_turn_lanes(result, {{3, "[ left | *through* ACTIVE | *through*;right VALID ]"},
                               {3, "[ left | *through* ACTIVE | *through*;right VALID ]"},
                               {2, "[ *through* ACTIVE | slight_right ]"},
                               {0, ""}});

  // Keep right
  result = gurka::do_action(valhalla::Options::route, map, {"A", "F"}, "auto");

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStayRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  validate_turn_lanes(result, {{3, "[ left | *through* VALID | *through*;right ACTIVE ]"},
                               {3, "[ left | *through* VALID | *through*;right ACTIVE ]"},
                               {2, "[ through | *slight_right* ACTIVE ]"},
                               {0, ""}});
}
