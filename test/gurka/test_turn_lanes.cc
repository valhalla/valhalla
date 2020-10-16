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

  const gurka::ways ways = {{"ABC", {{"highway", "primary"}, {"lanes", "4"}}},
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

  auto maneuver = result.directions().routes(0).legs(0).maneuver(1);
  EXPECT_EQ(maneuver.type(), DirectionsLeg_Maneuver_Type_kRight);

  odin::EnhancedTripLeg etl(*result.mutable_trip()->mutable_routes(0)->mutable_legs(0));
  auto prev_edge = etl.GetPrevEdge(maneuver.begin_path_index());

  ASSERT_TRUE(prev_edge);
  EXPECT_EQ(prev_edge->turn_lanes_size(), 3);
  EXPECT_EQ(prev_edge->TurnLanesToString(),
            "[ left | through | through;right ACTIVE, ACTIVE_DIR right ]");
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
  validate_turn_lanes(
      result,
      {
          {3,
           "[ left ACTIVE, ACTIVE_DIR left | left;through VALID, ACTIVE_DIR left | through;right ]"},
          {0, ""},
      });

  // A -> E - takes the leftmost through lane
  result = gurka::route(map, "A", "E", "auto");
  validate_turn_lanes(
      result,
      {
          {3,
           "[ left | left;through ACTIVE, ACTIVE_DIR through | through;right VALID, ACTIVE_DIR through ]"},
          {0, ""}, // TODO lanes are tossed when all are through
      });

  // A -> G - takes the rightmost through lane
  result = gurka::route(map, "A", "G", "auto");
  gurka::assert::osrm::expect_steps(result, {"Bronx and Pelham Parkway", "Pelham Parkway South"});
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kSlightRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(result,
                      {
                          {3, "[ left | left;through | through;right ACTIVE, ACTIVE_DIR right ]"},
                          {0, ""}, // TODO lanes are tossed when all are through
                      });

  // A -> H - takes the right lane
  result = gurka::route(map, "A", "H", "auto");
  validate_turn_lanes(result,
                      {
                          {3, "[ left | left;through | through;right ACTIVE, ACTIVE_DIR right ]"},
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
  validate_turn_lanes(
      result,
      {
          {3, "[ through ACTIVE, ACTIVE_DIR through | through VALID, ACTIVE_DIR through | right ]"},
          {3, "[ through ACTIVE, ACTIVE_DIR through | through VALID, ACTIVE_DIR through | right ]"},
          {3, "[ through ACTIVE, ACTIVE_DIR through | through VALID, ACTIVE_DIR through | right ]"},
          {0, ""},
      });

  // A -> C - right lane should always be active
  result = gurka::route(map, "A", "C", "auto");
  validate_turn_lanes(result, {
                                  {3, "[ through | through | right ACTIVE, ACTIVE_DIR right ]"},
                                  {0, ""},
                              });

  // A -> F - only right lane after B should be active, before that rightmost right lane should be
  // active
  result = gurka::route(map, "A", "F", "auto");
  validate_turn_lanes(
      result,
      {
          {3, "[ through VALID, ACTIVE_DIR through | through ACTIVE, ACTIVE_DIR through | right ]"},
          {3, "[ through | through | right ACTIVE, ACTIVE_DIR right ]"},
          {3, "[ through | through | right ACTIVE, ACTIVE_DIR right ]"},
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
  validate_turn_lanes(
      result,
      {
          {5,
           "[ left ACTIVE, ACTIVE_DIR left | left VALID, ACTIVE_DIR left | through | right | right ]"},
          {5,
           "[ left ACTIVE, ACTIVE_DIR left | left VALID, ACTIVE_DIR left | through | right | right ]"},
          {0, ""},
      });

  // A -> 3 - takes rightmost right lane
  result = gurka::route(map, "A", "3", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
  validate_turn_lanes(
      result,
      {
          {5,
           "[ left | left | through | right VALID, ACTIVE_DIR right | right ACTIVE, ACTIVE_DIR right ]"},
          {5,
           "[ left | left | through | right VALID, ACTIVE_DIR right | right ACTIVE, ACTIVE_DIR right ]"},
          {0, ""},
      });

  // A -> 4 - takes leftmost right lane
  result = gurka::route(map, "A", "4", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
  validate_turn_lanes(
      result,
      {
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right VALID, ACTIVE_DIR right ]"},
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right VALID, ACTIVE_DIR right ]"},
          {0, ""},
      });

  // A -> F - takes leftmost right lane (left side driving)
  result = gurka::route(map, "A", "F", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(
      result,
      {
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right VALID, ACTIVE_DIR right ]"},
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right VALID, ACTIVE_DIR right ]"},
          {0, ""},
      });

  // A -> G - takes both right lanes as destination is far away from previous transition
  result = gurka::route(map, "A", "G", "auto");
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  validate_turn_lanes(
      result,
      {
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right ACTIVE, ACTIVE_DIR right ]"},
          {5,
           "[ left | left | through | right ACTIVE, ACTIVE_DIR right | right ACTIVE, ACTIVE_DIR right ]"},
          {0, ""},
      });
}

TEST(Standalone, TurnLanesSerializedResponse) {
  const std::string ascii_map = R"(
        C
        |
    A---B
        |
        D--E
        |
        F
  )";

  const gurka::ways ways =
      {{"AB", {{"highway", "trunk"}, {"lanes", "3"}, {"turn:lanes", "left|right|right"}}},
       {"BC", {{"highway", "primary"}}},
       {"BD", {{"highway", "primary"}, {"lanes", "3"}, {"turn:lanes", "left|left;through|through"}}},
       {"DE", {{"highway", "primary"}}},
       {"DF", {{"highway", "primary"}, {"lanes", "2"}, {"turn:lanes", "through|through"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turn_lanes_5");

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

  // A->E : Both right lanes should be valid and left-most right lane should
  // be active. For following step, both left lanes should be valid and the
  // left-most left lane should be active.
  auto result = gurka::route(map, "A", "E", "auto");
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
    EXPECT_FALSE(step1_int0["lanes"][0].HasMember("active_indication"));
    // lane 1 - active: true, valid: true, active_ind: right
    EXPECT_TRUE(step1_int0["lanes"][1]["active"].GetBool());
    EXPECT_TRUE(step1_int0["lanes"][1]["valid"].GetBool());
    ASSERT_EQ(step1_int0["lanes"][1]["indications"].Size(), 1);
    EXPECT_EQ(step1_int0["lanes"][1]["indications"][0].GetString(), std::string("right"));
    ASSERT_TRUE(step1_int0["lanes"][1].HasMember("active_indication"));
    EXPECT_EQ(step1_int0["lanes"][1]["active_indication"].GetString(), std::string("right"));
    // lane 2 - active: false, valid: true, active_ind: right
    EXPECT_FALSE(step1_int0["lanes"][2]["active"].GetBool());
    EXPECT_TRUE(step1_int0["lanes"][2]["valid"].GetBool());
    ASSERT_EQ(step1_int0["lanes"][2]["indications"].Size(), 1);
    EXPECT_EQ(step1_int0["lanes"][2]["indications"][0].GetString(), std::string("right"));
    ASSERT_TRUE(step1_int0["lanes"][2].HasMember("active_indication"));
    EXPECT_EQ(step1_int0["lanes"][2]["active_indication"].GetString(), std::string("right"));
  }

  {
    // Validate lane object of third step
    const rapidjson::Value& step2_int0 = steps[2]["intersections"][0];
    ASSERT_TRUE(step2_int0.HasMember("lanes"));
    ASSERT_EQ(step2_int0["lanes"].Size(), 3);

    verify_lane_schema(step2_int0["lanes"]);
    // lane 0 - active: true, valid: true, active_ind: left
    EXPECT_TRUE(step2_int0["lanes"][0]["active"].GetBool());
    EXPECT_TRUE(step2_int0["lanes"][0]["valid"].GetBool());
    ASSERT_EQ(step2_int0["lanes"][0]["indications"].Size(), 1);
    EXPECT_EQ(step2_int0["lanes"][0]["indications"][0].GetString(), std::string("left"));
    ASSERT_TRUE(step2_int0["lanes"][0].HasMember("active_indication"));
    EXPECT_EQ(step2_int0["lanes"][0]["active_indication"].GetString(), std::string("left"));
    // lane 1 - active: false, valid: true, active_ind: left
    EXPECT_FALSE(step2_int0["lanes"][1]["active"].GetBool());
    EXPECT_TRUE(step2_int0["lanes"][1]["valid"].GetBool());
    ASSERT_EQ(step2_int0["lanes"][1]["indications"].Size(), 2);
    EXPECT_EQ(step2_int0["lanes"][1]["indications"][0].GetString(), std::string("left"));
    EXPECT_EQ(step2_int0["lanes"][1]["indications"][1].GetString(), std::string("straight"));
    ASSERT_TRUE(step2_int0["lanes"][1].HasMember("active_indication"));
    EXPECT_EQ(step2_int0["lanes"][1]["active_indication"].GetString(), std::string("left"));
    // lane 2 - active: false, valid: false
    EXPECT_FALSE(step2_int0["lanes"][2]["active"].GetBool());
    EXPECT_FALSE(step2_int0["lanes"][2]["valid"].GetBool());
    ASSERT_EQ(step2_int0["lanes"][2]["indications"].Size(), 1);
    EXPECT_EQ(step2_int0["lanes"][2]["indications"][0].GetString(), std::string("straight"));
    ASSERT_FALSE(step2_int0["lanes"][2].HasMember("active_indication"));
  }
}
