#include "gurka.h"
#include "mjolnir/osmway.h"
#include "test.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace mjolnir;
const std::unordered_map<std::string, std::string> build_config{{}};

struct range_t {
  range_t(float s, float e) : start(s), end(e){};
  range_t(float s) : start(s), end(s){};

  float start;
  float end;
};

class Indoor : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
              A
              |
              B
              |
              C---------x--------y
              |                  |
    D----E----F----G----H----I---J
    |         |
    N         K
    |         |
    O         L
              |
              M
    )";

    const gurka::ways ways = {
        {"AB",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}, {"level:ref", "Parking"}}},
        {"BC", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "0;1"}}},
        {"CF", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Lobby"}}},

        {"DE", {{"highway", "footway"}, {"level", "1"}}},
        {"EF", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Lobby"}}},

        {"FK", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Lobby"}}},
        {"KL", {{"highway", "steps"}, {"conveying", "yes"}, {"indoor", "yes"}, {"level", "1;2"}}},
        {"LM", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "2"}}},

        {"FG", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Lobby"}}},
        {"GH", {{"highway", "elevator"}, {"indoor", "yes"}, {"level", "1;2"}}},
        {"HI", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "2"}}},
        {"IJ", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "3"}}},

        {"DN", {{"highway", "steps"}}},
        {"NO", {{"highway", "footway"}}},

        {"Cx", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "-1;0-2"}}},
        {"xy", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "2;3"}}},
        {"yJ", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "3"}}},
    };

    const gurka::nodes nodes = {
        {"E", {{"entrance", "yes"}, {"indoor", "yes"}}},
        {"I", {{"highway", "elevator"}, {"indoor", "yes"}, {"level", "2;3"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_indoor", build_config);
  }
};
gurka::map Indoor::map = {};
std::string Indoor::ascii_map = {};
gurka::nodelayout Indoor::layout = {};

/*************************************************************/

TEST_F(Indoor, NodeInfo) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto node_id = gurka::findNode(graphreader, layout, "E");
  const auto* node = graphreader.nodeinfo(node_id);
  EXPECT_EQ(node->type(), baldr::NodeType::kBuildingEntrance);
  EXPECT_TRUE(node->access() & baldr::kPedestrianAccess);

  node_id = gurka::findNode(graphreader, layout, "I");
  node = graphreader.nodeinfo(node_id);
  EXPECT_EQ(graphreader.nodeinfo(node_id)->type(), baldr::NodeType::kElevator);
  EXPECT_TRUE(node->access() & baldr::kPedestrianAccess);
}

TEST_F(Indoor, DirectedEdge) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  const auto* directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "B", "C"));
  EXPECT_EQ(directededge->use(), baldr::Use::kSteps);
  EXPECT_TRUE(directededge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(directededge->reverseaccess() & baldr::kPedestrianAccess);

  directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "G", "H"));
  EXPECT_EQ(directededge->use(), baldr::Use::kElevator);
  EXPECT_TRUE(directededge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(directededge->reverseaccess() & baldr::kPedestrianAccess);

  directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "K", "L"));
  EXPECT_EQ(directededge->use(), baldr::Use::kEscalator);
  EXPECT_TRUE(directededge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(directededge->reverseaccess() & baldr::kPedestrianAccess);

  directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "D", "E"));
  EXPECT_EQ(directededge->indoor(), false);
  EXPECT_TRUE(directededge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(directededge->reverseaccess() & baldr::kPedestrianAccess);

  directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "E", "F"));
  EXPECT_EQ(directededge->indoor(), true);
  EXPECT_TRUE(directededge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_TRUE(directededge->reverseaccess() & baldr::kPedestrianAccess);
}

TEST_F(Indoor, ElevatorPenalty) {
  // first route should take the elevator node
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "J"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"EF", "FG", "GH", "HI", "IJ"});

  // second route should take the stairs because we gave the elevator a huge penalty
  result = gurka::do_action(valhalla::Options::route, map, {"E", "J"}, "pedestrian",
                            {{"/costing_options/pedestrian/elevator_penalty", "3600"}});
  gurka::assert::raw::expect_path(result, {"EF", "CF", "Cx", "xy", "yJ"});
}

TEST_F(Indoor, ElevatorManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "J"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"FG", "GH", "HI", "IJ"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kElevatorEnter,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kElevatorEnter,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify single maneuver prior to elevator
  int maneuver_index = 0;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Walk east on FG.", "Walk east.", "",
                                                            "Walk east on FG.",
                                                            "Continue for 500 meters.");

  // Verify elevator as a way instructions
  maneuver_index += 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the elevator to Level 2.", "", "",
                                                            "", "");

  // Verify elevator as a node instructions
  maneuver_index += 2;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the elevator to Level 3.", "", "",
                                                            "", "");
}

TEST_F(Indoor, IndoorStepsManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "A"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"CF", "BC", "AB"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStepsEnter,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify steps instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the stairs to Parking.", "", "", "",
                                                            "");
}

TEST_F(Indoor, OutdoorStepsManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "O"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"DE", "DN", "NO"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify steps instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Turn left onto DN.", "Turn left.",
                                                            "Turn left onto DN.",
                                                            "Turn left onto DN.",
                                                            "Continue for 200 meters.");
}

TEST_F(Indoor, EscalatorManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "M"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"FK", "KL", "LM"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kEscalatorEnter,
                                                DirectionsLeg_Maneuver_Type_kContinue,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify escalator instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the escalator to Level 2.", "", "",
                                                            "", "");
}

TEST_F(Indoor, EnterBuildingManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"D", "F"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"DE", "EF"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kBuildingEnter,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify enter building instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Enter the building, and continue on EF.",
                                                            "", "", "", "");
}

TEST_F(Indoor, ExitBuildingManeuver) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "D"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"EF", "DE"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kBuildingExit,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify exit building instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Exit the building, and continue on DE.",
                                                            "", "", "", "");
}

TEST_F(Indoor, CombineStepsManeuvers) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "J"}, "pedestrian",
                                 {{"/costing_options/pedestrian/elevator_penalty", "3600"}});
  gurka::assert::raw::expect_path(result, {"CF", "Cx", "xy", "yJ"});

  // Verify maneuver types
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kStepsEnter,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});

  // Verify steps instructions
  int maneuver_index = 1;
  gurka::assert::raw::expect_instructions_at_maneuver_index(result, maneuver_index,
                                                            "Take the stairs to Level 3.", "", "", "",
                                                            "");
}

/****************************************************************************************/

class Levels : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;
    ascii_map = R"(
    A-z-B-y-C-x-D-w-E-v-F-u-G-t-H-s-I-r-J
    )";

    const gurka::ways ways = {
        {"AB",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}, {"level:ref", "Parking"}}},
        {"BC", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0"}, {"level:ref", "Lobby"}}},
        {"CD",
         {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "2"}, {"level:ref", "2nd Floor"}}},
        {"DE", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "100"}}},
        {"EF", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "12;14"}}},
        {"FG", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "-1;0"}}},
        {"GH", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "85.5;12"}}},
        {"HI", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "-2;-1;0-3"}}},
        {"IJ", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "0-2.5"}}},
    };
    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_levels");
  }
};

gurka::map Levels::map = {};
std::string Levels::ascii_map = {};
gurka::nodelayout Levels::layout = {};

TEST_F(Levels, EdgeInfoIncludes) {

  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  std::vector<std::pair<std::array<std::string, 2>, std::vector<float>>> values = {
      {{"A", "B"}, {1}},
      {{"B", "C"}, {0}},
      {{"C", "D"}, {2}},
      {{"D", "E"}, {100}},
      {{"E", "F"}, {12, 14}},
      {{"F", "G"}, {-1, 0}},
      {{"G", "H"}, {85.5, 12}},
      {{"H", "I"}, {-2, -1, 0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3}},
      {{"I", "J"}, {0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5}},
  };

  for (const auto& [way, levels] : values) {

    auto edgeId = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, way[0], way[1]));
    for (float i = -2.0f; i <= 100.0f; i = i + 0.25f) {
      bool expected_includes_level = std::find(levels.begin(), levels.end(), i) != levels.end();
      bool includes = graphreader.edgeinfo(edgeId).includes_level(i);
      EXPECT_EQ(includes, expected_includes_level) << "Failing level " << i << "\n";
    }
  }
}

TEST_F(Levels, EdgeInfoJson) {

  auto graphreader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::string json;
  std::vector<std::string> locs = {
      "z", // AB
      "y", // BC
      "x", // CD
      "w", // DE
      "v", // EF
      "u", // FG
      "t", // GH
      "s", // HI
      "r", // IJ
  };
  auto result =
      gurka::do_action(valhalla::Options::locate, map, locs, "pedestrian", {}, graphreader, &json);

  std::unordered_map<std::string, std::vector<range_t>> expected_levels_map = {
      {"z", {{1.f}}},
      {"y", {{0.f}}},
      {"x", {{2.f}}},
      {"w", {{100.f}}},
      {"v", {{12.f}, {14.f}}},
      {"u", {{-1.f}, {0.f}}},
      {"t", {{12.0f}, {85.5f}}},
      {"s", {{-2.f}, {-1.f}, {0.f, 3.f}}},
      {"r", {{0.f, 2.5f}}},
  };
  rapidjson::Document response;
  response.Parse(json);

  if (response.HasParseError())
    throw std::runtime_error("bad json response");

  ASSERT_EQ(response.GetArray().Size(), 9);

  for (size_t i = 0; i < locs.size(); ++i) {
    auto name = locs[i];
    auto edges = rapidjson::Pointer("/" + std::to_string(i) + "/edges").Get(response)->GetArray();
    ASSERT_EQ(edges.Size(), 2);
    auto expected_levels = expected_levels_map[name];
    for (size_t edge_ix = 0; edge_ix < 2; ++edge_ix) {
      auto actual_levels = rapidjson::Pointer("/" + std::to_string(i) + "/edges/" +
                                              std::to_string(edge_ix) + "/edge_info/levels")
                               .Get(response)
                               ->GetArray();
      ASSERT_EQ(actual_levels.Size(), expected_levels.size());
      for (size_t j = 0; j < expected_levels.size(); ++j) {
        auto expected_level = expected_levels[j];
        auto ptr = rapidjson::Pointer("/" + std::to_string(i) + "/edges/" + std::to_string(edge_ix) +
                                      "/edge_info/levels/" + std::to_string(j))
                       .Get(response);
        if (expected_level.start == expected_level.end) {
          EXPECT_EQ(expected_level.start, ptr->GetFloat());
          continue;
        }
        auto actual_level = ptr->GetArray();
        ASSERT_EQ(actual_level.Size(), 2);
        for (size_t k = 0; k < 2; ++k) {
          auto expected_level_value = k == 0 ? expected_level.start : expected_level.end;
          auto actual_level_value =
              rapidjson::Pointer("/" + std::to_string(i) + "/edges/" + std::to_string(edge_ix) +
                                 "/edge_info/levels/" + std::to_string(j) + "/" + std::to_string(k))
                  .Get(response)
                  ->GetFloat();
          EXPECT_EQ(actual_level_value, expected_level_value);
        }
      }
    }
  }
}