#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

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

        {"Cx", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "1;2"}}},
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

TEST_F(Indoor, EdgeInfo) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto get_level = [&](auto from, auto to) {
    auto edgeId = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, from, to));
    return graphreader.edgeinfo(edgeId).level();
  };
  EXPECT_EQ(get_level("A", "B"), "0");
  EXPECT_EQ(get_level("B", "C"), "0:1");
  EXPECT_EQ(get_level("C", "F"), "1");

  auto get_level_ref = [&](auto from, auto to) {
    auto edge_id = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, from, to));
    return graphreader.edgeinfo(edge_id).level_ref();
  };
  EXPECT_EQ(get_level_ref("A", "B"), "Parking");
  EXPECT_EQ(get_level_ref("B", "C"), "");
  EXPECT_EQ(get_level_ref("C", "F"), "Lobby");
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