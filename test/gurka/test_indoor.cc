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
              |
              K
              |
              L
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

        {"Cx", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "1;2"}}},
        {"xy", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "2;3"}}},
        {"yJ", {{"highway", "steps"}, {"indoor", "yes"}, {"level", "3"}}},
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