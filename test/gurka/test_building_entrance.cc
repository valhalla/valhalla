#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class BuildingEntrance : public ::testing::Test {
protected:
  static gurka::map map;
  static std::string ascii_map;
  static gurka::nodelayout layout;

  static void SetUpTestSuite() {
    constexpr double gridsize_metres = 100;

    ascii_map = R"(
                          A
                          |
                          |
                          B
                          |
                          |
                          C
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "corridor"}, {"level", "1"}}},
        {"BC", {{"highway", "corridor"}, {"indoor", "yes"}, {"level", "1"}}},
    };

    const gurka::nodes nodes = {{"B", {{"entrance", "yes"}, {"indoor", "yes"}}}};

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_building_entrance", build_config);
  }
};
gurka::map BuildingEntrance::map = {};
std::string BuildingEntrance::ascii_map = {};
gurka::nodelayout BuildingEntrance::layout = {};

Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(BuildingEntrance, NodeType) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto node_id = gurka::findNode(graphreader, layout, "B");
  EXPECT_EQ(graphreader.nodeinfo(node_id)->type(), baldr::NodeType::kBuildingEntrance);
}

TEST_F(BuildingEntrance, Indoor) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "A", "B"));
  EXPECT_EQ(directededge->indoor(), false);
  directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "B", "C"));
  EXPECT_EQ(directededge->indoor(), true);
}