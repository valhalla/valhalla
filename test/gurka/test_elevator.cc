#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class Elevator : public ::testing::Test {
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
                          |
                          |
                          D
                          |
                          |
                          E                         
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "corridor"}, {"level", "1"}}},
        {"BC", {{"highway", "elevator"}, {"level", "1;2"}}},
        {"CD", {{"highway", "corridor"}, {"level", "2"}}},
        {"DE", {{"highway", "corridor"}, {"level", "3"}}},
    };

    const gurka::nodes nodes = {{"D", {{"highway", "elevator"}, {"level", "2;3"}}}};

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/gurka_elevator", build_config);
  }
};
gurka::map Elevator::map = {};
std::string Elevator::ascii_map = {};
gurka::nodelayout Elevator::layout = {};

Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(Elevator, Use) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "B", "C"));
  EXPECT_EQ(directededge->use(), baldr::Use::kElevator);
  auto node_id = gurka::findNode(graphreader, layout, "D");
  EXPECT_EQ(graphreader.nodeinfo(node_id)->type(), baldr::NodeType::kElevator);
}
