#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class Levels : public ::testing::Test {
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
    )";

    const gurka::ways ways = {
        {"AB", {{"highway", "corridor"}, {"level", "1"}, {"level:ref", "Ground Level"}}},
        {"BC", {{"highway", "steps"}, {"level", "1;2"}}},
        {"CD", {{"highway", "corridor"}, {"level", "2"}, {"level:ref", "Lobby"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_levels", build_config);
  }
};
gurka::map Levels::map = {};
std::string Levels::ascii_map = {};
gurka::nodelayout Levels::layout = {};

Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(Levels, Level) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto get_level = [&](auto from, auto to) {
    auto edgeId = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, from, to));
    return graphreader.edgeinfo(edgeId).level();
  };
  EXPECT_EQ(get_level("A", "B"), "1");
  EXPECT_EQ(get_level("B", "C"), "1:2");
  EXPECT_EQ(get_level("C", "D"), "2");
}

TEST_F(Levels, Level_ref) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto get_level_ref = [&](auto from, auto to) {
    auto edge_id = std::get<0>(gurka::findEdgeByNodes(graphreader, layout, from, to));
    return graphreader.edgeinfo(edge_id).level_ref();
  };
  EXPECT_EQ(get_level_ref("A", "B"), "Ground Level");
  EXPECT_EQ(get_level_ref("B", "C"), "");
  EXPECT_EQ(get_level_ref("C", "D"), "Lobby");
}
