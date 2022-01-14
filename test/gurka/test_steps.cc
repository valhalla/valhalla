#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
const std::unordered_map<std::string, std::string> build_config{{}};

class Steps : public ::testing::Test {
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
        {"AB", {{"highway", "corridor"}, {"level", "1"}}},
        {"BC", {{"highway", "steps"}, {"level", "1;2"}}},
        {"CD", {{"highway", "corridor"}, {"level", "2"}}},
    };

    layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres);
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_steps", build_config);
  }
};
gurka::map Steps::map = {};
std::string Steps::ascii_map = {};
gurka::nodelayout Steps::layout = {};

Api api;
rapidjson::Document d;

/*************************************************************/

TEST_F(Steps, Use) {
  baldr::GraphReader graphreader(map.config.get_child("mjolnir"));

  auto directededge = std::get<1>(gurka::findEdgeByNodes(graphreader, layout, "B", "C"));
  EXPECT_EQ(directededge->use(), baldr::Use::kSteps);
}
