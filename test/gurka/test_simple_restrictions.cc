#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class SimpleRestrictions : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    A---1B----C
    |    |
    D----E----F
    |    |
    G----H----I)";

    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},  {"BC", {{"highway", "primary"}}},
                              {"DEF", {{"highway", "primary"}}}, {"GHI", {{"highway", "primary"}}},
                              {"ADG", {{"highway", "primary"}}}, {"BE", {{"highway", "primary"}}},
                              {"EH", {{"highway", "primary"}}}};

    const gurka::relations relations = {{{{gurka::way_member, "BC", "from"},
                                          {gurka::way_member, "BE", "to"},
                                          {gurka::node_member, "B", "via"}},
                                         {{"type", "restriction"}, {"restriction", "no_left_turn"}}}};

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
    map = gurka::buildtiles(layout, ways, {}, relations, "test/data/simple_restrictions");
  }
};

gurka::map SimpleRestrictions::map = {};

/*************************************************************/
TEST_F(SimpleRestrictions, ForceDetour) {
  auto result = gurka::route(map, "C", "F", "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "AB", "ADG", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "DEF", "DEF"});
}
TEST_F(SimpleRestrictions, NoDetourWhenReversed) {
  auto result = gurka::route(map, "F", "C", "auto");
  gurka::assert::osrm::expect_steps(result, {"DEF", "BE", "BC"});
  gurka::assert::raw::expect_path(result, {"DEF", "BE", "BC"});
}
TEST_F(SimpleRestrictions, NoDetourFromDifferentStart) {
  auto result = gurka::route(map, "1", "F", "auto");
  gurka::assert::osrm::expect_steps(result, {"AB", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"AB", "BE", "DEF"});
}
