#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

class Accessibility : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    A----B----C
    |    .
    D----E----F
    |    . \
    G----H----I)";

    // BE and EH are highway=path, so no cars
    // EI is a shortcut that's not accessible to bikes
    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"DEF", {{"highway", "primary"}}},
                              {"GHI", {{"highway", "primary"}}},
                              {"ADG", {{"highway", "motorway"}}},
                              {"BE", {{"highway", "path"}}},
                              {"EI", {{"highway", "path"}, {"bicycle", "no"}}},
                              {"EH", {{"highway", "path"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/accessibility");
  }
};

gurka::map Accessibility::map = {};

/*************************************************************/
TEST_F(Accessibility, Auto1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "ADG", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "DEF", "DEF"});
}
TEST_F(Accessibility, Auto2) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"BC", "ADG", "GHI"});
  gurka::assert::raw::expect_path(result, {"BC", "AB", "ADG", "ADG", "GHI", "GHI"});
}
TEST_F(Accessibility, WalkUsesShortcut1) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}
TEST_F(Accessibility, WalkUsesBothShortcuts) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "EI"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "EI"});
}
TEST_F(Accessibility, BikeUsesShortcut) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "F"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "DEF"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "DEF"});
}
TEST_F(Accessibility, BikeAvoidsSecondShortcut) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"C", "I"}, "bicycle");
  gurka::assert::osrm::expect_steps(result, {"BC", "BE", "GHI"});
  gurka::assert::raw::expect_path(result, {"BC", "BE", "EH", "GHI"});
}
TEST_F(Accessibility, WalkAvoidsMotorway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "pedestrian");
  gurka::assert::osrm::expect_steps(result, {"AB", "BE", "GHI"});
  gurka::assert::raw::expect_path(result, {"AB", "BE", "EH", "GHI"});
}
TEST_F(Accessibility, AutoUsesMotorway) {
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::osrm::expect_steps(result, {"ADG"});
  gurka::assert::raw::expect_path(result, {"ADG", "ADG"});
}
