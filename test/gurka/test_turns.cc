#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
TEST(Standalone, TurnStraight) {

  const std::string ascii_map = R"(
    A----B----C
         |
         D)";

  const gurka::ways ways = {{"ABC", {{"highway", "primary"}}}, {"BD", {{"highway", "primary"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_turns_3");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "C"}, "auto");

  gurka::assert::osrm::expect_steps(result, {"ABC"});
  gurka::assert::raw::expect_path(result, {"ABC", "ABC"});
  gurka::assert::raw::expect_path_length(result, 1.0, .001);
}
/*************************************************************/

const std::string ascii_map_1 = R"(
    A----B----C
         |
         D)";

const gurka::ways map_1_ways = {{"ABC", {{"highway", "primary"}}}, {"BD", {{"highway", "primary"}}}};

const std::string ascii_map_2 = R"(
    E----B----C
    |    |
    F----D)";

const gurka::ways map_2_ways = {{"FEBC", {{"highway", "primary"}}},
                                {"FDB", {{"highway", "primary"}}}};

class Turns : public ::testing::Test {
protected:
  static gurka::map map_1;
  static gurka::map map_2;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const auto layout1 = gurka::detail::map_to_coordinates(ascii_map_1, gridsize);
    const auto layout2 = gurka::detail::map_to_coordinates(ascii_map_2, gridsize);
    map_1 = gurka::buildtiles(layout1, map_1_ways, {}, {}, "test/data/gurka_turns_1");
    map_2 = gurka::buildtiles(layout2, map_2_ways, {}, {}, "test/data/gurka_turns_2");
  }
};

gurka::map Turns::map_1 = {};
gurka::map Turns::map_2 = {};

/************************************************************************************/
TEST_F(Turns, TurnRight) {

  auto result = gurka::do_action(valhalla::Options::route, map_1, {"A", "D"}, "auto");

  gurka::assert::osrm::expect_steps(result, {"ABC", "BD"});
  gurka::assert::raw::expect_path(result, {"ABC", "BD"});

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kRight,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  gurka::assert::raw::expect_path_length(result, 0.7, 0.001);
}
/************************************************************************************/
TEST_F(Turns, TurnLeft) {

  auto result = gurka::do_action(valhalla::Options::route, map_2, {"C", "D"}, "auto");

  gurka::assert::osrm::expect_steps(result, {"FEBC", "FDB"});
  gurka::assert::raw::expect_path(result, {"FEBC", "FDB"});

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kLeft,
                                                DirectionsLeg_Maneuver_Type_kDestination});
  gurka::assert::raw::expect_path_length(result, 0.7, 0.001);
}
