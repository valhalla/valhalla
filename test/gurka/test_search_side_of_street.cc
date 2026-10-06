#include "gurka.h"

#include <gtest/gtest.h>

#include <format>

using namespace valhalla;

class SearchSideOfStreet : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
         7                            5
                       3
                                      6
        A1-------------2--------------B
        |                             |
        |              4              |
        |                             |
        |                             |
        D-----------------------------C
    )";
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);

    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}},
                              {"DA", {{"highway", "primary"}}}};

    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_search_side_of_street");
  }
};

gurka::map SearchSideOfStreet::map = {};

TEST_F(SearchSideOfStreet, InputStraight) {
  auto from = "1";
  auto to = "2";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
}

TEST_F(SearchSideOfStreet, InputLeft) {
  auto from = "1";
  auto to = "3";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputRight) {
  auto from = "1";
  auto to = "4";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
}

TEST_F(SearchSideOfStreet, InputRightDisplayLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "3";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f},"display_lat":{:.6f},"display_lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng(), map.nodes.at(display).lat(), map.nodes.at(display).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // display_ll is on the left and overrides the input point being on the right
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputLeftDisplayRight) {
  auto from = "1";
  auto to = "3";
  auto display = "4";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f},"display_lat":{:.6f},"display_lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng(), map.nodes.at(display).lat(), map.nodes.at(display).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
}

TEST_F(SearchSideOfStreet, InputRightDisplayAheadLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "5";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f},"display_lat":{:.6f},"display_lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng(), map.nodes.at(display).lat(), map.nodes.at(display).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 5 is left enough of the tangent line so is considered left side of street
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputRightDisplayAheadStraightLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "6";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f},"display_lat":{:.6f},"display_lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng(), map.nodes.at(display).lat(), map.nodes.at(display).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 6 is not left enough so is considered straight ahead
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
}

TEST_F(SearchSideOfStreet, InputRightDisplayBehindLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "7";
  const std::string& request = std::format(
      R"({{"locations":[{{"lat":{:.6f},"lon":{:.6f}}},{{"lat":{:.6f},"lon":{:.6f},"display_lat":{:.6f},"display_lon":{:.6f}}}],"costing":"auto"}})",
      map.nodes.at(from).lat(), map.nodes.at(from).lng(), map.nodes.at(to).lat(),
      map.nodes.at(to).lng(), map.nodes.at(display).lat(), map.nodes.at(display).lng());
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 7 is behind and left enough of the tangent line so is considered left side of street
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}
