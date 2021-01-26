#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

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
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
}

TEST_F(SearchSideOfStreet, InputLeft) {
  auto from = "1";
  auto to = "3";
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputRight) {
  auto from = "1";
  auto to = "4";
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
}

TEST_F(SearchSideOfStreet, InputRightDisplayLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "3";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"display_lat":%s,"display_lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) %
       std::to_string(map.nodes.at(display).lat()) % std::to_string(map.nodes.at(display).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // display_ll is on the left and overrides the input point being on the right
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputLeftDisplayRight) {
  auto from = "1";
  auto to = "3";
  auto display = "4";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"display_lat":%s,"display_lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) %
       std::to_string(map.nodes.at(display).lat()) % std::to_string(map.nodes.at(display).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationRight});
}

TEST_F(SearchSideOfStreet, InputRightDisplayAheadLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "5";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"display_lat":%s,"display_lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) %
       std::to_string(map.nodes.at(display).lat()) % std::to_string(map.nodes.at(display).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 5 is left enough of the tangent line so is considered left side of street
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}

TEST_F(SearchSideOfStreet, InputRightDisplayAheadStraightLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "6";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"display_lat":%s,"display_lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) %
       std::to_string(map.nodes.at(display).lat()) % std::to_string(map.nodes.at(display).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 6 is not left enough so is considered straight ahead
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestination});
}

TEST_F(SearchSideOfStreet, InputRightDisplayBehindLeft) {
  auto from = "1";
  auto to = "4";
  auto display = "7";
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"display_lat":%s,"display_lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()) %
       std::to_string(map.nodes.at(display).lat()) % std::to_string(map.nodes.at(display).lng()))
          .str();
  auto result = gurka::do_action(valhalla::Options::route, map, request);

  // point 7 is behind and left enough of the tangent line so is considered left side of street
  gurka::assert::raw::expect_maneuvers(result, {DirectionsLeg_Maneuver_Type_kStart,
                                                DirectionsLeg_Maneuver_Type_kDestinationLeft});
}
