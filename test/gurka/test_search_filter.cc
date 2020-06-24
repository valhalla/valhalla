#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
class SearchFilter : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    B---------C
    |   2     |
    |         ↑
    |1    4  3|
    A---------D
     \        |
      \ 5     |
       \   6  |
        F-----E
         )";
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC", {{"highway", "primary"}, {"tunnel", "yes"}}},
                              {"CD", {{"highway", "primary"}, {"oneway", "-1"}}},
                              {"AD", {{"highway", "primary"}}},
                              {"DE", {{"highway", "primary"}}},
                              {"EF", {{"highway", "primary"}, {"bridge", "yes"}}},
                              {"AF", {{"highway", "motorway_link"}}}};
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter");
  }
};

gurka::map SearchFilter::map = {};

TEST_F(SearchFilter, Unfiltered) {
  auto from = "1";
  auto to = "2";

  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::route(map, request);

  // should take the shortest path
  gurka::assert::osrm::expect_steps(result, {"AB", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "BC"});
}
TEST_F(SearchFilter, Heading) {
  auto from = "1";
  auto to = "2";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"heading":180,"heading_tolerance":45},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::route(map, request);

  // should take the long way around starting southbound due to heading at origin
  gurka::assert::osrm::expect_steps(result, {"AB", "AD", "CD", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "AD", "CD", "BC"});
}
TEST_F(SearchFilter, PreferredSide) {
  auto from = "1";
  auto to = "2";

  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"preferred_side":"same"}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::route(map, request);

  // should take the long way around starting southbound due to preferred side at destination
  gurka::assert::osrm::expect_steps(result, {"AB", "AD", "CD", "BC"});
  gurka::assert::raw::expect_path(result, {"AB", "AD", "CD", "BC"});
}
TEST_F(SearchFilter, MaxRoadClass) {
  auto from = "1";
  auto to = "2";

  // Should snap origin to CD as the search_filter disallows motorways
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"max_road_class":"primary"}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::route(map, request);
  gurka::assert::osrm::expect_steps(result, {"AD", "AB", "BC"});
  gurka::assert::raw::expect_path(result, {"AD", "AB", "BC"});
}
TEST_F(SearchFilter, MinRoadClass) {
  auto from = "1";
  auto to = "2";
  // Should snap destination to AB as the search_filter disallows primary
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"min_road_class":"motorway"}}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result = gurka::route(map, request);
  gurka::assert::osrm::expect_steps(result, {"AB"});
  gurka::assert::raw::expect_path(result, {"AB"});
}
TEST_F(SearchFilter, ExcludeTunnel) {
  auto from = "2";
  auto to = "1";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"BC", "AB"});
  gurka::assert::raw::expect_path(result_unfiltered, {"BC", "AB"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_tunnel":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);
  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "AB"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "AB"});
}
TEST_F(SearchFilter, ExcludeBridge) {
  auto from = "6";
  auto to = "3";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"EF", "DE", "CD"});
  gurka::assert::raw::expect_path(result_unfiltered, {"EF", "DE", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_bridge":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);
  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "CD"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "CD"});
}
TEST_F(SearchFilter, ExcludeRamp) {
  auto from = "5";
  auto to = "2";
  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_steps(result_unfiltered, {"AF", "AB", "BC"});
  gurka::assert::raw::expect_path(result_unfiltered, {"AF", "AB", "BC"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_ramp":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at(from).lat()) % std::to_string(map.nodes.at(from).lng()) %
       std::to_string(map.nodes.at(to).lat()) % std::to_string(map.nodes.at(to).lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);

  gurka::assert::osrm::expect_steps(result_filtered, {"AD", "AB", "BC"});
  gurka::assert::raw::expect_path(result_filtered, {"AD", "AB", "BC"});
}
