#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

/*************************************************************/
class HeadingFilter : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
    B----2----C
    |         |
    |1        |
    A---------D
         )";
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}},
                              {"AD", {{"highway", "primary"}}}};
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_heading");
  }
};

gurka::map HeadingFilter::map = {};

TEST_F(HeadingFilter, Unfiltered) {
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result = gurka::route(map, request);

  // should take the shortest path
  gurka::assert::osrm::expect_route(result, {"AB", "BC"});
}
TEST_F(HeadingFilter, Filtered) {
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"heading":180,"heading_tolerance":45},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result = gurka::route(map, request);

  // should take the long way around starting southbound
  gurka::assert::osrm::expect_route(result, {"AB", "AD", "CD", "BC"});
}

/*************************************************************/
class RoadClassFilter : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 100;

    const std::string ascii_map = R"(
      B--2--C
      |     |
      |     |
      |     |
      A 1   D
          )";

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                              {"BC", {{"highway", "primary"}}},
                              {"CD", {{"highway", "primary"}}}};
    map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_roadclass");
  }
};

gurka::map RoadClassFilter::map = {};

TEST_F(RoadClassFilter, Unfiltered) {
  // Should snap origin to AB as it's closest
  const std::string& request =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result = gurka::route(map, request);
  gurka::assert::osrm::expect_route(result, {"AB", "BC"});
}
TEST_F(RoadClassFilter, MaxRoadClass) {
  // Should snap origin to CD as the search_filter disallows motorways
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"max_road_class":"primary"}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result = gurka::route(map, request);
  gurka::assert::osrm::expect_route(result, {"CD", "BC"});
}
TEST_F(RoadClassFilter, MinRoadClass) {
  // Should snap destination to AB as the search_filter disallows primary
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"min_road_class":"motorway"}}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result = gurka::route(map, request);
  gurka::assert::osrm::expect_route(result, {"AB"});
}

/*************************************************************/
TEST(Standalone, TunnelFilter) {
  const std::string ascii_map = R"(
    B---1-----C
    |         |
    |        2|
    A         D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}, {"tunnel", "yes"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_tunnel");

  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_route(result_unfiltered, {"BC", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_tunnel":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);

  gurka::assert::osrm::expect_route(result_filtered, {"AB", "BC", "CD"});
}

/*************************************************************/
TEST(Standalone, BridgeFilter) {
  const std::string ascii_map = R"(
    B---1-----C
    |         |
    |        2|
    A         D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}, {"bridge", "yes"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_bridge");

  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_route(result_unfiltered, {"BC", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_bridge":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);
  gurka::assert::osrm::expect_route(result_filtered, {"AB", "BC", "CD"});
}

/*************************************************************/
TEST(Standalone, RampFilter) {
  const std::string ascii_map = R"(
            C-----D2
           /
         1/
         /
    A---B---------E
         )";

  const gurka::ways ways = {{"ABE", {{"highway", "primary"}}},
                            {"BC", {{"highway", "motorway_link"}}},
                            {"CD", {{"highway", "motorway"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_ramp");

  const std::string& request_unfiltered =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_unfiltered = gurka::route(map, request_unfiltered);
  gurka::assert::osrm::expect_route(result_unfiltered, {"BC", "CD"});

  const std::string& request_filtered =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"exclude_ramp":true}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_filtered = gurka::route(map, request_filtered);

  gurka::assert::osrm::expect_route(result_filtered, {"ABE", "BC", "CD"});
}
