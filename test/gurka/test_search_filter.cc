#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, HeadingFilter) {

  const std::string ascii_map = R"(
    B----2----C
    |         |
    |1        |
    A---------D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}},
                            {"AD", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_heading");

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

TEST_F(RoadClassFilter, Basic) {
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
