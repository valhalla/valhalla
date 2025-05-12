#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class BarrierUturns : public testing::TestWithParam<int> {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    const std::string ascii_map = R"(
  A----B----C
       1    |
       D    /
       |   /
       E--F)";

    // BD and BE are oneways that lead away, and toward the main road A-B-C
    const gurka::ways ways = {{"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
                              {"BD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
                              {"EF", {{"highway", "primary"}}}, {"CF", {{"highway", "primary"}}}};
    const gurka::nodes nodes = {{"D", {{"barrier", "fence"}, {"access", "no"}}}};
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);

    map = gurka::buildtiles(layout, ways, nodes, {}, "test/data/break_through");
  }
};

gurka::map BarrierUturns::map = {};

TEST_F(BarrierUturns, break_waypoint) {
  // Check with type="break" for middle waypoint (the default)
  auto result1 = gurka::do_action(valhalla::Options::route, map, {"A", "1", "F"}, "auto");
  gurka::assert::raw::expect_path(result1, {"AB", "BD", "BD", "BC", "CF"});
}

TEST_F(BarrierUturns, break_thruough_waypoint_into_barrier) {
  // Check with type="break_through" for middle waypoint
  auto result1 = gurka::do_action(valhalla::Options::route, map, {"A", "1", "F"}, "auto",
                                  {{"/locations/1/type", "break_through"}});
  gurka::assert::raw::expect_path(result1, {"AB", "BD", "BD", "BD", "BC", "CF"});
}

// Test various algorithms with forced heading
TEST_P(BarrierUturns, forced_heading_into_barrier_with_algorithm) {
  const std::string& request =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"heading":180,"heading_tolerance":45},{"lat":%s,"lon":%s}],
             "costing":"auto",
             "date_time": { "type": %d, "value": "2020-10-30T09:00"},
             "costing_options": { "auto": { "speed_types": ["constrained"]}}})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("F").lat()) % std::to_string(map.nodes.at("F").lng()) % GetParam())
          .str();
  auto result1 = gurka::do_action(valhalla::Options::route, map, request);
  gurka::assert::raw::expect_path(result1, {"BD", "BD", "BC", "CF"});
}

/**
 * 0 = current time (timedep_forward)
 * 1 = departure time (timedep_forward)
 * 2 = arrival time (timedep_reverse)
 * 3 = invariant time (bidirectional_astar)
 */
INSTANTIATE_TEST_SUITE_P(Algorithms, BarrierUturns, testing::Values(0, 1, 2, 3));