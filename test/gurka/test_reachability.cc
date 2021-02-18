#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Reachability, dont_snap_waypoints_to_deadends) {
  const std::string ascii_map = R"(
  A--1-B-2--C
      / \
     D   E)";

  // BD and BE are oneways that lead away, and toward the main road A-B-C
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"BD", {{"highway", "secondary"}, {"oneway","yes"}}},
                            {"BE", {{"highway", "secondary"}, {"oneway","-1"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 25);

  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/reachability");

  {
    // Verify simple waypoint routing works
    auto result1 = gurka::do_action(valhalla::Options::route, map, {"A", "1", "C"}, "auto");
    gurka::assert::raw::expect_path(result1, {"AB", "AB", "AB", "BC"});
    auto result2 = gurka::do_action(valhalla::Options::route, map, {"A", "2", "C"}, "auto");
    gurka::assert::raw::expect_path(result2, {"AB", "BC", "BC"});
  }
  {
    // BD is a oneway leading away from ABC with no escape
    // input coordinate at D should get snapped to the ABC way
    // Should give the same result as A->1->C
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D", "C"}, "auto");
    gurka::assert::raw::expect_path(result, {"AB", "AB", "AB", "BC"});
  }
  {
    // BE is a oneway leading towards ABC with no way in
    // input coordinate at E should get snapped to the ABC way
    // Should give the same result as A->2->C
    auto result = gurka::do_action(valhalla::Options::route, map, {"A", "E", "C"}, "auto");
    gurka::assert::raw::expect_path(result, {"AB", "BC", "BC"});
  }
}