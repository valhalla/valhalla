#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(TestSuite, TestName) {

  const std::string& ascii_map = R"(
        A----B
    )";
  const gurka::ways ways = {{"AB", {{"highway", "footway"}, {"footway", "crossing"}}}};

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 50);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/crosswalk");
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "B"}, "pedestrian");

  EXPECT_EQ(result.directions().routes_size(), 1);
  EXPECT_EQ(result.directions().routes(0).legs_size(), 1);
  gurka::assert::raw::expect_path(result, {"AB"});

  auto leg = result.trip().routes(0).legs(0);
  EXPECT_EQ(leg.node(0).edge().use(), valhalla::TripLeg_Use::TripLeg_Use_kPedestrianCrossingUse);
}
