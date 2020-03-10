#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, Maxspeed) {
  const std::string ascii_map = R"(
      A--B--C--D--E--F
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}, {"maxspeed", "50"}}},
      {"BC", {{"highway", "motorway"}, {"maxspeed", "60mph"}}},
      {"CD", {{"highway", "motorway"}, {"maxspeed", ""}}},
      {"DE", {{"highway", "motorway"}}},
      {"EF", {{"highway", "motorway"}, {"maxspeed", "none"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_maxspeed");
  auto result = gurka::route(map, "A", "F", "auto");

  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  auto leg = result.trip().routes(0).legs(0);

  EXPECT_EQ(leg.node(0).edge().speed_limit(), 50);  // AB
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 97);  // BC
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 0);   // CD
  EXPECT_EQ(leg.node(3).edge().speed_limit(), 0);   // DE
  EXPECT_EQ(leg.node(4).edge().speed_limit(), 255); // EF
}
