
#include "gurka.h"
#include "test.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, TruckSpeeds) {
  constexpr double gridsize = 500;

  const std::string ascii_map = R"(
      A---B---C---D---E
    )";

  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}, {"maxspeed:hgv:forward", "15"}}},
      {"BC", {{"highway", "residential"}, {"maxspeed:hgv:backward", "15"}}},
      // truck_speed > truck_speed_forward
      {"CD", {{"highway", "residential"}, {"maxspeed:hgv", "20"}, {"maxspeed:hgv:forward", "15"}}},
      // truck_speed < truck_speed_forward
      {"DE", {{"highway", "residential"}, {"maxspeed:hgv", "15"}, {"maxspeed:hgv:forward", "20"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  gurka::map map = gurka::buildtiles(layout, ways, {}, {}, "test/data/truck_speed");

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const DirectedEdge *de_fwd, *de_rev, *de_both_1, *de_both_2;
  std::tie(std::ignore, de_fwd) = gurka::findEdgeByNodes(reader, layout, "A", "B");
  std::tie(std::ignore, de_rev) = gurka::findEdgeByNodes(reader, layout, "C", "B");
  std::tie(std::ignore, de_both_1) = gurka::findEdgeByNodes(reader, layout, "C", "D");
  std::tie(std::ignore, de_both_2) = gurka::findEdgeByNodes(reader, layout, "D", "E");
  EXPECT_EQ(de_fwd->truck_speed(), 15);
  EXPECT_EQ(de_rev->truck_speed(), 15);
  EXPECT_EQ(de_both_1->truck_speed(), 15);
  EXPECT_EQ(de_both_2->truck_speed(), 15);
}
