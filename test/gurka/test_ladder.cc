#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(Ladder, PedestrianAccess) {
  const std::string ascii_map = R"(
      A---B---C---D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "footway"}}},
      {"BC", {{"highway", "ladder"}}},
      {"CD", {{"highway", "footway"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ladder");

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian");

  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});

  const auto& nodes = result.trip().routes(0).legs(0).node();
  bool found_ladder = false;
  for (const auto& node : nodes) {
    if (node.has_edge() && node.edge().use() == TripLeg_Use_kLadderUse) {
      found_ladder = true;
    }
  }
  EXPECT_TRUE(found_ladder);

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const auto* edge = std::get<1>(gurka::findEdgeByNodes(reader, layout, "B", "C"));

  EXPECT_EQ(edge->use(), baldr::Use::kLadder);
  EXPECT_TRUE(edge->forwardaccess() & baldr::kPedestrianAccess);
  EXPECT_FALSE(edge->forwardaccess() & baldr::kWheelchairAccess);
}

TEST(Ladder, WheelchairAccess) {
  const std::string ascii_map = R"(
      A---B---C---D
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "footway"}}},
      {"BC", {{"highway", "ladder"}}},
      {"CD", {{"highway", "footway"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_ladder_wheelchair");

  EXPECT_THROW(gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "pedestrian",
                                {{"/costing_options/pedestrian/type", "wheelchair"}}),
               std::runtime_error);
}
