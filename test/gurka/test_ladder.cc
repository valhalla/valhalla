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

  auto result = gurka::do_action(
      valhalla::Options::route, map, {"A", "D"}, "pedestrian");

  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD"});
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

  EXPECT_THROW(
      gurka::do_action(
          valhalla::Options::route, map, {"A", "D"}, "pedestrian",
          {{"/costing_options/pedestrian/type", "wheelchair"}}),
      std::runtime_error);
}
