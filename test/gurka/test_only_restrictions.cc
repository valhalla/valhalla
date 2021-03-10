#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

TEST(OnlyRestrictions, Straight) {
  const std::string ascii_map = R"(
   A-B-C-D-E-F
)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
      {"EF", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_straight_on"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_straight",
                               {{"mjolnir.concurrency", "1"}});

  for (const auto& from : map.nodes) {
    for (const auto& to : map.nodes) {
      if (from == to)
        continue;
      auto result = gurka::do_action(valhalla::Options::route, map, {from.first, to.first}, "auto");
      EXPECT_EQ(result.trip().routes_size(), 1);
    }
  }
}

TEST(OnlyRestrictions, OneNeighbour) {
  const std::string ascii_map = R"(
         1
        /
     B-C---D---E
)";

  const gurka::ways ways = {
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DE", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_left_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_one_neighbour",
                        {{"mjolnir.concurrency", "1"}});

  try {
    auto result = gurka::do_action(valhalla::Options::route, map, {"B", "1"}, "auto");
    gurka::assert::raw::expect_path(result, {"Unexpected path found"});
  } catch (const std::runtime_error& e) {
    EXPECT_STREQ(e.what(), "No path could be found for input");
  }
}

TEST(OnlyRestrictions, ManyNeighbours) {
  const std::string ascii_map = R"(
           1   3--5
           |   |
  A----B---C---D---E-----F
           |   |
           2   4
)";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"BC", {{"highway", "primary"}}},
      {"CD", {{"highway", "primary"}}},
      {"DE", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"EF", {{"highway", "primary"}}},
      {"C1", {{"highway", "primary"}}},
      {"C2", {{"highway", "primary"}}},
      {"D3", {{"highway", "primary"}}},
      {"D4", {{"highway", "primary"}}},
      {"35", {{"highway", "primary"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "CD", "via"},
           {gurka::way_member, "DE", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "only_right_turn"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, "test/data/only_restrictions_many_neighbours",
                        {{"mjolnir.concurrency", "1"}});

  for (const auto& from : {"A", "B"}) {
    for (const auto& to : {"1", "2", "3", "4", "5"}) {
      try {
        auto result = gurka::do_action(valhalla::Options::route, map, {from, to}, "auto");
        gurka::assert::raw::expect_path(result, {std::string("Unexpected path found for request ") +
                                                 from + " -> " + to});

      } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "No path could be found for input");
      }
    }
  }
}
