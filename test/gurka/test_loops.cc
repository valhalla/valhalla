#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

void check_opposing(const gurka::map& map) {
  // then we test the invariant that any edge should be the opposing edge of its opposing edge
  // edge == opposing(opposing(edge))
  valhalla::baldr::GraphReader reader(map.config.get_child("mjolnir"));
  for (auto tile_id : reader.GetTileSet()) {
    const auto* tile = reader.GetGraphTile(tile_id);
    for (auto edge = tile_id; edge.id() < tile->header()->directededgecount(); ++edge) {
      auto opposing = reader.GetOpposingEdgeId(edge);
      auto opposing_opposing = reader.GetOpposingEdgeId(opposing);
      EXPECT_EQ(edge, opposing_opposing);
    }
  }
}

TEST(loops, flat_loop) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(A-----B)";
  const gurka::ways ways = {
      {"ABA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_flat_loop");
  check_opposing(map);
}

TEST(loops, long_loop) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(A--B--C--D--E)";
  const gurka::ways ways = {
      {"ABCDEDCBA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_flat_loop");
  check_opposing(map);
}

TEST(loops, lollipop) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(
              C----D
             /      \
      A-----B        |
             \      /
              F----E)";
  const gurka::ways ways = {
      {"ABCDEFBA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_lollipop");
  check_opposing(map);
}

TEST(loops, split_hair) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(
  F--E
      \
       C-----D
      /
  A--B)";
  const gurka::ways ways = {
      {"ABCDCEF", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_hair");
  check_opposing(map);
}

TEST(loops, split_lolli) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(
  K--J         I----H
      \       /      \
       C--D--E        |
      /       \      /
  A--B         F----G)";
  const gurka::ways ways = {
      {"ABCDEFGHIEDCJK", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_lolli");
  check_opposing(map);
}

TEST(loops, eye) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(
          J----I
         /      \
  A--B--C        F--G--H
         \      /
          D----E)";
  const gurka::ways ways = {
      {"ABCDEFGHGFIJ", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_lolli");
  check_opposing(map);
}

TEST(loops, bubble) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(
          H----G
         /      \
  A--B--C--------D--E--F)";
  const gurka::ways ways = {
      {"ABCDEFEDGH", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_split_lolli");
  check_opposing(map);
}