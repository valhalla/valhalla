#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, FlatLoop) {
  // we create a way that doubles back on itself
  const std::string ascii_map = R"(A---B)";
  const gurka::ways ways = {
      {"ABA", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_opp_edge");

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