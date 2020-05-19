#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, FlatLoop) {
  const std::string ascii_map = R"(
      E----D----C
                |
                |
                B
                |
           Z----A----Y
  )";

  const gurka::ways ways = {
      {"ABCDEDB", {{"highway", "motorway"}}},
      {"ZAY", {{"highway", "motorway"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_opp_edge");
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