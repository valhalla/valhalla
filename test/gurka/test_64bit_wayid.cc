#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(UseDirectionOnWays, CheckNamesAndRefs) {
  // build a very simple graph
  const std::string ascii_map = R"(A----B----C)";
  const gurka::ways ways = {
      {"AB",
       {
           {"highway", "trunk"},
           {"osm_id", std::to_string(static_cast<uint64_t>(-1))},
       }},
      {"BC",
       {
           {"highway", "trunk"},
           {"osm_id", std::to_string(static_cast<uint64_t>(67132))},
       }},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {.1, .1});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_64bit_wayid",
                               {{"mjolnir.timezone", "/path/to/timezone.sqlite"}});

  // need to access the tiles
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // check all edges have correct edge info
  for (const auto& tile_id : reader.GetTileSet()) {
    const auto* tile = reader.GetGraphTile(tile_id);
    for (auto edge = tile_id; edge.id() < tile->header()->directededgecount(); ++edge) {
      auto info = tile->edgeinfo(tile->directededge(edge)->edgeinfo_offset());
      auto names = info.GetNames();
      if (names.at(0) == "AB")
        EXPECT_EQ(info.wayid(), static_cast<uint64_t>(-1));
      else if (names.at(0) == "BC")
        EXPECT_EQ(info.wayid(), static_cast<uint64_t>(67132));
      else
        FAIL() << "Got an edge with an unexpected name";
    }
  }
}
