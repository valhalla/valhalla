#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

TEST(WaysId, way_ids) {
  // build a very simple graph
  const std::string ascii_map = R"(ABCDFGHIJKLMNOPQRSTUVWXYZ)";
  gurka::ways ways;
  int node = 0;
  std::multiset<uint64_t> osm_way_ids;
  for (const auto& highway : std::vector<std::string>{"motorway", "secondary", "residential"}) {
    for (uint64_t i = 0; i < 8; ++i) {
      // compute way name and id
      std::string name = ascii_map.substr(node++, 2);
      uint64_t osm_id = (static_cast<uint64_t>(1) << (i * 8)) + highway[0];
      // we'll see the ids twice, one for each direction of the edge
      osm_way_ids.emplace(osm_id);
      osm_way_ids.emplace(osm_id);
      // add the way
      ways[name] = {{"highway", highway}, {"osm_id", std::to_string(osm_id)}};
    }
  }
  EXPECT_EQ(osm_way_ids.size(), 48);
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {.1, .1});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_64bit_wayid",
                               {{"mjolnir.timezone", "/path/to/timezone.sqlite"}});

  // need to access the tiles
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // check all edges have correct edge info
  for (const auto& tile_id : reader.GetTileSet()) {
    const auto* tile = reader.GetGraphTile(tile_id);
    for (auto edge = tile_id; edge.id() < tile->header()->directededgecount(); ++edge) {
      // we should find every way id in the tile set
      auto info = tile->edgeinfo(tile->directededge(edge)->edgeinfo_offset());
      auto found = osm_way_ids.find(info.wayid());
      EXPECT_FALSE(found == osm_way_ids.cend());
      osm_way_ids.erase(found);
    }
  }
  // and because we found them all there should be none left
  EXPECT_EQ(osm_way_ids.size(), 0);
}
