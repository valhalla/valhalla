#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

void find_ids(baldr::GraphReader& reader, std::multiset<uint64_t> osm_way_ids) {
  // check all edges have correct edge info
  for (const auto& tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    for (auto edge_id = tile_id; edge_id.id() < tile->header()->directededgecount(); ++edge_id) {
      // we should find every way id in the tile set, unless it's a shortcut, no way id there
      auto* edge = tile->directededge(edge_id);
      if (edge->is_shortcut()) {
        continue;
      }
      auto info = tile->edgeinfo(edge);
      auto id = info.wayid();
      auto found = osm_way_ids.find(id);
      if (found == osm_way_ids.cend()) {
        ASSERT_FALSE(found == osm_way_ids.cend()) << " couldnt find " << info.wayid();
        return;
      }
      osm_way_ids.erase(found);
    }
  }
  // and because we found them all there should be none left
  EXPECT_EQ(osm_way_ids.size(), 0) << " some expected osm way ids were not found";
}

TEST(WayIds, way_ids) {
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
  find_ids(reader, osm_way_ids);
}

TEST(WayIds, way_ids1) {
  const std::string ascii_map = R"(
    A---1B----C
    |    |
    D----E----F
    |
    G----H---2I)";

  std::vector<std::string> ids{
      "121", "122", "123", "124", "6472927700900931484", "125",
  };
  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}, {"osm_id", ids[0]}}},
      {"BC", {{"highway", "primary"}, {"osm_id", ids[1]}}},
      {"DEF", {{"highway", "primary"}, {"osm_id", ids[2]}}},
      {"GHI", {{"highway", "primary"}, {"osm_id", ids[3]}}},
      {"ADG", {{"highway", "primary"}, {"osm_id", ids[4]}}},
      {"BE", {{"highway", "primary"}, {"osm_id", ids[5]}}},
  };
  // keep one for each directed edge
  std::multiset<uint64_t> osm_way_ids;
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto& id = ids[i];
    osm_way_ids.emplace(std::stoull(id));
    osm_way_ids.emplace(std::stoull(id));
    // these are ways that become 2 edges
    if (i == 2 || i == 4) {
      osm_way_ids.emplace(std::stoull(id));
      osm_way_ids.emplace(std::stoull(id));
    }
  }

  const gurka::relations relations = {
      {{
           {gurka::way_member, "BC", "from"},
           {gurka::way_member, "BE", "to"},
           {gurka::node_member, "B", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_left_turn"},
       }},
      {{
           {gurka::way_member, "GHI", "from"},
           {gurka::way_member, "AB", "to"},
           {gurka::way_member, "ADG", "via"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_entry"},
       }},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_64bit_wayid1",
                               {{"mjolnir.hierarchy", "false"}, {"mjolnir.concurrency", "1"}});

  // need to access the tiles
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  find_ids(reader, osm_way_ids);
}
