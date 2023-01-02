#include "baldr/graphreader.h"
#include "gtest/gtest.h"
#include "test.h"

std::unordered_set<valhalla::baldr::GraphId> get_all_shortcuts(boost::property_tree::ptree conf) {
  auto reader = test::make_clean_graphreader(conf.get_child("mjolnir"));

  int total_shortcut_count = 0;
  std::unordered_set<valhalla::baldr::GraphId> found_shortcut_ids, all_shortcut_ids;

  auto tile_set = reader->GetTileSet();
  for (const valhalla::baldr::GraphId& tile_id : tile_set) {
    auto tile = reader->GetGraphTile(tile_id);

    valhalla::baldr::GraphId edge_id{tile_id};
    for (int32_t i = 0; i < tile->header()->directededgecount(); i++, ++edge_id) {
      auto directed_edge = tile->directededge(i);
      if (directed_edge->is_shortcut()) {
        all_shortcut_ids.emplace(edge_id);
        continue;
      }

      auto shortcut_id = reader->GetShortcut(edge_id);
      if (shortcut_id.Is_Valid()) {
        found_shortcut_ids.emplace(shortcut_id);
      }
    }
  }

  // We should have found all shortcuts in the graph at this point
  EXPECT_EQ(all_shortcut_ids.size(), found_shortcut_ids.size());
  EXPECT_EQ(all_shortcut_ids, found_shortcut_ids);

  for (const auto& shortcut_id : all_shortcut_ids) {
    auto found = found_shortcut_ids.find(shortcut_id);
    if (found == found_shortcut_ids.end())
      LOG_WARN("Shortcut that wasn't resolved properly: " + std::to_string(shortcut_id));
  }

  for (const auto& shortcut_id : found_shortcut_ids) {
    auto found = all_shortcut_ids.find(shortcut_id);
    if (found == all_shortcut_ids.end())
      LOG_WARN("Edge that was found but is not a shortcut: " + std::to_string(shortcut_id));
  }

  return found_shortcut_ids;
}

TEST(ShortcutLookup, Comparison) {
  auto conf_no_cache = test::make_config("data/utrecht_tiles",
                                         {{"mjolnir.tile_extract", "data/utrecht_tiles/tiles.tar"}});
  auto no_cache = get_all_shortcuts(conf_no_cache);

  auto conf_with_cache = test::make_config("data/utrecht_tiles",
                                           {{"mjolnir.tile_extract", "data/utrecht_tiles/tiles.tar"},
                                            {"mjolnir.shortcut_to_edge_cache", "true"},
                                            {"mjolnir.edge_to_shortcut_cache", "true"}});
  auto with_cache = get_all_shortcuts(conf_with_cache);

  EXPECT_EQ(no_cache, with_cache);
}