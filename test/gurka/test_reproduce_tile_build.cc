#include "gurka.h"

#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;

void assert_tile_equalish(const GraphTile& a, const GraphTile& b) {
  const GraphTileHeader *ah = a.header(), *bh = b.header();

  // expected size
  ASSERT_EQ(a.header()->end_offset(), b.header()->end_offset());

  // check the first chunk after the header
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + sizeof(GraphTileHeader),
                   reinterpret_cast<const char*>(b.header()) + sizeof(GraphTileHeader),
                   (reinterpret_cast<const char*>(b.GetBin(0, 0).begin()) -
                    reinterpret_cast<const char*>(b.header())) -
                       sizeof(GraphTileHeader)),
            0);

  // check bins
  for (size_t bin_index = 0; bin_index < kBinCount; ++bin_index) {
    ASSERT_EQ(ah->bin_offset(bin_index), bh->bin_offset(bin_index));
    auto a_bin = a.GetBin(bin_index);
    auto b_bin = b.GetBin(bin_index);
    GraphId* a_pos = a_bin.begin();
    GraphId* b_pos = b_bin.begin();

    while (true) {
      const auto diff = std::mismatch(a_pos, a_bin.end(), b_pos, b_bin.end());
      if (diff.first != a_bin.end()) {
        ADD_FAILURE() << "Bin[" << bin_index << "] mismatch at position "
                      << std::distance(a_bin.begin(), diff.first) << ": " << *diff.first
                      << " != " << *diff.second;

        a_pos = diff.first + 1;
        b_pos = diff.second + 1;
      } else {
        break;
      }
    }
  }

  // check the stuff after the bins
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + a.header()->edgeinfo_offset(),
                   reinterpret_cast<const char*>(b.header()) + b.header()->edgeinfo_offset(),
                   b.header()->end_offset() - b.header()->edgeinfo_offset()),
            0);

  // if the header is as expected
  ASSERT_EQ(ah->access_restriction_count(), bh->access_restriction_count());
  ASSERT_EQ(ah->admincount(), bh->admincount());
  ASSERT_EQ(ah->complex_restriction_forward_offset(), bh->complex_restriction_forward_offset());
  ASSERT_EQ(ah->complex_restriction_reverse_offset(), bh->complex_restriction_reverse_offset());
  ASSERT_EQ(ah->date_created(), bh->date_created());
  ASSERT_EQ(ah->density(), bh->density());
  ASSERT_EQ(ah->departurecount(), bh->departurecount());
  ASSERT_EQ(ah->directededgecount(), bh->directededgecount());
  ASSERT_EQ(ah->edgeinfo_offset(), bh->edgeinfo_offset());
  ASSERT_EQ(ah->exit_quality(), bh->exit_quality());
  ASSERT_EQ(ah->graphid(), bh->graphid());
  ASSERT_EQ(ah->name_quality(), bh->name_quality());
  ASSERT_EQ(ah->nodecount(), bh->nodecount());
  ASSERT_EQ(ah->routecount(), bh->routecount());
  ASSERT_EQ(ah->signcount(), bh->signcount());
  ASSERT_EQ(ah->speed_quality(), bh->speed_quality());
  ASSERT_EQ(ah->stopcount(), bh->stopcount());
  ASSERT_EQ(ah->textlist_offset(), bh->textlist_offset());
  ASSERT_EQ(ah->schedulecount(), bh->schedulecount());
  ASSERT_EQ(ah->version(), bh->version());

  // make sure the edges' shape and names match
  for (size_t i = 0; i < ah->directededgecount(); ++i) {
    const EdgeInfo a_info = a.edgeinfo(a.directededge(i));
    const EdgeInfo b_info = b.edgeinfo(b.directededge(i));
    ASSERT_EQ(a_info.encoded_shape(), b_info.encoded_shape());
    ASSERT_EQ(a_info.GetNames().size(), b_info.GetNames().size());
    for (size_t j = 0; j < a_info.GetNames().size(); ++j)
      ASSERT_EQ(a_info.GetNames()[j], b_info.GetNames()[j]);
  }
}

// 1. build tiles with the same input twice
// 2. check that the same tile sets are generated
struct ReproducibleBuild : ::testing::Test {
  void BuildTiles(const std::string& ascii_map, const gurka::ways& ways, const double gridsize) {
    const auto build_tiles = [&](const std::string& dir) -> gurka::map {
      const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
      const std::string workdir = "test/data/gurka_reproduce_tile_build/" + dir;
      return gurka::buildtiles(layout, ways, {}, {}, workdir, {});
    };
    const gurka::map first_map = build_tiles("1");
    const gurka::map second_map = build_tiles("2");

    baldr::GraphReader first_reader(first_map.config.get_child("mjolnir"));
    baldr::GraphReader second_reader(second_map.config.get_child("mjolnir"));
    const std::unordered_set<GraphId> first_tiles = first_reader.GetTileSet();
    ASSERT_EQ(first_tiles.size(), second_reader.GetTileSet().size())
        << "Got different tiles sets: tile count mismatch";
    for (const GraphId& tile_id : first_tiles) {
      graph_tile_ptr first_tile = first_reader.GetGraphTile(tile_id);
      ASSERT_TRUE(second_reader.DoesTileExist(tile_id))
          << "Tile " << GraphTile::FileSuffix(tile_id) << " isn't found in the second tile set";
      graph_tile_ptr second_tile = second_reader.GetGraphTile(tile_id);

      // human readable check
      assert_tile_equalish(*first_tile, *second_tile);

      // check that raw tiles are equal
      const auto raw_tile_bytes = [](const graph_tile_ptr& tile) -> std::string {
        const GraphTileHeader* header = tile->header();
        return std::string(reinterpret_cast<const char*>(header), header->end_offset());
      };
      const std::string first_raw_tile = raw_tile_bytes(first_tile);
      const std::string second_raw_tile = raw_tile_bytes(second_tile);
      const auto diff_positions = std::mismatch(first_raw_tile.begin(), first_raw_tile.end(),
                                                second_raw_tile.begin(), second_raw_tile.end());
      if (diff_positions.first != first_raw_tile.end())
        FAIL() << "Tile{" << GraphTile::FileSuffix(tile_id.Tile_Base())
               << "} mismatch at byte position "
               << std::distance(first_raw_tile.begin(), diff_positions.first);
    }
  }
};

TEST_F(ReproducibleBuild, OneHighway) {
  const std::string ascii_map = R"(A-----B)";
  const gurka::ways ways = {
      {"ABA", {{"highway", "motorway"}}},
  };
  BuildTiles(ascii_map, ways, 100);
}

TEST_F(ReproducibleBuild, BigGridSize) {
  const std::string ascii_map = R"(
    A----B----C
    |    .
    D----E----F
    |    . \
    G----H----I)";

  // BE and EH are highway=path, so no cars
  // EI is a shortcut that's not accessible to bikes
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"DEF", {{"highway", "primary"}}},
                            {"GHI", {{"highway", "primary"}}},
                            {"ADG", {{"highway", "motorway"}}},
                            {"BE", {{"highway", "path"}}},
                            {"EI", {{"highway", "path"}, {"bicycle", "no"}}},
                            {"EH", {{"highway", "path"}}}};
  BuildTiles(ascii_map, ways, 100000);
}
