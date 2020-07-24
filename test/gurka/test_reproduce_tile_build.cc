#include "gurka.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;

void assert_tile_equalish(const GraphTile& a, const GraphTile& b) {
  // expected size
  ASSERT_EQ(a.header()->end_offset(), b.header()->end_offset());

  // check the first chunk after the header
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + sizeof(GraphTileHeader),
                   reinterpret_cast<const char*>(b.header()) + sizeof(GraphTileHeader),
                   (reinterpret_cast<const char*>(b.GetBin(0, 0).begin()) -
                    reinterpret_cast<const char*>(b.header())) -
                       sizeof(GraphTileHeader)),
            0);

  // check the stuff after the bins
  ASSERT_EQ(memcmp(reinterpret_cast<const char*>(a.header()) + a.header()->edgeinfo_offset(),
                   reinterpret_cast<const char*>(b.header()) + b.header()->edgeinfo_offset(),
                   b.header()->end_offset() - b.header()->edgeinfo_offset()),
            0);

  // if the header is as expected
  const auto *ah = a.header(), *bh = b.header();
  if (ah->access_restriction_count() == bh->access_restriction_count() &&
      ah->admincount() == bh->admincount() &&
      ah->complex_restriction_forward_offset() == bh->complex_restriction_forward_offset() &&
      ah->complex_restriction_reverse_offset() == bh->complex_restriction_reverse_offset() &&
      ah->date_created() == bh->date_created() && ah->density() == bh->density() &&
      ah->departurecount() == bh->departurecount() &&
      ah->directededgecount() == bh->directededgecount() &&
      ah->edgeinfo_offset() == bh->edgeinfo_offset() && ah->exit_quality() == bh->exit_quality() &&
      ah->graphid() == bh->graphid() && ah->name_quality() == bh->name_quality() &&
      ah->nodecount() == bh->nodecount() && ah->routecount() == bh->routecount() &&
      ah->signcount() == bh->signcount() && ah->speed_quality() == bh->speed_quality() &&
      ah->stopcount() == bh->stopcount() && ah->textlist_offset() == bh->textlist_offset() &&
      ah->schedulecount() == bh->schedulecount() && ah->version() == bh->version()) {
    // make sure the edges' shape and names match
    for (size_t i = 0; i < ah->directededgecount(); ++i) {
      auto a_info = a.edgeinfo(a.directededge(i)->edgeinfo_offset());
      auto b_info = b.edgeinfo(b.directededge(i)->edgeinfo_offset());
      ASSERT_EQ(a_info.encoded_shape(), b_info.encoded_shape());
      ASSERT_EQ(a_info.GetNames().size(), b_info.GetNames().size());
      for (size_t j = 0; j < a_info.GetNames().size(); ++j)
        ASSERT_EQ(a_info.GetNames()[j], b_info.GetNames()[j]);
    }
  } else {
    FAIL() << "not equal";
  }
}

// 1. build tiles with the same input twice
// 2. check that the same tile sets are generated
TEST(GraphTileBuilder, TestBuildIsReproducible) {
  const auto build_tiles = [](const std::string& dir) -> gurka::map {
    const std::string ascii_map = R"(A-----B)";
    const gurka::ways ways = {
        {"ABA", {{"highway", "motorway"}}},
    };
    const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);
    const std::string workdir = "test/data/gurka_reproduce_tile_build/" + dir;
    auto map = gurka::buildtiles(layout, ways, {}, {}, workdir, {});
    return map;
  };

  const auto first_map = build_tiles("1");
  const auto second_map = build_tiles("2");

  baldr::GraphReader first_reader(first_map.config.get_child("mjolnir"));
  baldr::GraphReader second_reader(second_map.config.get_child("mjolnir"));
  const auto first_tiles = first_reader.GetTileSet();
  ASSERT_EQ(first_tiles.size(), second_reader.GetTileSet().size())
      << "Got different tiles sets: tile count mismatch";
  for (auto tile_id : first_tiles) {
    const auto* first_tile = first_reader.GetGraphTile(tile_id);
    ASSERT_TRUE(second_reader.DoesTileExist(tile_id))
        << "Tile " << GraphTile::FileSuffix(tile_id) << " isn't found in the second tile set";
    const auto* second_tile = second_reader.GetGraphTile(tile_id);

    // human readable check
    assert_tile_equalish(*first_tile, *second_tile);

    // check that raw tiles are equal
    auto raw_tile_bytes = [](const GraphTile* tile) -> std::string {
      const auto header = tile->header();
      return std::string(reinterpret_cast<const char*>(header), header->end_offset());
    };
    const auto first_raw_tile = raw_tile_bytes(first_tile);
    const auto second_raw_tile = raw_tile_bytes(second_tile);
    EXPECT_EQ(first_raw_tile, second_raw_tile)
        << "Tile{" << GraphTile::FileSuffix(tile_id) << "} mismatch";
  }
}
