#include "gurka.h"
#include "mjolnir/add_predicted_speeds.h"

#include <gtest/gtest.h>

#include <fstream>
#include <optional>

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
                   (reinterpret_cast<const char*>(b.GetBin(0, 0).data()) -
                    reinterpret_cast<const char*>(b.header())) -
                       sizeof(GraphTileHeader)),
            0);

  // check bins and circles
  for (size_t bin_index = 0; bin_index < kBinCount; ++bin_index) {
    ASSERT_EQ(ah->bin_offset(bin_index), bh->bin_offset(bin_index));
    auto a_bin = a.GetBin(bin_index);
    auto b_bin = b.GetBin(bin_index);
    auto a_pos = a_bin.begin();
    auto b_pos = b_bin.begin();

    auto a_circles = a.GetBoundingCircles(bin_index);
    auto b_circles = b.GetBoundingCircles(bin_index);
    auto a_circle = a_circles.begin();
    auto b_circle = b_circles.begin();

    while (a_circle != a_circles.end() && b_circle != b_circles.end()) {
      EXPECT_EQ(a_circle->y_offset(), b_circle->y_offset());
      EXPECT_EQ(a_circle->x_offset(), b_circle->x_offset());
      EXPECT_EQ(a_circle->radius_index(), b_circle->radius_index());
      a_circle++;
      b_circle++;
    }

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
  ASSERT_EQ(ah->bounding_circle_offset(), bh->bounding_circle_offset());
  ASSERT_EQ(ah->has_bounding_circles(), bh->has_bounding_circles());
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
    const auto build_tiles = [&](const std::string& dir) -> std::pair<gurka::map, std::string> {
      const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
      const std::string workdir = "test/data/gurka_reproduce_tile_build/" + dir;
      return std::make_pair(gurka::buildtiles(layout, ways, {}, {}, workdir, {}),
                            workdir + "/map.pbf");
    };
    const auto [first_map, first_pbf] = build_tiles("1");
    const auto [second_map, second_pbf] = build_tiles("2");

    baldr::GraphReader first_reader(first_map.config.get_child("mjolnir"));
    baldr::GraphReader second_reader(second_map.config.get_child("mjolnir"));
    const std::unordered_set<GraphId> first_tiles = first_reader.GetTileSet();
    ASSERT_EQ(first_tiles.size(), second_reader.GetTileSet().size())
        << "Got different tiles sets: tile count mismatch";

    // the low 48 bits hash the tile's own data, so the checksum is unique per tile but reproducible
    // across builds; the high 16 bits are a build id, identical across every tile and both builds
    uint16_t build_id = first_reader.GetGraphTile(*first_tiles.begin())->header()->build_id();
    for (const GraphId& tile_id : first_tiles) {
      graph_tile_ptr first_tile = first_reader.GetGraphTile(tile_id);
      ASSERT_TRUE(second_reader.DoesTileExist(tile_id))
          << "Tile " << GraphTile::FileSuffix(tile_id) << " isn't found in the second tile set";
      graph_tile_ptr second_tile = second_reader.GetGraphTile(tile_id);

      // human readable check
      assert_tile_equalish(*first_tile, *second_tile);
      EXPECT_GT(first_tile->header()->tile_checksum(), 0);
      EXPECT_EQ(first_tile->header()->tile_checksum(), second_tile->header()->tile_checksum());
      EXPECT_EQ(first_tile->header()->build_id(), build_id);
      EXPECT_EQ(second_tile->header()->build_id(), build_id);

      // check that raw tiles are equal
      const auto raw_tile_bytes = [](const graph_tile_ptr& tile) -> std::string {
        const GraphTileHeader* header = tile->header();
        return std::string(reinterpret_cast<const char*>(header) + sizeof(*header),
                           header->end_offset() - sizeof(*header));
      };
      const std::string first_raw_tile = raw_tile_bytes(first_tile);
      const std::string second_raw_tile = raw_tile_bytes(second_tile);
      const auto diff_positions = std::mismatch(first_raw_tile.begin(), first_raw_tile.end(),
                                                second_raw_tile.begin(), second_raw_tile.end());
      if (diff_positions.first != first_raw_tile.end())
        FAIL() << "Tile{" << GraphTile::FileSuffix(tile_id.tile_base())
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

// checksum_ packs a tileset-wide build id (high 16 bits) and a per-tile data hash (low 48 bits).
// Across a multi-tile build the build id is constant while each tile hashes its own data.
TEST(TileChecksum, BuildIdAndPerTileHash) {
  const std::string ascii_map = R"(
    A----B----C
    |    |
    D----E----F
    |    |
    G----H----I)";
  const gurka::ways ways = {{"ABC", {{"highway", "primary"}}},
                            {"ADG", {{"highway", "primary"}}},
                            {"DEF", {{"highway", "secondary"}}},
                            {"GHI", {{"highway", "secondary"}}},
                            {"BE", {{"highway", "residential"}}},
                            {"EH", {{"highway", "residential"}}}};
  const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 100000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_tile_checksum", {});

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const std::unordered_set<GraphId> tiles = reader.GetTileSet();
  ASSERT_GT(tiles.size(), 1) << "need multiple tiles to test per-tile uniqueness";

  std::optional<uint16_t> build_id;
  std::unordered_set<uint64_t> per_tile_hashes;
  for (const GraphId& tile_id : tiles) {
    const GraphTileHeader* header = reader.GetGraphTile(tile_id)->header();

    // the per-tile hash occupies only the low 48 bits and build_id is set
    EXPECT_LT(header->tile_checksum(), uint64_t(1) << kTileHashBits);
    EXPECT_GT(header->build_id(), 0u);

    // every tile of the build carries the same build id
    if (!build_id)
      build_id = header->build_id();
    EXPECT_EQ(header->build_id(), *build_id);

    per_tile_hashes.insert(header->tile_checksum());
  }

  // the low bits hash each tile's own data, so they vary across tiles rather than mirroring the
  // build id; equal hashes only occur for tiles with identical data
  EXPECT_GT(per_tile_hashes.size(), 1u);
}

// Adding predicted traffic rewrites a subset of tiles. The touched tiles' data hash must be refreshed
// and the tileset build id recomputed, so a tile_url client can tell the tileset changed.
TEST(TileChecksum, AddPredictedTrafficRefreshesChecksums) {
  const std::string ascii_map = R"(A----B----C----D)";
  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};
  const gurka::nodelayout layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_traffic_checksum", {});
  const std::string tile_dir = map.config.get<std::string>("mjolnir.tile_dir");

  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  const GraphId edge_id = std::get<0>(gurka::findEdge(reader, layout, "BC", "C"));
  ASSERT_TRUE(edge_id.is_valid());
  const GraphId tile_id = edge_id.tile_base();

  const GraphTileHeader* before = reader.GetGraphTile(tile_id)->header();
  const uint16_t build_id_before = before->build_id();
  const uint64_t hash_before = before->tile_checksum();

  // a single CSV row gives the edge free-flow/constrained speeds: level/tileid/edgeindex,ff,cf,
  const std::filesystem::path traffic_dir = "test/data/gurka_traffic_checksum_csv";
  std::filesystem::path csv = traffic_dir / GraphTile::FileSuffix(tile_id);
  csv.replace_extension(".csv");
  std::filesystem::create_directories(csv.parent_path());
  std::ofstream(csv) << tile_id.level() << "/" << tile_id.tileid() << "/" << edge_id.id()
                     << ",45,35,\n";

  mjolnir::ProcessTrafficTiles(tile_dir, traffic_dir, false, map.config);

  // fresh reader so we read the rewritten tile rather than the cached one
  baldr::GraphReader updated(map.config.get_child("mjolnir"));
  const graph_tile_ptr tile = updated.GetGraphTile(tile_id);
  EXPECT_EQ(tile->directededge(edge_id.id())->free_flow_speed(), 45) << "traffic wasn't applied";

  // the rewritten tile got a fresh data hash and the build id was recomputed for the tileset
  EXPECT_NE(tile->header()->tile_checksum(), hash_before);
  EXPECT_NE(tile->header()->build_id(), build_id_before);
}
