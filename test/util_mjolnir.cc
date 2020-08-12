#include <unordered_map>

#include "baldr/graphid.h"
#include "mjolnir/util.h"

#include "test.h"

namespace {
using valhalla::baldr::GraphId;
using valhalla::mjolnir::TileManifest;
using namespace valhalla::midgard;

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

#if !defined(VALHALLA_BINARY_DIR)
#define VALHALLA_BINARY_DIR
#endif

TEST(UtilMjolnir, TileManifestReadFromFile) {
  const std::string filename(VALHALLA_SOURCE_DIR "test/data/tile_manifest0.json");
  TileManifest read = TileManifest::ReadFromFile(filename);
  EXPECT_EQ(read.tileset.size(), 3);
  EXPECT_EQ(read.tileset[GraphId{5970538}], 0);
  EXPECT_EQ(read.tileset[GraphId{5970546}], 54);
  EXPECT_EQ(read.tileset[GraphId{5970554}], 450);
}

TEST(UtilMjolnir, TileManifestLogToFile) {
  const std::map<GraphId, size_t> tileset = {{GraphId{5970538}, 0},
                                             {GraphId{5970546}, 54},
                                             {GraphId{5970554}, 450}};
  TileManifest manifest{tileset};
  const std::string filename(VALHALLA_BINARY_DIR "dummy_tile_manifest0.json");
  manifest.LogToFile(filename);
  TileManifest read = TileManifest::ReadFromFile(filename);
  EXPECT_EQ(read.tileset.size(), 3);
  EXPECT_EQ(read.tileset[GraphId{5970538}], 0);
  EXPECT_EQ(read.tileset[GraphId{5970546}], 54);
  EXPECT_EQ(read.tileset[GraphId{5970554}], 450);
}

TEST(UtilMjolnir, EmptyTileManifestToString) {
  const std::map<GraphId, size_t> tileset = {};
  TileManifest manifest{tileset};
  EXPECT_EQ(manifest.ToString(), "{\"tiles\":[]}");
}

TEST(UtilMjolnir, NonEmptyTileManifestToString) {
  const std::map<GraphId, size_t> tileset = {{GraphId{5970538}, 0}};
  TileManifest manifest{tileset};
  EXPECT_EQ(
      manifest.ToString(),
      "{\"tiles\":[{\"node_index\":0,\"graphid\":{\"value\":5970538,\"id\":0,\"tile_id\":746317,\"level\":2}}]}");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
