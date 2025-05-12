#include <filesystem>
#include <sstream>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"
#include "mjolnir/util.h"

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

#if !defined(VALHALLA_BINARY_DIR)
#define VALHALLA_BINARY_DIR
#endif

namespace {
using boost::property_tree::ptree;
using valhalla::baldr::GraphId;
using valhalla::mjolnir::build_tile_set;
using valhalla::mjolnir::TileManifest;
using namespace valhalla;
using namespace valhalla::midgard;

// Verify that this function runs
TEST(UtilMjolnir, BuildTileSet) {
  ptree config;
  const std::string tile_dir("test/data/util_mjolnir_test_tiles");
  config.put("mjolnir.tile_dir", tile_dir);
  config.put("mjolnir.concurrency", 1);
  EXPECT_TRUE(build_tile_set(config, {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"},
                             mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kCleanup));
  // Clear the tile directory so it doesn't interfere with the next test.
  std::filesystem::remove_all(tile_dir);
  EXPECT_TRUE(!std::filesystem::exists(tile_dir));
}

TEST(UtilMjolnir, TileManifestReadFromFile) {
  const std::string filename(VALHALLA_SOURCE_DIR "test/data/tile_manifest0.json");
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
  // Serialize, explicitly check for JSON equality
  const std::map<GraphId, size_t> tileset = {{GraphId{5970538}, 0}};
  TileManifest manifest{tileset};
  std::stringstream buf;
  buf << manifest.ToString();
  ptree json;
  rapidjson::read_json(buf, json);
  // Check for expected:
  // "{\"tiles\":[{\"node_index\":0,\"graphid\":{\"value\":5970538,\"id\":0,\"tile_id\":746317,\"level\":2}}]}"
  size_t count = 0;
  for (const auto& tile_info : json.get_child("tiles")) {
    EXPECT_EQ(tile_info.second.get<size_t>("node_index"), 0);
    const auto& graph_id = tile_info.second.get_child("graphid");
    EXPECT_EQ(graph_id.get<uint64_t>("value"), 5970538);
    EXPECT_EQ(graph_id.get<uint32_t>("tile_id"), 746317);
    EXPECT_EQ(graph_id.get<uint32_t>("id"), 0);
    EXPECT_EQ(graph_id.get<uint32_t>("level"), 2);
    ++count;
  }
  // Only one element is present: vector/array access of this tree child is not possible.
  EXPECT_EQ(count, 1);
}

TEST(UtilMjolnir, TileManifestLogToFile) {
  const std::map<GraphId, size_t> src = {{GraphId{5970538}, 0},
                                         {GraphId{5970546}, 54},
                                         {GraphId{5970554}, 450}};
  TileManifest manifest{src};
  const std::string filename(VALHALLA_BINARY_DIR "dummy_tile_manifest0.json");
  manifest.LogToFile(filename);
  TileManifest read = TileManifest::ReadFromFile(filename);
  EXPECT_EQ(read.tileset.size(), 3);
  EXPECT_EQ(read.tileset[GraphId{5970538}], manifest.tileset[GraphId{5970538}]);
  EXPECT_EQ(read.tileset[GraphId{5970546}], manifest.tileset[GraphId{5970546}]);
  EXPECT_EQ(read.tileset[GraphId{5970554}], manifest.tileset[GraphId{5970554}]);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
