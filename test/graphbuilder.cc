#include "mjolnir/graphbuilder.h"
#include "baldr/graphreader.h"
#include "midgard/sequence.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/util.h"

#include <sstream>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "test.h"

using boost::property_tree::ptree;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using valhalla::baldr::GraphId;
using valhalla::baldr::GraphReader;
using valhalla::mjolnir::build_tile_set;
using valhalla::mjolnir::TileManifest;

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

#if !defined(VALHALLA_BINARY_DIR)
#define VALHALLA_BINARY_DIR
#endif

namespace {

const std::string pbf_file = {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"};
const std::string tile_dir = "test/data/graphbuilder_tiles";
const size_t id_table_size = 1000;

const std::string access_file = "test_access_harrisburg.bin";
const std::string bss_file = "test_bss_nodes_harrisburg.bin";
const std::string edges_file = "test_edges_harrisburg.bin";
const std::string from_restriction_file = "test_from_complex_restrictions_harrisburg.bin";
const std::string nodes_file = "test_nodes_harrisburg.bin";
const std::string to_restriction_file = "test_to_complex_restrictions_harrisburg.bin";
const std::string way_nodes_file = "test_way_nodes_harrisburg.bin";
const std::string ways_file = "test_ways_harrisburg.bin";

// Test output from construct edges and that the expected number of tiles are produced from the
// build tiles step that follows.
TEST(GraphBuilder, TestConstructEdges) {
  ptree config;
  config.put<std::string>("mjolnir.tile_dir", tile_dir);
  config.put<size_t>("mjolnir.id_table_size", id_table_size);
  OSMData osm_data{0};
  osm_data.read_from_temp_files(tile_dir);
  std::map<baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file);
  EXPECT_EQ(tiles.size(), 4);
  EXPECT_EQ(tiles[GraphId{5993698}], 0);
  EXPECT_EQ(tiles[GraphId{5993706}], 3084);
  EXPECT_EQ(tiles[GraphId{6005218}], 3113);
  EXPECT_EQ(tiles[GraphId{6005226}], 8953);
  // This directory should be empty
  filesystem::remove_all(tile_dir);
  GraphBuilder::Build(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file,
                      from_restriction_file, to_restriction_file, tiles);
  GraphReader reader(config.get_child("mjolnir"));
  EXPECT_EQ(reader.GetTileSet(2).size(), 4);
  // Clear the tile directory so it doesn't interfere with the next test with graphreader.
  EXPECT_TRUE(filesystem::remove_all(tile_dir));
}

// Test that only a subset of tiles are built when explicitly asked for.
TEST(graphbuilder, TestConstructEdgesSubset) {
  ptree config;
  config.put<std::string>("mjolnir.tile_dir", tile_dir);
  config.put<size_t>("mjolnir.id_table_size", id_table_size);
  OSMData osm_data{0};
  osm_data.read_from_temp_files(tile_dir);
  std::map<baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file);
  // Redefine tiles to that we only build a single tile.
  tiles = {{GraphId{5993698}, 0}};
  // This directory should be empty
  filesystem::remove_all(tile_dir);
  GraphBuilder::Build(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file,
                      from_restriction_file, to_restriction_file, tiles);
  GraphReader reader(config.get_child("mjolnir"));
  EXPECT_EQ(reader.GetTileSet(2).size(), 1);
  EXPECT_TRUE(reader.DoesTileExist(GraphId{5993698}));
}

class HarrisburgTestSuiteEnv : public ::testing::Environment {
public:
  void SetUp() override {
    ptree config;
    config.put<std::string>("mjolnir.tile_dir", tile_dir);
    config.put<size_t>("mjolnir.id_table_size", id_table_size);
    const auto& mjolnir_config = config.get_child("mjolnir");
    const std::vector<std::string>& input_files = {pbf_file};
    OSMData osmdata = PBFGraphParser::ParseWays(mjolnir_config, input_files, ways_file,
                                                way_nodes_file, access_file);
    PBFGraphParser::ParseRelations(mjolnir_config, input_files, from_restriction_file,
                                   to_restriction_file, osmdata);
    PBFGraphParser::ParseNodes(mjolnir_config, input_files, ways_file, way_nodes_file, bss_file,
                               osmdata);
  }

  void TearDown() override {
    filesystem::remove(ways_file);
    filesystem::remove(way_nodes_file);
    filesystem::remove(access_file);
    filesystem::remove(from_restriction_file);
    filesystem::remove(to_restriction_file);
    filesystem::remove(bss_file);
  }
};

} // namespace

int main(int argc, char* argv[]) {
  // Disable the logging noise
  logging::Configure({{"type", ""}});
  testing::AddGlobalTestEnvironment(new HarrisburgTestSuiteEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
