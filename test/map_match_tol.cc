#include "filesystem.h"
#include "midgard/sequence.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/osmnode.h"
#include "mjolnir/pbfgraphparser.h"

#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <fstream>

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"

#include "test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla::mjolnir;
using namespace valhalla::baldr;

namespace {

const std::string config_file = "test/test_map_match_tol";

// Remove a temporary file if it exists
void remove_temp_file(const std::string& fname) {
  if (filesystem::exists(fname)) {
    filesystem::remove(fname);
  }
}

void write_config(const std::string& filename) {
  std::ofstream file;
  try {
    file.open(filename, std::ios_base::trunc);
    file << "{ \
      \"mjolnir\": { \
      \"concurrency\": 1, \
      \"id_table_size\": 1000, \
       \"tile_dir\": \"test/data/palermo_tiles\", \
        \"admin\": \"" VALHALLA_SOURCE_DIR "test/data/palermo_admin.sqlite\", \
         \"timezone\": \"" VALHALLA_SOURCE_DIR "test/data/not_needed.sqlite\" \
      } \
    }";
  } catch (...) {}
  file.close();
}

const auto node_predicate = [](const OSMWayNode& a, const OSMWayNode& b) {
  return a.node.osmid_ < b.node.osmid_;
};

OSMNode GetNode(uint64_t node_id, sequence<OSMWayNode>& way_nodes) {
  auto found = way_nodes.find({node_id}, node_predicate);
  EXPECT_NE(found, way_nodes.end()) << "Couldn't find node: " + std::to_string(node_id);
  return (*found).node;
}

auto way_predicate = [](const OSMWay& a, const OSMWay& b) { return a.osmwayid_ < b.osmwayid_; };

OSMWay GetWay(uint32_t way_id, sequence<OSMWay>& ways) {
  auto found = ways.find({way_id}, way_predicate);
  EXPECT_NE(found, ways.end()) << "Couldn't find way: " + std::to_string(way_id);
  return *found;
}

TEST(MapMatchTol, Test) {

  // make a config file
  write_config(config_file);

  boost::property_tree::ptree conf;
  rapidjson::read_json(config_file, conf);

  // setup and purge
  GraphReader graph_reader(conf.get_child("mjolnir"));
  for (const auto& level : TileHierarchy::levels()) {
    auto level_dir = graph_reader.tile_dir() + "/" + std::to_string(level.level);
    if (filesystem::exists(level_dir) && !filesystem::is_empty(level_dir)) {
      filesystem::remove_all(level_dir);
    }
  }

  // Set up the temporary (*.bin) files used during processing
  std::string ways_file = "test_ways_palermo.bin";
  std::string way_nodes_file = "test_way_nodes_palermo.bin";
  std::string nodes_file = "test_nodes_palermo.bin";
  std::string edges_file = "test_edges_palermo.bin";
  std::string access_file = "test_access_palermo.bin";
  std::string cr_from_file = "test_from_cr_palermo.bin";
  std::string cr_to_file = "test_to_cr_palermo.bin";
  std::string bss_nodes_file = "test_bss_nodes_palermo.bin";

  // Parse Amsterdam OSM data
  auto osmdata = PBFGraphParser::ParseWays(conf.get_child("mjolnir"),
                                           {VALHALLA_SOURCE_DIR "test/data/palermo.osm.pbf"},
                                           ways_file, way_nodes_file, access_file);

  PBFGraphParser::ParseRelations(conf.get_child("mjolnir"),
                                 {VALHALLA_SOURCE_DIR "test/data/palermo.osm.pbf"}, cr_from_file,
                                 cr_to_file, osmdata);

  PBFGraphParser::ParseNodes(conf.get_child("mjolnir"),
                             {VALHALLA_SOURCE_DIR "test/data/palermo.osm.pbf"}, way_nodes_file,
                             bss_nodes_file, osmdata);

  std::map<valhalla::baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(conf.get_child("mjolnir"), ways_file, way_nodes_file, nodes_file,
                               edges_file);

  // Build the graph using the OSMNodes and OSMWays from the parser
  GraphBuilder::Build(conf, osmdata, ways_file, way_nodes_file, nodes_file, edges_file, cr_from_file,
                      cr_to_file, tiles);

  // load a tile and test the default access.
  GraphId id(1, 2, 3);
  graph_tile_ptr t = GraphTile::Create("test/data/palermo_tiles", id);
  ASSERT_TRUE(t);

  GraphTileBuilder tilebuilder(graph_reader.tile_dir(), id, true);


  // Remove temporary files
  remove_temp_file(ways_file);
  remove_temp_file(way_nodes_file);
  remove_temp_file(nodes_file);
  remove_temp_file(edges_file);
  remove_temp_file(access_file);
  remove_temp_file(cr_from_file);
  remove_temp_file(cr_to_file);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
