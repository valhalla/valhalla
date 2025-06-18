#include "mjolnir/graphbuilder.h"
#include "baldr/datetime.h"
#include "baldr/graphreader.h"
#include "mjolnir/admin.h"
#include "mjolnir/directededgebuilder.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/pbfgraphparser.h"

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <string>

using boost::property_tree::ptree;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;
using valhalla::baldr::GraphId;
using valhalla::baldr::GraphReader;

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

#if !defined(VALHALLA_BINARY_DIR)
#define VALHALLA_BINARY_DIR
#endif

namespace {

const std::string pbf_file = {VALHALLA_SOURCE_DIR "test/data/harrisburg.osm.pbf"};
const std::string tile_dir = {VALHALLA_BUILD_DIR "test/data/graphbuilder_tiles"};
const size_t id_table_size = 1000;

const std::string access_file = "test_access_harrisburg.bin";
const std::string bss_file = "test_bss_nodes_harrisburg.bin";
const std::string edges_file = "test_edges_harrisburg.bin";
const std::string from_restriction_file = "test_from_complex_restrictions_harrisburg.bin";
const std::string nodes_file = "test_nodes_harrisburg.bin";
const std::string to_restriction_file = "test_to_complex_restrictions_harrisburg.bin";
const std::string way_nodes_file = "test_way_nodes_harrisburg.bin";
const std::string ways_file = "test_ways_harrisburg.bin";
const std::string linguistic_node_file = "test_linguistic_node_harrisburg.bin";

// Test output from construct edges and that the expected number of tiles are produced from the
// build tiles step that follows.
TEST(GraphBuilder, TestConstructEdges) {
  ptree config;
  config.put("mjolnir.tile_dir", tile_dir);
  config.put("mjolnir.concurrency", 1);
  OSMData osm_data{0};
  osm_data.read_from_temp_files(tile_dir);
  std::map<baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(config, ways_file, way_nodes_file, nodes_file, edges_file);
  EXPECT_EQ(tiles.size(), 4);
  EXPECT_EQ(tiles[GraphId{5993698}], 0);
  EXPECT_EQ(tiles[GraphId{5993706}], 3125);
  EXPECT_EQ(tiles[GraphId{6005218}], 3154);
  EXPECT_EQ(tiles[GraphId{6005226}], 8997);
  // This directory should be empty
  std::filesystem::remove_all(tile_dir);
  GraphBuilder::Build(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file,
                      from_restriction_file, to_restriction_file, linguistic_node_file, tiles);
  GraphReader reader(config.get_child("mjolnir"));
  EXPECT_EQ(reader.GetTileSet(2).size(), 4);
  // Clear the tile directory so it doesn't interfere with the next test with graphreader.
  std::filesystem::remove_all(tile_dir);
  EXPECT_TRUE(!std::filesystem::exists(tile_dir));
}

// Test that only a subset of tiles are built when explicitly asked for.
TEST(Graphbuilder, TestConstructEdgesSubset) {
  ptree config;
  config.put<std::string>("mjolnir.tile_dir", tile_dir);
  config.put("mjolnir.concurrency", 1);
  OSMData osm_data{0};
  osm_data.read_from_temp_files(tile_dir);
  std::map<baldr::GraphId, size_t> tiles =
      GraphBuilder::BuildEdges(config, ways_file, way_nodes_file, nodes_file, edges_file);
  // Redefine tiles to that we only build a single tile.
  tiles = {{GraphId{5993698}, 0}};
  // This directory should be empty
  std::filesystem::remove_all(tile_dir);
  GraphBuilder::Build(config, osm_data, ways_file, way_nodes_file, nodes_file, edges_file,
                      from_restriction_file, to_restriction_file, linguistic_node_file, tiles);
  GraphReader reader(config.get_child("mjolnir"));
  EXPECT_EQ(reader.GetTileSet(2).size(), 1);
  EXPECT_TRUE(reader.DoesTileExist(GraphId{5993698}));
}

TEST(Graphbuilder, TestDEBuilderLength) {

  std::vector<PointLL> shape1{{-160.096619f, 21.997619f},
                              {-90.037697f, 41.004531},
                              {-160.096619f, 21.997619f}};
  ASSERT_NO_THROW(DirectedEdgeBuilder edge_builder({}, GraphId(123, 2, 8), true,
                                                   valhalla::midgard::length(shape1), 1, 1,
                                                   Use::kRoad, baldr::RoadClass::kMotorway, 0, false,
                                                   false, false, false, 0, 0, false,
                                                   baldr::RoadClass::kInvalid));

  std::vector<PointLL> shape2{{-160.096619f, 21.997619f},
                              {-90.037697f, 41.004531},
                              {-160.096619f, 21.997619f},
                              {-90.037697f, 41.004531}};
  ASSERT_THROW(DirectedEdgeBuilder edge_builder({}, GraphId(123, 2, 8), true,
                                                valhalla::midgard::length(shape2), 1, 1, Use::kRoad,
                                                baldr::RoadClass::kMotorway, 0, false, false, false,
                                                false, 0, 0, false, baldr::RoadClass::kInvalid),
               std::runtime_error);
}

// test new timezones here instead of a new mjolnir test
class TestNodeInfo : NodeInfo {
public:
  using NodeInfo::set_timezone;

  uint32_t get_raw_timezone_field() const {
    return timezone_;
  }

  uint32_t get_raw_timezone_ext1_field() const {
    return timezone_ext_1_;
  }
};

TEST(Graphbuilder, NewTimezones) {
  TestNodeInfo test_node;
  auto sql_db = AdminDB::open(VALHALLA_BUILD_DIR "test/data/tz.sqlite");
  ASSERT_TRUE(sql_db);

  const auto& tzdb = DateTime::get_tz_db();

  // America/Ciudad_Juarez
  auto ciudad_juarez_polys = GetTimeZones(*sql_db, {-106.450948, 31.669746, -106.386046, 31.724371});
  EXPECT_EQ(ciudad_juarez_polys.begin()->first, tzdb.to_index("America/Ciudad_Juarez"));
  test_node.set_timezone(ciudad_juarez_polys.begin()->first);
  EXPECT_EQ(test_node.get_raw_timezone_field(), tzdb.to_index("America/Ojinaga"));
  EXPECT_EQ(test_node.get_raw_timezone_ext1_field(), 1);

  // Asia/Qostanay
  auto qostanay_polys = GetTimeZones(*sql_db, {62.41766759, 51.37601571, 64.83104595, 52.71089583});
  EXPECT_EQ(qostanay_polys.begin()->first, tzdb.to_index("Asia/Qostanay"));
  test_node.set_timezone(qostanay_polys.begin()->first);
  EXPECT_EQ(test_node.get_raw_timezone_field(), tzdb.to_index("Asia/Qyzylorda"));
  EXPECT_EQ(test_node.get_raw_timezone_ext1_field(), 1);
}

TEST(Graphbuilder, AdminBbox) {
  auto admin_db = AdminDB::open(VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite");
  ASSERT_TRUE(admin_db);

  const auto& tiling = TileHierarchy::levels().back().tiles;

  // Problematic tile in Belgium where boost::geometry::intersection(box, polygon) fails to produce
  // multiple polygons.
  const GraphId id(811462);
  GraphTileBuilder graphtile(tile_dir, id, false);
  std::unordered_map<uint32_t, bool> drive_on_right;
  std::unordered_map<uint32_t, bool> allow_intersection_names;
  language_poly_index language_polys;

  const AABB2<PointLL> bbox = tiling.TileBounds(id);
  auto admin_polys = GetAdminInfo(*admin_db, drive_on_right, allow_intersection_names, language_polys,
                                  bbox, graphtile);

  const auto admin_name = [&](const Admin& admin) {
    return admin.country_iso() + "/" + admin.state_iso();
  };

  ASSERT_EQ(admin_polys.size(), 3);
  EXPECT_EQ(admin_name(graphtile.admins_builder(0)), "/"); // default empty strings
  EXPECT_EQ(admin_name(graphtile.admins_builder(1)), "BE/VLG");
  EXPECT_EQ(admin_name(graphtile.admins_builder(2)), "BE/WAL");
  EXPECT_EQ(admin_name(graphtile.admins_builder(3)), "BE/");

  // fr, nl, nl, fr
  ASSERT_EQ(language_polys.size(), 4);

  // Ensure that tile corners are handled correctly
  EXPECT_EQ(admin_name(graphtile.admins_builder(
                GetMultiPolyId(admin_polys, PointLL(bbox.minx(), bbox.miny()), graphtile))),
            "BE/VLG");
  EXPECT_EQ(admin_name(graphtile.admins_builder(
                GetMultiPolyId(admin_polys, PointLL(bbox.maxx(), bbox.miny()), graphtile))),
            "BE/VLG");
  EXPECT_EQ(admin_name(graphtile.admins_builder(
                GetMultiPolyId(admin_polys, PointLL(bbox.minx(), bbox.maxy()), graphtile))),
            "BE/VLG");
  EXPECT_EQ(admin_name(graphtile.admins_builder(
                GetMultiPolyId(admin_polys, PointLL(bbox.maxx(), bbox.maxy()), graphtile))),
            "BE/VLG");

  EXPECT_EQ(admin_name(graphtile.admins_builder(
                GetMultiPolyId(admin_polys, PointLL((bbox.minx() + bbox.maxx()) / 2, bbox.miny()),
                               graphtile))),
            "BE/WAL");
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
    PBFGraphParser::ParseNodes(mjolnir_config, input_files, way_nodes_file, bss_file,
                               linguistic_node_file, osmdata);
  }

  void TearDown() override {
    std::filesystem::remove(ways_file);
    std::filesystem::remove(way_nodes_file);
    std::filesystem::remove(access_file);
    std::filesystem::remove(from_restriction_file);
    std::filesystem::remove(to_restriction_file);
    std::filesystem::remove(bss_file);
    std::filesystem::remove(linguistic_node_file);
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
