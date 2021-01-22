#include <gtest/gtest.h>

#include "gurka.h"

#include "baldr/admin.h"
#include "mjolnir/admin.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/pbfgraphparser.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

namespace {

// GetAdminInfo() requires a sqlite db handle and tiles. This creates a mock
// graph with two states part of the same country - with a highway between them.
valhalla::gurka::map BuildPBFandTiles(const std::string& workdir) {
  const std::string ascii_map = R"(
        A-------B-------C
        |       |       |
        |   G---|---H   |
        |       |       |
        F-------E-------D
  )";

  // To define an administrative boundary, the nodes must form a closed polygon.
  const gurka::ways ways = {{"ABCDEFA", {}},
                            {"ABEFA", {}},
                            {"BCDEB", {}},
                            {"GH",
                             {
                                 {"highway", "primary"},
                             }}};

  const gurka::relations relations = {{{{{gurka::way_member, "ABEFA", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "4"},
                                        {"name", "Colorado"}}},
                                      {{{{gurka::way_member, "BCDEB", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "4"},
                                        {"name", "Utah"}}},
                                      {{{{gurka::way_member, "ABCDEFA", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "2"},
                                        {"name", "USA"}}}};

  constexpr double gridsize = 10;
  auto nodes = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto admin_map = gurka::buildtiles(nodes, ways, {}, relations, workdir);

  return admin_map;
}


uint32_t GetContainingState(GraphTileBuilder & tilebuilder,
                            const std::unordered_multimap<uint32_t, multi_polygon_type>& polys,
                            const PointLL& ll) {
  uint32_t index = 0;
  point_type p(ll.lng(), ll.lat());
  for (const auto& poly : polys) {
    if (boost::geometry::covered_by(p, poly.second)) {
      uint32_t poly_id = poly.first;
      const Admin & admin = tilebuilder.admins_builder(poly_id);
      // if state_offset is non-zero, it must be a state (not a country)
      if ( admin.state_offset() != 0 )
        return admin.state_offset();
    }
  }
  return 0;
}

} // anonymous namespace


// Test that BuildAdminFromPBF() works.
//
// This test creates a mock pbf AND tile data. Both are required to prove
// that BuildAdminFromPBF() is working.
//
// Given map.pbf, BuildAdminFromPBF() creates test/data/admin.sqlite. Hence,
// this test needs to prove that the contents of admin.sqlite are correct.
// Before we get into how I propose to prove correctness, some thoughts must
// be written.
//
// The mock graph we're using has three "relations" (which are just administrative
// boundaries). BuildAdminFromPBF() will create three rows in the admin.sqlite::admins
// table. The rows in the admin table do not store spatial/positional information
// about the nodes that form the administrative area. This is instead stored in the
// tile-data.
//
// GetAdminInfo() will query the tile-data AND the sqlite file to retrieve information
// about the administrative polygons stored within. You'll see that a map of "polys"
// is returned - but each "poly" returned is just polygonal data. The remaining admin
// area metadata is stored in the GraphTileBuilder object that was passed into GetAdminInfo().
//
// So how do you get the admin area metadata? When you get the "polys" map back from
// GetAdminInfo(), the integral map key can be passed to GraphTileBuilder::admins_builder()
// to retrieve each admin area's metadata.
//
// As you can see in the ascii-map, there is a simple two node highway that spans
// the two states. I thought it'd be interesting to see if I could prove that the
// node "G" lives in "Colorado" and that "H" lives in "Utah".
//
// To prove the admin.sqlite is built correctly, we need to ensure the offsets/
// indicies stored for each entry in the admins table is correct - which
// requires the presence of the tile-data.
//
// Basically

TEST(AdminTest, TestAdminPolygonBasic) {
  // Creates test/data/admin/map.pbf and tile data
  const std::string workdir = "test/data/admin";
  auto admin_map = BuildPBFandTiles(workdir);

  boost::property_tree::ptree pt;
  pt.put("mjolnir.concurrency", 1);
  pt.put("mjolnir.id_table_size", 1000);
  pt.put("mjolnir.tile_dir", "test/data/admin");
  pt.put("mjolnir.admin", "test/data/admin.sqlite");
  pt.put("mjolnir.timezone", "test/data/not_needed.sqlite");

  // Given map.pbf, BuildAdminFromPBF() creates test/data/admin.sqlite.
  std::vector<std::string> input_files = {workdir + "/map.pbf"};
  BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);

  // reverse engineer the tile_id from the nodes that make up
  // our mock map.pbf. Its probably overkill to check every node
  // for its tile-id if I know they are all in the same tile-id...
  std::unordered_set<GraphId> tile_ids;
  for (const auto& node : admin_map.nodes) {
    const midgard::PointLL& latlon = node.second;
    GraphId tile_id = TileHierarchy::GetGraphId(latlon, 0);
    tile_ids.insert(tile_id);
  }
  ASSERT_EQ(tile_ids.size(), 1);

  GraphId tile_id(*tile_ids.begin());

  GraphReader graph_reader(pt.get_child("mjolnir"));
  auto t = GraphTile::Create(graph_reader.tile_dir(), tile_id);
  ASSERT_TRUE(t);

  sqlite3* db_handle = NULL;
  std::string dbname = pt.get<std::string>("mjolnir.admin");
  uint32_t ret = sqlite3_open_v2(dbname.c_str(), &db_handle, SQLITE_OPEN_READONLY, NULL);
  EXPECT_EQ(ret, SQLITE_OK);
  std::unordered_map<uint32_t, bool> drive_on_right;
  std::unordered_map<uint32_t, bool> allow_intersection_names;
  AABB2<PointLL> world_box(-180.0f, -90.0f, 180.f, 90.f);
  GraphTileBuilder tilebuilder(graph_reader.tile_dir(), tile_id, true);
  std::unordered_multimap<uint32_t, multi_polygon_type> polys;
  polys = GetAdminInfo(db_handle, drive_on_right, allow_intersection_names, world_box, tilebuilder);
  sqlite3_close(db_handle);

  // For two states part of one country, there should be three polys, etc.
  ASSERT_EQ(polys.size(), 3);
  ASSERT_EQ(drive_on_right.size(), 3);
  ASSERT_EQ(allow_intersection_names.size(), 3);

  // find the GH way/edge in the graph, make sure its there
  auto bigt = findEdge(graph_reader, admin_map.nodes, "GH", "H", tile_id);
  const DirectedEdge * dir_edge = std::get<1>(bigt);
  ASSERT_NE(dir_edge, nullptr);

  // See that nodes G and H are in the correct state
  for ( const auto & node : admin_map.nodes ) {
      const std::string & node_name = node.first;
      if ( node_name == "G" ) {
          uint32_t co_offset = tilebuilder.AddName( "Colorado" );
          const midgard::PointLL &node_latlon = node.second;
          uint32_t state_offset = GetContainingState(tilebuilder, polys, node_latlon);
          ASSERT_EQ( state_offset, co_offset );
      }
      else if ( node_name == "H" ) {
        uint32_t ut_offset = tilebuilder.AddName( "Utah" );
        const midgard::PointLL &node_latlon = node.second;
        uint32_t state_offset = GetContainingState(tilebuilder, polys, node_latlon);
        ASSERT_EQ( state_offset, ut_offset );
      }
  }
}
