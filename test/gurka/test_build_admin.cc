#include <gtest/gtest.h>

#include "baldr/admin.h"
#include "filesystem.h"
#include "gurka.h"
#include "mjolnir/admin.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/pbfadminparser.h"
#include "mjolnir/pbfgraphparser.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

namespace {

// GetAdminInfo() requires a sqlite db handle and tiles. This creates a mock
// graph with two states part of the same country - with a highway between them.
valhalla::gurka::map BuildPBF(const std::string& workdir) {
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
  auto node_layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, {}, relations, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = node_layout;
  return result;
}

void BuildTiles(valhalla::gurka::map& admin_map, const std::vector<std::string>& pbf_filenames) {
  build_tile_set(admin_map.config, pbf_filenames, mjolnir::BuildStage::kInitialize,
                 mjolnir::BuildStage::kValidate, false);
}

void GetAdminData(const std::string& dbname,
                  std::set<std::string>& countries,
                  std::set<std::string>& states) {
  countries.clear();
  states.clear();

  // Load the admin.sqlite table we just created
  sqlite3* db_handle = NULL;
  uint32_t ret = sqlite3_open_v2(dbname.c_str(), &db_handle, SQLITE_OPEN_READONLY, NULL);
  EXPECT_EQ(ret, SQLITE_OK);

  sqlite3_stmt* stmt = 0;
  std::string sql = "SELECT admin_level, name from admins;";

  uint32_t result = 0;
  bool dor = true;
  bool intersection_name = false;
  ret = sqlite3_prepare_v2(db_handle, sql.c_str(), sql.length(), &stmt, 0);

  if (ret == SQLITE_OK || ret == SQLITE_ERROR) {
    result = sqlite3_step(stmt);

    if (result == SQLITE_DONE) {
      sqlite3_finalize(stmt);
      stmt = 0;
      return;
    }
  }

  while (result == SQLITE_ROW) {
    int admin_level = 0;
    if (sqlite3_column_type(stmt, 0) == SQLITE_INTEGER)
      admin_level = sqlite3_column_int(stmt, 0);
    EXPECT_TRUE((admin_level == 4) || (admin_level == 2));

    std::string name;
    if (sqlite3_column_type(stmt, 1) == SQLITE_TEXT)
      name = (char*)sqlite3_column_text(stmt, 1);
    EXPECT_TRUE(!name.empty());

    if (admin_level == 4)
      states.insert(name);
    else
      countries.insert(name);

    result = sqlite3_step(stmt);
  }

  if (stmt) {
    sqlite3_finalize(stmt);
    stmt = 0;
  }
}

} // anonymous namespace

TEST(AdminTest, TestBuildAdminFromPBF) {
  //======================================================================
  // part I: create a mock graph, build a pbf from it, build a sqlite
  // from that pbf, read a few bits from the sqlite to prove its there
  // and has the things we'd expect.
  //======================================================================

  // Create test/data/admin/map.pbf
  const std::string workdir = "test/data/admin";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map admin_map = BuildPBF(workdir);

  boost::property_tree::ptree& pt = admin_map.config;
  pt.put("mjolnir.concurrency", 1);
  pt.put("mjolnir.id_table_size", 1000);
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", workdir + "/admin.sqlite");
  pt.put("mjolnir.timezone", workdir + "/not_needed.sqlite");

  std::string dbname = pt.get<std::string>("mjolnir.admin");

  // Given map.pbf, BuildAdminFromPBF() creates test/data/admin.sqlite.
  std::vector<std::string> input_files = {workdir + "/map.pbf"};
  BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);

  // Load the sqlite and read the countries/states from the admin table
  std::set<std::string> countries, states;
  GetAdminData(dbname, countries, states);

  std::set<std::string> exp_countries = {"USA"};
  EXPECT_EQ(countries, exp_countries);

  std::set<std::string> exp_states = {"Colorado", "Utah"};
  EXPECT_EQ(states, exp_states);

  //======================================================================
  // part II: build the tile data. Call GetAdminInfo() which reads
  // sqlite and tile data. Confirm the polygonal data is correct
  // with a few simple containment tests.
  //======================================================================
  BuildTiles(admin_map, input_files);

  // reverse engineer the tile_id from the nodes that make up
  // our mock map.pbf. Its probably overkill to check every node
  // for its tile-id if I know they are all in the same tile-id,
  // but its a good exercise.
  std::unordered_set<GraphId> tile_ids;
  for (const auto& node : admin_map.nodes) {
    const midgard::PointLL& latlon = node.second;
    GraphId tile_id = TileHierarchy::GetGraphId(latlon, 0);
    tile_ids.insert(tile_id);
  }
  EXPECT_EQ(tile_ids.size(), 1);

  GraphId tile_id(*tile_ids.begin());

  // Create a GraphReader so we can create a GraphTileBuilder
  GraphReader graph_reader(pt.get_child("mjolnir"));
  auto t = GraphTile::Create(graph_reader.tile_dir(), tile_id);
  EXPECT_TRUE(t);
  GraphTileBuilder tilebuilder(graph_reader.tile_dir(), tile_id, true);

  // Load the admin.sqlite table we just created
  sqlite3* db_handle = nullptr;
  uint32_t ret = sqlite3_open_v2(dbname.c_str(), &db_handle, SQLITE_OPEN_READONLY, nullptr);
  EXPECT_EQ(ret, SQLITE_OK);

  // Call GetAdminInfo.
  std::unordered_multimap<uint32_t, multi_polygon_type> polys;
  std::unordered_map<uint32_t, bool> drive_on_right;
  std::unordered_map<uint32_t, bool> allow_intersection_names;
  AABB2<PointLL> world_box(-180.0f, -90.0f, 180.f, 90.f);
  polys = GetAdminInfo(db_handle, drive_on_right, allow_intersection_names, world_box, tilebuilder);
  sqlite3_close(db_handle);

  // For two states part of one country, there should be three polys, etc.
  EXPECT_EQ(polys.size(), 3);
  EXPECT_EQ(drive_on_right.size(), 3);
  EXPECT_EQ(allow_intersection_names.size(), 3);

  // find the GH way/edge in the graph, make sure its there
  GraphId edge_id;
  const DirectedEdge* edge = nullptr;
  GraphId opp_edge_id;
  const DirectedEdge* opp_edge = nullptr;
  std::tie(edge_id, edge, opp_edge_id, opp_edge) =
      findEdge(graph_reader, admin_map.nodes, "GH", "H", tile_id);
  EXPECT_NE(edge, nullptr);
  EXPECT_NE(opp_edge, nullptr);

  // H is the end node of edge GH
  {
    auto H_tile = graph_reader.GetGraphTile(edge_id);
    const DirectedEdge* GH_edge = H_tile->directededge(edge_id);
    EXPECT_EQ(GH_edge, edge);
    GraphId H_node_id = GH_edge->endnode();
    const NodeInfo* H_node = H_tile->node(H_node_id);
    AdminInfo H_admin = H_tile->admininfo(H_node->admin_index());
    EXPECT_EQ(H_admin.state_text(), "Utah");
    EXPECT_EQ(H_admin.country_text(), "USA");
  }

  // G is the end node of opp_edge HG
  {
    auto G_tile = graph_reader.GetGraphTile(opp_edge_id);
    const DirectedEdge* HG_edge = G_tile->directededge(opp_edge_id);
    EXPECT_EQ(HG_edge, opp_edge);
    GraphId G_node_id = HG_edge->endnode();
    const NodeInfo* G_node = G_tile->node(G_node_id);
    AdminInfo G_admin = G_tile->admininfo(G_node->admin_index());
    EXPECT_EQ(G_admin.state_text(), "Colorado");
    EXPECT_EQ(G_admin.country_text(), "USA");
  }
}
