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
        A-------B-------C-------I-------J
        |       |       |   W   |       |
        |       |       |   |   |       |
        |   G---|---H---|---X---|---Y   |
        |       |       |   |   |       |
        |       |       |   Z   |       |
        F-------E-------D-------L-------K
  )";

  // To define an administrative boundary, the nodes must form a closed polygon.
  const gurka::ways ways = {{"ABCDEFA", {}},
                            {"ABEFA", {}},
                            {"BCDEB", {}},
                            {"CIJKLDC", {}},
                            {"CILDC", {}},
                            {"IJKLI", {}},
                            {"GH",
                             {
                                 {"highway", "primary"},
                             }},
                            {"HX",
                             {
                                 {"highway", "primary"},
                             }},
                            {"XY",
                             {
                                 {"highway", "primary"},
                             }},
                            {"WX",
                             {
                                 {"highway", "primary"},
                             }},
                            {"XZ",
                             {
                                 {"highway", "primary"},
                             }}};

  // X lives Japan which allows named intersections - and is named.
  // gurka automatically names the nodes by their name in the ascii map
  // if you want to make sure there is no name you need to send empty string
  const gurka::nodes nodes = {
      {"X", {{"junction", "named"}, {"name", "namae wa nan desu ka"}}},
      {"Y", {{"junction", "yes"}, {"name", ""}}},
  };

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
                                        {"name", "USA"}}},
                                      {{{{gurka::way_member, "CILDC", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "4"},
                                        {"name", "Hyogo"}}},
                                      {{{{gurka::way_member, "IJKLI", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "4"},
                                        {"name", "Kyoto"}}},
                                      {{{{gurka::way_member, "CIJKLDC", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "2"},
                                        {"name", "Japan"}}}};

  constexpr double gridsize = 10;
  auto node_layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, nodes, relations, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = node_layout;
  return result;
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

  std::set<std::string> exp_countries = {"Japan", "USA"};
  EXPECT_EQ(countries, exp_countries);

  std::set<std::string> exp_states = {"Colorado", "Hyogo", "Kyoto", "Utah"};
  EXPECT_EQ(states, exp_states);

  //======================================================================
  // part II: build the tile data. Query/assert things about the highway
  // nodes that span the two countries.
  //======================================================================
  build_tile_set(admin_map.config, input_files, mjolnir::BuildStage::kInitialize,
                 mjolnir::BuildStage::kValidate, false);

  GraphReader graph_reader(pt.get_child("mjolnir"));

  //----------------------------
  // Get/assert info on G & H
  //----------------------------
  GraphId GH_edge_id;
  const DirectedEdge* GH_edge = nullptr;
  GraphId HG_edge_id;
  const DirectedEdge* HG_edge = nullptr;
  std::tie(GH_edge_id, GH_edge, HG_edge_id, HG_edge) =
      findEdge(graph_reader, admin_map.nodes, "GH", "H");
  EXPECT_NE(GH_edge, nullptr);
  EXPECT_NE(HG_edge, nullptr);

  // H is the end node of edge GH
  {
    GraphId H_node_id = GH_edge->endnode();
    auto H_tile = graph_reader.GetGraphTile(H_node_id);
    const NodeInfo* H_node = H_tile->node(H_node_id);
    EXPECT_EQ(H_node->drive_on_right(), true);
    EXPECT_EQ(H_node->named_intersection(), false);
    AdminInfo H_admin = H_tile->admininfo(H_node->admin_index());
    EXPECT_EQ(H_admin.state_text(), "Utah");
    EXPECT_EQ(H_admin.country_text(), "USA");
  }

  // G is the end node of edge HG
  {
    GraphId G_node_id = HG_edge->endnode();
    auto G_tile = graph_reader.GetGraphTile(G_node_id);
    const NodeInfo* G_node = G_tile->node(G_node_id);
    EXPECT_EQ(G_node->drive_on_right(), true);
    EXPECT_EQ(G_node->named_intersection(), false);
    AdminInfo G_admin = G_tile->admininfo(G_node->admin_index());
    EXPECT_EQ(G_admin.state_text(), "Colorado");
    EXPECT_EQ(G_admin.country_text(), "USA");
  }

  //----------------------------
  // Get/assert info on X & Y
  //----------------------------
  GraphId XY_edge_id;
  const DirectedEdge* XY_edge = nullptr;
  GraphId YX_edge_id;
  const DirectedEdge* YX_edge = nullptr;
  std::tie(XY_edge_id, XY_edge, YX_edge_id, YX_edge) =
      findEdge(graph_reader, admin_map.nodes, "XY", "Y");
  EXPECT_NE(XY_edge, nullptr);
  EXPECT_NE(YX_edge, nullptr);

  // Y is the end node of edge XY
  {
    GraphId Y_node_id = XY_edge->endnode();
    auto Y_tile = graph_reader.GetGraphTile(Y_node_id);
    const NodeInfo* Y_node = Y_tile->node(Y_node_id);
    EXPECT_EQ(Y_node->drive_on_right(), false);
    EXPECT_EQ(Y_node->named_intersection(), false);
    AdminInfo Y_admin = Y_tile->admininfo(Y_node->admin_index());
    EXPECT_EQ(Y_admin.state_text(), "Kyoto");
    EXPECT_EQ(Y_admin.country_text(), "Japan");
  }

  // X is the end node of edge YX
  {
    GraphId X_node_id = YX_edge->endnode();
    auto X_tile = graph_reader.GetGraphTile(X_node_id);
    const NodeInfo* X_node = X_tile->node(X_node_id);
    EXPECT_EQ(X_node->drive_on_right(), false);
    EXPECT_EQ(X_node->named_intersection(), true);
    EXPECT_EQ(X_tile->GetSigns(X_node_id.id(), true).at(0).text(), "namae wa nan desu ka");
    AdminInfo X_admin = X_tile->admininfo(X_node->admin_index());
    EXPECT_EQ(X_admin.state_text(), "Hyogo");
    EXPECT_EQ(X_admin.country_text(), "Japan");
  }
}
