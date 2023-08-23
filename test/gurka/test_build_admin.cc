#include <filesystem>

#include <gtest/gtest.h>

#include "baldr/admin.h"
#include "gurka.h"
#include "mjolnir/adminbuilder.h"
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



        M-------N-------O           d
        |    S-----T    |           |
        |    |     |    |           |
        |  a-|--b--|-c  |           |
        |    |     |    |           |
        |    U-----V    |           |
        Q-------R-------P           e-----f

        g----h----i
         \       /
          \     /
           \   /
            \ /
             j


        k-------------l
        |             |
        |   o     q   |
        |   |\   /|   |
        |   |0\ /1|   |
        |   || p ||   |
        |   |2/ \3|   |
        |   |/   \|   |
        |   s     r   |
        |             |
        n-------------m
  )";

  // To define an administrative boundary, the nodes must form a closed polygon.
  const gurka::ways ways = {
      {"AB", {}},
      {"AF", {}},
      {"EF", {}},
      {"EB", {}},
      {"BCDE", {}},
      {"CIJKLDC", {}},
      {"CILDC", {}},
      {"IJKLI", {}},
      {"MNOPRQM", {}},
      {"STVUS", {}},
      {"de", {}}, // not a closed ring
      {"ef", {}}, // not a closed ring
      {"gh", {}}, // not a closed ring
      {"hi", {}}, // not a closed ring
      {"ij", {}}, // not a closed ring
      {"klmnk", {}},
      {"opqrpso", {}},
      {"opso", {}},
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
       }},
      {"ab",
       {
           {"highway", "primary"},
       }},
      {"bc",
       {
           {"highway", "primary"},
       }},
      {"02",
       {
           {"highway", "primary"},
       }},
      {"13",
       {
           {"highway", "primary"},
       }},
  };

  // X lives in Japan which allows named intersections - and is named.
  // gurka automatically names the nodes by their name in the ascii map
  // if you want to make sure there is no name you need to send empty string
  const gurka::nodes nodes = {
      {"X", {{"junction", "named"}, {"name", "namae wa nan desu ka"}}},
      {"Y", {{"junction", "yes"}, {"name", ""}}},
  };

  const gurka::relations relations = {
      {{{
           {gurka::way_member, "AB", "outer"},
           {gurka::way_member, "EB", "outer"},
           {gurka::way_member, "EF", "outer"},
           {gurka::way_member, "AF", "outer"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "4"},
        {"name", "Colorado"}}},

      {{{
           {gurka::way_member, "BCDE", "outer"},
           {gurka::way_member, "EB", "outer"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "4"},
        {"name", "Utah"}}},

      {{{
           {gurka::way_member, "AB", "outer"},
           {gurka::way_member, "BCDE", "outer"},
           {gurka::way_member, "EF", "outer"},
           {gurka::way_member, "AF", "outer"},
       }},
       {{"type", "boundary"}, {"boundary", "administrative"}, {"admin_level", "2"}, {"name", "USA"}}},

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
        {"name", "Japan"}}},

      {{{
           {gurka::way_member, "MNOPRQM", "outer"},
           {gurka::way_member, "STVUS", "inner"}, // austrian enclave, wound wrong
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Germany"}}},

      {{{
           {gurka::way_member, "STVUS", "outer"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Austria"}}},

      {{{
           {gurka::way_member, "de", "outer"},
           {gurka::way_member, "ef", "outer"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Mexico"}}},

      {{{
           {gurka::way_member, "gh", "outer"},
           {gurka::way_member, "hi", "outer"},
           {gurka::way_member, "ij", "outer"},
           {gurka::way_member, "jg", "outer"}, // way is missing from extract
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Madagasgar"}}},

      {{{
           {gurka::way_member, "klmnk", "outer"},
           {gurka::way_member, "opqrpso", "inner"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Netherlands"}}},

      {{{
           {gurka::way_member, "opso", "outer"},
       }},
       {{"type", "boundary"},
        {"boundary", "administrative"},
        {"admin_level", "2"},
        {"name", "Belgium"}}},
  };

  constexpr double gridsize = 100000;
  auto node_layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, nodes, relations, pbf_filename, 0, false);

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

  if (!std::filesystem::exists(workdir)) {
    bool created = std::filesystem::create_directories(workdir);
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
  bool ret = BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);
  EXPECT_TRUE(ret);

  // Load the sqlite and read the countries/states from the admin table
  std::set<std::string> countries, states;
  GetAdminData(dbname, countries, states);

  std::set<std::string> exp_countries = {"Austria", "Germany",     "Japan",
                                         "USA",     "Netherlands", "Belgium"};
  ASSERT_EQ(countries, exp_countries);

  std::set<std::string> exp_states = {"Colorado", "Hyogo", "Kyoto", "Utah"};
  ASSERT_EQ(states, exp_states);

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

  // check a and c are in Germany
  {
    auto a_id = findNode(graph_reader, admin_map.nodes, "a");
    EXPECT_TRUE(a_id.Is_Valid());
    auto c_id = findNode(graph_reader, admin_map.nodes, "c");
    EXPECT_TRUE(c_id.Is_Valid());
    const auto* a = graph_reader.nodeinfo(a_id);
    EXPECT_EQ(a->drive_on_right(), true);
    EXPECT_EQ(a->named_intersection(), false);
    const auto* c = graph_reader.nodeinfo(c_id);
    EXPECT_EQ(c->drive_on_right(), true);
    EXPECT_EQ(c->named_intersection(), false);
    AdminInfo a_admin = graph_reader.GetGraphTile(a_id)->admininfo(a->admin_index());
    EXPECT_EQ(a_admin.state_text(), "");
    EXPECT_EQ(a_admin.country_text(), "Germany");
    AdminInfo c_admin = graph_reader.GetGraphTile(c_id)->admininfo(c->admin_index());
    EXPECT_EQ(c_admin.state_text(), "");
    EXPECT_EQ(c_admin.country_text(), "Germany");
  }

  // b is in an austrian enclave
  {
    auto b_id = findNode(graph_reader, admin_map.nodes, "b");
    EXPECT_TRUE(b_id.Is_Valid());
    const auto* b = graph_reader.nodeinfo(b_id);
    EXPECT_EQ(b->drive_on_right(), true);
    EXPECT_EQ(b->named_intersection(), false);
    AdminInfo b_admin = graph_reader.GetGraphTile(b_id)->admininfo(b->admin_index());
    EXPECT_EQ(b_admin.state_text(), "");
    EXPECT_EQ(b_admin.country_text(), "Austria");
  }

  // 0 is in a belgian enclave
  {
    auto zero_id = findNode(graph_reader, admin_map.nodes, "0");
    EXPECT_TRUE(zero_id.Is_Valid());
    const auto* zero = graph_reader.nodeinfo(zero_id);
    EXPECT_EQ(zero->drive_on_right(), true);
    EXPECT_EQ(zero->named_intersection(), false);
    AdminInfo zero_admin = graph_reader.GetGraphTile(zero_id)->admininfo(zero->admin_index());
    EXPECT_EQ(zero_admin.state_text(), "");
    EXPECT_EQ(zero_admin.country_text(), "Belgium");
  }

  // 1 is orphaned in an enclave of the netherlands with no admin coverage
  {
    auto one_id = findNode(graph_reader, admin_map.nodes, "1");
    EXPECT_TRUE(one_id.Is_Valid());
    const auto* one = graph_reader.nodeinfo(one_id);
    EXPECT_EQ(one->drive_on_right(), false);
    EXPECT_EQ(one->named_intersection(), false);
    AdminInfo one_admin = graph_reader.GetGraphTile(one_id)->admininfo(one->admin_index());
    EXPECT_EQ(one_admin.state_text(), "None");
    EXPECT_EQ(one_admin.country_text(), "None");
  }
}
