#include <filesystem>

#include <gtest/gtest.h>

#include "baldr/admin.h"
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
        |               |
        |  G--------H   |
        |               |
        |  I--------J   |
        |               |
        |  K--------L   |
        |               |
        F-------E-------D
  )";

  // To define an administrative boundary, the nodes must form a closed polygon.
  const gurka::ways ways =
      {{"ABCDEFA", {}},
       {"GH", {{"highway", "footway"}}},
       {"IJ", {{"highway", "footway"}, {"footway", "sidewalk"}}},
       {"KL", {{"highway", "footway"}, {"crossing", "zebra"}, {"footway", "crossing"}}}};

  const gurka::relations relations = {{{{{gurka::way_member, "ABCDEFA", "outer"}}},
                                       {{"type", "boundary"},
                                        {"boundary", "administrative"},
                                        {"admin_level", "2"},
                                        {"name", "Belarus"}}}};

  constexpr double gridsize = 10;
  auto node_layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(node_layout, ways, {}, relations, pbf_filename);

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
  const std::string workdir = "test/data/admin_belarus";

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

  std::set<std::string> exp_countries = {"Belarus"};
  EXPECT_EQ(countries, exp_countries);

  std::set<std::string> exp_states = {};
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

  // footway only
  EXPECT_EQ(GH_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(GH_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));

  EXPECT_EQ(HG_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(HG_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));

  //----------------------------
  // Get/assert info on I & J
  //----------------------------
  GraphId IJ_edge_id;
  const DirectedEdge* IJ_edge = nullptr;
  GraphId JI_edge_id;
  const DirectedEdge* JI_edge = nullptr;
  std::tie(IJ_edge_id, IJ_edge, JI_edge_id, JI_edge) =
      findEdge(graph_reader, admin_map.nodes, "IJ", "J");
  EXPECT_NE(IJ_edge, nullptr);
  EXPECT_NE(JI_edge, nullptr);

  // sidewalk
  EXPECT_EQ(IJ_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(IJ_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));

  EXPECT_EQ(JI_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(JI_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));

  //----------------------------
  // Get/assert info on K & L
  //----------------------------
  GraphId KL_edge_id;
  const DirectedEdge* KL_edge = nullptr;
  GraphId LK_edge_id;
  const DirectedEdge* LK_edge = nullptr;
  std::tie(KL_edge_id, KL_edge, LK_edge_id, LK_edge) =
      findEdge(graph_reader, admin_map.nodes, "KL", "L");
  EXPECT_NE(KL_edge, nullptr);
  EXPECT_NE(LK_edge, nullptr);

  // sidewalk
  EXPECT_EQ(KL_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(KL_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));

  EXPECT_EQ(LK_edge->forwardaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
  EXPECT_EQ(LK_edge->reverseaccess(), (kPedestrianAccess | kWheelchairAccess | kBicycleAccess));
}
