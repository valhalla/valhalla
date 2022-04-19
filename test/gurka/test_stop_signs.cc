#include "gurka.h"
#include "test/test.h"
#include <gtest/gtest.h>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;
using namespace valhalla::mjolnir;

valhalla::gurka::map BuildPBF(const std::string& workdir) {
  const std::string ascii_map = R"(
               
               F        G         H                 P
               |        |         |                 |
               |        |/-----2--D--------L--------M-----4--R--------S
      A--------B-----1--C                           |
               |        |\-----3--E--------N--------O--------T-----5--U
               |        |         |                 |
               I        J         K                 Q
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "primary"}}},
      {"B1C", {{"highway", "primary"}}},
      {"C3E", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"EN", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"NO", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"OT", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"T5U", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"ML", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"SR", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"R4M", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"LD", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"D2C", {{"highway", "primary"}, {"oneway", "yes"}}},
      {"DH", {{"highway", "residential"}}},
      {"EK", {{"highway", "residential"}}},
      {"CG", {{"highway", "residential"}}},
      {"CJ", {{"highway", "residential"}}},
      {"BF", {{"highway", "residential"}}},
      {"BI", {{"highway", "residential"}}},
      {"OQ", {{"highway", "residential"}}},
      {"OM", {{"highway", "residential"}}},
      {"MP", {{"highway", "residential"}}},
  };

  const gurka::nodes nodes = {{"1", {{"highway", "stop"}, {"direction", "backward"}}},
                              {"2", {{"highway", "stop"}, {"direction", "forward"}}},
                              {"3", {{"highway", "stop"}}},
                              {"B", {{"highway", "stop"}, {"stop", "minor"}}},
                              {"O", {{"highway", "stop"}}},
                              {"4", {{"highway", "stop"}, {"direction", "forward"}}},
                              {"5", {{"highway", "stop"}}}};

  constexpr double gridsize = 100;

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize, {5.1079374, 52.0887174});

  auto pbf_filename = workdir + "/map.pbf";
  detail::build_pbf(layout, ways, nodes, {}, pbf_filename);

  valhalla::gurka::map result;
  result.nodes = layout;

  return result;
}

TEST(Standalone, StopSigns) {

  const std::string workdir = "test/data/gurka_stop_signs";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  valhalla::gurka::map map = BuildPBF(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/netherlands_admin.sqlite"};
  boost::property_tree::ptree& pt = map.config;
  pt.put("mjolnir.tile_dir", workdir + "/tiles");
  pt.put("mjolnir.admin", sqlite);

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  build_tile_set(pt, input_files, mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kValidate,
                 false);

  GraphReader graph_reader(pt.get_child("mjolnir"));

  // edges should not be marked with a stop sign as they are not minor
  {
    GraphId AB_edge_id;
    const DirectedEdge* AB_edge = nullptr;
    GraphId BA_edge_id;
    const DirectedEdge* BA_edge = nullptr;
    std::tie(AB_edge_id, AB_edge, BA_edge_id, BA_edge) = findEdge(graph_reader, map.nodes, "AB", "B");
    EXPECT_NE(AB_edge, nullptr);
    EXPECT_NE(BA_edge, nullptr);

    EXPECT_EQ(AB_edge->stop_sign(), false);
    EXPECT_EQ(BA_edge->stop_sign(), false);
  }

  // CB edge should be marked with a stop sign as there is a stop sign in the backward direction
  {
    GraphId BC_edge_id;
    const DirectedEdge* BC_edge = nullptr;
    GraphId CB_edge_id;
    const DirectedEdge* CB_edge = nullptr;
    std::tie(BC_edge_id, BC_edge, CB_edge_id, CB_edge) =
        findEdge(graph_reader, map.nodes, "B1C", "C");
    EXPECT_NE(BC_edge, nullptr);
    EXPECT_NE(CB_edge, nullptr);

    EXPECT_EQ(BC_edge->stop_sign(), false);
    EXPECT_EQ(CB_edge->stop_sign(), true);
  }

  // FB should be marked with a stop sign
  {
    GraphId BF_edge_id;
    const DirectedEdge* BF_edge = nullptr;
    GraphId FB_edge_id;
    const DirectedEdge* FB_edge = nullptr;
    std::tie(BF_edge_id, BF_edge, FB_edge_id, FB_edge) = findEdge(graph_reader, map.nodes, "BF", "F");
    EXPECT_NE(BF_edge, nullptr);
    EXPECT_NE(FB_edge, nullptr);

    EXPECT_EQ(BF_edge->stop_sign(), false);
    EXPECT_EQ(FB_edge->stop_sign(), true);
  }

  // BI should be marked with a stop sign
  {
    GraphId BI_edge_id;
    const DirectedEdge* BI_edge = nullptr;
    GraphId IB_edge_id;
    const DirectedEdge* IB_edge = nullptr;
    std::tie(BI_edge_id, BI_edge, IB_edge_id, IB_edge) = findEdge(graph_reader, map.nodes, "BI", "I");
    EXPECT_NE(BI_edge, nullptr);
    EXPECT_NE(IB_edge, nullptr);

    EXPECT_EQ(BI_edge->stop_sign(), false);
    EXPECT_EQ(IB_edge->stop_sign(), true);
  }

  // edge DC should only be marked with a stop sign
  {
    GraphId DC_edge_id;
    const DirectedEdge* DC_edge = nullptr;
    GraphId CD_edge_id;
    const DirectedEdge* CD_edge = nullptr;
    std::tie(DC_edge_id, DC_edge, CD_edge_id, CD_edge) =
        findEdge(graph_reader, map.nodes, "D2C", "C");
    EXPECT_NE(DC_edge, nullptr);
    EXPECT_NE(CD_edge, nullptr);

    EXPECT_EQ(DC_edge->stop_sign(), true);
    EXPECT_EQ(CD_edge->stop_sign(), false);
  }

  // edges CE should be marked with a stop sign as there is no direction.
  // due to the onewayness on EC, we toss the stop sign
  {
    GraphId CE_edge_id;
    const DirectedEdge* CE_edge = nullptr;
    GraphId EC_edge_id;
    const DirectedEdge* EC_edge = nullptr;
    std::tie(CE_edge_id, CE_edge, EC_edge_id, EC_edge) =
        findEdge(graph_reader, map.nodes, "C3E", "E");
    EXPECT_NE(CE_edge, nullptr);
    EXPECT_NE(EC_edge, nullptr);

    EXPECT_EQ(CE_edge->stop_sign(), true);
    EXPECT_EQ(EC_edge->stop_sign(), false);
  }

  {
    GraphId NO_edge_id;
    const DirectedEdge* NO_edge = nullptr;
    GraphId ON_edge_id;
    const DirectedEdge* ON_edge = nullptr;
    std::tie(NO_edge_id, NO_edge, ON_edge_id, ON_edge) = findEdge(graph_reader, map.nodes, "NO", "O");
    EXPECT_NE(NO_edge, nullptr);
    EXPECT_NE(ON_edge, nullptr);

    EXPECT_EQ(NO_edge->stop_sign(), true);
    EXPECT_EQ(ON_edge->stop_sign(), false);

    GraphId OM_edge_id;
    const DirectedEdge* OM_edge = nullptr;
    GraphId MO_edge_id;
    const DirectedEdge* MO_edge = nullptr;
    std::tie(OM_edge_id, OM_edge, MO_edge_id, MO_edge) = findEdge(graph_reader, map.nodes, "OM", "M");
    EXPECT_NE(OM_edge, nullptr);
    EXPECT_NE(MO_edge, nullptr);

    EXPECT_EQ(OM_edge->stop_sign(), false);
    EXPECT_EQ(MO_edge->stop_sign(), true);

    GraphId OQ_edge_id;
    const DirectedEdge* OQ_edge = nullptr;
    GraphId QO_edge_id;
    const DirectedEdge* QO_edge = nullptr;
    std::tie(OQ_edge_id, OQ_edge, QO_edge_id, QO_edge) = findEdge(graph_reader, map.nodes, "OQ", "Q");
    EXPECT_NE(OQ_edge, nullptr);
    EXPECT_NE(QO_edge, nullptr);

    EXPECT_EQ(OQ_edge->stop_sign(), false);
    EXPECT_EQ(QO_edge->stop_sign(), true);

    GraphId OT_edge_id;
    const DirectedEdge* OT_edge = nullptr;
    GraphId TO_edge_id;
    const DirectedEdge* TO_edge = nullptr;
    std::tie(OT_edge_id, OT_edge, TO_edge_id, TO_edge) = findEdge(graph_reader, map.nodes, "OT", "T");
    EXPECT_NE(OT_edge, nullptr);
    EXPECT_NE(TO_edge, nullptr);

    EXPECT_EQ(TO_edge->stop_sign(), false); // false here due to being a oneway edge
    EXPECT_EQ(OT_edge->stop_sign(), false);
  }

  {
    GraphId RM_edge_id;
    const DirectedEdge* RM_edge = nullptr;
    GraphId MR_edge_id;
    const DirectedEdge* MR_edge = nullptr;
    std::tie(RM_edge_id, RM_edge, MR_edge_id, MR_edge) =
        findEdge(graph_reader, map.nodes, "R4M", "M");
    EXPECT_NE(RM_edge, nullptr);
    EXPECT_NE(MR_edge, nullptr);

    EXPECT_EQ(RM_edge->stop_sign(), true);
    EXPECT_EQ(MR_edge->stop_sign(), false);
  }

  {
    GraphId TU_edge_id;
    const DirectedEdge* TU_edge = nullptr;
    GraphId UT_edge_id;
    const DirectedEdge* UT_edge = nullptr;
    std::tie(TU_edge_id, TU_edge, UT_edge_id, UT_edge) =
        findEdge(graph_reader, map.nodes, "T5U", "U");
    EXPECT_NE(TU_edge, nullptr);
    EXPECT_NE(UT_edge, nullptr);

    EXPECT_EQ(TU_edge->stop_sign(), true);
    EXPECT_EQ(UT_edge->stop_sign(), false); // no stop sign as we are a oneway road
  }
}
