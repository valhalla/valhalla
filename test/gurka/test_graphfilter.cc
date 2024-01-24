#include "baldr/graphreader.h"
#include "mjolnir/util.h"

#include "gurka.h"
#include "test/test.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;

const std::string work_dir = {VALHALLA_BUILD_DIR "test/data/gurka_graphfilter"};

TEST(Standalone, SimpleFilter) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

             M              P         R              W    Y
             |              |         |              |    |
             |              |         |              |    |
        A----B----C----D----E----F----G----H----I----J----K----L
                  |    |         |         |    |         |
                  |    |         Q----S----T    |         |
                  N    O              |         V         X
                                      |
                                      U 
  )";

  const gurka::ways ways = {
      {"ABCDEFGHIJKL", {{"highway", "residential"}, {"osm_id", "100"}, {"name", "East Fort Avenue"}}},
      {"BM", {{"highway", "residential"}, {"osm_id", "101"}, {"name", "Hull Street"}}},
      {"CN",
       {{"highway", "residential"},
        {"osm_id", "102"},
        {"name", "Latrobe Park Terrace"},
        {"oneway", "yes"}}},
      {"DO", {{"highway", "footway"}, {"osm_id", "103"}, {"foot", "yes"}}},
      {"EP",
       {{"highway", "residential"},
        {"osm_id", "104"},
        {"name", "Cooksie Street"},
        {"oneway", "yes"}}},
      {"FQ", {{"highway", "footway"}, {"osm_id", "105"}, {"foot", "yes"}}},
      {"QST", {{"highway", "footway"}, {"osm_id", "106"}, {"foot", "yes"}}},
      {"SU", {{"highway", "footway"}, {"osm_id", "107"}, {"foot", "yes"}}},
      {"GR", {{"highway", "residential"}, {"osm_id", "108"}, {"name", "Towson Street"}}},
      {"HT", {{"highway", "footway"}, {"osm_id", "109"}, {"foot", "yes"}}},
      {"IV", {{"highway", "footway"}, {"osm_id", "110"}, {"foot", "yes"}}},
      {"JW", {{"highway", "residential"}, {"osm_id", "111"}, {"name", "Richardson Street"}}},
      {"YKX", {{"highway", "residential"}, {"osm_id", "112"}, {"name", "Andre Street"}}},
  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-76.59223, 39.26825});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});

  // all footways will be deleted.  check for DO
  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));
    GraphId DO_edge_id;
    const DirectedEdge* DO_edge = nullptr;
    GraphId OD_edge_id;
    const DirectedEdge* OD_edge = nullptr;
    std::tie(DO_edge_id, DO_edge, OD_edge_id, OD_edge) =
        findEdge(graph_reader, map.nodes, "DO", "O", baldr::GraphId{});
    EXPECT_EQ(DO_edge, nullptr);
    EXPECT_EQ(OD_edge, nullptr);
  }

  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");
  gurka::assert::raw::expect_path(result, {"East Fort Avenue", "East Fort Avenue", "East Fort Avenue",
                                           "East Fort Avenue", "East Fort Avenue", "East Fort Avenue",
                                           "East Fort Avenue"});

  // reconvert again, but include pedestrian edges
  map = gurka::buildtiles(layout, ways, {}, {}, work_dir,
                          {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                           {"mjolnir.include_pedestrian", "true"}});

  // all footways will not be deleted
  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));
    GraphId DO_edge_id;
    const DirectedEdge* DO_edge = nullptr;
    GraphId OD_edge_id;
    const DirectedEdge* OD_edge = nullptr;
    std::tie(DO_edge_id, DO_edge, OD_edge_id, OD_edge) =
        findEdge(graph_reader, map.nodes, "DO", "O", baldr::GraphId{});
    EXPECT_NE(DO_edge, nullptr);
    EXPECT_NE(OD_edge, nullptr);
  }

  // more edges will exist
  result = gurka::do_action(valhalla::Options::route, map, {"A", "L"}, "auto");
  gurka::assert::raw::expect_path(result, {"East Fort Avenue", "East Fort Avenue", "East Fort Avenue",
                                           "East Fort Avenue", "East Fort Avenue", "East Fort Avenue",
                                           "East Fort Avenue", "East Fort Avenue", "East Fort Avenue",
                                           "East Fort Avenue", "East Fort Avenue"});
}

TEST(Standalone, SimpleFilter2) {

  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

        J----K----L----M----------------------N
        |    |    |    |                      |
        |    H    I    |                      |
        |    |    |    |                      |
        A----B----C----D----E------------F----G
        |              |     \          /     |
        |              |      \        /      |
        O              P       Q------R       S                  
  )";

  const gurka::ways ways = {
      {"JKLMN",
       {{"highway", "secondary"},
        {"osm_id", "100"},
        {"bicycle", "no"},
        {"foot", "no"},
        {"name", "Vondellaan"},
        {"oneway", "yes"}}},
      {"ABCDEFG", {{"highway", "cycleway"}, {"osm_id", "101"}, {"oneway", "yes"}}},
      {"JAO",
       {{"highway", "residential"},
        {"osm_id", "102"},
        {"bicycle", "no"},
        {"foot", "no"},
        {"name", "Croesestraat"},
        {"oneway", "yes"},
        {"oneway:bicycle", "no"}}},
      {"BHK", {{"highway", "cycleway"}, {"osm_id", "103"}, {"oneway", "yes"}}},
      {"CIL", {{"highway", "footway"}, {"osm_id", "104"}}},
      {"MDP", {{"highway", "service"}, {"osm_id", "105"}, {"access", "private"}}},
      {"EQRF",
       {{"highway", "service"}, {"osm_id", "106"}, {"access", "delivery"}, {"oneway", "yes"}}},
      {"NG", {{"highway", "unclassified"}, {"osm_id", "107"}, {"name", "Oosterkade"}}},
      {"GS", {{"highway", "cycleway"}, {"osm_id", "108"}}},
  };

  const gurka::nodes nodes = {
      {"H", {{"highway", "traffic_signals"}}},
      {"I", {{"highway", "traffic_signals"}, {"traffic_signals:direction", "backward"}}},
  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {5.11668, 52.07826});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});

  // CIL should be deleted
  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));
    GraphId CL_edge_id;
    const DirectedEdge* CL_edge = nullptr;
    GraphId LC_edge_id;
    const DirectedEdge* LC_edge = nullptr;
    std::tie(CL_edge_id, CL_edge, LC_edge_id, LC_edge) =
        findEdge(graph_reader, map.nodes, "CIL", "L", baldr::GraphId{});
    EXPECT_EQ(CL_edge, nullptr);
    EXPECT_EQ(LC_edge, nullptr);
  }

  auto result = gurka::do_action(valhalla::Options::route, map, {"J", "N"}, "auto");
  gurka::assert::raw::expect_path(result, {"Vondellaan", "Vondellaan", "Vondellaan"});

  // reconvert again, but include pedestrian edges
  map = gurka::buildtiles(layout, ways, nodes, {}, work_dir,
                          {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                           {"mjolnir.include_pedestrian", "true"}});
  GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

  // CIL should not be deleted
  {
    GraphId CL_edge_id;
    const DirectedEdge* CL_edge = nullptr;
    GraphId LC_edge_id;
    const DirectedEdge* LC_edge = nullptr;
    std::tie(CL_edge_id, CL_edge, LC_edge_id, LC_edge) =
        findEdge(graph_reader, map.nodes, "CIL", "L", baldr::GraphId{});
    EXPECT_NE(CL_edge, nullptr);
    EXPECT_NE(LC_edge, nullptr);
  }

  // should have an extra edge since CIL was not deleted
  result = gurka::do_action(valhalla::Options::route, map, {"J", "N"}, "auto");
  gurka::assert::raw::expect_path(result, {"Vondellaan", "Vondellaan", "Vondellaan", "Vondellaan"});
}

TEST(Standalone, FilterTestComplexRestrictionsSignals) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(
                          L
                          |
                          |
                          M--N
                          |  | 
                          |  |
       A------------------B--C----D-----------E
       |                  |  |
       F------------------G--H----I-----------J
                          |
                          |
                          K
  )";

  const gurka::ways ways = {
      {"ED",
       {{"highway", "primary"},
        {"osm_id", "100"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"DCB",
       {{"highway", "primary"},
        {"osm_id", "101"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"BA",
       {{"highway", "primary"},
        {"osm_id", "102"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"AF", {{"highway", "tertiary"}, {"osm_id", "103"}}},
      {"FG",
       {{"highway", "primary"},
        {"osm_id", "104"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"GHI",
       {{"highway", "primary"},
        {"osm_id", "105"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"IJ",
       {{"highway", "primary"},
        {"osm_id", "106"},
        {"oneway", "yes"},
        {"name", "East Houston Street"}}},
      {"LMB", {{"highway", "tertiary"}, {"osm_id", "107"}, {"name", "Avenue B"}}},
      {"BG", {{"highway", "tertiary"}, {"osm_id", "108"}, {"name", "Avenue B"}}},
      {"GK", {{"highway", "tertiary"}, {"osm_id", "109"}, {"name", "Avenue B"}}},
      {"MN", {{"highway", "footway"}, {"osm_id", "110"}, {"foot", "yes"}}},
      {"NCH", {{"highway", "footway"}, {"osm_id", "111"}, {"foot", "yes"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "DCB", "from"},
           {gurka::way_member, "BG", "via"},
           {gurka::way_member, "GHI", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
  };

  const gurka::nodes nodes = {
      {"M", {{"highway", "traffic_signals"}}},
  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-73.98311, 40.72122});
  auto map =
      gurka::buildtiles(layout, ways, nodes, relations, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});
  GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

  // even though we filitered pedestrian edges, make sure the restriction is not deleted.
  auto result = gurka::do_action(valhalla::Options::route, map, {"E", "I"}, "auto");
  gurka::assert::raw::expect_path(result, {"East Houston Street", "East Houston Street",
                                           "East Houston Street", "East Houston Street", "AF",
                                           "East Houston Street", "East Houston Street",
                                           "East Houston Street"});

  // even though we filitered pedestrian edges, make sure we don't split at the traffic signal
  result = gurka::do_action(valhalla::Options::route, map, {"L", "K"}, "auto");
  gurka::assert::raw::expect_path(result, {"Avenue B", "Avenue B", "Avenue B", "Avenue B"});
}

TEST(Standalone, FilterTestSimpleRestrictions) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(


     H----------D----------I
               / \
              /   \
     J-------E-----C-------K
            /	    \
           F	     B
           |         |
           |         |
           |         |
           G         A
  )";

  const gurka::ways ways = {
      {"ABCD",
       {{"highway", "primary"}, {"osm_id", "100"}, {"oneway", "yes"}, {"name", "Washington Avenue"}}},
      {"DEFG",
       {{"highway", "primary"}, {"osm_id", "101"}, {"oneway", "yes"}, {"name", "Washington Avenue"}}},
      {"HD", {{"highway", "tertiary"}, {"osm_id", "102"}, {"name", "State Street"}}},
      {"DI", {{"highway", "tertiary"}, {"osm_id", "103"}, {"name", "T Street"}}},
      {"JECK", {{"highway", "footway"}, {"osm_id", "104"}, {"foot", "yes"}}},
  };

  const gurka::relations relations = {
      {{
           {gurka::way_member, "ABCD", "from"},
           {gurka::node_member, "D", "via"},
           {gurka::way_member, "DEFG", "to"},
       },
       {
           {"type", "restriction"},
           {"restriction", "no_u_turn"},
       }},
  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-73.94913, 42.81490});
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});
  GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

  // even though we filitered pedestrian edges, make sure the simple restriction is not deleted.  We
  // make a uturn at the H node
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "G"}, "auto");
  gurka::assert::raw::expect_path(result, {"Washington Avenue", "Washington Avenue", "State Street",
                                           "State Street", "Washington Avenue"});

  // JECK should be deleted
  {
    GraphId JK_edge_id;
    const DirectedEdge* JK_edge = nullptr;
    GraphId KJ_edge_id;
    const DirectedEdge* KJ_edge = nullptr;
    std::tie(JK_edge_id, JK_edge, KJ_edge_id, KJ_edge) =
        findEdge(graph_reader, map.nodes, "JECK", "K", baldr::GraphId{});
    EXPECT_EQ(JK_edge, nullptr);
    EXPECT_EQ(KJ_edge, nullptr);
  }
}

TEST(Standalone, FilterTestNodeTypeSignals) {
  constexpr double gridsize_metres = 10;

  const std::string ascii_map = R"(

                F
                |
                G--H
                |  | 
                |  |
     A----------B--C-------D
                |
                |
                E
  
  )";

  const gurka::ways ways = {
      {"ABCD", {{"highway", "secondary"}, {"osm_id", "100"}}},
      {"FGBE", {{"highway", "secondary"}, {"osm_id", "101"}}},
      {"GHC", {{"highway", "footway"}, {"osm_id", "102"}, {"foot", "yes"}}},
  };

  const gurka::nodes nodes = {
      {"C", {{"highway", "traffic_signals"}}},
      {"G", {{"barrier", "gate"}}},
  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-73.94913, 42.81490});
  auto map =
      gurka::buildtiles(layout, ways, nodes, {}, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});
  GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

  // even though we filitered pedestrian edges there should be 3 edges as ABCD has a signal set on the
  // edge
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "D"}, "auto");
  gurka::assert::raw::expect_path(result, {"ABCD", "ABCD", "ABCD"});

  // even though we filitered pedestrian edges there should be 3 edges as G is a gate
  result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "auto");
  gurka::assert::raw::expect_path(result, {"FGBE", "FGBE", "FGBE"});

  // GHC should be deleted
  {
    GraphId GC_edge_id;
    const DirectedEdge* GC_edge = nullptr;
    GraphId CG_edge_id;
    const DirectedEdge* CG_edge = nullptr;
    std::tie(GC_edge_id, GC_edge, CG_edge_id, CG_edge) =
        findEdge(graph_reader, map.nodes, "GHC", "C", baldr::GraphId{});
    EXPECT_EQ(GC_edge, nullptr);
    EXPECT_EQ(CG_edge, nullptr);
  }
}

TEST(Standalone, FilterTestMultipleEdges) {
  constexpr double gridsize_metres = 50;

  const std::string ascii_map = R"(

     A---B---C---D---E---F---G---H---I
         |   |   |   |   |   |   |
         |   |   |   |   |   |   |
         J   K   L   M   N   O   P
  )";

  const gurka::ways ways = {
      {"AB", {{"highway", "secondary"}, {"osm_id", "100"}}},
      {"BC", {{"highway", "secondary"}, {"osm_id", "101"}}},
      {"CD", {{"highway", "secondary"}, {"osm_id", "102"}}},
      {"DE", {{"highway", "secondary"}, {"osm_id", "103"}}},
      {"EF", {{"highway", "secondary"}, {"osm_id", "104"}}},
      // note these next 2 have same way id
      {"FG", {{"highway", "secondary"}, {"osm_id", "105"}}},
      {"GH", {{"highway", "secondary"}, {"osm_id", "105"}}},
      {"HI", {{"highway", "secondary"}, {"osm_id", "106"}}},
      {"BJ", {{"highway", "residential"}, {"osm_id", "107"}}},
      {"CK", {{"highway", "residential"}, {"osm_id", "108"}}},
      {"DL", {{"highway", "footway"}, {"osm_id", "109"}, {"foot", "yes"}}},
      {"EM", {{"highway", "residential"}, {"osm_id", "110"}}},
      {"FN", {{"highway", "residential"}, {"osm_id", "111"}}},
      {"GO", {{"highway", "footway"}, {"osm_id", "112"}, {"foot", "yes"}}},
      {"HP", {{"highway", "residential"}, {"osm_id", "113"}}},

  };

  auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-73.94913, 42.81490});
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, work_dir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.include_pedestrian", "false"}});

  // there should be 7 edges from A to I as even though DJ is a footway, CD and DE have different way
  // ids FG and GH will be aggregated as they have the same way id and are separated by a footway.
  auto result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");
  gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "HI"});

  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

    // DL should be deleted
    GraphId DL_edge_id;
    const DirectedEdge* DL_edge = nullptr;
    GraphId LD_edge_id;
    const DirectedEdge* LD_edge = nullptr;
    std::tie(DL_edge_id, DL_edge, LD_edge_id, LD_edge) =
        findEdge(graph_reader, map.nodes, "DL", "L", baldr::GraphId{});
    EXPECT_EQ(DL_edge, nullptr);
    EXPECT_EQ(LD_edge, nullptr);

    // GO should be deleted
    GraphId GO_edge_id;
    const DirectedEdge* GO_edge = nullptr;
    GraphId OG_edge_id;
    const DirectedEdge* OG_edge = nullptr;
    std::tie(GO_edge_id, GO_edge, OG_edge_id, OG_edge) =
        findEdge(graph_reader, map.nodes, "GO", "O", baldr::GraphId{});
    EXPECT_EQ(GO_edge, nullptr);
    EXPECT_EQ(OG_edge, nullptr);

    // FG should be deleted as it was aggregated
    GraphId FG_edge_id;
    const DirectedEdge* FG_edge = nullptr;
    GraphId GF_edge_id;
    const DirectedEdge* GF_edge = nullptr;
    std::tie(FG_edge_id, FG_edge, GF_edge_id, GF_edge) =
        findEdge(graph_reader, map.nodes, "FG", "G", baldr::GraphId{});
    EXPECT_EQ(FG_edge, nullptr);
    EXPECT_EQ(GF_edge, nullptr);

    // GH should be deleted as it was aggregated
    GraphId GH_edge_id;
    const DirectedEdge* GH_edge = nullptr;
    GraphId HG_edge_id;
    const DirectedEdge* HG_edge = nullptr;
    std::tie(GH_edge_id, GH_edge, HG_edge_id, HG_edge) =
        findEdge(graph_reader, map.nodes, "GH", "H", baldr::GraphId{});
    EXPECT_EQ(GH_edge, nullptr);
    EXPECT_EQ(HG_edge, nullptr);

    // FH is the new aggregated edge.  Make sure is equal to wayid 105
    GraphId FH_edge_id;
    const DirectedEdge* FH_edge = nullptr;
    GraphId HF_edge_id;
    const DirectedEdge* HF_edge = nullptr;
    std::tie(FH_edge_id, FH_edge, HF_edge_id, HF_edge) =
        findEdge(graph_reader, map.nodes, "FG", "H", baldr::GraphId{}, 105);
    EXPECT_NE(FH_edge, nullptr);
    EXPECT_NE(HF_edge, nullptr);

    auto FH = gurka::findEdgeByNodes(graph_reader, layout, "F", "H");
    auto tile = graph_reader.GetGraphTile(std::get<1>(FH)->endnode());
    EXPECT_EQ(tile->edgeinfo(std::get<1>(FH)).wayid(), 105);
  }

  // save and check later
  auto shape = result.trip().routes(0).legs().begin()->shape();

  // Now run again with include_pedestrian = true.  all footways should be back.
  map = gurka::buildtiles(layout, ways, {}, {}, work_dir,
                          {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                           {"mjolnir.include_pedestrian", "true"}});

  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));

    // there should be 8 edges from A to I
    result = gurka::do_action(valhalla::Options::route, map, {"A", "I"}, "auto");
    gurka::assert::raw::expect_path(result, {"AB", "BC", "CD", "DE", "EF", "FG", "GH", "HI"});
    // DL should not be deleted

    GraphId DL_edge_id;
    const DirectedEdge* DL_edge = nullptr;
    GraphId LD_edge_id;
    const DirectedEdge* LD_edge = nullptr;
    std::tie(DL_edge_id, DL_edge, LD_edge_id, LD_edge) =
        findEdge(graph_reader, map.nodes, "DL", "L", baldr::GraphId{});
    EXPECT_NE(DL_edge, nullptr);
    EXPECT_NE(LD_edge, nullptr);

    // GO should not be deleted
    GraphId GO_edge_id;
    const DirectedEdge* GO_edge = nullptr;
    GraphId OG_edge_id;
    const DirectedEdge* OG_edge = nullptr;
    std::tie(GO_edge_id, GO_edge, OG_edge_id, OG_edge) =
        findEdge(graph_reader, map.nodes, "GO", "O", baldr::GraphId{});
    EXPECT_NE(GO_edge, nullptr);
    EXPECT_NE(OG_edge, nullptr);

    // FG should not be deleted
    GraphId FG_edge_id;
    const DirectedEdge* FG_edge = nullptr;
    GraphId GF_edge_id;
    const DirectedEdge* GF_edge = nullptr;
    std::tie(FG_edge_id, FG_edge, GF_edge_id, GF_edge) =
        findEdge(graph_reader, map.nodes, "FG", "G", baldr::GraphId{});
    EXPECT_NE(FG_edge, nullptr);
    EXPECT_NE(GF_edge, nullptr);

    auto tile = graph_reader.GetGraphTile(FG_edge->endnode());
    EXPECT_EQ(tile->edgeinfo(FG_edge).wayid(), 105);

    tile = graph_reader.GetGraphTile(GF_edge->endnode());
    EXPECT_EQ(tile->edgeinfo(GF_edge).wayid(), 105);

    // GH should not be deleted
    GraphId GH_edge_id;
    const DirectedEdge* GH_edge = nullptr;
    GraphId HG_edge_id;
    const DirectedEdge* HG_edge = nullptr;
    std::tie(GH_edge_id, GH_edge, HG_edge_id, HG_edge) =
        findEdge(graph_reader, map.nodes, "GH", "H", baldr::GraphId{});
    EXPECT_NE(GH_edge, nullptr);
    EXPECT_NE(HG_edge, nullptr);

    tile = graph_reader.GetGraphTile(GH_edge->endnode());
    EXPECT_EQ(tile->edgeinfo(GH_edge).wayid(), 105);

    tile = graph_reader.GetGraphTile(HG_edge->endnode());
    EXPECT_EQ(tile->edgeinfo(HG_edge).wayid(), 105);

    // FH should not exist
    ASSERT_THROW(gurka::findEdgeByNodes(graph_reader, layout, "F", "H"), std::runtime_error);
  }
  // the shape between including the pedestrian edges or not, should match
  EXPECT_EQ(shape, result.trip().routes(0).legs().begin()->shape());
}
