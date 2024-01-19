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
    GraphReader graph_reader(map.config.get_child("mjolnir"));
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
    GraphReader graph_reader(map.config.get_child("mjolnir"));
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

TEST(Standalone, AccessFilter) {

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

  // CIL should not be deleted
  {
    GraphReader graph_reader = GraphReader(map.config.get_child("mjolnir"));
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
