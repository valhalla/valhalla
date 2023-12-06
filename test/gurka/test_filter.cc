#include "baldr/graphreader.h"
#include "mjolnir/util.h"

#include "gurka.h"
#include "test/test.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;

valhalla::gurka::map the_map = {};

class FilterData : public ::testing::Test {
protected:
  valhalla::gurka::map BuildPBF(const std::string& workdir) {
    constexpr double gridsize_metres = 100;

    const std::string ascii_map = R"(
		A--------B--------C
                 |
                 |
                 |
                 D
    )";

    /*
     * Notice that we have a different name in the forward
     * and backward direction.  This test confirms that
     * if we filter out footways (i.e., pedestrian edges),
     * the names are still created correctly in the forward
     * and backward direction
     */
    const gurka::ways ways = {
        {"ABC",
         {{"highway", "residential"},
          {"name:forward", "name in forward dir"},
          {"name:backward", "name in backward dir"},
          {"osm_id", "100"}}},
        {"BD", {{"highway", "footway"}}},
    };

    if (!filesystem::exists(workdir)) {
      bool created = filesystem::create_directories(workdir);
      EXPECT_TRUE(created);
    }

    constexpr double gridsize = 100;

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-82.68811, 40.22535});

    auto pbf_filename = workdir + "/map.pbf";
    detail::build_pbf(layout, ways, {}, {}, pbf_filename);

    valhalla::gurka::map result;
    result.nodes = layout;
    return result;
  }
};

///////////////////////////////////////////////////////////////////////////////
TEST_F(FilterData, CheckStreetNamesAfterFilter) {

  const std::string workdir = "test/data/gurka_filter";

  if (!filesystem::exists(workdir)) {
    bool created = filesystem::create_directories(workdir);
    EXPECT_TRUE(created);
  }

  the_map = BuildPBF(workdir);

  const std::string sqlite = {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"};
  the_map.config =
      test::make_config(workdir,
                        {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                         {"mjolnir.tile_dir", workdir + "/tiles"},
                         {"mjolnir.include_pedestrian", "false"}});

  std::vector<std::string> input_files = {workdir + "/map.pbf"};

  build_tile_set(the_map.config, input_files, mjolnir::BuildStage::kInitialize,
                 mjolnir::BuildStage::kCleanup, false);

  GraphReader graph_reader(the_map.config.get_child("mjolnir"));

  GraphId ABC_edge_id;
  const DirectedEdge* ABC_edge = nullptr;
  GraphId CBA_edge_id;
  const DirectedEdge* CBA_edge = nullptr;
  std::tie(ABC_edge_id, ABC_edge, CBA_edge_id, CBA_edge) =
      findEdge(graph_reader, the_map.nodes, "", "C", baldr::GraphId{}, 100);
  EXPECT_NE(ABC_edge, nullptr);
  EXPECT_NE(CBA_edge, nullptr);

  GraphId node_id = ABC_edge->endnode();
  auto tile = graph_reader.GetGraphTile(node_id);
  auto edgeinfo = tile->edgeinfo(ABC_edge);
  std::vector<uint8_t> types;
  auto names_and_types = edgeinfo.GetNamesAndTypes(true);
  ASSERT_EQ(names_and_types.size(), 1);
  ASSERT_EQ(std::get<0>(names_and_types.at(0)), "name in forward dir");

  node_id = CBA_edge->endnode();
  tile = graph_reader.GetGraphTile(node_id);
  edgeinfo = tile->edgeinfo(CBA_edge);

  names_and_types = edgeinfo.GetNamesAndTypes(true);
  ASSERT_EQ(names_and_types.size(), 1);
  ASSERT_EQ(std::get<0>(names_and_types.at(0)), "name in backward dir");
}
