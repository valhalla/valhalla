#include "baldr/graphreader.h"
#include "gurka.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::gurka;

class FilterData : public ::testing::Test {
protected:
  static gurka::map the_map;

  static void SetUpTestSuite() {
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

    const auto layout =
        gurka::detail::map_to_coordinates(ascii_map, gridsize_metres, {-82.68811, 40.22535});

    the_map =
        gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/gurka_filter",
                          {{"mjolnir.admin", {VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"}},
                           {"mjolnir.include_pedestrian", "false"}});
  }
};

gurka::map FilterData::the_map = {};

///////////////////////////////////////////////////////////////////////////////
TEST_F(FilterData, CheckStreetNamesAfterFilter) {
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

TEST(Standalone, FerryEdges) {
  // Ferry edges get their speed based on the `duration` tag. It might go wrong if car and pedestrian
  // ramps are mapped as separate edge so with `--mjolnir-include-pedestrian=False` when small edge
  // between two ramps merges with ferry edge, the overall speed gets screwed.

  const std::string ascii_map = R"(
    A---B-C                                G
         \ \                              /
          D=E============================F
    )";
  const gurka::ways ways = {
      {"ABD", {{"highway", "service"}, {"foot", "yes"}}},
      {"BCE", {{"highway", "footway"}}},
      {"DEF", {{"route", "ferry"}, {"duration", "1:00"}}}, // 1h
      {"FG", {{"highway", "service"}, {"foot", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 200);

  {
    auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_filter_ferry",
                                 {{"mjolnir.include_pedestrian", "true"}});
    baldr::GraphReader reader(map.config.get_child("mjolnir"));
    const auto big = gurka::findEdge(reader, layout, "DEF", "F");
    EXPECT_EQ(std::get<1>(big)->speed(), 6);
    EXPECT_EQ(std::get<1>(big)->length(), 5800);
    EXPECT_EQ(std::get<3>(big)->speed(), 6);
    EXPECT_EQ(std::get<3>(big)->length(), 5800);

    // Smaller edge should have the same speed.
    const auto small = gurka::findEdge(reader, layout, "DEF", "D");
    EXPECT_EQ(std::get<1>(small)->speed(), 6);
    EXPECT_EQ(std::get<1>(small)->length(), 400);
    EXPECT_EQ(std::get<3>(small)->speed(), 6);
    EXPECT_EQ(std::get<3>(small)->length(), 400);
  }

  {
    auto map =
        gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_filter_ferry_no_pedestrian",
                          {{"mjolnir.admin", VALHALLA_SOURCE_DIR "test/data/language_admin.sqlite"},
                           {"mjolnir.include_pedestrian", "false"}});
    baldr::GraphReader reader(map.config.get_child("mjolnir"));

    const auto big = gurka::findEdge(reader, layout, "DEF", "F");
    EXPECT_EQ(std::get<1>(big)->speed(), 6);
    EXPECT_EQ(std::get<1>(big)->length(), 6199); // merged
    EXPECT_EQ(std::get<3>(big)->speed(), 6);
    EXPECT_EQ(std::get<3>(big)->length(), 6199);

    // Small and big edges got merged.
    const auto small = gurka::findEdge(reader, layout, "DEF", "D");
    EXPECT_EQ(std::get<1>(small)->speed(), 6);
    EXPECT_EQ(std::get<1>(small)->length(), 6199);
    EXPECT_EQ(std::get<3>(small)->speed(), 6);
    EXPECT_EQ(std::get<3>(small)->length(), 6199);
    EXPECT_EQ(std::get<0>(big), std::get<2>(small));
    EXPECT_EQ(std::get<2>(big), std::get<0>(small));
  }
}
