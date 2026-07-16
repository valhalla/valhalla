#include "gurka.h"
#include "mjolnir/osmdata.h"
#include "mjolnir/osmway.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace valhalla;
using namespace valhalla::baldr;

TEST(area_routing, square_skipped_routes_around) {
  const std::string ascii_map = R"(
    F--------G
    |        |
    A----B   |
    |    |   |
    D----C   |
         |   |
         E---H
  )";

  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}, {"name", "square"}}},
      {"FG", {{"highway", "footway"}, {"name", "top"}}},
      {"GH", {{"highway", "footway"}, {"name", "right"}}},
      {"EH", {{"highway", "footway"}, {"name", "bottom"}}},
      {"FA", {{"highway", "footway"}, {"name", "entry"}}},
      {"CE", {{"highway", "footway"}, {"name", "exit"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_square",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}});

  const auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));

  // we don't want to route the perimeter of an area, so if it is a pedestrian area, do not make edges
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "A", "B"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "B", "C"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "C", "D"), std::runtime_error);
  EXPECT_THROW(gurka::findEdgeByNodes(*reader, map.nodes, "D", "A"), std::runtime_error);

  // if the square is correctly skipped it should route around the outside (F -> G -> H -> E).
  // once area routing is implemented the route should cross the square
  auto result = gurka::do_action(valhalla::Options::route, map, {"F", "E"}, "pedestrian");
  gurka::assert::raw::expect_path(result, {"top", "right", "bottom"});
}

TEST(area_routing, area_bit_is_set) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";
  const gurka::ways ways = {
      {"ABCDA", {{"highway", "pedestrian"}, {"area", "yes"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);

  // build with pedestrian_areas enabled, stop after parsing ways
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_area_bit_on",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                               mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseWays);

  auto way = gurka::findWay(map, "ABCDA");
  // we expect the area_bit to be true
  EXPECT_TRUE(way.area());
}

TEST(area_routing, area_member_ways_collected) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";

  // the member way has no routable tags, so lua discards it during ParseWays
  const gurka::ways ways = {
      {"ABCDA", {}},
  };

  // the relation carries the area tags
  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string workdir = "test/data/gurka_area_members";

  // run up to ParseRelations, the member way was discarded by lua
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, workdir,
                        {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                        mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseRelations);

  EXPECT_THROW(gurka::findWay(map, "ABCDA"), std::runtime_error);

  // run the new stage on top of the partial build
  gurka::buildtiles(layout, ways, {}, relations, workdir,
                    {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                    mjolnir::BuildStage::kParseAreaWays, mjolnir::BuildStage::kParseAreaWays);

  // now the member way was collected with the area bit set
  auto way = gurka::findWay(map, "ABCDA");
  EXPECT_TRUE(way.area());

  // its geometry was collected too. Coords aren't yet since ParseNodes runs after ParseAreaWays
  auto b_nodes = gurka::findWayNodes(map, "B");
  ASSERT_FALSE(b_nodes.empty());
  EXPECT_FALSE(b_nodes.front().node.latlng().IsValid());
}

TEST(area_routing, area_relations_collected) {
  const std::string ascii_map = R"(
    A----B
    |    |
    D----C
  )";

  // the member way has no routable tags, so lua discards it during ParseWays
  const gurka::ways ways = {
      {"ABCDA", {}},
  };

  // the relation carries the area tags
  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  const std::string workdir = "test/data/gurka_area_relations";

  // run up to ParseRelations so area_relations is filled and written to temp files
  auto map =
      gurka::buildtiles(layout, ways, {}, relations, workdir,
                        {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                        mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kParseRelations);

  // load the OSMData from the temp files and check the area relation members
  const auto tile_dir = map.config.get<std::string>("mjolnir.tile_dir");
  mjolnir::OSMData osmdata{};
  osmdata.read_from_temp_files(tile_dir);

  // the area relation collected its member way as an outer
  ASSERT_FALSE(osmdata.area_relations.empty());
  const auto way_id = map.way_osm_ids.at("ABCDA");
  bool found_outer = false;
  for (const auto& [rel_id, member] : osmdata.area_relations) {
    if (member.way_id == way_id && member.is_outer) {
      found_outer = true;
    }
  }
  EXPECT_TRUE(found_outer);
}

TEST(area_routing, area_polygons_assembled) {
  const std::string ascii_map = R"(
    W         X      Y         Z
    |         |      |         |
    A---------B      M---------N
    |         |      |         |
    |  E---F  |      |  Q---R  |
    |  |   |  |      |  |   |  |
    |  G---H  |      |  S---T  |
    |         |      |         |
    D---------C      P---------O
  )";

  const gurka::ways ways = {
      {"ABCDA", {}},
      {"EFHGE", {}},
      {"MNO", {}},
      {"OPM", {}},
      {"QRTSQ", {}},
      {"WA", {{"highway", "footway"}}},
      {"XB", {{"highway", "footway"}}},
      {"YM", {{"highway", "footway"}}},
      {"ZN", {{"highway", "footway"}}},
  };

  const gurka::relations relations = {
      {{{
           {gurka::way_member, "ABCDA", "outer"},
           {gurka::way_member, "EFHGE", "inner"},
           {gurka::way_member, "MNO", "outer"},
           {gurka::way_member, "OPM", "outer"},
           {gurka::way_member, "QRTSQ", "inner"},
       }},
       {{"type", "multipolygon"}, {"highway", "pedestrian"}, {"area", "yes"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 10);

  auto map = gurka::buildtiles(layout, ways, {}, relations, "test/data/gurka_area_polygons",
                               {{"mjolnir.concurrency", "1"}, {"mjolnir.pedestrian_areas", "true"}},
                               mjolnir::BuildStage::kInitialize, mjolnir::BuildStage::kBuildAreas);

  SUCCEED();
}